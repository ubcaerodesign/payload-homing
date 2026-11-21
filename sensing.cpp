#include <Arduino.h>
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_timer.h"
#include "config.h"
#include "sensing.h"

static const int NUM_ADC1 = 10;
static const int NUM_ADC2 = 7;
static const int ADC2_GPIO[NUM_ADC2] = {11, 12, 13, 14, 16, 17, 18};
static const adc_channel_t ADC2_CH[NUM_ADC2] = {ADC_CHANNEL_0, ADC_CHANNEL_1, ADC_CHANNEL_2, ADC_CHANNEL_3,
                                                ADC_CHANNEL_5, ADC_CHANNEL_6, ADC_CHANNEL_7};

static const int NUM_BINS = 4;
static const float TONE_HZ[NUM_BINS] = {500, 800, 1340, 1110};

static float goertzel_magnitude(float a, float b, float c) { return sqrtf(fmaxf(0, a * a + b * b - c * a * b)); }

static const uint32_t SAMPLE_HZ = 50000;
static const int N1 = 500;
static const uint32_t FRAME_CONV = NUM_ADC1 * 50;
static const uint32_t FRAME_BYTES = FRAME_CONV * SOC_ADC_DIGI_RESULT_BYTES;
static const uint32_t POOL_BYTES = FRAME_BYTES * 16;

static adc_continuous_handle_t adc1 = NULL;
static uint8_t raw[FRAME_BYTES];
static adc_continuous_data_t parsed[FRAME_CONV];
static float coeff1[NUM_BINS];
static float s1a[NUM_ADC1][NUM_BINS], s2a[NUM_ADC1][NUM_BINS];
static float amp_a[NUM_ADC1][NUM_BINS];
static uint32_t in_window1 = 0;

static const int64_t ROUND_HZ = 2740;
static const int N2 = 274;
static const int64_t LATE_US = 50;
static adc_oneshot_unit_handle_t adc2 = NULL;
static float coeff2[NUM_BINS];
static float s1b[NUM_ADC2][NUM_BINS], s2b[NUM_ADC2][NUM_BINS];
static int in_window2 = 0;
static int64_t first_round_us = 0, rounds = 0;
static int64_t max_late_us = 0;
static volatile uint32_t seq = 0;
static float amp_b[NUM_ADC2][NUM_BINS];
static volatile uint32_t windows_b = 0, late_b = 0;
static volatile int32_t max_late_b = 0;

static void (*window_cb)(const ctrl_in *in, const window_stats *stats) = NULL;
static void (*tick_cb)() = NULL;
static ctrl_in input;

static bool IRAM_ATTR on_overflow(adc_continuous_handle_t h, const adc_continuous_evt_data_t *e, void *user) {
  return false;
}

static void check(esp_err_t err, const char *what) {
  if (err == ESP_OK) return;
  while (true) {
    Serial.printf("%s failed: %s\n", what, esp_err_to_name(err));
    delay(1000);
  }
}

static void finish_window() {
  float b2[NUM_ADC2][NUM_BINS];
  uint32_t before, n_b, l_b;
  int32_t m_b;
  do {
    before = seq;
    memcpy(b2, amp_b, sizeof b2);
    n_b = windows_b;
    l_b = late_b;
    m_b = max_late_b;
  } while ((before & 1) || before != seq);
  static uint32_t last_n = 0, last_l = 0;
  window_stats stats = {n_b - last_n, l_b - last_l, m_b};
  last_n = n_b;
  last_l = l_b;

  for (int d = 0; d < NDET; d++) {
    int pin = DET_PIN[d];
    const float *a = NULL;
    if (pin <= 10) a = amp_a[pin - 1];
    else
      for (int k = 0; k < NUM_ADC2; k++)
        if (ADC2_GPIO[k] == pin) a = b2[k];
    for (int b = 0; b < NBEACON; b++) input.magnitude[d][b] = a[b] / GAIN[d];
    input.quiet_magnitude[d] = a[3] / GAIN[d];
  }
  window_cb(&input, &stats);
}

static void adc1_task(void *arg) {
  for (;;) {
    uint32_t got = 0;
    esp_err_t err = adc_continuous_read(adc1, raw, FRAME_BYTES, &got, 1000);
    if (err == ESP_ERR_TIMEOUT) {
      Serial.printf("P %.1f no ADC1 data for 1 s\n", esp_timer_get_time() / 1000.0);
      continue;
    }
    check(err, "adc_continuous_read");
    uint32_t n = 0;
    check(adc_continuous_parse_data(adc1, raw, got, parsed, &n), "adc_continuous_parse_data");
    for (uint32_t i = 0; i < n; i++) {
      if (!parsed[i].valid || parsed[i].unit != ADC_UNIT_1 || parsed[i].channel >= NUM_ADC1) continue;
      int k = parsed[i].channel;
      int x = parsed[i].raw_data;
      for (int b = 0; b < NUM_BINS; b++) {
        float s0 = x + coeff1[b] * s1a[k][b] - s2a[k][b];
        s2a[k][b] = s1a[k][b];
        s1a[k][b] = s0;
      }
      if (++in_window1 == (uint32_t)(N1 * NUM_ADC1)) {
        for (int e = 0; e < NUM_ADC1; e++)
          for (int b = 0; b < NUM_BINS; b++) {
            amp_a[e][b] = 2 * goertzel_magnitude(s1a[e][b], s2a[e][b], coeff1[b]) / N1;
            s1a[e][b] = s2a[e][b] = 0;
          }
        in_window1 = 0;
        finish_window();
      }
    }
    tick_cb();
  }
}

void sensing_begin(void (*on_window)(const ctrl_in *in, const window_stats *stats), void (*on_tick)()) {
  window_cb = on_window;
  tick_cb = on_tick;
  for (int b = 0; b < NUM_BINS; b++) {
    coeff1[b] = 2.0f * cosf(2.0f * PI * TONE_HZ[b] / (SAMPLE_HZ / NUM_ADC1));
    coeff2[b] = 2.0f * cosf(2.0f * PI * TONE_HZ[b] / ROUND_HZ);
  }

  adc_continuous_handle_cfg_t handle_cfg = {};
  handle_cfg.max_store_buf_size = POOL_BYTES;
  handle_cfg.conv_frame_size = FRAME_BYTES;
  check(adc_continuous_new_handle(&handle_cfg, &adc1), "adc_continuous_new_handle");
  adc_digi_pattern_config_t pattern[NUM_ADC1];
  for (int k = 0; k < NUM_ADC1; k++) {
    pattern[k].atten = ADC_ATTEN_DB_12;
    pattern[k].channel = k;
    pattern[k].unit = ADC_UNIT_1;
    pattern[k].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;
  }
  adc_continuous_config_t acfg = {};
  acfg.pattern_num = NUM_ADC1;
  acfg.adc_pattern = pattern;
  acfg.sample_freq_hz = SAMPLE_HZ;
  acfg.conv_mode = ADC_CONV_SINGLE_UNIT_1;
  acfg.format = ADC_DIGI_OUTPUT_FORMAT_TYPE2;
  check(adc_continuous_config(adc1, &acfg), "adc_continuous_config");
  adc_continuous_evt_cbs_t cbs = {};
  cbs.on_pool_ovf = on_overflow;
  check(adc_continuous_register_event_callbacks(adc1, &cbs, NULL), "adc_continuous_register_event_callbacks");

  adc_oneshot_unit_init_cfg_t unit_cfg = {};
  unit_cfg.unit_id = ADC_UNIT_2;
  check(adc_oneshot_new_unit(&unit_cfg, &adc2), "adc_oneshot_new_unit");
  adc_oneshot_chan_cfg_t chan_cfg = {};
  chan_cfg.atten = ADC_ATTEN_DB_12;
  chan_cfg.bitwidth = ADC_BITWIDTH_12;
  for (int k = 0; k < NUM_ADC2; k++)
    check(adc_oneshot_config_channel(adc2, ADC2_CH[k], &chan_cfg), "adc_oneshot_config_channel");

  check(adc_continuous_start(adc1), "adc_continuous_start");
  xTaskCreatePinnedToCore(adc1_task, "adc1", 16384, NULL, 5, NULL, 0);
  first_round_us = esp_timer_get_time() + 1000;
}

void sensing_poll_adc2() {
  int64_t due = first_round_us + rounds * 1000000 / ROUND_HZ;
  int64_t start = esp_timer_get_time();
  while (start < due) start = esp_timer_get_time();
  int64_t late_by = start - due;
  if (late_by > max_late_us) max_late_us = late_by;
  if (late_by > LATE_US) late_b = late_b + 1;
  rounds++;
  for (int k = 0; k < NUM_ADC2; k++) {
    int x = 0;
    adc_oneshot_read(adc2, ADC2_CH[k], &x);
    for (int b = 0; b < NUM_BINS; b++) {
      float s0 = x + coeff2[b] * s1b[k][b] - s2b[k][b];
      s2b[k][b] = s1b[k][b];
      s1b[k][b] = s0;
    }
  }
  if (++in_window2 == N2) {
    seq++;
    for (int k = 0; k < NUM_ADC2; k++)
      for (int b = 0; b < NUM_BINS; b++) {
        amp_b[k][b] = 2 * goertzel_magnitude(s1b[k][b], s2b[k][b], coeff2[b]) / N2;
        s1b[k][b] = s2b[k][b] = 0;
      }
    windows_b = windows_b + 1;
    max_late_b = (int32_t)max_late_us;
    seq++;
    max_late_us = 0;
    in_window2 = 0;
  }
}
