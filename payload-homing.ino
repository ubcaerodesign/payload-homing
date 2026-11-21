#include "esp_timer.h"
#include "config.h"
#include "controller.h"
#include "motors.h"
#include "sensing.h"
#include "winch.h"

ctrl_cfg config;
ctrl_state state;
ctrl_out output;

volatile int run_state = 0;
int run_windows = 0;
char line[16];
int line_len = 0;

double ms() { return esp_timer_get_time() / 1000.0; }

void build_config() {
  memset(&config, 0, sizeof config);
  config.beacon_xy[0][0] = -TRACK / 2;
  config.beacon_xy[0][1] = -BASE / 3;
  config.beacon_xy[1][0] = TRACK / 2;
  config.beacon_xy[1][1] = -BASE / 3;
  config.beacon_xy[2][0] = 0;
  config.beacon_xy[2][1] = 2 * BASE / 3;
  for (int b = 0; b < NBEACON; b++) config.hoop_radius[b] = HOOP_RADIUS[b];
  config.payload_radius = PAYLOAD_RADIUS;
  for (int d = 0; d < NDET; d++) {
    double a = (d * 22.5 + AXIS_OFFSET_DEG[d]) * PI / 180;
    config.detector_angle[d] = atan2(sin(a), cos(a));
  }
  for (int m = 0; m < NWHEEL; m++) config.wheel_angle[m] = WHEEL_DEG[m] * PI / 180;
  config.top_speed = TOP_SPEED;
  config.dodge_clearance = DODGE_CLEARANCE;
  config.taper_start = TAPER_START;
  config.memory_time = MEMORY_TIME;
  config.hidden_margin = HIDDEN_MARGIN_DEG * PI / 180;
  config.resect_limit = RESECT_LIMIT_M_PER_DEG / (PI / 180);
  config.motion_trust = MOTION_TRUST;
  config.fix_trust = FIX_TRUST;
  config.lost_limit = LOST_LIMIT;
}

void winch_in(const char *why) {
  winch_run(WINCH_IN_US, WINCH_IN_MS);
  Serial.printf("P %.1f winch %s: %d us for %lu ms\n", ms(), why, WINCH_IN_US, (unsigned long)WINCH_IN_MS);
}

void winch_off(const char *why) {
  if (winch_stop()) Serial.printf("P %.1f winch %s, no pulses\n", ms(), why);
}

void set_state(int s, const char *why) {
  run_state = s;
  if (s == 1) {
    ctrl_init(&config, &state);
    run_windows = 0;
  }
  if (s != 1) stop_motors();
  Serial.printf("P %.1f %s\n", ms(), why);
}

void on_window(const ctrl_in *in, const window_stats *stats) {
  if (run_state != 1) return;

  int64_t t0 = esp_timer_get_time();
  ctrl_step(&config, &state, in, &output);
  uint32_t step_us = (uint32_t)(esp_timer_get_time() - t0);
  run_windows++;
  if (output.arrived) {
    set_state(2, "stopped: the controller believes it is on the centroid");
    winch_in("pulling in");
  } else if (run_windows >= MAX_RUN_WINDOWS) {
    set_state(2, "stopped: timeout, winch left alone");
  } else {
    drive(output.wheel_speed);
  }

  int detected = 0, used = 0;
  for (int b = 0; b < NBEACON; b++) {
    detected |= output.detected[b] << b;
    used |= output.used[b] << b;
  }
  Serial.printf("W %.1f %d %d %d %d %d  %.4f %.4f  %.4f %.4f %.4f  %d %d %d  %.4f %.4f  %lu ", ms(), run_state,
                output.branch, detected, used, output.arrived, output.vx, output.vy, output.wheel_speed[0],
                output.wheel_speed[1], output.wheel_speed[2], motor_duty[0], motor_duty[1], motor_duty[2],
                output.centroid_known ? output.centroid_x : NAN, output.centroid_known ? output.centroid_y : NAN,
                (unsigned long)step_us);
  for (int d = 0; d < NDET; d++)
    for (int b = 0; b < NBEACON; b++) Serial.printf(" %.1f", in->magnitude[d][b]);
  for (int d = 0; d < NDET; d++) Serial.printf(" %.2f", in->quiet_magnitude[d]);
  Serial.printf("  %lu %lu %ld\n", (unsigned long)stats->adc2_new_windows, (unsigned long)stats->adc2_late_rounds,
                (long)stats->adc2_max_late_us);
}

void command(const char *c) {
  if (c[0] == 's') {
    set_state(2, "stopped by command");
    winch_off("stopped by command");
  } else if (c[0] == 'g') {
    winch_off("stopped for a new run");
    set_state(1, "started by command");
  } else if (c[0] == 'w' || c[0] == 'u') {
    if (run_state == 1) {
      Serial.printf("P %.1f the winch waits until the run is over\n", ms());
      return;
    }
    if (c[0] == 'w') {
      winch_in("in by command");
    } else {
      winch_run(WINCH_OUT_US, WINCH_OUT_MS);
      Serial.printf("P %.1f winch out by command: %d us for %lu ms\n", ms(), WINCH_OUT_US, (unsigned long)WINCH_OUT_MS);
    }
  } else {
    Serial.printf("P %.1f unknown command: %s\n", ms(), c);
  }
}

void on_tick() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r' || c == '\n') {
      line[line_len] = 0;
      if (line_len > 0) command(line);
      line_len = 0;
    } else if (line_len < (int)sizeof line - 1) {
      line[line_len++] = c;
    }
  }
  if (winch_done()) Serial.printf("P %.1f winch done, no pulses\n", ms());
  static bool auto_checked = false;
  if (!auto_checked && millis() >= AUTO_START_MS) {
    auto_checked = true;
    if (run_state == 0 && !HWCDC::isPlugged()) set_state(1, "started on its own, no USB");
  }
}

void setup() {
  Serial.setTxBufferSize(8192);
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);
  motors_begin();
  winch_begin();
  build_config();
  sensing_begin(on_window, on_tick);
  Serial.printf("P %.1f ready: g starts, s stops, w winds in, u lets out\n", ms());
}

void loop() { sensing_poll_adc2(); }
