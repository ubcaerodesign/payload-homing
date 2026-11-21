#include <Arduino.h>
#include "config.h"
#include "winch.h"

static bool running = false;
static uint32_t end_ms = 0;

static void servo_pulse(int us) {
  ledcWrite(SERVO_PIN, us > 0 ? (uint32_t)((long)us * (1 << SERVO_BITS) / (1000000 / SERVO_HZ)) : 0);
}

void winch_begin() {
  ledcAttach(SERVO_PIN, SERVO_HZ, SERVO_BITS);
  servo_pulse(0);
}

void winch_run(int us, uint32_t for_ms) {
  servo_pulse(us);
  running = true;
  end_ms = millis() + for_ms;
}

bool winch_stop() {
  servo_pulse(0);
  bool was = running;
  running = false;
  return was;
}

bool winch_done() {
  if (running && (int32_t)(millis() - end_ms) >= 0) return winch_stop();
  return false;
}
