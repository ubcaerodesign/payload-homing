#include <Arduino.h>
#include "motors.h"

int motor_duty[NUM_MOTORS];

static int duty_for(double v) {
  double deg_s = fabs(v) / WHEEL_FROM_CENTRE * 180 / PI;
  if (deg_s <= 0) return 0;
  if (deg_s >= SPIN_CAL_DEG_S[SPIN_CAL_N - 1]) return 255;
  int i = 1;
  while (SPIN_CAL_DEG_S[i] < deg_s) i++;
  double f = (deg_s - SPIN_CAL_DEG_S[i - 1]) / (SPIN_CAL_DEG_S[i] - SPIN_CAL_DEG_S[i - 1]);
  return (int)lround(SPIN_CAL_DUTY[i - 1] + f * (SPIN_CAL_DUTY[i] - SPIN_CAL_DUTY[i - 1]));
}

void motors_begin() {
  for (int m = 0; m < NUM_MOTORS; m++) {
    ledcAttach(MOTOR_PIN_A[m], PWM_HZ, PWM_BITS);
    ledcAttach(MOTOR_PIN_B[m], PWM_HZ, PWM_BITS);
  }
  stop_motors();
}

void drive(const double *wheel_speed) {
  for (int m = 0; m < NUM_MOTORS; m++) {
    double v = MOTOR_SIGN[m] * wheel_speed[m];
    int d = duty_for(v);
    ledcWrite(MOTOR_PIN_A[m], v > 0 ? d : 0);
    ledcWrite(MOTOR_PIN_B[m], v < 0 ? d : 0);
    motor_duty[m] = v < 0 ? -d : d;
  }
}

void stop_motors() {
  double zero[NUM_MOTORS] = {0, 0, 0};
  drive(zero);
}
