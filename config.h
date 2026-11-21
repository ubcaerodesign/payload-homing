#pragma once

#include <stdint.h>

#define NUM_MOTORS 3

const int DET_PIN[16] = {9, 10, 13, 11, 12, 14, 8, 7, 2, 17, 18, 16, 3, 5, 6, 4};
const double AXIS_OFFSET_DEG[16] = {3.8, 1.8, 0.4, -0.4, 1.3, -3.2, -3.6, -0.5, 2.7, -1.0, -3.9, -2.1, 0.6, 1.1, 0.5, 2.8};
const double GAIN[16] = {1.02, 0.93, 1.02, 0.91, 0.93, 1.00, 1.17, 0.96, 1.00, 0.98, 0.90, 0.96, 0.99, 1.09, 1.07, 1.09};

const int MOTOR_PIN_A[NUM_MOTORS] = {21, 39, 41};
const int MOTOR_PIN_B[NUM_MOTORS] = {40, 38, 42};
const double WHEEL_DEG[NUM_MOTORS] = {300, 60, 180};
const int MOTOR_SIGN[NUM_MOTORS] = {-1, 1, -1};
const int PWM_HZ = 5000;
const int PWM_BITS = 8;
const int SPIN_CAL_N = 8;
const double SPIN_CAL_DUTY[SPIN_CAL_N] = {0, 40, 60, 80, 100, 150, 200, 255};
const double SPIN_CAL_DEG_S[SPIN_CAL_N] = {0, 10, 30, 43, 63, 95, 127, 162};
const double WHEEL_FROM_CENTRE = 0.08;

const int SERVO_PIN = 47;
const int SERVO_HZ = 50;
const int SERVO_BITS = 14;
const int WINCH_IN_US = 500;
const uint32_t WINCH_IN_MS = 4000;
const int WINCH_OUT_US = 1750;
const uint32_t WINCH_OUT_MS = 3000;

const double INCH = 0.0254;
const double TRACK = 26.3 * INCH;
const double BASE = 13.5 * INCH;
const double HOOP_RADIUS[3] = {0.05, 0.05, 0.04};
const double PAYLOAD_RADIUS = 3 * INCH;

const double TOP_SPEED = 0.14;
const double DODGE_CLEARANCE = 0.06;
const double TAPER_START = 0.30;
const double MEMORY_TIME = 2;
const double HIDDEN_MARGIN_DEG = 4;
const double RESECT_LIMIT_M_PER_DEG = 0.025;
const double MOTION_TRUST = 1;
const double FIX_TRUST = 2;
const double LOST_LIMIT = 0.05;

const int MAX_RUN_WINDOWS = 30 + 450;
const uint32_t AUTO_START_MS = 5000;
