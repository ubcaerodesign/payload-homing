#pragma once

#include "config.h"

void motors_begin();
void drive(const double *wheel_speed);
void stop_motors();
extern int motor_duty[NUM_MOTORS];
