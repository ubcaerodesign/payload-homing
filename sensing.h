#pragma once

#include <stdint.h>
#include "controller.h"

typedef struct {
  uint32_t adc2_new_windows;
  uint32_t adc2_late_rounds;
  int32_t adc2_max_late_us;
} window_stats;

void sensing_begin(void (*on_window)(const ctrl_in *in, const window_stats *stats), void (*on_tick)());
void sensing_poll_adc2();
