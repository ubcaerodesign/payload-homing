#pragma once

#include <stdint.h>

void winch_begin();
void winch_run(int us, uint32_t for_ms);
bool winch_stop();
bool winch_done();
