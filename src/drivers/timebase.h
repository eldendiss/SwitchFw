// drivers/timebase.h
#pragma once
#include <stdint.h>
void timebase_init_us();
uint32_t us_now32();
uint64_t us_now();
