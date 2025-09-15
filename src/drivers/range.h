// drivers/range.h
#pragma once
#include <stdint.h>

void range_init();
uint8_t range_get_input_code();
void range_request(uint8_t code);     // request switch (0..3)
bool range_is_busy();

void range_service(uint32_t now_us, uint16_t fb_counts, int16_t fb_dcounts);

// Params
extern uint32_t RANGE_BREAK_US;     // open time before make
extern uint16_t RANGE_STABLE_DTH;   // |dFB/dms| threshold to consider stable
