#pragma once
#include <stdint.h>
#include "types.h"

void controller_init_defaults(ControlParams& c);
void controller_reset(ControlState& s, uint32_t now_us);

// One control step: returns fb (counts) and derivative (counts/ms)
void controller_step(uint32_t now_us,
                     ControlParams& c,
                     ControlState& s,
                     uint16_t fb_raw_counts,
                     uint8_t oc_fault_now,
                     bool allow_switch,
                     uint16_t& fb_out,
                     int16_t& dcounts_out);
