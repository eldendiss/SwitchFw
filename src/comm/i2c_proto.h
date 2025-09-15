// comm/i2c_proto.h
#pragma once
#include <stdint.h>
#include "../cfg/config.h"
#include "../control/types.h"

void i2c_init(uint8_t addr);
void i2c_bind(AppConfig* cfg, ControlState* state);

// Simple map (little-endian):
// 0x00: u16 fb_set_cnt (R/W)
// 0x02: u16 fb_ov_trip_cnt (R/W)
// 0x04: u16 fb_ov_clear_cnt (R/W)
// 0x06: f32 KP (R/W) ... etc (add as needed)
// 0x40: cmd (W): 1=save cfg to EEPROM, 2=reset controller
// 0x50: status (R): u8 run_state, u8 fault_code, u16 fb, i16 dcounts
