// drivers/status_led.h
#pragma once
#include <stdint.h>

// Fault LED codes (STATUS1/LED_FAULT)
enum FaultCode : uint8_t {
  F_NONE=0, F_CFG=1, F_OV=2, F_OC=3, F_EE=4
};

// Status LED (STATUS0/LED_OK) patterns
enum RunState : uint8_t {
  S_BOOT=0, S_WAIT_EN, S_PRECHARGE, S_RAMP, S_REGULATING, S_BURST_HOLD, S_RANGE_SWITCH
};

void led_init();
void led_fault_run(uint32_t now_us, FaultCode code);
void led_status_run(uint32_t now_us, RunState st);
