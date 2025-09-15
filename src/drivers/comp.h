// drivers/comp.h
#pragma once
#include <stdint.h>
void comp_init();
void comp_arm_after_us(uint32_t delay_us, uint32_t now_us);
void comp_service(uint32_t now_us);
uint8_t comp_oc_fault(); // momentary OC
uint8_t comp_oc_pulse_sticky();    // NEW: one-shot sticky, cleared by reader
extern volatile uint8_t g_allow_switch;
