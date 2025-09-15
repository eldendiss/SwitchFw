#pragma once
#include <stdint.h>

// Timer1 OC1A (PB1) driver.
// Fast PWM mode 14, TOP = ICR1, prescaler = 1.
// Base PWM freq set by ICR1 (defaults to 31.25 kHz @ 8 MHz).

void     pwm_init();

void     pwm_enable();                 // attach OC1A (non-inverting)
void     pwm_disable();                // detach + drive gate LOW
bool     pwm_is_enabled();
void     pwm_enable_sync_bottom();     // attach exactly at next BOTTOM

void     pwm_set_duty(float duty);     // 0..1, hard-clamped internally (default 20%)
void     pwm_set_max(float max_duty);  // set absolute ceiling (0.05..0.95), default 0.20
float    pwm_get_duty();

float    pwm_min_step();               // duty LSB = 1/(ICR1+1) (≈0.39% with ICR1=255)
uint32_t pwm_clk_hz();                 // timer clock (F_CPU_CFG)

// ---- PFM one-shot ----
// Emit one fixed-width pulse (ton_us) starting at the next BOTTOM,
// then auto-detach OC1A at the end of the pulse and keep the gate LOW.
void     pwm_emit_oneshot_us(uint16_t ton_us);
