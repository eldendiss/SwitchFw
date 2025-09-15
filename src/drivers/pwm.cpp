#include <avr/io.h>
#include <avr/interrupt.h>
#include "pwm.h"
#include "pins.h"
#include "config_clock.h"   // F_CPU_CFG (e.g. 8000000)

static float duty_cache  = 0.0f;
static float duty_hw_max = 0.20f;                 // absolute safety ceiling
static volatile uint8_t oneshot_active = 0;

#define PWM_FREQ 31250UL                            // 31.25 kHz @ 8 MHz -> ICR1=255

void pwm_init() {
  // Fast PWM, TOP=ICR1, prescaler=1
  TCCR1A = 0;
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10);
  TCCR1A |= _BV(WGM11);

  ICR1  = (uint16_t)((F_CPU_CFG / PWM_FREQ) - 1);  // 255 at 8 MHz
  OCR1A = 0;

  DDRB  |= _BV(GATE_OUT);
  PORTB &= ~_BV(GATE_OUT);                         // gate hard LOW initially
}

void pwm_set_max(float m) {
  if (m < 0.05f) m = 0.05f;
  if (m > 0.95f) m = 0.95f;
  duty_hw_max = m;
}

void pwm_enable()               { TCCR1A |=  _BV(COM1A1); }
bool pwm_is_enabled()           { return (TCCR1A & _BV(COM1A1)) != 0; }

void pwm_disable() {
  TCCR1A &= ~_BV(COM1A1);                           // detach OC1A
  DDRB   |=  _BV(GATE_OUT);
  PORTB  &= ~_BV(GATE_OUT);                         // force gate LOW
  TIMSK1 &= ~_BV(OCIE1A);
  oneshot_active = 0;
}

void pwm_enable_sync_bottom() {
  if (pwm_is_enabled()) return;
  // Wait (≤ 32 µs @ 31.25 kHz) for OVF (== BOTTOM), then attach
  while (!(TIFR1 & _BV(TOV1))) { /* spin ≤ one period */ }
  TIFR1  = _BV(TOV1);                                 // clear OVF we saw
  TCCR1A |= _BV(COM1A1);                              // non-inverting
}

void pwm_set_duty(float d) {
  if (d < 0.0f) d = 0.0f;
  if (d > duty_hw_max) d = duty_hw_max;
  duty_cache = d;

  const uint16_t top = ICR1;                          // buffered TOP
  uint32_t cmp = (uint32_t)((float)(top + 1) * d);
  if (cmp > top) cmp = top;
  OCR1A = (uint16_t)cmp;                              // buffered; takes effect at BOTTOM
}

float    pwm_get_duty() { return duty_cache; }
float    pwm_min_step() { return 1.0f / (float)(ICR1 + 1); }
uint32_t pwm_clk_hz()   { return F_CPU_CFG; }

// ---- PFM one-shot ----
void pwm_emit_oneshot_us(uint16_t ton_us)
{
  if (ton_us == 0) return;

  // Convert µs -> timer counts (prescaler=1)
  uint32_t counts = (uint32_t)ton_us * (F_CPU_CFG / 1000000UL);
  if (counts < 1) counts = 1;
  if (counts > ICR1) counts = ICR1;                  // cap within period

  OCR1A = (uint16_t)counts;                          // high from BOTTOM..COMPA
  oneshot_active = 1;
  TIMSK1 |= _BV(OCIE1A);                             // detach at end of pulse
  pwm_enable_sync_bottom();                          // start exactly at BOTTOM
}

// End the one-shot when OCR1A fires
ISR(TIMER1_COMPA_vect)
{
  if (oneshot_active) {
    TCCR1A &= ~_BV(COM1A1);                          // detach OC1A
    DDRB   |=  _BV(GATE_OUT);
    PORTB  &= ~_BV(GATE_OUT);                        // hold gate LOW
    oneshot_active = 0;
  }
  TIMSK1 &= ~_BV(OCIE1A);                            // stop COMPA IRQ
}
