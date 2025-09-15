#pragma once
#include <Arduino.h>

// --- User pin assignments (bits, not Arduino numbers) ---
#define GATE_OUT PB1
#define FB_IN PC0
#define STATUS0 PC2
#define STATUS1 PC3

#define OC_IN PD6 // AIN0 (comparator input)
#define EN_IN PD3
#define RANGE0 PD4
#define RANGE1 PD5

#define LED_OK PC2
#define LED_FAULT PC3

#define R_SW0 PB6
#define R_SW1 PB7
#define R_SW2 PD0
#define R_SW3 PD1

// --- Port helpers (direct I/O) ---
#define _BV8(b) (uint8_t(1U) << (b))

// LEDs
static inline void led_ok_set(bool on)
{
  if (on)
    PORTC |= _BV8(LED_OK);
  else
    PORTC &= ~_BV8(LED_OK);
}
static inline void led_fault_set(bool on)
{
  if (on)
    PORTC |= _BV8(LED_FAULT);
  else
    PORTC &= ~_BV8(LED_FAULT);
}

// EN (active-low)
static inline void EN_init_input()
{
  // Input, NO internal pull-up (external pulldown).
  DDRD &= ~_BV(PD3);
  PORTD &= ~_BV(PD3); // ensure pull-up is OFF
  // Make sure Timer2 isn't driving OC2B on PD3
  TCCR2A &= ~(_BV(COM2B1) | _BV(COM2B0));
}
static inline bool EN_is_active() { return (PIND & _BV(PD3)) != 0; }

// Range inputs (1-of-4)
static inline void range_inputs_init()
{
  DDRD &= ~(_BV8(RANGE0) | _BV8(RANGE1));
  PORTD |= (_BV8(RANGE0) | _BV8(RANGE1)); // pull-ups
}
static inline uint8_t range_code_read()
{ // 0..3
  uint8_t r0 = (PIND & _BV8(RANGE0)) ? 1 : 0;
  uint8_t r1 = (PIND & _BV8(RANGE1)) ? 1 : 0;
  return (uint8_t)((!r1) << 1 | (!r0)); // active-low to code
}

// R_SWs (outputs that make the divider)
static inline void rsw_init()
{
  DDRB |= _BV8(R_SW0) | _BV8(R_SW1);
  DDRD |= _BV8(R_SW2) | _BV8(R_SW3);
}
static inline void rsw_all_open()
{
  PORTB &= ~(_BV8(R_SW0) | _BV8(R_SW1));
  PORTD &= ~(_BV8(R_SW2) | _BV8(R_SW3));
}
static inline void rsw_make(uint8_t code)
{
  // 0..3 maps to one of the four switches. Adjust mapping to your HW.
  rsw_all_open();
  switch (code & 3)
  {
  case 0:
    PORTB |= _BV8(R_SW0);
    break;
  case 1:
    PORTB |= _BV8(R_SW1);
    break;
  case 2:
    PORTD |= _BV8(R_SW2);
    break;
  case 3:
    PORTD |= _BV8(R_SW3);
    break;
  }
}
