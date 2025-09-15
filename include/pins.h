#pragma once
#include <Arduino.h>

/**
 * \file
 * \brief Direct I/O helpers and pin map (AVR bit indices, not Arduino pin numbers).
 * \details
 * Defines port bit positions for a flyback/PSU board and provides small inline helpers:
 * - LED control on PC2/PC3
 * - External EN input on PD3
 * - 2-bit range code read on PD4/PD5 (active-low)
 * - Resistor-switch controls on PB6/PB7/PD0/PD1
 *
 * \note All pin macros below are **bit numbers** (e.g., \c PD4 == 4) to be used with
 * DDRx/PORTx/PINx and \c _BV(). They are **not** Arduino digital pin IDs.
 *
 * \warning \c STATUS0 and \c STATUS1 are set to PD4/PD5, which overlap with \c RANGE0/\c RANGE1.
 * If both are intended, reassign to avoid contention.
 */

// --- User pin assignments (bits, not Arduino numbers) ---

/** \brief Gate drive output (bit index on PORTB). */
#define GATE_OUT PB1
/** \brief Feedback input (bit index on PORTC). Typically analog/ADC node. */
#define FB_IN PC0

/** \brief Status line 0 on PORTD.
 *  \warning Overlaps with \c RANGE0 (PD4). */
#define STATUS0 PD4
/** \brief Status line 1 on PORTD.
 *  \warning Overlaps with \c RANGE1 (PD5). */
#define STATUS1 PD5

/** \brief Over-current / analog comparator input (AIN0) on PORTD. */
#define OC_IN PD6 // AIN0 (comparator input)
/** \brief External enable input on PORTD (see EN helpers below). */
#define EN_IN PD3

/** \brief Range select input bit 0 (active-low) on PORTD with pull-up. */
#define RANGE0 PD4
/** \brief Range select input bit 1 (active-low) on PORTD with pull-up. */
#define RANGE1 PD5

/** \brief Green OK LED on PORTC bit 2. */
#define LED_OK PC2
/** \brief Red FAULT LED on PORTC bit 3. */
#define LED_FAULT PC3

/** \brief Resistor-switch output 0 (divider control) on PORTB. */
#define R_SW0 PB6
/** \brief Resistor-switch output 1 on PORTB. */
#define R_SW1 PB7
/** \brief Resistor-switch output 2 on PORTD. */
#define R_SW2 PD0
/** \brief Resistor-switch output 3 on PORTD. */
#define R_SW3 PD1

// --- Port helpers (direct I/O) ---

/**
 * \brief 8-bit variant of \c _BV() for clarity with uint8_t expressions.
 * \param b Bit position (0..7).
 * \return (uint8_t)1U << b
 */
#define _BV8(b) (uint8_t(1U) << (b))

// -----------------------------------------------------------------------------
// LEDs
// -----------------------------------------------------------------------------

/**
 * \brief Drive the OK LED on PC2.
 * \param on \c true turns the LED on (if wired active-high), \c false turns it off.
 * \note Assumes DDRC for PC2 was configured as output elsewhere.
 */
static inline void led_ok_set(bool on)
{
  if (on)
    PORTC |= _BV8(LED_OK);
  else
    PORTC &= ~_BV8(LED_OK);
}

/**
 * \brief Drive the FAULT LED on PC3.
 * \param on \c true turns the LED on (if wired active-high), \c false turns it off.
 * \note Assumes DDRC for PC3 was configured as output elsewhere.
 */
static inline void led_fault_set(bool on)
{
  if (on)
    PORTC |= _BV8(LED_FAULT);
  else
    PORTC &= ~_BV8(LED_FAULT);
}

// -----------------------------------------------------------------------------
// EN (active-low external input on PD3)
// -----------------------------------------------------------------------------

/**
 * \brief Configure PD3 as a plain input for external EN (no internal pull-up).
 * \details
 * - Sets DDRD.PD3 = 0 (input) and clears PORTD.PD3 (disables pull-up).
 * - Clears TCCR2A.COM2B[1:0] to ensure Timer2 isn't driving OC2B on PD3.
 */
static inline void EN_init_input()
{
  // Input, NO internal pull-up (external pulldown).
  DDRD &= ~_BV(PD3);
  PORTD &= ~_BV(PD3); // ensure pull-up is OFF
  // Make sure Timer2 isn't driving OC2B on PD3
  TCCR2A &= ~(_BV(COM2B1) | _BV(COM2B0));
}

/**
 * \brief Read the EN input level.
 * \return \c true if PD3 is high, \c false if low.
 * \note EN is active-high, \c false means "disabled".
 */
static inline bool EN_is_active() { return (PIND & _BV(PD3)) != 0; }

// -----------------------------------------------------------------------------
// Range inputs (1-of-4) on PD4/PD5 with pull-ups (active-low)
// -----------------------------------------------------------------------------

/**
 * \brief Configure PD4 and PD5 as inputs with internal pull-ups for range code.
 */
static inline void range_inputs_init()
{
  DDRD &= ~(_BV8(RANGE0) | _BV8(RANGE1));
  PORTD |= (_BV8(RANGE0) | _BV8(RANGE1)); // pull-ups
}

/**
 * \brief Read the 2-bit range code (active-low).
 * \return Value 0..3 computed as \c (!PD5)<<1 | (!PD4).
 * \note Because of pull-ups, a grounded/low pin reads as 0; inversion yields the code.
 */
static inline uint8_t range_code_read()
{ // 0..3
  uint8_t r0 = (PIND & _BV8(RANGE0)) ? 1 : 0;
  uint8_t r1 = (PIND & _BV8(RANGE1)) ? 1 : 0;
  return (uint8_t)((!r1) << 1 | (!r0)); // active-low to code
}

// -----------------------------------------------------------------------------
// Resistor-divider switches (outputs)
// -----------------------------------------------------------------------------

/**
 * \brief Initialize R_SW pins as outputs (state unchanged).
 * \details Sets DDRB bits for PB6/PB7 and DDRD bits for PD0/PD1.
 * \note Call rsw_all_open() after init if you need a defined low/off state.
 */
static inline void rsw_init()
{
  DDRB |= _BV8(R_SW0) | _BV8(R_SW1);
  DDRD |= _BV8(R_SW2) | _BV8(R_SW3);
}

/**
 * \brief Open all resistor-switches (drive control lines low).
 * \details Clears PORTB bits PB6/PB7 and PORTD bits PD0/PD1.
 * \note Adjust if your hardware uses opposite polarity.
 */
static inline void rsw_all_open()
{
  PORTB &= ~(_BV8(R_SW0) | _BV8(R_SW1));
  PORTD &= ~(_BV8(R_SW2) | _BV8(R_SW3));
}

/**
 * \brief Close exactly one resistor-switch according to a 2-bit code.
 * \param code Lower 2 bits select which switch: 0→PB6, 1→PB7, 2→PD0, 3→PD1.
 * \details Clears all switches via rsw_all_open(), then sets the selected one.
 * \warning Verify mapping vs. PCB; change the switch/case if wiring differs.
 */
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
