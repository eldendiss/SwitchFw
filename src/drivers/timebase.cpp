// drivers/timebase.cpp
#include <avr/io.h>
#include <avr/interrupt.h>
#include "timebase.h"

/**
 * \file
 * \brief 1 µs-resolution system timebase using Timer2 (8-bit) in normal mode.
 * \details
 * - Timer2 runs with prescaler /8 at 8 MHz CPU → 1 MHz timer clock → 1 tick = 1 µs.
 * - An overflow ISR accumulates high-order microseconds in \c us_high (+=256 each OVF).
 * - \c us_now32() reads a coherent 32-bit µs counter with overflow check.
 * - \c us_now() extends to 64-bit by tracking 32-bit wrap-around in software.
 *
 * \note This configuration assumes \c F_CPU (or effective clock) is 8 MHz. If you
 * change the CPU clock or prescaler, update the Timer2 prescale accordingly to keep 1 µs ticks.
 */

/// \internal High-order microseconds accumulated in 256-µs blocks by the OVF ISR.
static volatile uint32_t us_high = 0;

/**
 * \brief Initialize Timer2 for a 1 MHz tick (1 µs per count) and enable OVF ISR.
 * \details
 * - Normal mode (no PWM), OC2A/OC2B disconnected.
 * - Prescaler = /8 (at 8 MHz CPU → 1 MHz timer clock).
 * - Clears pending overflow and enables \c TIMER2_OVF_vect.
 */
void timebase_init_us()
{
  // CPU = 8 MHz. Timer2 prescaler = 8  => 1 MHz timer clock => 1 tick = 1 µs
  TCCR2A = 0;         // normal mode, OC2A/OC2B disconnected
  TCCR2B = _BV(CS21); // prescaler /8
  TCNT2 = 0;
  TIFR2 = _BV(TOV2);    // clear pending OVF
  TIMSK2 |= _BV(TOIE2); // enable overflow interrupt
}

/**
 * \internal
 * \brief Timer2 overflow ISR — accumulate high-order microseconds.
 * \details Each OVF covers 256 timer counts → 256 µs at 1 MHz tick.
 */
ISR(TIMER2_OVF_vect) { us_high += 256; }

/**
 * \brief Get a coherent 32-bit microsecond timestamp.
 * \return Monotonic time in µs since \c timebase_init_us(), modulo 2^32.
 * \details
 * Reads \c TCNT2 and \c TIFR2 atomically and compensates for an overflow that
 * may have occurred between reading \c TCNT2 and \c us_high.
 */
uint32_t us_now32()
{
  uint32_t base;
  uint8_t t, tifr, s = SREG;
  cli();
  t = TCNT2;
  tifr = TIFR2;
  base = us_high;
  SREG = s;
  if ((tifr & _BV(TOV2)) && (t < 255))
    base += 256;
  return base + t;
}

/**
 * \brief Get a 64-bit microsecond timestamp (wrap-extended).
 * \return Monotonic time in µs since \c timebase_init_us(), 64-bit.
 * \details
 * Extends the 32-bit counter returned by \c us_now32() by detecting wrap-around
 * and accumulating a 64-bit high part. Suitable for long uptimes.
 */
uint64_t us_now()
{
  static uint32_t last = 0;
  static uint64_t high = 0;
  uint32_t cur = us_now32();
  if (cur < last)
    high += (1ULL << 32);
  last = cur;
  return (high | cur);
}
