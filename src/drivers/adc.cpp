// drivers/adc.cpp
#include <avr/io.h>
#include <util/atomic.h>
#include "adc.h"
#include "config_clock.h"
#include "pins.h"

/**
 * \file
 * \brief ADC driver: free-running on ADC0 with 8× decimation.
 * \details
 * - Input: ADC0, external AREF (ADMUX.REFS=00).
 * - Mode: Free-running (\c ADATE), interrupt enabled (\c ADIE).
 * - Clock: Prescaler chosen to target ~125 kHz from \c F_CPU_CFG.
 * - Filtering: Simple 8-sample boxcar average in ISR; latest averaged value
 *   exposed via \c adc_fb_decim() with atomic read.
 */

/** \brief Latest 8×-decimated (averaged) sample (volatile; updated in ISR). */
static volatile uint16_t fb_decim = 0;
/** \brief Accumulator for 8 samples (ISR scope). */
static uint16_t acc8 = 0;
/** \brief Sample counter modulo 8 (ISR scope). */
static uint8_t cnt8 = 0;

/**
 * \internal
 * \brief Compute ADPS[2:0] bits for ~125 kHz ADC clock from \c F_CPU_CFG.
 * \return Bit mask for ADCSRA.ADPS[2:0].
 * \note Keeps ADC clock within datasheet-recommended range for 10-bit conversions.
 */
static uint8_t _adps_bits()
{
  // choose closest to 125 kHz using F_CPU_CFG
  if (F_CPU_CFG >= 16000000UL)
    return _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0); // /128
  else if (F_CPU_CFG >= 8000000UL)
    return _BV(ADPS2) | _BV(ADPS1); // /64 -> 125k @8M
  else if (F_CPU_CFG >= 4000000UL)
    return _BV(ADPS2) | _BV(ADPS0); // /32 -> 125k @4M
  else if (F_CPU_CFG >= 2000000UL)
    return _BV(ADPS2); // /16 -> 125k @2M
  else
    return _BV(ADPS1) | _BV(ADPS0); // /8  -> 125k @1M
}

/**
 * \brief Initialize ADC0 in free-running mode with interrupt and averaging.
 * \details
 * - ADMUX = 0 → REFS=00 (external AREF), MUX=0000 (ADC0).
 * - ADCSRB = 0 → free-running, AC mux off.
 * - DIDR0 disables digital buffer on ADC0.
 * - ADCSRA enables ADC, auto-trigger, interrupt, prescaler (via \c _adps_bits()).
 * - Starts conversions by setting \c ADSC.
 */
void adc_init()
{
  // ADC0, AREF external
  ADMUX = 0;          // REFS=00, MUX=0000
  ADCSRB = 0;         // free running, AC mux off
  DIDR0 = _BV(ADC0D); // disable digital buffer
  ADCSRA = _BV(ADEN) | _BV(ADATE) | _BV(ADIE) | _adps_bits();
  ADCSRA |= _BV(ADSC);
}

/**
 * \internal
 * \brief ADC conversion-complete ISR: 8× boxcar average into \c fb_decim.
 * \note Runs at end of every conversion; keep as light as possible.
 */
ISR(ADC_vect)
{
  acc8 += ADC;
  if (++cnt8 >= 8)
  {
    fb_decim = acc8 >> 3; // divide by 8
    acc8 = 0;
    cnt8 = 0;
  }
}

/**
 * \brief Get latest 8×-averaged ADC0 sample (atomic read).
 * \return Decimated ADC count (0..1023 for 10-bit ADC).
 */
uint16_t adc_fb_decim()
{
  uint16_t v;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { v = fb_decim; }
  return v;
}
