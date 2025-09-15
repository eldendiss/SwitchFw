#pragma once
#include <avr/io.h>
#include <avr/interrupt.h>

/**
 * \file
 * \brief AVR clock prescaler utilities.
 * \details
 * Provides a compile-time configuration macro to derive a configured CPU clock
 * from \c F_CPU and a small helper to change the clock prescaler at runtime
 * using the required timed sequence (CLKPCE unlock). Interrupts are masked
 * around the sequence and previous \c SREG is restored.
 *
 * Typical AVR prescaler options are 1,2,4,8,16,32,64,128,256 (i.e. 2^\e n with
 * \e n in [0,8]). Check your device datasheet for exact limits.
 *
 * \note Ensure \c F_CPU is defined by your build system to the nominal clock
 * before prescaling (e.g., via compiler flag \c -DF_CPU=16000000UL).
 */

/**
 * \def CPU_DIV_LOG2
 * \brief Base-2 logarithm of the desired CPU prescaler at compile time.
 * \details
 * The effective CPU frequency after prescaling is:
 * \code
 * F_CPU_CFG = F_CPU >> CPU_DIV_LOG2;
 * \endcode
 * For example, to run at \c F_CPU/8 use \c CPU_DIV_LOG2=3.
 *
 * \note Default is 0 (no prescale). Override via compiler flags or before this header.
 */
#ifndef CPU_DIV_LOG2
#define CPU_DIV_LOG2 0
#endif

/**
 * \def F_CPU_CFG
 * \brief Effective CPU frequency after applying the compile-time prescaler.
 * \details Computed as \c (F_CPU >> CPU_DIV_LOG2). Useful for delay routines,
 * timers, or baud calculations that should reflect the configured prescale.
 */
#define F_CPU_CFG (F_CPU >> CPU_DIV_LOG2)

/**
 * \brief Set the CPU clock prescaler (runtime).
 * \details
 * Performs the CLKPCE unlock then writes CLKPR with the provided prescaler code.
 * Interrupts are disabled for the critical section and previous \c SREG is restored.
 *
 * \param log2_div Base-2 logarithm of the division factor to set (typically 0..8).
 * \note The value is masked with \c 0x0F; valid range is device-dependent.
 * \warning Changing the CPU clock affects all timing (UART baud, timers, delays).
 * Reconfigure peripherals accordingly.
 */
static inline void clock_set_div(uint8_t log2_div)
{
  uint8_t s = SREG;
  cli();
  CLKPR = _BV(CLKPCE);       // enable change within 4 cycles
  CLKPR = (log2_div & 0x0F); // write prescaler bits (CLKPS3:0)
  SREG = s;
}
