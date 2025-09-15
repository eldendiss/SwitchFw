// drivers/timebase.h
#pragma once
#include <stdint.h>

/**
 * \file
 * \brief Microsecond-resolution timebase API.
 * \details
 * Initializes a 1 µs tick timebase (Timer2 in normal mode) and exposes
 * monotonic timestamps as 32-bit and wrap-extended 64-bit counters.
 */

/**
 * \brief Initialize the 1 µs-resolution timebase.
 * \details Configures the hardware timer and enables its overflow ISR.
 */
void timebase_init_us();

/**
 * \brief Get a 32-bit microsecond timestamp.
 * \return Monotonic time in µs since \c timebase_init_us(), modulo 2^32.
 */
uint32_t us_now32();

/**
 * \brief Get a 64-bit microsecond timestamp (wrap-extended).
 * \return Monotonic time in µs since \c timebase_init_us(), 64-bit.
 */
uint64_t us_now();
