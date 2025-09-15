// drivers/adc.h
#pragma once
#include <stdint.h>

/**
 * \file
 * \brief ADC driver public API (free-running ADC0 with 8× averaging).
 */

/**
 * \brief Initialize ADC0 in free-running mode with interrupt and 8× boxcar average.
 * \details Uses external AREF, targets ~125 kHz ADC clock based on F_CPU_CFG.
 */
void adc_init();

/**
 * \brief Get latest 8×-averaged ADC0 sample.
 * \return Decimated ADC counts (0..1023).
 * \note Atomic read of an ISR-updated value.
 */
uint16_t adc_fb_decim(); // returns decimated ADC counts (0..1023)
