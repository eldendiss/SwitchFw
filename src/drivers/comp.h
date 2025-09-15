// drivers/comp.h
#pragma once
#include <stdint.h>

/**
 * \file
 * \brief Analog-comparator OC cut-off interface with PWM-cycle blanking.
 * \details
 * Provides initialization, timed arming, periodic service, and OC flags.
 */

/**
 * \brief Initialize the comparator/PWM blanking hooks.
 * \details Configures AIN0 vs 1.1 V bandgap (falling-edge IRQ) and enables
 * Timer1 OVF/OC1B ISRs used to blank the comparator at period start.
 */
void comp_init();

/**
 * \brief Disarm comparator now and re-arm it after a delay.
 * \param delay_us Re-arm delay in microseconds.
 * \param now_us   Current monotonic time in microseconds.
 * \details Clears ACIE immediately; call \c comp_service() periodically to re-enable on deadline.
 */
void comp_arm_after_us(uint32_t delay_us, uint32_t now_us);

/**
 * \brief Periodic service that re-enables the comparator after \c comp_arm_after_us().
 * \param now_us Current monotonic time in microseconds.
 */
void comp_service(uint32_t now_us);

/**
 * \brief Momentary OC flag (one-shot per PWM period).
 * \return Nonzero if an OC event occurred since the last Timer1 overflow.
 */
uint8_t comp_oc_fault(); // momentary OC

/**
 * \brief Sticky OC pulse indicator.
 * \return 1 if any OC occurred since the last read; clears the sticky flag.
 */
uint8_t comp_oc_pulse_sticky(); // NEW: one-shot sticky, cleared by reader

/**
 * \brief Allows PWM output to be re-enabled after an OC event at next period start.
 * \details Set to 1 to allow switching after OC; set to 0 to keep PWM disabled.
 */
extern volatile uint8_t g_allow_switch;
