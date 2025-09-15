// drivers/pwm.h
#pragma once
#include <stdint.h>

/**
 * \file
 * \brief Timer1 OC1A gate driver (Fast PWM, TOP=ICR1, prescaler=1).
 * \details
 * - Mode 14 (Fast PWM, ICR1 as TOP), non-inverting on OC1A (PB1).
 * - Base PWM frequency set by \c ICR1 (default ≈31.25 kHz when \c F_CPU_CFG=8 MHz).
 * - Duty is fractional (0..1) and hard-clamped internally to a safety ceiling
 *   (default 0.20, configurable via \c pwm_set_max()).
 */

/**
 * \brief Initialize Timer1 and the OC1A gate pin.
 * \details
 * Configures Fast PWM (TOP=ICR1), prescaler=1, computes TOP from clock and target
 * frequency, sets OC1A pin as output and forces it LOW. PWM remains detached
 * after init; call \c pwm_enable() or \c pwm_enable_sync_bottom().
 */
void pwm_init();

/**
 * \brief Attach OC1A (non-inverting) to start driving the gate immediately.
 */
void pwm_enable();

/**
 * \brief Detach OC1A and force the gate LOW (fail-safe).
 * \details Also disables any one-shot in progress.
 */
void pwm_disable();

/**
 * \brief Query whether OC1A is currently attached (PWM active).
 * \return \c true if PWM is enabled, otherwise \c false.
 */
bool pwm_is_enabled();

/**
 * \brief Attach OC1A exactly at the next period BOTTOM (phase-aligned start).
 * \details Useful to avoid mid-period enable glitches.
 */
void pwm_enable_sync_bottom();

/**
 * \brief Set PWM duty (fraction 0..1).
 * \param duty Requested duty cycle.
 * \details The value is clamped to the current absolute ceiling (see \c pwm_set_max()).
 */
void pwm_set_duty(float duty);

/**
 * \brief Set absolute duty ceiling.
 * \param max_duty New ceiling in 0..1 (internally constrained to 0.05..0.95).
 * \details Default ceiling is 0.20.
 */
void pwm_set_max(float max_duty);

/**
 * \brief Get the last commanded duty (fraction 0..1).
 */
float pwm_get_duty();

/**
 * \brief Minimum duty step for the current TOP.
 * \return Duty LSB = \c 1/(ICR1+1) (≈0.39% when ICR1=255).
 */
float pwm_min_step();

/**
 * \brief Timer clock driving PWM.
 * \return Timer clock in Hz (typically \c F_CPU_CFG).
 */
uint32_t pwm_clk_hz();

// ---- PFM one-shot -----------------------------------------------------------

/**
 * \brief Emit one fixed-width pulse starting at the next BOTTOM.
 * \param ton_us Pulse on-time in microseconds.
 * \details
 * Drives OC1A high from BOTTOM for \c ton_us (clamped to [1, TOP] counts),
 * then auto-detaches OC1A and holds the gate LOW at pulse end.
 * \warning Overrides \c OCR1A during the pulse; do not interleave with \c pwm_set_duty().
 */
void pwm_emit_oneshot_us(uint16_t ton_us);
