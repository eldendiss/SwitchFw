// controller.h
#pragma once
#include <stdint.h>
#include "types.h"

/**
 * \file
 * \brief Public controller API (params/state init and one control step).
 * \details
 * Provides:
 * - \c controller_init_defaults(): populate \c ControlParams with safe defaults.
 * - \c controller_reset(): reset \c ControlState and seed scheduler timebase.
 * - \c controller_step(): execute one control iteration (PRECHARGE / PI(+D) PWM or PFM microburst).
 *
 * **Units**
 * - Timebase argument \c now_us is in microseconds (monotonic).
 * - Feedback is in ADC counts (e.g., 10-bit 0..1023).
 * - Duty is fractional 0..1 (quantized by PWM backend).
 */

/**
 * \brief Initialize controller parameters with safe defaults.
 * \param[out] c Parameter block to populate.
 * \note Defaults include 1 kHz control loop, soft ramp, conservative duty caps,
 * PI gains, derivative filter alpha, PFM microburst settings, and deadband.
 */
void controller_init_defaults(ControlParams &c);

/**
 * \brief Reset dynamic controller state and enter PRECHARGE phase.
 * \param[out] s    State block to reset (integrator, latches, filters, etc.).
 * \param now_us    Current monotonic timestamp in microseconds to seed the scheduler.
 * \details Clears OV/OC indicators, duty/integrator, EMA/derivative filters,
 * ramp/precharge timers, and sets \c s.phase = \c PH_PRECHARGE.
 */
void controller_reset(ControlState &s, uint32_t now_us);

/**
 * \brief Execute one control iteration.
 * \param now_us         Current monotonic time in microseconds.
 * \param[in]  c         Tunable controller parameters (constant during the step).
 * \param[in,out] s      Controller state (integrators, latches, phase, etc.).
 * \param fb_raw_counts  Latest raw feedback sample (ADC counts).
 * \param oc_fault_now   Nonzero if an over-current condition is present this step.
 * \param allow_switch   If \c false, forces PWM off regardless of other conditions.
 * \param[out] fb_out    EMA-filtered feedback (counts).
 * \param[out] dcounts_out First difference derivative in counts/step
 *                         (~counts/ms when \c c.control_period_us = 1000).
 * \details
 * - Enforces the scheduler period \c c.control_period_us; if called early it returns immediately.
 * - Latches OV above \c c.fb_ov_trip_cnt and clears below \c c.fb_ov_clear_cnt.
 * - Faults (OV/OC) force PWM off and reset ramp/precharge.
 * - When ramp is complete and FB is near setpoint, enters silent PFM microburst mode;
 *   otherwise runs PRECHARGE then PI(+filtered D) PWM control.
 * \warning Requires a configured PWM backend (see \c drivers/pwm.h). Call from a single context.
 */
void controller_step(uint32_t now_us,
                     ControlParams &c,
                     ControlState &s,
                     uint16_t fb_raw_counts,
                     uint8_t oc_fault_now,
                     bool allow_switch,
                     uint16_t &fb_out,
                     int16_t &dcounts_out);
