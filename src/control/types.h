// control/types.h
#pragma once
#include <stdint.h>

/**
 * \file
 * \brief Controller parameter/state types and small enums.
 * \details
 * Units:
 * - Feedback values are in ADC counts (e.g., 10-bit 0..1023).
 * - Duty is fractional 0..1.
 * - Times are µs or ms as indicated per field.
 */

/**
 * \brief Tunable controller parameters.
 * \details
 * Contains ADC-count targets, PI(D) gains, PWM limits, timing, deadband,
 * and PFM (microburst) thresholds/shape. See \c controller_init_defaults()
 * for safe starting values.
 */
struct ControlParams
{
  // --- Targets / thresholds (ADC counts) ---
  /** \brief ACTIVE setpoint used by controller */
  uint16_t fb_set_cnt;
  /** \brief ACTIVE OV trip threshold (latches OV fault when FB > this). */
  uint16_t fb_ov_trip_cnt;
  /** \brief ACTIVE OV clear threshold (clears OV fault when FB < this). */
  uint16_t fb_ov_clear_cnt;
  /** \brief per-range setpoints (R0..R3). I²C writes read/modify these. */
  uint16_t fb_set_cnt_tab[4];
  /** \brief per-range OV trip threshold */
  uint16_t fb_ov_trip_cnt_tab[4];
  /** \brief per-range OV clear threshold */
  uint16_t fb_ov_clear_cnt_tab[4];

  // --- PI(D) gains ---
  /** \brief Proportional gain. */
  float KP;
  /** \brief Integral gain (per control step). */
  float KI;
  /** \brief Derivative gain (on measured FB, filtered). */
  float KD;
  /** \brief Derivative IIR alpha (0..1), higher = less smoothing. */
  float deriv_alpha;

  // --- PWM limits (applied during ramp/settle) ---
  /** \brief Max duty slew per control step. */
  float duty_step_max;
  /** \brief Absolute duty ceiling (safety), e.g. ≤0.20. */
  float duty_abs_max;
  /** \brief Soft cap during ramp, e.g. ≤0.12. */
  float duty_ramp_cap;
  /** \brief Post-ramp steady PWM cap, e.g. ≤0.12. */
  float duty_post_cap;
  /** \brief Integrator leak factor per step (0..1). */
  float i_leak;

  // --- Timing / precharge ---
  /** \brief Control scheduler period in µs (1000 → 1 kHz). */
  uint16_t control_period_us;
  /** \brief Setpoint ramp duration in ms. */
  uint16_t ramp_time_ms;
  /** \brief Precharge duration in ms (fixed small duty). */
  uint16_t precharge_time_ms;
  /** \brief Precharge duty (fraction 0..1). */
  float precharge_duty;
  /** \brief Fraction of ramped setpoint to exit precharge (0..1). */
  float precharge_exit_frac;

  // --- Error deadband ---
  /** \brief Symmetric deadband around zero error (ADC counts). */
  uint16_t err_deadband_cnt;

  // --- PFM hysteretic hold / microburst ---
  /** \brief 1 = enable PFM mode after ramp completes. */
  uint8_t pfm_enable;
  /** \brief Legacy one-shot pulse width (µs). Not used in microburst mode. */
  uint16_t pfm_ton_us;
  /** \brief Minimum off/cooldown after a pulse or burst (ms). */
  uint16_t pfm_min_off_ms;
  /** \brief Start when FB ≤ (fb_set_cnt − pfm_lo_sub_cnt). */
  uint16_t pfm_lo_sub_cnt;
  /** \brief Allow PFM when FB ≥ (fb_set_cnt − pfm_enter_sub_cnt). */
  uint16_t pfm_enter_sub_cnt;
  /** \brief Microburst length (ms), e.g., 2–4. */
  uint16_t pfm_burst_ms;
  /** \brief Duty during microburst (fraction 0..1), e.g., 0.03f. */
  float pfm_burst_duty;
};

/**
 * \brief Controller phase (high-level state).
 */
typedef enum
{
  /** \brief Fixed small duty soft-start until time/FB threshold. */
  PH_PRECHARGE,
  /** \brief Main PWM regulation with PI(+filtered D). */
  PH_PI
} phase_t;

/**
 * \brief Regulation mode indicator.
 */
typedef enum
{
  /** \brief PWM regulation. */
  REG_PWM,
  /** \brief PFM/microburst regulation. */
  REG_PFM
} regmode_t;

/**
 * \brief Dynamic controller state and latches.
 * \details
 * Includes scheduler bookkeeping, PI/D terms, OV/OC latches, EMA, phase/mode,
 * and PFM cooldown/microburst counters.
 */
struct ControlState
{
  // --- Scheduler / timing ---
  /** \brief Timestamp of last executed step (µs). */
  uint32_t last_us;
  /** \brief Elapsed ramp time (ms). */
  uint16_t ramp_ms;
  /** \brief Elapsed precharge time (ms). */
  uint16_t pre_ms;

  // --- Control outputs / integrator ---
  /** \brief Current commanded PWM duty (0..1). */
  float duty;
  /** \brief Integrator state (accumulator, 0..cap). */
  float integ;
  /** \brief 0..3 active range */
  uint8_t active_range;

  // --- Derivative helper (on measured FB) ---
  /** \brief Previous FB sample (counts) for first difference. */
  int32_t fb_prev;
  /** \brief Filtered derivative state. */
  float d_filt;
  /** \brief 0 until derivative path has been initialized. */
  uint8_t d_inited;

  // --- UI helpers / indicators ---
  /** \brief 1 while PFM is “holding” (used for LED/UI). */
  uint8_t burst_skip;
  /** \brief Cooldown counter used for PWM path; reused by PFM as needed (ms). */
  uint16_t burst_off_ms;

  // --- Fault latches ---
  /** \brief OV latch (1 = faulted until below clear threshold). */
  uint8_t ov_fault;
  /** \brief OC latched indicator decay for UI (ms). */
  uint16_t oc_latch_ms;

  // --- Feedback smoothing (EMA) ---
  /** \brief EMA-filtered FB (counts, Q0). */
  uint32_t fb_ema;
  /** \brief 0 until EMA seeded with first sample. */
  uint8_t fb_ema_inited;

  // --- Phase / mode ---
  /** \brief Current phase (PRECHARGE or PI). */
  phase_t phase;
  /** \brief Regulation mode hint (PWM or PFM). May be informational. */
  regmode_t mode;

  // --- PFM bookkeeping ---
  /** \brief Legacy PFM cooldown (ms). Not used by microburst path currently. */
  uint16_t pfm_cool_ms;
  /** \brief Remaining microburst time (ms). */
  uint16_t pfm_mb_ms_left;
};
