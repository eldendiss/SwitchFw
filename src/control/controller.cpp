#include <math.h>
#include "controller.h"
#include "drivers/pwm.h"

/**
 * \file
 * \brief Closed-loop controller implementation for flyback HV supply.
 * \details
 * Implements:
 * - Default parameter initialization (\c controller_init_defaults)
 * - Runtime state reset (\c controller_reset)
 * - 1 kHz control step combining PRECHARGE, PI(+Dmeas), and a silent PFM "microburst" mode (\c controller_step)
 *
 * **Units and conventions**
 * - Time: microseconds for scheduler period, milliseconds for ramps/cooldowns.
 * - Duty: fractional 0..1; quantized to timer resolution via \c pwm_min_step().
 * - Feedback (FB): ADC counts (e.g., 10-bit 0..1023). \c fb_set_cnt is the target.
 * - Derivative output: \c dcounts_out in raw counts per control step (~counts/ms at 1 kHz).
 * - Over-voltage (OV) latch: trips above \c fb_ov_trip_cnt, clears below \c fb_ov_clear_cnt.
 * - Over-current (OC) is provided by the caller as a latched/instantaneous flag.
 *
 * **Phases**
 * - \c PH_PRECHARGE : soft start at fixed small duty until either time elapses or FB reaches a fraction
 *   of the ramped setpoint.
 * - \c PH_PI : main PWM control loop with PI + filtered derivative feedback.
 *
 * **PFM microburst mode**
 * - After ramp completion, if FB is close to setpoint (\c pfm_enter_sub_cnt) the loop switches to a
 *   low-audibility scheme: short PWM "microbursts" of small duty followed by a randomized cooldown
 *   (\c pfm_min_off_ms ± jitter). This reduces audible noise and average power when near target.
 */

// ---- helpers ----------------------------------------------------------------

/**
 * \brief Clamp helper for floats.
 * \param x Value.
 * \param lo Lower bound (inclusive).
 * \param hi Upper bound (inclusive).
 * \return \c x limited to [lo, hi].
 */
static inline float clampf(float x, float lo, float hi)
{
  return (x < lo) ? lo : (x > hi) ? hi
                                  : x;
}

// ---- API --------------------------------------------------------------------

/**
 * \brief Initialize controller parameters with safe, production-friendly defaults.
 * \param[out] c Parameter block to populate.
 * \details
 * - 1 kHz control loop (\c control_period_us = 1000).
 * - Soft ramp, short precharge at very low duty.
 * - Conservative PWM caps and step slew to avoid audible artifacts.
 * - PI gains and leaky integrator to minimize windup.
 * - PFM microburst defaults enabled (exact thresholds set via configuration in counts).
 * \note Derivative term uses a simple 1st-order IIR with \c deriv_alpha.
 */
void controller_init_defaults(ControlParams &c)
{
  // timing
  c.control_period_us = 1000; // 1 kHz control loop
  c.ramp_time_ms = 1500;
  c.precharge_time_ms = 200;
  c.precharge_duty = 0.015f;
  c.precharge_exit_frac = 0.85f;

  // gains
  c.KP = 0.00045f;
  c.KI = 0.000006f;
  c.KD = 0.00018f;
  c.deriv_alpha = 0.05f;

  // PWM limits (ramp/PI only)
  c.duty_abs_max = 0.20f;  // absolute safety (driver also clamps)
  c.duty_ramp_cap = 0.12f; // ≤12% during ramp/PI
  c.duty_post_cap = 0.12f; // ≤12% steady PWM
  c.duty_step_max = 0.0008f;
  c.i_leak = 0.9995f;

  // deadband
  c.err_deadband_cnt = 0;

  // PFM (microburst) hold — set exact counts in cfg
  c.pfm_enable = 1;
  c.pfm_min_off_ms = 300;  // long cooldown = very low average pps
  c.pfm_lo_sub_cnt = 0;    // e.g., counts_from_volts(8–12 V) in cfg
  c.pfm_enter_sub_cnt = 0; // e.g., counts_from_volts(12–15 V) in cfg

  // microburst shape (inaudible)
  c.pfm_burst_ms = 3;       // 2–4 ms of 31.25 kHz PWM
  c.pfm_burst_duty = 0.03f; // ~3% duty during microburst (cap still applies)

  // oneshot fields kept for compatibility (unused here)
  c.pfm_ton_us = 0;
}

/**
 * \brief Reset dynamic controller state.
 * \param[out] s  State block to reset.
 * \param now_us  Current timestamp in microseconds (used to seed scheduler).
 * \details
 * Clears latches, integrators, filters, ramps; sets phase to \c PH_PRECHARGE and marks OV as faulted
 * (requires below-clear threshold to unlatch).
 */
void controller_reset(ControlState &s, uint32_t now_us)
{
  s.last_us = now_us;
  s.ramp_ms = 0;
  s.pre_ms = 0;

  s.duty = 0.0f;
  s.integ = 0.0f;

  s.fb_prev = 0;
  s.d_filt = 0.0f;
  s.d_inited = 0;

  s.burst_skip = 0;
  s.burst_off_ms = 0;   // re-used as PFM cooldown
  s.pfm_mb_ms_left = 0; // microburst time remaining

  s.ov_fault = 1;
  s.oc_latch_ms = 0;

  s.fb_ema = 0;
  s.fb_ema_inited = 0;

  s.phase = PH_PRECHARGE;
}

/**
 * \brief Execute one control iteration (nominally every \c control_period_us).
 * \param now           Current timestamp in microseconds (monotonic).
 * \param[in]  c        Tunable control parameters (constant during step).
 * \param[in,out] s     Controller state (integrators, latches, phase, etc.).
 * \param fb_raw        Latest raw feedback (ADC counts).
 * \param oc_now        Over-current flag (non-zero means fault this step).
 * \param allow_switch  If \c false, PWM is disabled regardless of state/faults.
 * \param[out] fb_out   EMA-filtered feedback (counts) exported to caller.
 * \param[out] dcounts_out  First-difference derivative (counts/step, ~counts/ms @1 kHz).
 * \details
 * - 1-tap EMA on \c fb_raw, alpha=1/8.
 * - OV latch and OC handling gate the state machine and force PWM off.
 * - Scheduler enforces 1 kHz; if called early, it returns immediately.
 * - When ramp done and near setpoint, uses PFM microbursts; otherwise PRECHARGE → PI(+D) PWM.
 * - Duty is quantized to the PWM grid and slewed by \c duty_step_max to avoid audible jumps.
 * - Integrator uses leak factor \c i_leak and conditional anti-windup against saturation.
 * \note Requires timer/PWM backend: \c pwm_min_step(), \c pwm_set_duty(), \c pwm_enable_sync_bottom(),
 * \c pwm_disable(), \c pwm_is_enabled().
 */
void controller_step(uint32_t now,
                     ControlParams &c,
                     ControlState &s,
                     uint16_t fb_raw,
                     uint8_t oc_now,
                     bool allow_switch,
                     uint16_t &fb_out,
                     int16_t &dcounts_out)
{
  // === EMA (alpha = 1/8) ===
  if (!s.fb_ema_inited)
  {
    s.fb_ema = fb_raw;
    s.fb_ema_inited = 1;
  }
  s.fb_ema += ((int32_t)fb_raw - (int32_t)s.fb_ema) >> 3;
  uint16_t fb = (uint16_t)s.fb_ema;
  fb_out = fb;

  // Helpers
  const float DLSB = pwm_min_step(); // ≈0.39% at 31.25 kHz
  const float Q_EPS = 1e-9f;
  auto quant = [&](float d) -> float { // quantize to PWM grid
    float q = roundf(d / DLSB) * DLSB;
    if (q < 0.0f)
      q = 0.0f;
    return q;
  };

  // === OV latch ===
  if (fb > c.fb_ov_trip_cnt)
    s.ov_fault = 1;
  if (s.ov_fault && fb < c.fb_ov_clear_cnt)
    s.ov_fault = 0;

  // OC latch visibility (LED/UI)
  if (oc_now)
  {
    s.oc_latch_ms = 1500;
  }
  else if (s.oc_latch_ms)
  {
    --s.oc_latch_ms;
  }

  // === 1 kHz scheduler ===
  if ((uint32_t)(now - s.last_us) < c.control_period_us)
  {
    dcounts_out = 0;
    return;
  }
  s.last_us += c.control_period_us;

  // === Fault handling ===
  if (oc_now || s.ov_fault)
  {
    s.ramp_ms = 0;
    s.pre_ms = 0;
    s.phase = PH_PRECHARGE;
    s.duty = 0.0f;
    s.integ = 0.0f;
    pwm_set_duty(0.0f);
    s.d_filt = 0.0f;
    s.d_inited = 0;
    s.fb_prev = fb;
    s.burst_skip = 0;
    s.burst_off_ms = 0;
    s.pfm_mb_ms_left = 0;
    pwm_disable();
    dcounts_out = 0;
    return;
  }

  if (!allow_switch)
  {
    pwm_disable();
    dcounts_out = 0;
    return;
  }

  // === Ramp setpoint (for PWM path) ===
  if (s.ramp_ms < c.ramp_time_ms)
    ++s.ramp_ms;
  const float ramp = (float)s.ramp_ms / (float)c.ramp_time_ms;
  const uint16_t fb_set = (uint16_t)(c.fb_set_cnt * ramp + 0.5f);

  // Duty ceilings (PWM)
  float cap_soft = c.duty_ramp_cap * ramp;
  if (cap_soft > c.duty_ramp_cap)
    cap_soft = c.duty_ramp_cap;
  float cap = (cap_soft < c.duty_abs_max) ? cap_soft : c.duty_abs_max;
  if (s.ramp_ms >= c.ramp_time_ms && cap > c.duty_post_cap)
    cap = c.duty_post_cap;

  // Derivative (counts/ms) for external logic (e.g., range)
  int32_t d_raw = 0;
  if (!s.d_inited)
  {
    s.d_filt = 0.0f;
    s.fb_prev = fb;
    s.d_inited = 1;
  }
  else
  {
    d_raw = s.fb_prev - (int32_t)fb;
    s.fb_prev = fb;
  }
  dcounts_out = (int16_t)d_raw;

  // === Decide if PFM (microburst) is allowed ===
  const bool ramp_done = (s.ramp_ms >= c.ramp_time_ms);
  const uint16_t pfm_enter_th =
      (c.fb_set_cnt > c.pfm_enter_sub_cnt) ? (uint16_t)(c.fb_set_cnt - c.pfm_enter_sub_cnt) : 0;
  const bool allow_pfm = c.pfm_enable && ramp_done && (fb >= pfm_enter_th);

  // === PFM microburst hold (silent) ===
  if (allow_pfm)
  {
    s.burst_skip = 1; // for LED

    // Cooldown between microbursts
    if (s.pfm_mb_ms_left == 0)
    {
      if (s.burst_off_ms > 0)
      {
        --s.burst_off_ms;
        pwm_disable();
        return;
      }

      // Use FINAL setpoint thresholds (not ramped)
      const uint16_t lo = (c.fb_set_cnt > c.pfm_lo_sub_cnt)
                              ? (uint16_t)(c.fb_set_cnt - c.pfm_lo_sub_cnt)
                              : 0;

      // Start a microburst only if low
      if (fb <= lo)
      {
        s.pfm_mb_ms_left = c.pfm_burst_ms; // e.g., 2–4 ms
        // Prepare PWM with small duty, bottom-synced
        float dcmd = c.pfm_burst_duty;
        float capb = c.duty_post_cap; // keep ≤12%
        if (dcmd > capb)
          dcmd = capb;
        dcmd = quant(dcmd);
        if (dcmd < 2.0f * DLSB)
          dcmd = 2.0f * DLSB; // ensure switching
        s.duty = dcmd;
        pwm_set_duty(s.duty);
        pwm_enable_sync_bottom();
      }
      else
      {
        pwm_disable();
        return;
      }
    }

    // Run microburst window at 31.25 kHz, small duty
    if (s.pfm_mb_ms_left > 0)
    {
      --s.pfm_mb_ms_left;

      // (Optional small slew toward target to avoid sudden step)
      float target = quant(c.pfm_burst_duty);
      if (target > c.duty_post_cap)
        target = c.duty_post_cap;
      if (target < 2.0f * DLSB)
        target = 2.0f * DLSB;

      float du = target - s.duty;
      float step = (c.duty_step_max < DLSB) ? DLSB : c.duty_step_max;
      if (du > step)
        du = step;
      if (du < -step)
        du = -step;
      s.duty += du;
      pwm_set_duty(s.duty);

      if (s.pfm_mb_ms_left == 0)
      {
        // End of microburst → gate low and start jittered cooldown
        pwm_disable();
        static uint8_t lfsr = 0xA5;
        lfsr = (uint8_t)((lfsr >> 1) ^ (-(lfsr & 1u) & 0xB8u));
        int8_t jitter = (int8_t)(lfsr & 0x1F) - 16; // -16..+15 (~±20%)
        int32_t off = (int32_t)c.pfm_min_off_ms + jitter;
        if (off < 20)
          off = 20;
        s.burst_off_ms = (uint16_t)off;
      }
    }
    return;
  }

  // === PWM path (ramp/PI) ===
  s.burst_skip = 0;

  if (!pwm_is_enabled())
  {
    float pre = 2.0f * DLSB;
    if (s.duty < pre)
      s.duty = pre;
    pwm_set_duty(s.duty);
    pwm_enable_sync_bottom();
  }

  // --- PRECHARGE ---
  if (s.phase == PH_PRECHARGE)
  {
    if (s.pre_ms < c.precharge_time_ms)
      ++s.pre_ms;

    float dcmd = c.precharge_duty;
    if (dcmd > cap)
      dcmd = cap;

    float target = quant(dcmd);
    float du = target - s.duty;
    float step = (c.duty_step_max < DLSB) ? DLSB : c.duty_step_max;
    if (du > step)
      du = step;
    if (du < -step)
      du = -step;
    s.duty += du;
    pwm_set_duty(s.duty);

    const uint16_t fb85 = (uint16_t)(fb_set * c.precharge_exit_frac + 0.5f);
    if (s.pre_ms >= c.precharge_time_ms || fb >= fb85)
    {
      s.phase = PH_PI;
      s.integ = s.duty;
      s.d_filt = 0.0f;
      s.d_inited = 0;
      s.fb_prev = fb;
    }
    return;
  }

  // --- PI + D(meas) ---
  int32_t err_raw = (int32_t)fb_set - (int32_t)fb;
  int32_t err = 0;
  if (err_raw > (int32_t)c.err_deadband_cnt)
    err = err_raw - (int32_t)c.err_deadband_cnt;
  else if (err_raw < -(int32_t)c.err_deadband_cnt)
    err = err_raw + (int32_t)c.err_deadband_cnt;

  s.d_filt += c.deriv_alpha * ((float)d_raw - s.d_filt);

  float u = c.KP * (float)err + s.integ + c.KD * s.d_filt;
  float u_sat = clampf(u, 0.0f, cap);

  // Anti-windup
  s.integ *= c.i_leak;
  if ((fabsf(u - u_sat) < Q_EPS) || (u > cap && err < 0) || (u < 0.0f && err > 0))
  {
    s.integ += c.KI * (float)err;
  }
  if (s.integ < 0.0f)
    s.integ = 0.0f;
  if (s.integ > cap)
    s.integ = cap;

  float target = quant(u_sat);
  float du = target - s.duty;
  float step = (c.duty_step_max < DLSB) ? DLSB : c.duty_step_max;
  if (du > step)
    du = step;
  if (du < -step)
    du = -step;
  s.duty += du;
  pwm_set_duty(s.duty);
}
