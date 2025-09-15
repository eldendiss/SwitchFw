// control/controller.cpp
#include <math.h>
#include "controller.h"
#include "drivers/pwm.h"

static inline float clampf(float x, float lo, float hi) {
  return (x < lo) ? lo : (x > hi) ? hi : x;
}

void controller_init_defaults(ControlParams &c)
{
  // timing
  c.control_period_us   = 1000;    // 1 kHz control loop
  c.ramp_time_ms        = 1500;
  c.precharge_time_ms   = 200;
  c.precharge_duty      = 0.015f;
  c.precharge_exit_frac = 0.85f;

  // gains
  c.KP = 0.00045f;
  c.KI = 0.000006f;
  c.KD = 0.00018f;
  c.deriv_alpha = 0.05f;

  // PWM limits (ramp/PI only)
  c.duty_abs_max  = 0.20f;  // absolute safety (driver also clamps)
  c.duty_ramp_cap = 0.12f;  // ≤12% during ramp/PI
  c.duty_post_cap = 0.12f;  // ≤12% steady PWM
  c.duty_step_max = 0.0008f;
  c.i_leak        = 0.9995f;

  // deadband
  c.err_deadband_cnt = 0;

  // PFM hysteretic hold (set exact thresholds in cfg)
  c.pfm_enable        = 1;
  c.pfm_ton_us        = 1;      // 1.5–3 us typical
  c.pfm_min_off_ms    = 300;     // 60–120 ms
  c.pfm_lo_sub_cnt    = 0;      // counts_from_volts(~3–5 V) in cfg
  c.pfm_enter_sub_cnt = 0;      // counts_from_volts(~5 V) in cfg
}

void controller_reset(ControlState &s, uint32_t now_us)
{
  s.last_us = now_us;
  s.ramp_ms = 0;
  s.pre_ms  = 0;

  s.duty    = 0.0f;
  s.integ   = 0.0f;

  s.fb_prev = 0;
  s.d_filt  = 0.0f;
  s.d_inited = 0;

  s.burst_skip   = 0;
  s.burst_off_ms = 0;

  s.ov_fault   = 1;
  s.oc_latch_ms = 0;

  s.fb_ema = 0;
  s.fb_ema_inited = 0;

  s.phase = PH_PRECHARGE;
}

void controller_step(uint32_t now,
                     ControlParams &c,
                     ControlState &s,
                     uint16_t fb_raw,
                     uint8_t oc_now,
                     bool allow_switch,
                     uint16_t &fb_out,
                     int16_t  &dcounts_out)
{
  // === EMA (alpha = 1/8) ===
  if (!s.fb_ema_inited) { s.fb_ema = fb_raw; s.fb_ema_inited = 1; }
  s.fb_ema += ((int32_t)fb_raw - (int32_t)s.fb_ema) >> 3;
  uint16_t fb = (uint16_t)s.fb_ema;
  fb_out = fb;

  // Helpers
  const float DLSB   = pwm_min_step();                     // ≈0.39% at 31.25 kHz
  const float Q_EPS  = 1e-9f;
  auto quant = [&](float d)->float {                       // quantize to PWM grid
    float q = roundf(d / DLSB) * DLSB;
    if (q < 0.0f) q = 0.0f;
    return q;
  };

  // === OV latch ===
  if (fb > c.fb_ov_trip_cnt) s.ov_fault = 1;
  if (s.ov_fault && fb < c.fb_ov_clear_cnt) s.ov_fault = 0;

  // OC latch visibility (for LED/UI)
  if (oc_now) { s.oc_latch_ms = 1500; }
  else if (s.oc_latch_ms) { --s.oc_latch_ms; }

  // === 1 kHz scheduler ===
  if ((uint32_t)(now - s.last_us) < c.control_period_us) { dcounts_out = 0; return; }
  s.last_us += c.control_period_us;

  // === Fault handling ===
  if (oc_now || s.ov_fault) {
    s.ramp_ms = 0;
    s.pre_ms  = 0;
    s.phase   = PH_PRECHARGE;
    s.duty    = 0.0f;
    s.integ   = 0.0f;
    pwm_set_duty(0.0f);
    s.d_filt  = 0.0f; s.d_inited = 0; s.fb_prev = fb;
    s.burst_skip = 0;
    s.burst_off_ms = 0;
    pwm_disable();
    dcounts_out = 0;
    return;
  }

  if (!allow_switch) {
    pwm_disable();
    dcounts_out = 0;
    return;
  }

  // === Ramp setpoint ===
  if (s.ramp_ms < c.ramp_time_ms) ++s.ramp_ms;
  const float    ramp   = (float)s.ramp_ms / (float)c.ramp_time_ms;
  const uint16_t fb_set = (uint16_t)(c.fb_set_cnt * ramp + 0.5f);

  // Duty ceilings (PWM path)
  float cap_soft = c.duty_ramp_cap * ramp;
  if (cap_soft > c.duty_ramp_cap) cap_soft = c.duty_ramp_cap;
  float cap = (cap_soft < c.duty_abs_max) ? cap_soft : c.duty_abs_max;
  if (s.ramp_ms >= c.ramp_time_ms && cap > c.duty_post_cap) cap = c.duty_post_cap;

  // Derivative (counts/ms) for external logic (e.g., range)
  int32_t d_raw = 0;
  if (!s.d_inited) { s.d_filt = 0.0f; s.fb_prev = fb; s.d_inited = 1; }
  else             { d_raw = s.fb_prev - (int32_t)fb; s.fb_prev = fb; }
  dcounts_out = (int16_t)d_raw;

  // === Arm PFM only when NEAR the final setpoint (not just when time is up) ===
  const bool ramp_done = (s.ramp_ms >= c.ramp_time_ms);
  const uint16_t pfm_enter_th =
      (c.fb_set_cnt > c.pfm_enter_sub_cnt) ? (uint16_t)(c.fb_set_cnt - c.pfm_enter_sub_cnt) : 0;
  const bool allow_pfm = c.pfm_enable && ramp_done && (fb >= pfm_enter_th);

  // === PFM hysteretic hold ===
  if (allow_pfm) {
    // mark holding for LEDs
    s.burst_skip = 1;

    // ensure continuous PWM is OFF in PFM
    if (pwm_is_enabled()) pwm_disable();

    // cooldown (ms)
    if (s.burst_off_ms > 0) { --s.burst_off_ms; return; }

    // low threshold around the FINAL setpoint (not ramped)
    const uint16_t lo = (c.fb_set_cnt > c.pfm_lo_sub_cnt)
                          ? (uint16_t)(c.fb_set_cnt - c.pfm_lo_sub_cnt) : 0;

    if (fb <= lo) {
      // one fixed-width pulse; driver auto-detaches after COMPA
      pwm_emit_oneshot_us(c.pfm_ton_us);

      // jittered cooldown to avoid tonal ticks (±~20% of min_off)
      static uint8_t lfsr = 0xA5;
      lfsr = (uint8_t)((lfsr >> 1) ^ (-(lfsr & 1u) & 0xB8u));
      int8_t jitter = (int8_t)(lfsr & 0x1F) - 16;         // -16..+15
      int32_t off = (int32_t)c.pfm_min_off_ms + jitter;   // ms
      if (off < 20) off = 20;                             // floor
      s.burst_off_ms = (uint16_t)off;
    }
    return;
  }

  // === PWM path (ramp/PI) ===
  s.burst_skip = 0; // not holding

  // Ensure PWM attached before commanding duty
  if (!pwm_is_enabled()) {
    float pre = 2.0f * DLSB;               // guarantee switching
    if (s.duty < pre) s.duty = pre;
    pwm_set_duty(s.duty);
    pwm_enable_sync_bottom();
  }

  // --- PRECHARGE ---
  if (s.phase == PH_PRECHARGE) {
    if (s.pre_ms < c.precharge_time_ms) ++s.pre_ms;

    float dcmd = c.precharge_duty;
    if (dcmd > cap) dcmd = cap;

    float target = quant(dcmd);
    float du = target - s.duty;
    float step = (c.duty_step_max < DLSB) ? DLSB : c.duty_step_max;
    if (du >  step) du =  step;
    if (du < -step) du = -step;
    s.duty += du; pwm_set_duty(s.duty);

    const uint16_t fb85 = (uint16_t)(fb_set * c.precharge_exit_frac + 0.5f);
    if (s.pre_ms >= c.precharge_time_ms || fb >= fb85) {
      s.phase   = PH_PI;
      s.integ   = s.duty;
      s.d_filt  = 0.0f; s.d_inited = 0; s.fb_prev = fb;
    }
    return;
  }

  // --- PI + D(meas) ---
  int32_t err_raw = (int32_t)fb_set - (int32_t)fb;
  int32_t err = 0;
  if      (err_raw >  (int32_t)c.err_deadband_cnt) err = err_raw - (int32_t)c.err_deadband_cnt;
  else if (err_raw < -(int32_t)c.err_deadband_cnt) err = err_raw + (int32_t)c.err_deadband_cnt;

  s.d_filt += c.deriv_alpha * ((float)d_raw - s.d_filt);

  float u     = c.KP * (float)err + s.integ + c.KD * s.d_filt;
  float u_sat = clampf(u, 0.0f, cap);

  // Anti-windup
  s.integ *= c.i_leak;
  if ((fabsf(u - u_sat) < Q_EPS) || (u > cap && err < 0) || (u < 0.0f && err > 0)) {
    s.integ += c.KI * (float)err;
  }
  if (s.integ < 0.0f) s.integ = 0.0f;
  if (s.integ > cap)  s.integ = cap;

  float target = quant(u_sat);
  float du = target - s.duty;
  float step = (c.duty_step_max < DLSB) ? DLSB : c.duty_step_max;
  if (du >  step) du =  step;
  if (du < -step) du = -step;
  s.duty += du; pwm_set_duty(s.duty);
}
