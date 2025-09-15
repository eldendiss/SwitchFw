#pragma once
#include <stdint.h>

struct ControlParams {
  // ADC-count targets
  uint16_t fb_set_cnt, fb_ov_trip_cnt, fb_ov_clear_cnt;

  // PI(D) gains
  float KP, KI, KD, deriv_alpha;

  // PWM limits (for ramp/settle)
  float duty_step_max;
  float duty_abs_max;      // ≤ 0.20
  float duty_ramp_cap;     // ≤ 0.12
  float duty_post_cap;     // ≤ 0.12
  float i_leak;

  // timing
  uint16_t control_period_us;    // 1000 = 1 kHz
  uint16_t ramp_time_ms;
  uint16_t precharge_time_ms;
  float    precharge_duty;
  float    precharge_exit_frac;

  // error deadband
  uint16_t err_deadband_cnt;

  // --- PFM hysteretic hold (counts + times) ---
  uint8_t  pfm_enable;           // 1 = use PFM after ramp
  uint16_t pfm_ton_us;           // pulse width (µs)
  uint16_t pfm_min_off_ms;       // min off time after a pulse
  uint16_t pfm_lo_sub_cnt;       // fire when fb <= fb_set - this
  uint16_t pfm_enter_sub_cnt;
  uint16_t pfm_burst_ms;       // microburst length in ms (e.g., 2–4)
  float    pfm_burst_duty;     // duty during microburst (e.g., 0.03f)
};

typedef enum { PH_PRECHARGE, PH_PI } phase_t;
typedef enum { REG_PWM, REG_PFM }    regmode_t;

struct ControlState {
  uint32_t last_us;
  uint16_t ramp_ms, pre_ms;

  float    duty, integ;

  int32_t  fb_prev;
  float    d_filt;
  uint8_t  d_inited;

  // LED helpers (kept)
  uint8_t  burst_skip;           // 1 while "holding" in PFM
  uint16_t burst_off_ms;         // unused in PFM

  uint8_t  ov_fault;
  uint16_t oc_latch_ms;

  uint32_t fb_ema;
  uint8_t  fb_ema_inited;

  phase_t   phase;
  regmode_t mode;                // PWM or PFM
  uint16_t  pfm_cool_ms;         // cooldown counter (ms)
  uint16_t pfm_mb_ms_left;
};
