#include <Arduino.h>
#include "pins.h"
#include "adc.h"
#include "ac.h"
#include "pwm.h"
#include "timebase.h"

// ========= Scaling / Targets =========
#define VOUT_TARGET_V     900.0f
#define VOUT_OV_TRIP_V    990.0f
#define VOUT_OV_CLEAR_V   950.0f

static inline uint16_t counts_from_volts(float v) {
    return (uint16_t)(v * (1023.0f / 1077.5f) + 0.5f); // 431 * 2.5 = 1077.5
}
static inline float volts_from_counts(uint16_t c) {
    return 431.0f * ((c * 2.5f) / 1023.0f);
}

#define FB_SET_COUNTS   (counts_from_volts(VOUT_TARGET_V))
#define FB_OV_TRIP_CNT  (counts_from_volts(VOUT_OV_TRIP_V))
#define FB_OV_CLR_CNT   (counts_from_volts(VOUT_OV_CLEAR_V))

// ========= LEDs =========
#define LED_OK    PC3
#define LED_FAULT PC2

// Fault LED pattern timings (microseconds)
#define LED_BLINK_ON_US     120000u   // length of each short blink
#define LED_BLINK_OFF_US    120000u   // gap between short blinks
#define LED_BLINK_PAUSE_US  600000u   // pause after the group

static inline void led_ok_set(bool on)    { if (on) PORTC |= _BV(LED_OK);   else PORTC &= ~_BV(LED_OK); }
static inline void led_fault_set(bool on) { if (on) PORTC |= _BV(LED_FAULT); else PORTC &= ~_BV(LED_FAULT); }

// ========= Control Tuning =========
#define CONTROL_PERIOD_US   1000u  // 1 kHz control
#define RAMP_TIME_MS        1500u  // 1.5 s total ramp
#define PRECHARGE_DUTY      0.015f
#define PRECHARGE_TIME_MS   200u
#define PRECHARGE_EXIT_FRAC 0.85f

#define KP 0.00045f
#define KI 0.000006f         // per 1 ms tick
#define KD 0.00018f          // derivative on measurement (counts/ms)
#define DERIV_ALPHA 0.05f    // EMA on derivative (~12 ms @1 kHz)

#define DUTY_STEP_MAX  0.0008f
#define DUTY_ABS_MAX   0.20f
#define DUTY_RAMP_CAP  0.12f
#define DUTY_POST_CAP  0.16f   // more headroom after ramp

#define ERR_DEADBAND_CNT  counts_from_volts(1.5f)
#define I_LEAK_FACTOR     0.9995f

// ====== Burst/skip (enabled only AFTER ramp) ======
#define BURST_HI_V        0.5f    // stop when FB >= set + 0.5 V
#define BURST_LO_V        1.2f    // resume when FB <= set − 1.2 V
#define BURST_MIN_OFF_MS  10u     // min off-time to reduce ripple

// ========= State =========
static uint32_t lastCtrlUs = 0;
static uint16_t ramp_ms = 0;         // 0..RAMP_TIME_MS (true ms)
static uint16_t pre_ms  = 0;
static float duty  = 0.0f;
static float integ = 0.0f;

static int32_t fb_prev = 0;          // last filtered FB (counts)
static float   d_filt  = 0.0f;       // filtered derivative (counts/ms)
static uint8_t d_inited = 0;

static bool     burst_skip   = false;
static uint16_t burst_off_ms = 0;

static bool ovFault = true;          // OV latch

static uint32_t fbFilt = 0;          // EMA filtered FB (counts)
static uint8_t  fbFilt_inited = 0;

// ---- OC latch for LED pattern ----
static uint16_t oc_latch_ms = 0;     // keep OC visible for some time

typedef enum { PH_PRECHARGE, PH_PI } phase_t;
static phase_t phase = PH_PRECHARGE;

// ========= Small helpers =========
static inline float clampf(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}
static inline void setDutySlew(float target, float stepMax) {
    float delta = target - duty;
    if (delta > stepMax) delta = stepMax;
    if (delta < -stepMax) delta = -stepMax;
    duty += delta;
    setDuty(duty);
}

// ========= Fault LED pattern engine =========
// Codes: 0 = off, 2 = OV (two blinks), 3 = OC (three blinks). Priority OC > OV.
static void run_fault_led(uint32_t now_us, uint8_t code) {
    static uint8_t cur_code = 0;
    static uint8_t blinks_left = 0;
    static enum { IDLE, ON_SEG, OFF_SEG, PAUSE_SEG } st = IDLE;
    static uint32_t t0 = 0;

    if (code != cur_code) { // reset on code change
        cur_code = code;
        st = (code == 0) ? IDLE : ON_SEG;
        blinks_left = code;
        t0 = now_us;
        led_fault_set(code ? true : false);
        if (!code) led_fault_set(false);
        return;
    }
    if (code == 0) { led_fault_set(false); return; }

    switch (st) {
        case ON_SEG:
            if ((uint32_t)(now_us - t0) >= LED_BLINK_ON_US) {
                led_fault_set(false);
                t0 = now_us;
                st = OFF_SEG;
            }
            break;
        case OFF_SEG:
            if ((uint32_t)(now_us - t0) >= LED_BLINK_OFF_US) {
                if (--blinks_left > 0) {
                    led_fault_set(true);
                    t0 = now_us;
                    st = ON_SEG;
                } else {
                    t0 = now_us;
                    st = PAUSE_SEG;
                }
            }
            break;
        case PAUSE_SEG:
            if ((uint32_t)(now_us - t0) >= LED_BLINK_PAUSE_US) {
                blinks_left = code;
                led_fault_set(true);
                t0 = now_us;
                st = ON_SEG;
            }
            break;
        default: st = IDLE; led_fault_set(false); break;
    }
}

// ========= Setup =========
void setup() {
    DDRC |= _BV(LED_OK) | _BV(LED_FAULT);

    // PB6 high (as in your original)
    DDRB |= _BV(PB6);
    PORTB |= _BV(PB6);

    cli();
    initPwm();
    initAdc();
    initAc();
    init_time_us();
    sei();

    enablePwmOutput();

    // Arm OC later and wait ~100 ms for AREF/ADC settle
    armComparatorAfter_us(80000, us_now());
    const uint64_t start = us_now();
    while (us_now() - start <= 100000) { /* settle */ }

    lastCtrlUs = (uint32_t)us_now();  // align 1 kHz scheduler
}

// ========= Main Loop =========
void loop() {
    const uint32_t now = us_now();

    acService(now);

    // --- FB read + EMA ---
    uint16_t fb_raw = getFB();
    if (!fbFilt_inited) { fbFilt = fb_raw; fbFilt_inited = 1; }
    fbFilt += ((int32_t)fb_raw - (int32_t)fbFilt) >> 3;
    fbFilt += ((int32_t)fb_raw - (int32_t)fbFilt) >> 3;
    const uint16_t fb = (uint16_t)fbFilt;

    // --- OV latch with hysteresis ---
    if (fb > FB_OV_TRIP_CNT) ovFault = true;
    if (ovFault && fb < FB_OV_CLR_CNT) ovFault = false;

    // Track OC (short events) for LED pattern
    if (getOcFault()) oc_latch_ms = 1500;               // hold 1.5 s
    else if (oc_latch_ms) --oc_latch_ms;                // 1 ms per control tick (below)

    // --- Control @ 1 kHz ---
    if ((uint32_t)(now - lastCtrlUs) >= CONTROL_PERIOD_US) {
        lastCtrlUs += CONTROL_PERIOD_US;

        // Fault → reset
        if (getOcFault() || ovFault) {
            ramp_ms = 0;
            pre_ms  = 0;
            phase   = PH_PRECHARGE;
            duty = 0.0f; integ = 0.0f; setDuty(0.0f);
            d_filt = 0.0f; d_inited = 0;
            fb_prev = (int32_t)fb;
            burst_skip = false;
            burst_off_ms = 0;
            disablePwmOutput();
        } else if (ramp_ms < RAMP_TIME_MS) {
            ++ramp_ms;
        }

        // Ramped setpoint (counts)
        const float ramp = (float)ramp_ms / (float)RAMP_TIME_MS;
        const uint16_t fb_set_counts = (uint16_t)(FB_SET_COUNTS * ramp + 0.5f);

        // Duty ceilings
        float cap_soft = DUTY_RAMP_CAP * ramp;
        if (cap_soft > DUTY_RAMP_CAP) cap_soft = DUTY_RAMP_CAP;
        const float cap_abs = DUTY_ABS_MAX;
        float cap = (cap_soft < cap_abs) ? cap_soft : cap_abs;
        if (ramp_ms >= RAMP_TIME_MS && cap > DUTY_POST_CAP) cap = DUTY_POST_CAP;

        // Allow burst only after ramp completes
        const bool burst_allowed = (ramp_ms >= RAMP_TIME_MS) && !ovFault && !getOcFault();

        // ===== Burst/refresh (post-ramp only) =====
        if (burst_allowed) {
            const uint16_t burst_hi_cnt = (uint16_t)(fb_set_counts + counts_from_volts(BURST_HI_V));
            const uint16_t burst_lo_cnt = (fb_set_counts > counts_from_volts(BURST_LO_V))
                                          ? (uint16_t)(fb_set_counts - counts_from_volts(BURST_LO_V)) : 0;

            if (!burst_skip) {
                if (fb >= burst_hi_cnt) {
                    burst_skip   = true;
                    burst_off_ms = 0;
                    duty = 0.0f; integ = 0.0f; setDuty(0.0f);
                    disablePwmOutput();
                }
            } else {
                if (burst_off_ms < BURST_MIN_OFF_MS) ++burst_off_ms;
                if (burst_off_ms >= BURST_MIN_OFF_MS &&
                    fb <= burst_lo_cnt &&
                    !ovFault && !getOcFault())
                {
                    burst_skip = false;
                    d_filt = 0.0f; d_inited = 0; fb_prev = (int32_t)fb;
                    enablePwmOutput();
                }
            }
        } else {
            // During ramp or fault recovery: no burst
            burst_skip = false;
            burst_off_ms = 0;
            if (!ovFault && !getOcFault()) enablePwmOutput();
        }

        // ===== Control Phases =====
        if (!ovFault && !getOcFault()) {
            if (burst_skip) {
                setDuty(0.0f);
            } else {
                // Ensure PWM is enabled while active
                enablePwmOutput();

                if (phase == PH_PRECHARGE) {
                    if (pre_ms < PRECHARGE_TIME_MS) ++pre_ms;

                    float d_cmd = PRECHARGE_DUTY;
                    if (d_cmd > cap) d_cmd = cap;
                    setDutySlew(d_cmd, DUTY_STEP_MAX);

                    const uint16_t fb85 = (uint16_t)(fb_set_counts * PRECHARGE_EXIT_FRAC + 0.5f);
                    if (pre_ms >= PRECHARGE_TIME_MS || fb >= fb85) {
                        phase  = PH_PI;
                        integ  = duty;
                        d_filt = 0.0f; d_inited = 0;
                        fb_prev = (int32_t)fb;
                    }
                } else {
                    // PI + D on measurement
                    int32_t err_raw = (int32_t)fb_set_counts - (int32_t)fb;
                    int32_t err;
                    if      (err_raw >  (int32_t)ERR_DEADBAND_CNT) err = err_raw - (int32_t)ERR_DEADBAND_CNT;
                    else if (err_raw < -(int32_t)ERR_DEADBAND_CNT) err = err_raw + (int32_t)ERR_DEADBAND_CNT;
                    else                                           err = 0;

                    if (!d_inited) { d_filt = 0.0f; fb_prev = (int32_t)fb; d_inited = 1; }
                    int32_t d_raw = fb_prev - (int32_t)fb;  // counts per 1 ms
                    fb_prev = (int32_t)fb;
                    d_filt += DERIV_ALPHA * ((float)d_raw - d_filt);

                    float u = KP * (float)err + integ + KD * d_filt;
                    float u_sat = clampf(u, 0.0f, cap);

                    integ *= I_LEAK_FACTOR;
                    if ((u == u_sat) || (u > cap && err < 0) || (u < 0.0f && err > 0)) {
                        integ += KI * (float)err;
                    }
                    if (integ < 0.0f) integ = 0.0f;
                    if (integ > cap)  integ = cap;

                    setDutySlew(u_sat, DUTY_STEP_MAX);
                }
            }
        }
    }

    // ========= LED policy =========
    // OK LED: ON while in burst/hold, OFF otherwise (never on during faults)
    if (ovFault || oc_latch_ms) {
        led_ok_set(false);
    } else {
        led_ok_set(burst_skip);  // illuminate only in burst mode
    }

    // FAULT LED: blink codes (OC=3 blinks > OV=2 blinks)
    uint8_t code = 0;
    if (oc_latch_ms) code = 3;
    else if (ovFault) code = 2;
    run_fault_led(now, code);
}
