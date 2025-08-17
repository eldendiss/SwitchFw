#include <Arduino.h>
#include "pins.h"
#include "adc.h"
#include "ac.h"
#include "pwm.h"
#include "timebase.h"

// ========= Scaling / Targets =========
#define VOUT_TARGET_V 900.0f
#define VOUT_OV_TRIP_V 990.0f
#define VOUT_OV_CLEAR_V 950.0f

static inline uint16_t counts_from_volts(float v)
{
    return (uint16_t)(v * (1023.0f / 1077.5f) + 0.5f); // 431 * 2.5 = 1077.5
}
static inline float volts_from_counts(uint16_t c)
{
    return 431.0f * ((c * 2.5f) / 1023.0f);
}

#define FB_SET_COUNTS (counts_from_volts(VOUT_TARGET_V))
#define FB_OV_TRIP_CNT (counts_from_volts(VOUT_OV_TRIP_V))
#define FB_OV_CLR_CNT (counts_from_volts(VOUT_OV_CLEAR_V))

// ========= LEDs / Blink =========
#define LED_OK PC3
#define LED_FAULT PC2
#define OK_BLINK_US 1000000ull    // 1000 ms
#define OV_FLT_BLINK_US 50000ull  // 50 ms
#define OC_FLT_BLINK_US 200000ull // 200 ms

// ========= Control Tuning =========
#define CONTROL_PERIOD_US 1000u // 1 kHz
#define RAMP_TIME_MS 1500u      // 1.5 s total ramp
#define PRECHARGE_DUTY 0.015f
#define PRECHARGE_TIME_MS 200u
#define PRECHARGE_EXIT_FRAC 0.85f

#define KP 0.00045f
#define KI 0.000006f      // per 1 ms tick
#define KD 0.00018f       // derivative on measurement (counts/ms)
#define DERIV_ALPHA 0.05f // EMA on derivative (~12 ms @1 kHz)

#define DUTY_STEP_MAX 0.0008f
#define DUTY_ABS_MAX 0.20f
#define DUTY_RAMP_CAP 0.12f
#define DUTY_POST_CAP 0.10f

#define ERR_DEADBAND_CNT counts_from_volts(1.5f)
#define I_LEAK_FACTOR 0.9995f

// ====== Burst/skip (A): tighter window + min off-time ======
#define BURST_HI_V 0.8f      // stop when FB >= set + 0.8 V
#define BURST_LO_V 2.0f      // resume when FB <= set - 2.0 V
#define BURST_MIN_OFF_MS 40u // minimum time to stay off after stop

// ========= State =========
static uint32_t lastCtrlUs = 0;
static uint16_t ramp_ms = 0; // 0..RAMP_TIME_MS (true ms)
static uint16_t pre_ms = 0;
static float duty = 0.0f;
static float integ = 0.0f;

static int32_t fb_prev = 0; // last filtered FB (counts)
static float d_filt = 0.0f; // filtered derivative (counts/ms)
static uint8_t d_inited = 0;

static bool burst_skip = false;
static uint16_t burst_off_ms = 0;

static bool ovFault = true;

static uint64_t lastOkBlinkUs = 0;
static uint64_t lastErrBlinkUs = 0;
static uint32_t fbFilt = 0; // EMA filtered FB (counts)
static uint8_t fbFilt_inited = 0;

typedef enum
{
    PH_PRECHARGE,
    PH_PI
} phase_t;
static phase_t phase = PH_PRECHARGE;

// ========= Small helpers =========
static inline float clampf(float x, float lo, float hi)
{
    if (x < lo)
        return lo;
    if (x > hi)
        return hi;
    return x;
}
static inline void setDutySlew(float target, float stepMax)
{
    float delta = target - duty;
    if (delta > stepMax)
        delta = stepMax;
    if (delta < -stepMax)
        delta = -stepMax;
    duty += delta;
    setDuty(duty);
}

// ========= Setup =========
void setup()
{
    DDRC |= (1 << LED_OK) | (1 << LED_FAULT);

    // PB6 high
    DDRB |= (1 << PB6);
    PORTB |= (1 << PB6);

    cli();
    initPwm();
    initAdc();
    initAc();
    init_time_us();
    sei();

    enablePwmOutput();

    // Arm OC later and wait ~100 ms for AREF/ADC settle
    armComparatorAfter_us(80000, us_now());
    uint64_t start = us_now();
    while (us_now() - start <= 100000)
    {
    }
}

// ========= Main Loop =========
void loop()
{
    const uint32_t now = us_now();

    acService(now);

    // --- FB read + EMA (alpha = 1/8) ---
    uint16_t fb_raw = getFB();
    if (!fbFilt_inited)
    {
        fbFilt = fb_raw;
        fbFilt_inited = 1;
    }
    fbFilt += ((int32_t)fb_raw - (int32_t)fbFilt) >> 3;
    fbFilt += ((int32_t)fb_raw - (int32_t)fbFilt) >> 3;
    uint16_t fb = (uint16_t)fbFilt;

    // --- OV latch with hysteresis ---
    if (fb > FB_OV_TRIP_CNT)
        ovFault = true;
    if (ovFault && fb < FB_OV_CLR_CNT)
        ovFault = false;

    // --- Apply OV action ---
    if (ovFault)
    {
        disablePwmOutput();
        duty = 0.0f;
        integ = 0.0f;
        setDuty(0.0f);
    }
    else
    {
        enablePwmOutput();
    }

    // --- Control @ 1 kHz ---
    if ((uint32_t)(now - lastCtrlUs) >= CONTROL_PERIOD_US)
    {
        lastCtrlUs += CONTROL_PERIOD_US;

        // Fault → reset ramp & controller
        if (getOcFault() || ovFault)
        {
            ramp_ms = 0;
            pre_ms = 0;
            phase = PH_PRECHARGE;
            duty = 0.0f;
            integ = 0.0f;
            d_filt = 0.0f;
            d_inited = 0;
            fb_prev = (int32_t)fb;
            burst_skip = false;
            burst_off_ms = 0;
        }
        else if (ramp_ms < RAMP_TIME_MS)
        {
            ++ramp_ms;
        }

        // Ramped setpoint (counts)
        const float ramp = (float)ramp_ms / (float)RAMP_TIME_MS;
        const uint16_t fb_set_counts = (uint16_t)(FB_SET_COUNTS * ramp + 0.5f);

        // Duty ceilings
        float cap_soft = DUTY_RAMP_CAP * ramp;
        if (cap_soft > DUTY_RAMP_CAP)
            cap_soft = DUTY_RAMP_CAP;
        const float cap_abs = DUTY_ABS_MAX;
        float cap = (cap_soft < cap_abs) ? cap_soft : cap_abs;
        // after ramp completes, hold a smaller ceiling for stability at no-load
        if (ramp_ms >= RAMP_TIME_MS && cap > DUTY_POST_CAP)
            cap = DUTY_POST_CAP;

        // --- Burst/skip (A): tighter window + minimum off-time ---
        /*const uint16_t burst_hi_cnt = fb_set_counts + counts_from_volts(BURST_HI_V);
        const uint16_t burst_lo_cnt = (fb_set_counts > counts_from_volts(BURST_LO_V))
                                      ? fb_set_counts - counts_from_volts(BURST_LO_V) : 0;

        if (!burst_skip) {
          if (fb >= burst_hi_cnt) {
            burst_skip   = true;
            burst_off_ms = 0;           // start counting off-time
            duty = 0.0f; setDuty(0.0f);
          }
        } else {
          if (burst_off_ms < BURST_MIN_OFF_MS) ++burst_off_ms;
          // allow resume only after min off-time AND below low threshold
          if (burst_off_ms >= BURST_MIN_OFF_MS &&
              fb <= burst_lo_cnt && !ovFault && !getOcFault()) {
            burst_skip = false;
            d_filt = 0.0f; d_inited = 0; fb_prev = (int32_t)fb; // clean re-entry
          }
        }*/

        if (ovFault)
        {
            disablePwmOutput();
            duty = 0.0f;
            integ = 0.0f;
            setDuty(0.0f);
        }
        else
        {
            enablePwmOutput();

            // ===== Phase 1: precharge (gentle fill, no PI) =====
            if (phase == PH_PRECHARGE)
            {
                if (pre_ms < PRECHARGE_TIME_MS)
                    ++pre_ms;

                if (burst_skip)
                {
                    duty = 0.0f;
                    setDuty(0.0f);
                }
                else
                {
                    float d_cmd = PRECHARGE_DUTY;
                    if (d_cmd > cap)
                        d_cmd = cap;
                    setDutySlew(d_cmd, DUTY_STEP_MAX);
                }

                // Exit by time or 85% of *ramped* target
                const uint16_t fb85 = (uint16_t)(fb_set_counts * PRECHARGE_EXIT_FRAC + 0.5f);
                if (pre_ms >= PRECHARGE_TIME_MS || fb >= fb85)
                {
                    phase = PH_PI;
                    integ = duty; // seed integrator
                    d_filt = 0.0f;
                    d_inited = 0;
                    fb_prev = (int32_t)fb;
                }
            }
            // ===== Phase 2: PID (P + I + D on measurement) =====
            else
            {
                if (burst_skip)
                {
                    duty = 0.0f;
                    setDuty(0.0f);
                }
                else
                {
                    int32_t err_raw = (int32_t)fb_set_counts - (int32_t)fb;

                    // deadband around setpoint
                    int32_t err;
                    if (err_raw > (int32_t)ERR_DEADBAND_CNT)
                        err = err_raw - (int32_t)ERR_DEADBAND_CNT;
                    else if (err_raw < -(int32_t)ERR_DEADBAND_CNT)
                        err = err_raw + (int32_t)ERR_DEADBAND_CNT;
                    else
                        err = 0;

                    if (!d_inited)
                    {
                        d_filt = 0.0f;
                        fb_prev = (int32_t)fb;
                        d_inited = 1;
                    }
                    int32_t d_raw = fb_prev - (int32_t)fb; // counts per 1 ms
                    fb_prev = (int32_t)fb;

                    d_filt += DERIV_ALPHA * ((float)d_raw - d_filt); // LPF derivative

                    float u = KP * (float)err + integ + KD * d_filt;
                    float u_sat = clampf(u, 0.0f, cap);

                    integ *= I_LEAK_FACTOR;

                    // anti-windup: integrate only when inside or driving inward
                    if ((u == u_sat) || (u > cap && err < 0) || (u < 0.0f && err > 0))
                    {
                        integ += KI * (float)err;
                    }

                    if (integ < 0.0f)
                        integ = 0.0f;
                    if (integ > cap)
                        integ = cap;

                    setDutySlew(u_sat, DUTY_STEP_MAX);
                }
            }
        }
    }

    // --- LEDs (non-blocking) ---
    if (getOcFault())
    {
        if ((uint32_t)(now - lastErrBlinkUs) >= OC_FLT_BLINK_US)
        {
            PORTC ^= _BV(PC3);
            lastErrBlinkUs = now; // OK LED fast on OC
        }
    }
    else if (ovFault)
    {
        if ((uint32_t)(now - lastErrBlinkUs) >= OV_FLT_BLINK_US)
        {
            PORTC ^= _BV(PC3);
            lastErrBlinkUs = now; // OK LED faster on OV
        }
    }
    else
    {
        if ((uint32_t)(now - lastOkBlinkUs) >= OK_BLINK_US)
        {
            PORTC ^= _BV(PC2);
            lastOkBlinkUs = now; // FAULT LED slow when OK
        }
    }
}
