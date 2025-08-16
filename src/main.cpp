#include <Arduino.h>
#include "pins.h"
#include "adc.h"
#include "ac.h"
#include "pwm.h"
#include "timebase.h"

// ===== Targets (200 V) =====
#define VOUT_TARGET_V     200.0f
#define VOUT_OV_TRIP_V    220.0f
#define VOUT_OV_CLEAR_V   210.0f

// 431:1 divider, AREF=2.50 V  ->  full-scale 1077.5 V
static inline uint16_t fb_counts_from_volts(float v) {
    return (uint16_t)(v * (1023.0f / 1077.5f) + 0.5f);  // 1077.5 = 431 * 2.5
}
static inline float volts_from_counts(uint16_t c) {
    float vfb = (c * 2.5f) / 1023.0f;
    return 431.0f * vfb;
}

#define FB_SET_COUNTS    (fb_counts_from_volts(VOUT_TARGET_V))     // ~190
#define FB_OV_TRIP_CNT   (fb_counts_from_volts(VOUT_OV_TRIP_V))    // ~209
#define FB_OV_CLR_CNT    (fb_counts_from_volts(VOUT_OV_CLEAR_V))   // ~199

// ===== LEDs =====
#define LED_OK    PC3
#define LED_FAULT PC2

// ===== Timing =====
#define CONTROL_PERIOD_US   1000ull     // 1 kHz control
#define OK_BLINK_US         1000000ull  // 1000 ms
#define OV_FLT_BLINK_US     50000ull    // 50 ms
#define OC_FLT_BLINK_US     200000ull   // 200 ms
#define RAMP_TIME_MS        5000L

// ===== State =====
static uint64_t lastCtrl        = 0;
static bool     ovFault         = true;
static float    duty            = 0.0f;
static bool     rampActive      = true;
static uint64_t ramp_ms         = 0;
static uint64_t lastOkBlinkUs   = 0;
static uint64_t lastErrBlinkUs  = 0;
static uint32_t fbFilt          = 0;     // EMA filtered FB (counts)
static uint8_t  fbFilt_inited   = 0;

void setup() {
    // LEDs
    DDRC |= (1 << LED_OK) | (1 << LED_FAULT);

    // PB6 high (as requested)
    DDRB  |= (1 << PB6);
    PORTB |= (1 << PB6);

    cli();
    initPwm();
    initAdc();
    initAc();
    init_time_us();
    sei();

    enablePwmOutput();
    armComparatorAfter_us(80000, us_now());  // arm OC after ~80 ms
}

void loop() {
    const uint32_t now = us_now();

    acService(now);

    // --- FB read + EMA (alpha = 1/8) ---
    uint16_t fb = getFB();
    if (!fbFilt_inited) { fbFilt = fb; fbFilt_inited = 1; }
    fbFilt += ((int32_t)fb - (int32_t)fbFilt) >> 3;
    fb = (uint16_t)fbFilt;

    // --- OV latch with hysteresis ---
    if (fb > FB_OV_TRIP_CNT) {
        ovFault = true;
    }
    if (ovFault && fb < FB_OV_CLR_CNT) {
        ovFault = false;
    }

    // Apply OV action
    if (ovFault) {
        disablePwmOutput();
        duty = 0.0f;
        setDuty(0.0f);
    } else {
        enablePwmOutput();
    }

    // Soft-start ramp bookkeeping
    if (getOcFault() || ovFault) {
        rampActive = true;
        ramp_ms = 0;
    } else if (rampActive) {
        if (ramp_ms < RAMP_TIME_MS) ++ramp_ms;
        else rampActive = false;
    }

    // --- Control at 1 kHz ---
    if ((now - lastCtrl) >= CONTROL_PERIOD_US) {
        lastCtrl = now;

        // Setpoint counts: 0..FB_SET_COUNTS over RAMP_TIME_MS
        uint16_t fb_set_counts = (uint16_t)(
            ((uint32_t)FB_SET_COUNTS * (uint32_t)ramp_ms + (RAMP_TIME_MS/2)) /
            (uint32_t)RAMP_TIME_MS
        );

        // Duty cap grows 0..10% during ramp
        float duty_cap = 0.10f * ((float)ramp_ms / (float)RAMP_TIME_MS);
        if (duty_cap > 0.10f) duty_cap = 0.10f;

        if (!getOcFault() && !ovFault) {
            const float Kp = 0.0004f;         // conservative
            const int32_t err = (int32_t)fb_set_counts - (int32_t)fb;

            duty += Kp * (float)err;
            if (duty < 0.0f) duty = 0.0f;
            if (duty > duty_cap) duty = duty_cap;

            setDuty(duty);
            // keep PWM enabled when healthy
            enablePwmOutput();
        } else {
            duty = 0.0f;
            setDuty(0.0f);
            if (ovFault) disablePwmOutput();
        }
    }

    // --- LEDs (non-blocking) ---
    if (getOcFault()) {
        if ((uint32_t)(now - lastErrBlinkUs) >= OC_FLT_BLINK_US) {
            PORTC ^= _BV(PC3);      // blink OK LED fast on OC
            lastErrBlinkUs = now;
        }
    } else if (ovFault) {
        if ((uint32_t)(now - lastErrBlinkUs) >= OV_FLT_BLINK_US) {
            PORTC ^= _BV(PC3);      // blink OK LED faster on OV
            lastErrBlinkUs = now;
        }
    } else {
        if ((uint32_t)(now - lastOkBlinkUs) >= OK_BLINK_US) {
            PORTC ^= _BV(PC2);      // blink FAULT LED slow when OK
            lastOkBlinkUs = now;
        }
    }
}
