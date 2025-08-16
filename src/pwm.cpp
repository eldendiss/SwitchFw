/* pwm.cpp */

#include "pwm.h"
#include "pins.h"

#define PWM_FREQ 20000       // 20 kHz switching
#define DUTY_MIN 0.0f
#define DUTY_MAX 0.20f        // cap at 20%

volatile uint16_t topValue;

int initPwm(){
    // Start with output disabled; enable after init/soft-start
    TCCR1A = 0;                            // keep OC1A disabled initially
    TCCR1B = (1<<WGM13) | (1<<WGM12) | (1<<CS10);
    TCCR1A |= (1<<WGM11);                  // complete mode 14

    topValue = (uint16_t)((F_CPU / PWM_FREQ) - 1); // 8e6/20k - 1 = 399
    ICR1 = topValue;

    OCR1A = 0;
    DDRB |= (1 << PB1);
    PORTB &= ~(1 << PB1);
    return 0;
}

void enablePwmOutput(void)  { TCCR1A |=  (1 << COM1A1); }  // non-inverting
void disablePwmOutput(void) { TCCR1A &= ~(1 << COM1A1); }

void setDuty(float d) {
    if (d < DUTY_MIN) d = DUTY_MIN;
    if (d > DUTY_MAX) d = DUTY_MAX;
    uint16_t cmp = (uint16_t)(d * (float)(ICR1 + 1)); // Fast PWM uses (TOP+1)
    if (cmp > ICR1) cmp = ICR1;
    OCR1A = cmp;
}

