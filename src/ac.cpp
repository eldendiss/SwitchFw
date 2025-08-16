/* ac.cpp */

#include "ac.h"
#include "pins.h"

volatile uint8_t ocFault = 0;
static volatile uint8_t ac_armed = 0;

static const uint16_t OC_BLANK_COUNTS = 8;

static uint64_t ac_arm_deadline_us = 0;

int initAc(){
    // Disable digital buffers on both AINs
    DIDR1 = 1 << AIN1D | 1 << AIN0D;

    // Comparator: + = 1.1V bandgap (ACBG=1), − = AIN1.
    // ACO goes LOW when AIN1 > 1.1V  -> use FALLING edge.
    ACSR = (1 << ACBG) | (1 << ACIE) | (1 << ACIS1); // ACIS1:0 = 10 (falling)

    TIMSK1 |= (1 << TOIE1) | (1 << OCIE1B);
    ac_armed = 0;
    // ACIS0 left as 0.

    return 0;
}

uint8_t getOcFault(){ return ocFault;}

void armComparatorAfter_us(uint64_t delay_us, uint64_t usNow){
    ac_arm_deadline_us = usNow + delay_us;
    ACSR &= ~(1 << ACIE);    // keep disabled until deadline
    ac_armed = 0;
}

void acService(uint64_t usNow){
    if (!ac_armed && (int64_t)(usNow - ac_arm_deadline_us) >= 0){
        ACSR |= (1 << ACIE); // enable comparator interrupts
        ac_armed = 1;
    }
}

//AC IRQ handler - stop all switching on OC condition - TURN OFF TIMER
ISR(ANALOG_COMP_vect) {
    TCCR1A &= ~(1 << COM1A1);   // disable OC1A compare output control
    DDRB   |=  (1 << PB1);      // ensure PB1 as GPIO output
    PORTB  &= ~(1 << PB1);      // drive gate LOW
    OCR1A   = 0;                // clear compare value (optional, keeps duty 0 if re-enabled blindly)
    ocFault = 1;
}

ISR(TIMER1_OVF_vect) {          // new PWM cycle
    if (ocFault) {
        TCCR1A |= (1 << COM1A1);   // re-enable output for new cycle
        ocFault = 0;             // clear fault flag
    }
    ACSR &= ~(1 << ACIE);
    OCR1B = OC_BLANK_COUNTS;  // schedule re-enable
}

ISR(TIMER1_COMPB_vect){
    if (ac_armed) ACSR |= (1 << ACIE);
}