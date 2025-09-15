// drivers/comp.cpp
#include <avr/io.h>
#include <avr/interrupt.h>
#include "comp.h"
#include "pwm.h"
#include "pins.h"

volatile uint8_t g_allow_switch=0;
static volatile uint8_t ocFault=0;
static volatile uint8_t ocSticky = 0;
static volatile uint8_t ac_armed=0;
static uint32_t arm_deadline_us=0;
static const uint16_t OC_BLANK_COUNTS = 20; // OC blank into the cycle

void comp_init(){
  // AIN0 (PD6) vs 1.1V bandgap: trigger when AIN0 > 1.1V (falling edge)
  DIDR1 = _BV(AIN0D)|_BV(AIN1D);
  ACSR  = _BV(ACBG)|_BV(ACIS1); // ACIS1:0=10 falling
  TIMSK1 |= _BV(TOIE1)|_BV(OCIE1B);       // PWM cycle hooks
  ac_armed=1;
}
void comp_arm_after_us(uint32_t delay_us, uint32_t now_us){
  arm_deadline_us = now_us + delay_us;
  ACSR &= ~_BV(ACIE); ac_armed=0;
}
void comp_service(uint32_t now_us){
  if (!ac_armed && (int32_t)(now_us - arm_deadline_us) >= 0){
    ACSR |= _BV(ACIE); ac_armed=1;
  }
}
uint8_t comp_oc_fault(){ return ocFault; }

uint8_t comp_oc_pulse_sticky(){
  uint8_t s = ocSticky;
  ocSticky = 0;
  return s;
}

// Comparator ISR: kill gate immediately
ISR(ANALOG_COMP_vect){
  TCCR1A &= ~_BV(COM1A1);   // disable OC1A
  DDRB   |=  _BV(GATE_OUT);
  PORTB  &= ~_BV(GATE_OUT); // hard low
  OCR1A   = 0;
  ocFault = 1;
  ocSticky = 1;
}
// New PWM cycle
ISR(TIMER1_OVF_vect){
  if (ocFault){
    if (g_allow_switch){ TCCR1A |= _BV(COM1A1); } // re-enable if allowed
    ocFault=0;
  }
  ACSR &= ~_BV(ACIE);         // disable until blanking ends
  OCR1B = OC_BLANK_COUNTS;    // schedule re-enable
}
ISR(TIMER1_COMPB_vect){
  if (ac_armed) ACSR |= _BV(ACIE);
}
