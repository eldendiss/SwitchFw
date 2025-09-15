// drivers/adc.cpp
#include <avr/io.h>
#include <util/atomic.h>
#include "adc.h"
#include "config_clock.h"
#include "pins.h"

static volatile uint16_t fb_decim=0;
static uint16_t acc8=0; static uint8_t cnt8=0;

static uint8_t _adps_bits(){
  // choose closest to 125 kHz using F_CPU_CFG
  if      (F_CPU_CFG >= 16000000UL) return _BV(ADPS2)|_BV(ADPS1)|_BV(ADPS0); // /128
  else if (F_CPU_CFG >=  8000000UL) return _BV(ADPS2)|_BV(ADPS1);            // /64 -> 125k @8M
  else if (F_CPU_CFG >=  4000000UL) return _BV(ADPS2)|_BV(ADPS0);            // /32 -> 125k @4M
  else if (F_CPU_CFG >=  2000000UL) return _BV(ADPS2);                        // /16 -> 125k @2M
  else                               return _BV(ADPS1)|_BV(ADPS0);            // /8  -> 125k @1M
}

void adc_init(){
  // ADC0, AREF external
  ADMUX = 0;           // REFS=00, MUX=0000
  ADCSRB= 0;           // free running, AC mux off
  DIDR0 = _BV(ADC0D);  // disable digital buffer
  ADCSRA = _BV(ADEN)|_BV(ADATE)|_BV(ADIE)|_adps_bits();
  ADCSRA |= _BV(ADSC);
}
ISR(ADC_vect){
  acc8 += ADC; if (++cnt8>=8){ fb_decim = acc8>>3; acc8=0; cnt8=0; }
}
uint16_t adc_fb_decim(){ uint16_t v; ATOMIC_BLOCK(ATOMIC_RESTORESTATE){ v=fb_decim; } return v; }
