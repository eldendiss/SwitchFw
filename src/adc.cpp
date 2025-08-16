/* adc.cpp */

#include "adc.h"
#include "pins.h"

#define ADC_REF_VOLTAGE 2.50f // External reference voltage on AREF

volatile uint16_t fbADC_decim = 0;  // decimated avg every 8 samples (used by control)
static   uint16_t acc8 = 0;
static   uint8_t  cnt8 = 0;

int initAdc() {
    // ADC mux perm on AD0, external REF
    ADMUX = 0;

    // enable ADC, start freerunning mode, prescaler to fastest speed possible on 8 Mhz
    ADCSRA = (1 << ADEN) | (1 << ADATE) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1);

    // free running mode, AC mux disabled
    ADCSRB = 0;

    //disable digital buffer on ADC pins
    DIDR0 = 1 << ADC0D;

    ADCSRA |= (1 << ADSC); // Start conversions

    return 0;
}

ISR(ADC_vect) {
    acc8 += ADC;               // sum 8 consecutive samples
    if (++cnt8 >= 8) {
        fbADC_decim = acc8 >> 3;  // divide by 8
        acc8 = 0;
        cnt8 = 0;
    }
}

uint16_t getFB(void) {
    return fbADC_decim;
}