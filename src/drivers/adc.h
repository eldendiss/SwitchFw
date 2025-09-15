// drivers/adc.h
#pragma once
#include <stdint.h>
void adc_init();
uint16_t adc_fb_decim(); // returns decimated ADC counts (0..1023)
