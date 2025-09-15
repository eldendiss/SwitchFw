#pragma once
#include <avr/io.h>
#include <avr/interrupt.h>

// Run core at /8
#ifndef CPU_DIV_LOG2
#define CPU_DIV_LOG2 0
#endif

#define F_CPU_CFG (F_CPU >> CPU_DIV_LOG2)

static inline void clock_set_div(uint8_t log2_div){
  uint8_t s=SREG; cli();
  CLKPR=_BV(CLKPCE); CLKPR=(log2_div & 0x0F);
  SREG=s;
}
