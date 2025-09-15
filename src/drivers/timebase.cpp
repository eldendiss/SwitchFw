// drivers/timebase.cpp
#include <avr/io.h>
#include <avr/interrupt.h>
#include "timebase.h"

static volatile uint32_t us_high = 0;

void timebase_init_us()
{
  // CPU = 8 MHz. Timer2 prescaler = 8  => 1 MHz timer clock => 1 tick = 1 µs
  TCCR2A = 0;         // normal mode, OC2A/OC2B disconnected
  TCCR2B = _BV(CS21); // prescaler /8
  TCNT2 = 0;
  TIFR2 = _BV(TOV2);    // clear pending OVF
  TIMSK2 |= _BV(TOIE2); // enable overflow interrupt
}
ISR(TIMER2_OVF_vect) { us_high += 256; }

uint32_t us_now32()
{
  uint32_t base;
  uint8_t t, tifr, s = SREG;
  cli();
  t = TCNT2;
  tifr = TIFR2;
  base = us_high;
  SREG = s;
  if ((tifr & _BV(TOV2)) && (t < 255))
    base += 256;
  return base + t;
}
uint64_t us_now()
{
  static uint32_t last = 0;
  static uint64_t high = 0;
  uint32_t cur = us_now32();
  if (cur < last)
    high += (1ULL << 32);
  last = cur;
  return (high | cur);
}
