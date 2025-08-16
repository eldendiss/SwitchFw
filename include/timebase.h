#include <avr/io.h>
#include <avr/interrupt.h>

// High part accumulates in microseconds, step +256 on each overflow
static volatile uint32_t us_high = 0;

void init_time_us(void) {
    // Timer2: prescaler = 8  => 8 MHz / 8 = 1 MHz => 1 tick = 1 us
    // Free-running normal mode, overflow every 256 us
    TCCR2A = 0;                 // normal
    TCCR2B = (1 << CS21);       // prescaler 8
    TCNT2  = 0;
    TIFR2  = (1 << TOV2);       // clear pending OVF
    TIMSK2 |= (1 << TOIE2);     // enable overflow interrupt
}

ISR(TIMER2_OVF_vect) {
    us_high += 256;             // each overflow adds 256 us
}

// Race-safe read of 32-bit time in us
uint32_t us_now(void) {
    uint8_t  t1, t2;
    uint8_t  ovf1, ovf2;
    uint32_t base;

    // First snapshot
    ovf1 = TIFR2 & (1 << TOV2);
    t1   = TCNT2;
    base = us_high;

    // If OVF happened during read, re-sample
    ovf2 = TIFR2 & (1 << TOV2);
    t2   = TCNT2;
    if (ovf2 && (t2 < t1)) {
        // Account for the overflow we raced with
        base += 256;
        t1 = t2;
        // Clear the OVF flag we just consumed
        TIFR2 = (1 << TOV2);
    }

    return base + t1;
}
