// drivers/comp.cpp
#include <avr/io.h>
#include <avr/interrupt.h>
#include "comp.h"
#include "pwm.h"
#include "pins.h"

/**
 * \file
 * \brief Analog comparator–based over-current (OC) cut-off with PWM-cycle blanking.
 * \details
 * - Uses the on-chip analog comparator to detect an OC event and immediately force the gate low.
 * - Comparator is referenced to the internal 1.1 V bandgap (ACBG=1) and generates an interrupt
 *   on a falling edge (ACIS=10). Exact polarity/threshold depends on your external wiring.
 * - At every PWM period start (Timer1 overflow), the comparator interrupt is disabled and then
 *   re-enabled after a short blanking window (Timer1 OCR1B compare) to avoid switch edge spikes.
 *
 * ISRs:
 * - \c ANALOG_COMP_vect: Hard-kills the gate output, latches OC, and marks a sticky pulse.
 * - \c TIMER1_OVF_vect : Start of a new PWM cycle — clears the one-shot OC flag, optionally
 *   re-enables OC output driver if allowed, and arms the blanking timer.
 * - \c TIMER1_COMPB_vect: Ends blanking window — re-enables comparator interrupt (if armed).
 *
 * \warning The comparator ISR forcibly overrides OC1A by clearing COM1A1 and also drives the
 * gate pin low via DDR/PORT to ensure a hard turn-off. Ensure \c GATE_OUT mapping matches your PCB.
 */

/** \brief External gatekeeper: 1 allows PWM output to be re-enabled after an OC event. */
volatile uint8_t g_allow_switch = 0;
/** \brief One-shot OC flag cleared at each PWM period start. */
static volatile uint8_t ocFault = 0;
/** \brief Sticky pulse flag (latched until read via \c comp_oc_pulse_sticky()). */
static volatile uint8_t ocSticky = 0;
/** \brief Comparator armed state (controls ACIE). */
static volatile uint8_t ac_armed = 0;
/** \brief Timestamp when the comparator should be re-armed (µs). */
static uint32_t arm_deadline_us = 0;
/** \brief Blanking delay from period start to AC enable, in Timer1 counts. */
static const uint16_t OC_BLANK_COUNTS = 20; // OC blank into the cycle

/**
 * \brief Initialize comparator for OC detection and hook Timer1 cycle ISRs.
 * \details
 * - Disables digital input on AIN0/AIN1.
 * - Sets ACBG=1 (bandgap reference) and ACIS[1:0]=10 (falling-edge interrupt).
 * - Enables Timer1 overflow and OCR1B compare interrupts for blanking control.
 * - Arms comparator (internal flag only; ACIE is managed each cycle).
 */
void comp_init()
{
  // AIN0/AIN1 digital disable
  DIDR1 = _BV(AIN0D) | _BV(AIN1D);
  // Bandgap reference, falling-edge interrupt (ACIE managed by arm/blank)
  ACSR = _BV(ACBG) | _BV(ACIS1); // ACIS1:0=10 falling
  // Hook PWM-cycle service ISRs
  TIMSK1 |= _BV(TOIE1) | _BV(OCIE1B);
  ac_armed = 1;
}

/**
 * \brief Disarm comparator now and schedule re-arm after a delay.
 * \param delay_us  Re-arm delay in microseconds.
 * \param now_us    Current time (µs).
 * \details Clears ACIE immediately; \c comp_service() re-enables it once time elapses.
 */
void comp_arm_after_us(uint32_t delay_us, uint32_t now_us)
{
  arm_deadline_us = now_us + delay_us;
  ACSR &= ~_BV(ACIE);
  ac_armed = 0;
}

/**
 * \brief Periodic service to re-enable comparator interrupt after a timed delay.
 * \param now_us Current time (µs).
 */
void comp_service(uint32_t now_us)
{
  if (!ac_armed && (int32_t)(now_us - arm_deadline_us) >= 0)
  {
    ACSR |= _BV(ACIE);
    ac_armed = 1;
  }
}

/**
 * \brief Get and clear the current OC fault (one-shot, cleared each PWM period start).
 * \return Nonzero if an OC was detected since the last Timer1 overflow.
 */
uint8_t comp_oc_fault() { return ocFault; }

/**
 * \brief Read and clear the sticky OC pulse indicator.
 * \return 1 if any OC event occurred since the last read; 0 otherwise.
 */
uint8_t comp_oc_pulse_sticky()
{
  uint8_t s = ocSticky;
  ocSticky = 0;
  return s;
}

// ----------------------------------------------------------------------------
// ISRs
// ----------------------------------------------------------------------------

/**
 * \internal
 * \brief Analog Comparator ISR — immediate hard gate kill and latch OC.
 * \details
 * - Disables OC1A hardware drive (COM1A1=0).
 * - Forces the gate pin low via DDR/PORT for fail-safe turn-off.
 * - Clears OCR1A for good measure and latches OC/sticky flags.
 */
ISR(ANALOG_COMP_vect)
{
  TCCR1A &= ~_BV(COM1A1);  // disable OC1A (disconnect PWM driver)
  DDRB |= _BV(GATE_OUT);   // ensure pin is output
  PORTB &= ~_BV(GATE_OUT); // hard low
  OCR1A = 0;
  ocFault = 1;
  ocSticky = 1;
}

/**
 * \internal
 * \brief Timer1 overflow ISR — start of PWM period (blank comparator, optionally re-enable PWM).
 * \details
 * - If an OC was latched in the previous cycle and switching is allowed
 *   (\c g_allow_switch), re-enable OC1A output driver.
 * - Clears the one-shot OC flag.
 * - Disables comparator interrupt (ACIE=0) and programs OCR1B for the blanking window.
 */
ISR(TIMER1_OVF_vect)
{
  if (ocFault)
  {
    if (g_allow_switch)
    {
      TCCR1A |= _BV(COM1A1);
    } // re-enable if allowed
    ocFault = 0;
  }
  ACSR &= ~_BV(ACIE);      // disable until blanking ends
  OCR1B = OC_BLANK_COUNTS; // schedule re-enable
}

/**
 * \internal
 * \brief Timer1 compare-B ISR — end of blanking; re-enable comparator interrupt if armed.
 */
ISR(TIMER1_COMPB_vect)
{
  if (ac_armed)
    ACSR |= _BV(ACIE);
}
