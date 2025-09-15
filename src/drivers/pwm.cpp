// drivers/pwm.cpp
#include <avr/io.h>
#include <avr/interrupt.h>
#include "pwm.h"
#include "pins.h"
#include "config_clock.h" // F_CPU_CFG (e.g. 8000000)

/**
 * \file
 * \brief Timer1-based PWM and one-shot pulse generator for the gate drive.
 * \details
 * - Timer: Fast PWM, TOP = ICR1, prescaler = 1.
 * - Frequency: \c PWM_FREQ (default 31.25 kHz when \c F_CPU_CFG=8 MHz → ICR1=255).
 * - Duty is fractional (0..1). Hardware compare uses \c OCR1A against TOP.
 * - Provides:
 *   - \c pwm_init() : configure timer and output pin, gate forced LOW
 *   - \c pwm_enable() / \c pwm_disable() with hard LOW on disable
 *   - \c pwm_enable_sync_bottom() : attach exactly at BOTTOM (period boundary)
 *   - \c pwm_set_duty() / \c pwm_get_duty() / \c pwm_min_step() / \c pwm_clk_hz()
 *   - \c pwm_emit_oneshot_us() : single on-time pulse starting at BOTTOM
 *
 * \warning This module directly manipulates Timer1 (TCCR1x/ICR1/OCR1A) and OC1A pin.
 * Ensure no other code/peripheral uses Timer1 or OC1A.
 */

/// \internal Cached last commanded duty (0..1).
static float duty_cache = 0.0f;
/// \internal Absolute safety ceiling for duty (0..1). Change via \c pwm_set_max().
static float duty_hw_max = 0.20f;
/// \internal One-shot pulse active flag (managed by COMPA ISR).
static volatile uint8_t oneshot_active = 0;

/// \brief Nominal PWM frequency in Hz (TOP computed from this and \c F_CPU_CFG).
#define PWM_FREQ 31250UL // 31.25 kHz @ 8 MHz -> ICR1=255

/**
 * \brief Initialize Timer1 for Fast PWM (TOP=ICR1, prescaler=1) and gate pin.
 * \details
 * - Sets WGM13:WGM11 and CS10 for Fast PWM, no prescaling.
 * - Computes TOP: \c ICR1 = F_CPU_CFG / PWM_FREQ - 1 (buffered).
 * - Initializes \c OCR1A = 0, sets gate pin as output and forces LOW.
 * \note PWM is left detached after init; call \c pwm_enable() or \c pwm_enable_sync_bottom().
 */
void pwm_init()
{
  // Fast PWM, TOP=ICR1, prescaler=1
  TCCR1A = 0;
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10);
  TCCR1A |= _BV(WGM11);

  ICR1 = (uint16_t)((F_CPU_CFG / PWM_FREQ) - 1); // 255 at 8 MHz
  OCR1A = 0;

  DDRB |= _BV(GATE_OUT);
  PORTB &= ~_BV(GATE_OUT); // gate hard LOW initially
}

/**
 * \brief Set absolute max allowed duty (safety clamp for \c pwm_set_duty()).
 * \param m New ceiling (0..1); constrained internally to [0.05, 0.95].
 */
void pwm_set_max(float m)
{
  if (m < 0.05f)
    m = 0.05f;
  if (m > 0.95f)
    m = 0.95f;
  duty_hw_max = m;
}

/**
 * \brief Attach OC1A (non-inverting) to start driving the gate.
 */
void pwm_enable() { TCCR1A |= _BV(COM1A1); }

/**
 * \brief Check if OC1A is currently attached (PWM driving).
 * \return \c true if enabled, \c false otherwise.
 */
bool pwm_is_enabled() { return (TCCR1A & _BV(COM1A1)) != 0; }

/**
 * \brief Detach OC1A and force the gate pin LOW (fail-safe).
 * \details
 * - Clears \c COM1A1 to detach hardware PWM.
 * - Drives \c GATE_OUT as output and pulls it LOW.
 * - Disables COMPA interrupt used for one-shot pulses.
 */
void pwm_disable()
{
  TCCR1A &= ~_BV(COM1A1); // detach OC1A
  DDRB |= _BV(GATE_OUT);
  PORTB &= ~_BV(GATE_OUT); // force gate LOW
  TIMSK1 &= ~_BV(OCIE1A);
  oneshot_active = 0;
}

/**
 * \brief Attach OC1A exactly at the next BOTTOM (period boundary).
 * \details
 * Spins until \c TOV1 is set (≤ 1 period), clears it, then attaches OC1A.
 * \note Keeps phase alignment and avoids mid-period duty steps on enable.
 */
void pwm_enable_sync_bottom()
{
  if (pwm_is_enabled())
    return;
  // Wait (≤ 32 µs @ 31.25 kHz) for OVF (== BOTTOM), then attach
  while (!(TIFR1 & _BV(TOV1)))
  { /* spin ≤ one period */
  }
  TIFR1 = _BV(TOV1);     // clear OVF we saw
  TCCR1A |= _BV(COM1A1); // non-inverting
}

/**
 * \brief Set PWM duty (fraction 0..1), quantized to timer resolution and clamped to \c pwm_set_max().
 * \param d Requested duty (0..1).
 * \details
 * - Updates \c duty_cache and buffered \c OCR1A. New compare takes effect at BOTTOM.
 * - Clamps to \c duty_hw_max.
 */
void pwm_set_duty(float d)
{
  if (d < 0.0f)
    d = 0.0f;
  if (d > duty_hw_max)
    d = duty_hw_max;
  duty_cache = d;

  const uint16_t top = ICR1; // buffered TOP
  uint32_t cmp = (uint32_t)((float)(top + 1) * d);
  if (cmp > top)
    cmp = top;
  OCR1A = (uint16_t)cmp; // buffered; takes effect at BOTTOM
}

/**
 * \brief Get the last commanded duty (fraction 0..1).
 */
float pwm_get_duty() { return duty_cache; }

/**
 * \brief Minimum duty step (quantization) for the current TOP.
 * \return 1/(ICR1+1), i.e., the granularity of \c pwm_set_duty().
 */
float pwm_min_step() { return 1.0f / (float)(ICR1 + 1); }

/**
 * \brief Return the PWM timer clock in Hz (\c F_CPU_CFG).
 */
uint32_t pwm_clk_hz() { return F_CPU_CFG; }

// ---- PFM one-shot -----------------------------------------------------------

/**
 * \brief Emit a single on-time pulse starting at the next BOTTOM.
 * \param ton_us Pulse on-time in microseconds (clamped to [1, TOP] counts).
 * \details
 * - Converts \c ton_us to Timer1 counts with prescaler=1 and writes \c OCR1A.
 * - Arms COMPA interrupt to detach OC1A and force gate LOW at pulse end.
 * - Uses \c pwm_enable_sync_bottom() to align the pulse to the period start.
 *
 * \warning This routine temporarily uses the COMPA ISR and overrides any
 * previously programmed \c OCR1A. Do not call concurrently with \c pwm_set_duty().
 */
void pwm_emit_oneshot_us(uint16_t ton_us)
{
  if (ton_us == 0)
    return;

  // Convert µs -> timer counts (prescaler=1)
  uint32_t counts = (uint32_t)ton_us * (F_CPU_CFG / 1000000UL);
  if (counts < 1)
    counts = 1;
  if (counts > ICR1)
    counts = ICR1; // cap within period

  OCR1A = (uint16_t)counts; // high from BOTTOM..COMPA
  oneshot_active = 1;
  TIMSK1 |= _BV(OCIE1A);    // detach at end of pulse
  pwm_enable_sync_bottom(); // start exactly at BOTTOM
}

/**
 * \internal
 * \brief COMPA ISR — end the one-shot pulse: detach PWM and force gate LOW.
 */
ISR(TIMER1_COMPA_vect)
{
  if (oneshot_active)
  {
    TCCR1A &= ~_BV(COM1A1); // detach OC1A
    DDRB |= _BV(GATE_OUT);
    PORTB &= ~_BV(GATE_OUT); // hold gate LOW
    oneshot_active = 0;
  }
  TIMSK1 &= ~_BV(OCIE1A); // stop COMPA IRQ
}
