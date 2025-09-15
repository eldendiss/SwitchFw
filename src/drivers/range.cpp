// drivers/range.cpp
#include "range.h"
#include "status_led.h"
#include "pins.h"
#include "drivers/pwm.h"

/**
 * \file
 * \brief Auto-ranging sequencer for resistor-divider (R_SW*) with stability guard.
 * \details
 * Implements a small state machine that:
 * 1) Opens all range switches,
 * 2) waits for a blanking time and for the feedback slope to settle,
 * 3) then closes the requested switch.
 *
 * This avoids switching ranges while the feedback is still slewing, which can
 * produce false OC/OV triggers or audible artifacts.
 *
 * \section states States
 * - 0: IDLE
 * - 1: OPEN (all R_SWx off)
 * - 2: WAIT_STABLE (blank + |dFB/dt| below threshold)
 * - 3: MAKE (close requested range)
 *
 * \note Input range code is read from \c RANGE0/RANGE1 (active-low) via \c range_code_read().
 * \note R_SW outputs drive the divider legs; mapping is in \c rsw_make().
 */

/** \brief Open interval after breaking the range (µs). */
uint32_t RANGE_BREAK_US = 5000; // 5 ms open
/** \brief Stability threshold for derivative (counts/ms). */
uint16_t RANGE_STABLE_DTH = 6; // counts/ms

/// \internal Currently engaged range code (0..3).
static uint8_t cur_code = 0, req_code = 0;
/// \internal State machine (0 idle, 1 open, 2 wait-stable, 3 make).
static uint8_t state = 0;
/// \internal Timestamp of last state transition (µs).
static uint32_t t0 = 0;
/// \internal Busy flag (1 when ranging sequence active).
static uint8_t busy = 0;

/**
 * \brief Returns whether the range sequencer is in progress.
 * \return 1 if busy, 0 if idle.
 */
bool range_is_busy() { return busy; }

/**
 * \brief Initialize range I/O and apply the current range.
 * \details
 * - Configures \c RANGE0/RANGE1 inputs with pull-ups.
 * - Sets R_SW pins as outputs.
 * - Re-applies \c cur_code to the R_SW network.
 */
void range_init()
{
  range_inputs_init();
  rsw_init();
  rsw_make(cur_code);
}

/**
 * \brief Read current external range-select inputs (active-low).
 * \return Code 0..3 derived from RANGE1:RANGE0.
 */
uint8_t range_get_input_code() { return range_code_read(); }

/**
 * @brief report current made range
 * @return Code 0..3
 */
uint8_t range_get_current_code() { return cur_code; }

/**
 * \brief Request a new divider range.
 * \param code Desired range code (0..3).
 * \details Starts the open→wait-stable→make sequence if \c code differs from current.
 */
void range_request(uint8_t code)
{
  req_code = (code & 3);
  if (req_code == cur_code)
    return;
  // begin sequence
  state = 1;
  t0 = 0;
  busy = 1;
}

/**
 * \brief Periodic service for the auto-ranging state machine.
 * \param now       Current time in microseconds (monotonic).
 * \param fb        Current feedback (counts). (Unused here, reserved for future heuristics.)
 * \param dfb_ms    Feedback derivative in counts/ms (signed).
 * \details
 * - State 1: opens all R_SW outputs; leaves PWM untouched.
 * - State 2: waits at least \c RANGE_BREAK_US and for \c |dfb_ms| ≤ \c RANGE_STABLE_DTH.
 * - State 3: closes the requested R_SW and returns to idle.
 * \note Caller should invoke this at ~1 kHz or faster to maintain timing granularity.
 */
void range_service(uint32_t now, uint16_t fb, int16_t dfb_ms)
{
  switch (state)
  {
  case 0:
    busy = 0;
    break;

  case 1:
    rsw_all_open(); /* do not touch PWM here */
    t0 = now;
    state = 2;
    break;

  case 2:
    if ((uint32_t)(now - t0) >= RANGE_BREAK_US)
    {
      if ((dfb_ms < 0 ? -dfb_ms : dfb_ms) <= RANGE_STABLE_DTH)
      {
        state = 3;
      }
    }
    break;

  case 3:
    rsw_make(req_code);
    cur_code = req_code;
    state = 0;
    busy = 0;
    break;
  }
}
