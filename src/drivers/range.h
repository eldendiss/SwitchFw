// drivers/range.h
#pragma once
#include <stdint.h>

/**
 * \file
 * \brief Auto-ranging control for resistor-divider (R_SWx) with stability guard.
 * \details
 * Provides a small state machine to switch ranges safely:
 * open → wait blanking + |dFB/dt| stable → make.
 * Units: time in µs; feedback in ADC counts; derivative in counts/ms.
 */

/**
 * \brief Initialize range I/O and apply current range.
 * \details Configures RANGE0/1 inputs (with pull-ups) and R_SW outputs.
 */
void range_init();

/**
 * \brief Read current external range-select inputs (active-low).
 * \return Code 0..3 derived from RANGE1:RANGE0.
 */
uint8_t range_get_input_code();

/**
 * @brief report current made range (0..3)
 *
 * @return Code 0..3
 */
uint8_t range_get_current_code();

/**
 * \brief Request switching to a new divider range.
 * \param code Desired range code (0..3).
 * \details Starts the open→wait-stable→make sequence if different from current.
 */
void range_request(uint8_t code); // request switch (0..3)

/**
 * \brief Whether a range switch sequence is in progress.
 * \return true if busy, false if idle.
 */
bool range_is_busy();

/**
 * \brief Service the auto-ranging state machine.
 * \param now_us     Current time (µs, monotonic).
 * \param fb_counts  Current feedback (ADC counts). (Reserved for future heuristics.)
 * \param fb_dcounts Feedback derivative (counts/ms, signed).
 * \details
 * Call periodically (≈1 kHz). Performs:
 * - OPEN: all R_SW off and start blanking timer.
 * - WAIT_STABLE: wait ≥ RANGE_BREAK_US and |fb_dcounts| ≤ RANGE_STABLE_DTH.
 * - MAKE: close requested R_SW and return to idle.
 */
void range_service(uint32_t now_us, uint16_t fb_counts, int16_t fb_dcounts);

// ---- Parameters -------------------------------------------------------------

/**
 * \brief Open time between breaking the old range and making the new one (µs).
 * \details Default 5000 (5 ms).
 */
extern uint32_t RANGE_BREAK_US; // open time before make

/**
 * \brief Stability threshold for |dFB/dt| to consider feedback settled (counts/ms).
 * \details Default 6 counts/ms.
 */
extern uint16_t RANGE_STABLE_DTH; // |dFB/dms| threshold to consider stable
