// drivers/status_led.h
#pragma once
#include <stdint.h>

/**
 * \file
 * \brief Public API for status (OK) and fault LED patterns.
 * \details
 * Fault LED encodes a \c FaultCode as N blinks, pause, repeat.
 * OK LED indicates \c RunState with distinct blink/steady patterns.
 * Time base for the run functions is microseconds.
 */

/**
 * \brief Fault LED codes (shown on STATUS1 / LED_FAULT).
 */
enum FaultCode : uint8_t
{
  F_NONE = 0, //!< No fault (LED off).
  F_CFG = 1,  //!< Configuration error.
  F_OV = 2,   //!< Over-voltage fault.
  F_OC = 3,   //!< Over-current fault.
  F_EE = 4    //!< EEPROM/storage error.
};

/**
 * \brief Run-state patterns for the OK LED (STATUS0 / LED_OK).
 */
enum RunState : uint8_t
{
  S_BOOT = 0,    //!< 2 Hz blink.
  S_WAIT_EN,     //!< Short blip every 1 s.
  S_PRECHARGE,   //!< Fast blink.
  S_RAMP,        //!< Medium blink.
  S_REGULATING,  //!< Steady ON.
  S_BURST_HOLD,  //!< 1 Hz blink.
  S_RANGE_SWITCH //!< Double-blink pattern.
};

/**
 * \brief Initialize LED subsystem.
 * \details Pins are configured as outputs elsewhere (main); this is a no-op stub.
 */
void led_init();

/**
 * \brief Drive the fault LED to encode \c code as N blinks, pause, repeat.
 * \param now_us Current time in microseconds (monotonic).
 * \param code   Fault code to display; \c F_NONE turns LED off.
 */
void led_fault_run(uint32_t now_us, FaultCode code);

/**
 * \brief Drive the OK LED according to the current run state.
 * \param now_us Current time in microseconds (monotonic).
 * \param st     Current \c RunState.
 */
void led_status_run(uint32_t now_us, RunState st);
