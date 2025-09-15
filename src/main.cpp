#include <Arduino.h>
#include "pins.h"
#include "config_clock.h"

#include "drivers/timebase.h"
#include "drivers/pwm.h"
#include "drivers/adc.h"
#include "drivers/comp.h"
#include "drivers/range.h"
#include "drivers/status_led.h"

#include "control/types.h"
#include "control/controller.h"

#include "cfg/config.h"
#include "comm/i2c_proto.h"

/**
 * \file
 * \brief Firmware entry points: hardware bring-up, main control loop, and UI.
 * \details
 * Responsibilities:
 * - Enforce safe gate state on boot; set CPU prescaler via \c clock_set_div().
 * - Init low-level drivers (PWM, ADC, comparator, timebase) and range IO.
 * - Load configuration (EEPROM with wear leveling) and reset controller state.
 * - Bind and start the I²C slave (addr 0x2A) for config/status access.
 * - Run the 1 kHz control loop, range switching state machine, and LEDs.
 *
 * LED policy (see \c drivers/status_led.*):
 * - Fault LED encodes fault codes (OC/OV, etc.).
 * - OK LED indicates run state (boot, wait EN, precharge, ramp, PFM hold, etc.).
 */

// Shared with comparator ISR
extern volatile uint8_t g_allow_switch; //!< Allows PWM re-enable after OC in next period.

/** \brief Global configuration blob (loaded from EEPROM or defaults). */
static AppConfig g_cfg;
/** \brief Runtime controller state (integrators, latches, EMA, etc.). */
static ControlState g_state;

extern uint8_t i2c_get_range_src();
extern void i2c_set_range_src(uint8_t src);

static uint32_t last_i2c_select_us = 0;
static const uint32_t I2C_HOLD_US = 0; // set to e.g. 30*1000000 for AUTO fallback after 30 s

/**
 * \brief Arduino setup: put hardware in a safe state and initialize all subsystems.
 * \details
 * Steps:
 * 1) Force gate LOW and detach OC1A.
 * 2) Apply CPU prescaler (\c CPU_DIV_LOG2 → \c F_CPU_CFG).
 * 3) Configure LEDs, EN input, range IO, and a board strap.
 * 4) Initialize PWM/ADC/comparator/timebase (interrupt-safe).
 * 5) Load configuration; reset controller timebase/phase.
 * 6) Arm comparator after delay (AREF settle), then start I²C at 0x2A.
 */
void setup()
{
  cli();
  // Force gate LOW and detach timer output
  DDRB |= _BV(PB1);                       // PB1 as output
  PORTB &= ~_BV(PB1);                     // drive LOW
  TCCR1A &= ~(_BV(COM1A1) | _BV(COM1A0)); // detach OC1A from Timer1
  sei();

  clock_set_div(CPU_DIV_LOG2);

  // LEDs
  DDRC |= _BV(LED_OK) | _BV(LED_FAULT);
  led_init();

  // EN + ranges
  EN_init_input();
  range_init();

  _delay_ms(3000);

  noInterrupts();
  pwm_init();
  adc_init();
  comp_init();
  timebase_init_us();
  interrupts();

  // Config
  bool loaded = config_load(g_cfg);
  controller_reset(g_state, us_now32());

  // Arm OC later & settle AREF
  comp_arm_after_us(80000UL, us_now32());
  uint32_t t0 = us_now32();
  while (us_now32() - t0 < 100000UL)
  { /* settle */
  }

  // I2C
  i2c_bind(&g_cfg, &g_state);
  i2c_init(0x2A); // slave addr
  (void)loaded;
}

/**
 * \brief Main loop: gating, range handling, control step, and LED/UI updates.
 * \details
 * - EN gating: controller runs only if EN is active and range sequencer is idle.
 * - External range inputs trigger a safe break→wait-stable→make sequence.
 * - Controller executes 1 kHz steps with OV/OC handling and optional PFM hold.
 * - OC sticky pulses extend the OC indicator for UI visibility.
 * - LEDs reflect current run/fault states.
 */
void loop()
{
  uint32_t now = us_now32();

  // --- arbitration state (persist across calls) ---
  static uint32_t last_i2c_select_us = 0;
  static uint8_t last_hw_code = 0xFF;
  static uint8_t last_made = 0xFF;
  const uint32_t I2C_HOLD_US = 0; // >0 to AUTO-fallback to HW after idle; 0 = disabled

  // ---- source selection: HW vs I2C (0x21) ----
  uint8_t src = i2c_get_range_src();

  // I2C range requests (only if not forced-HW)
  uint8_t req;
  if ((src != RANGE_SRC_HW) && i2c_take_range_request(&req))
  {
    last_i2c_select_us = now;
    if (!range_is_busy())
      range_request((uint8_t)(req & 0x03));
    if (src == RANGE_SRC_AUTO)
      i2c_set_range_src(RANGE_SRC_I2C); // I²C takes over
  }

  // Optional AUTO fallback to HW after inactivity
  if (src == RANGE_SRC_AUTO && I2C_HOLD_US > 0)
  {
    if ((int32_t)(now - last_i2c_select_us) >= (int32_t)I2C_HOLD_US)
    {
      i2c_set_range_src(RANGE_SRC_HW);
      src = RANGE_SRC_HW;
    }
  }

  // HW pins drive range only in HW mode
  if (src == RANGE_SRC_HW)
  {
    uint8_t hw = range_get_input_code();
    if (hw != last_hw_code && !range_is_busy())
    {
      range_request((uint8_t)(hw & 0x03));
      last_hw_code = hw;
    }
  }

  // ---- EN gating computed AFTER arbitration ----
  bool allow = EN_is_active() && !range_is_busy();
  g_allow_switch = allow;
  comp_service(now);

  // Controller inputs
  uint16_t fb_raw = adc_fb_decim();

  if (!allow)
  {
    // Keep gate off but continue the range FSM
    pwm_disable();
    pwm_set_duty(0.0f);
    range_service(now, fb_raw, 0);

    // Apply per-range ACTIVE profile when relay makes
    uint8_t made = range_get_current_code();
    if (made != last_made && made < 4)
    {
      last_made = made;
      g_cfg.ctrl.fb_set_cnt = g_cfg.ctrl.fb_set_cnt_tab[made];
      g_cfg.ctrl.fb_ov_trip_cnt = g_cfg.ctrl.fb_ov_trip_cnt_tab[made];
      g_cfg.ctrl.fb_ov_clear_cnt = g_cfg.ctrl.fb_ov_clear_cnt_tab[made];
      g_state.active_range = made;

      // gentle re-entry
      g_state.ramp_ms = 0;
      g_state.pre_ms = 0;
      g_state.phase = PH_PRECHARGE;
    }

    led_status_run(now, range_is_busy() ? S_RANGE_SWITCH : S_WAIT_EN);
    led_fault_run(now, F_NONE);
    return;
  }

  // ---- Normal control path ----
  uint16_t fb;
  int16_t dcounts;
  uint8_t oc_now = comp_oc_fault();
  controller_step(now, g_cfg.ctrl, g_state, fb_raw, oc_now, allow, fb, dcounts);

  uint8_t oc_event = comp_oc_pulse_sticky();
  if (oc_event)
    g_state.oc_latch_ms = 1500;

  // Advance range FSM with real derivative (stability check meaningful)
  range_service(now, fb, dcounts);

  // Apply per-range ACTIVE profile when relay makes
  uint8_t made = range_get_current_code();
  if (made != last_made && made < 4)
  {
    last_made = made;
    g_cfg.ctrl.fb_set_cnt = g_cfg.ctrl.fb_set_cnt_tab[made];
    g_cfg.ctrl.fb_ov_trip_cnt = g_cfg.ctrl.fb_ov_trip_cnt_tab[made];
    g_cfg.ctrl.fb_ov_clear_cnt = g_cfg.ctrl.fb_ov_clear_cnt_tab[made];
    g_state.active_range = made;

    // restart gently after relay change
    g_state.ramp_ms = 0;
    g_state.pre_ms = 0;
    g_state.phase = PH_PRECHARGE;
    pwm_disable();
  }

  // ---- LEDs ----
  FaultCode fcode = F_NONE;
  RunState rstate = S_WAIT_EN;

  if (g_state.ov_fault)
  {
    fcode = F_OV;
    rstate = S_RAMP;
  }
  else if (g_state.oc_latch_ms)
  {
    fcode = F_OC;
    rstate = S_RAMP;
  }
  else if (g_state.phase == PH_PRECHARGE)
  {
    rstate = S_PRECHARGE;
  }
  else if (g_state.burst_skip)
  {
    rstate = S_BURST_HOLD;
  }
  else if (pwm_get_duty() > 0.0f)
  {
    rstate = S_REGULATING;
  }
  else
  {
    rstate = S_RAMP;
  }

  led_status_run(now, rstate);
  led_fault_run(now, fcode);
}
