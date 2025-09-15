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

  // EN gating
  bool allow = EN_is_active() && !range_is_busy();
  g_allow_switch = allow;
  comp_service(now);

  // I2C-requested range changes
  uint8_t req;
  if (i2c_take_range_request(&req)) {
    // only start a new switch when not already switching
    if (!range_is_busy()) {
      range_request(req);
    }
  }

  // Track when the physical relay actually made, then apply the table->active setpoint
  static uint8_t last_made = 0xFF;
  uint8_t made = range_get_current_code();
  if (made != last_made) {
    last_made = made;
    if (made < 4) {
      g_cfg.ctrl.fb_set_cnt = g_cfg.ctrl.fb_set_cnt_tab[made];
      g_state.active_range = made; // if you added this to ControlState
    }
  }

  // Controller step (only if enabled)
  uint16_t fb;
  int16_t dcounts;
  uint16_t fb_raw = adc_fb_decim();
  FaultCode fcode = F_NONE;
  RunState rstate = S_WAIT_EN;

  if (!allow)
  {
    pwm_disable();
    pwm_set_duty(0.0f);
    led_status_run(now, range_is_busy() ? S_RANGE_SWITCH : S_WAIT_EN);
    led_fault_run(now, F_NONE);
    return;
  }

  // OC latch reflection for LED
  uint8_t oc_now = comp_oc_fault();
  controller_step(now, g_cfg.ctrl, g_state, fb_raw, oc_now, allow, fb, dcounts);

  uint8_t oc_event = comp_oc_pulse_sticky(); // use sticky
  if (oc_event)
    g_state.oc_latch_ms = 1500; // LED will always show OC click

  // Range service uses derivative to decide “stable”
  range_service(now, fb, dcounts);

  // Select LED state
  if (g_state.ov_fault)
  {
    fcode = F_OV;
    rstate = S_RAMP;
  } // showing OV fault
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

  // LEDs
  led_status_run(now, rstate);
  led_fault_run(now, fcode);
}
