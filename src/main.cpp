#include <Arduino.h>
#include <avr/sleep.h>
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
 * Range select is I²C-only (register 0x20). PD4/PD5 are free for other use.
 *
 * Power policy when EN is deasserted:
 * - ADC and Timer1 ISRs (OVF/COMPB) are stopped — not needed with gate off.
 * - CPU enters IDLE sleep; Timer2 OVF (every 256 µs) and I²C events wake it.
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

  // EN + range switches (no HW range inputs — I2C only)
  EN_init_input();
  range_init();

  // I2C
  i2c_bind(&g_cfg, &g_state);
  i2c_init(0x2A); // slave addr

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

  (void)loaded;
}

/**
 * \brief Main loop: gating, range handling, control step, and LED/UI updates.
 * \details
 * When EN is deasserted the CPU enters IDLE sleep after handling housekeeping.
 * Timer2 OVF (256 µs) and I²C events provide wake-up; Timer1 ISRs and ADC are
 * silenced in this state so they do not prevent the CPU from sleeping.
 */
void loop()
{
  uint32_t now = us_now32();

  static uint8_t last_made = 0xFF;

  // ---- I2C range requests (only source — HW pins repurposed) ----
  uint8_t req;
  if (i2c_take_range_request(&req))
  {
    if (!range_is_busy())
      range_request((uint8_t)(req & 0x03));
  }

  // ---- EN gating ----
  bool allow = EN_is_active() && !range_is_busy();
  g_allow_switch = allow;

  if (!allow)
  {
    // Stop peripherals that are only needed while switching
    adc_stop();
    comp_suspend(); // disarms comparator + silences Timer1 OVF/COMPB ISRs

    pwm_disable();
    pwm_set_duty(0.0f);
    range_service(now, 0, 0);

    // Apply per-range profile when relay makes during disabled state
    uint8_t made = range_get_current_code();
    if (made != last_made && made < 4)
    {
      last_made = made;
      g_cfg.ctrl.fb_set_cnt      = g_cfg.ctrl.fb_set_cnt_tab[made];
      g_cfg.ctrl.fb_ov_trip_cnt  = g_cfg.ctrl.fb_ov_trip_cnt_tab[made];
      g_cfg.ctrl.fb_ov_clear_cnt = g_cfg.ctrl.fb_ov_clear_cnt_tab[made];
      g_state.active_range = made;
      g_state.ramp_ms = 0;
      g_state.pre_ms  = 0;
      g_state.phase   = PH_PRECHARGE;
    }

    led_status_run(now, range_is_busy() ? S_RANGE_SWITCH : S_WAIT_EN);
    led_fault_run(now, F_NONE);

    // Sleep until Timer2 OVF (256 µs) or I²C interrupt
    set_sleep_mode(SLEEP_MODE_IDLE);
    sleep_mode();
    return;
  }

  // ---- Active path: restart peripherals silenced during disabled state ----
  adc_start();
  comp_resume(now); // re-enables Timer1 ISRs, schedules comparator re-arm
  comp_service(now);

  // ---- Normal control path ----
  uint16_t fb_raw = adc_fb_decim();
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
    g_cfg.ctrl.fb_set_cnt      = g_cfg.ctrl.fb_set_cnt_tab[made];
    g_cfg.ctrl.fb_ov_trip_cnt  = g_cfg.ctrl.fb_ov_trip_cnt_tab[made];
    g_cfg.ctrl.fb_ov_clear_cnt = g_cfg.ctrl.fb_ov_clear_cnt_tab[made];
    g_state.active_range = made;
    g_state.ramp_ms = 0;
    g_state.pre_ms  = 0;
    g_state.phase   = PH_PRECHARGE;
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
