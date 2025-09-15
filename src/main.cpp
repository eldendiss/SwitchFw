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

// Shared with comparator ISR
extern volatile uint8_t g_allow_switch;

static AppConfig g_cfg;
static ControlState g_state;

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
  DDRC |= _BV(STATUS0) | _BV(STATUS1);
  led_init();

  // EN + ranges
  EN_init_input();
  range_init();

  // Board strap
  DDRB |= _BV(PB6);
  PORTB |= _BV(PB6);

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
  {
  }

  // I2C
  i2c_bind(&g_cfg, &g_state);
  i2c_init(0x2A); // slave addr
  (void)loaded;
}

void loop()
{
  uint32_t now = us_now32();

  // EN gating
  bool allow = EN_is_active() && !range_is_busy();
  g_allow_switch = allow;
  comp_service(now);

  // Range control based on external pins
  static uint8_t last_range = 0xFF;
  uint8_t rc = range_get_input_code();
  if (rc != last_range)
  {
    range_request(rc);
    last_range = rc;
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
