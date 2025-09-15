// drivers/status_led.cpp
#include "status_led.h"
#include "pins.h"

#define ON_US 120000UL
#define OFF_US 120000UL
#define PAUSE_US 600000UL

void led_init() { /* pins already outputs in main */ }

void led_fault_run(uint32_t now, FaultCode code)
{
  static uint8_t cur = 255, left = 0;
  static uint8_t st = 0;
  static uint32_t t0 = 0;
  enum
  {
    IDLE,
    ON,
    OFF,
    PAUSE
  };
  if (code != cur)
  {
    cur = code;
    left = code;
    t0 = now;
    st = (code == 0) ? IDLE : ON;
    led_fault_set(code != 0);
    if (!code)
      led_fault_set(false);
    return;
  }
  if (!code)
  {
    led_fault_set(false);
    return;
  }
  switch (st)
  {
  case ON:
    if (now - t0 >= ON_US)
    {
      led_fault_set(false);
      t0 = now;
      st = OFF;
    }
    break;
  case OFF:
    if (now - t0 >= OFF_US)
    {
      if (--left > 0)
      {
        led_fault_set(true);
        t0 = now;
        st = ON;
      }
      else
      {
        t0 = now;
        st = PAUSE;
      }
    }
    break;
  case PAUSE:
    if (now - t0 >= PAUSE_US)
    {
      left = code;
      led_fault_set(true);
      t0 = now;
      st = ON;
    }
    break;
  default:
    led_fault_set(false);
    break;
  }
}

void led_status_run(uint32_t now, RunState st)
{
  // Simple patterns on LED_OK:
  // BOOT: 2 Hz blink; WAIT_EN: 1 blink every 1s; PRECHARGE: fast blink; RAMP: medium;
  // REGULATING: ON; BURST_HOLD: slow blink; RANGE_SWITCH: double-blink pattern
  static RunState last = RunState(255);
  static uint32_t t0 = 0;
  if (st != last)
  {
    last = st;
    t0 = now;
    led_ok_set(false);
  }
  uint32_t dt = now - t0;
  switch (st)
  {
  case S_BOOT:
    led_ok_set((dt % 500000UL) < 250000UL);
    break; // 2 Hz
  case S_WAIT_EN:
    led_ok_set((dt % 1000000UL) < 120000UL);
    break; // short blip
  case S_PRECHARGE:
    led_ok_set((dt % 200000UL) < 80000UL);
    break; // fast
  case S_RAMP:
    led_ok_set((dt % 400000UL) < 180000UL);
    break; // medium
  case S_REGULATING:
    led_ok_set(true);
    break; // steady
  case S_BURST_HOLD:
    led_ok_set((dt % 1000000UL) < 500000UL);
    break; // 1 Hz
  case S_RANGE_SWITCH:
    led_ok_set((dt % 400000UL) < 60000UL || ((dt + 120000UL) % 400000UL) < 60000UL);
    break;
  default:
    led_ok_set(false);
    break;
  }
}
