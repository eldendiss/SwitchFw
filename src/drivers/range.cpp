// drivers/range.cpp
#include "range.h"
#include "status_led.h"
#include "pins.h"
#include "drivers/pwm.h"

uint32_t RANGE_BREAK_US = 5000; // 5 ms open
uint16_t RANGE_STABLE_DTH = 6;  // counts/ms

static uint8_t cur_code = 0, req_code = 0;
static uint8_t state = 0; // 0 idle, 1 open, 2 wait-stable, 3 make
static uint32_t t0 = 0;
static uint8_t busy = 0;

bool range_is_busy() { return busy; }

void range_init()
{
  range_inputs_init();
  rsw_init();
  rsw_make(cur_code);
}

uint8_t range_get_input_code() { return range_code_read(); }

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
