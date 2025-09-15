// comm/i2c_proto.cpp
#include <Wire.h>
#include "i2c_proto.h"
#include "../drivers/range.h"

/**
 * \file
 * \brief I²C glue: exposes ControlParams image + range/command/status registers.
 * \details
 * - Config image is written/read byte-wise (little-endian) starting at reg_ptr.
 * - Reserved registers:
 *   - 0x20: range select (W=0..3, R=last request)
 *   - 0x40: command      (W=code, R=0x00)
 *   - 0x50: status (8B): {run,fault,fb,d,active,busy}
 *
 * \warning Wire callbacks run in ISR context; only set flags / copy bytes.
 */

static AppConfig *g_cfg = nullptr;
static ControlState *g_state = nullptr;
static uint8_t reg_ptr = 0;

static volatile uint8_t g_range_req_pending = 0;
static volatile uint8_t g_range_req_code = 0;

static volatile uint8_t g_cmd_pending = 0;
static volatile uint8_t g_cmd_code = I2C_CMD_NONE;

static volatile uint8_t g_range_src = RANGE_SRC_AUTO; // default AUTO

uint8_t i2c_get_range_src() { return g_range_src; }
void i2c_set_range_src(uint8_t src)
{
  if (src > RANGE_SRC_AUTO)
    src = RANGE_SRC_AUTO;
  g_range_src = src;
}

void i2c_bind(AppConfig *cfg, ControlState *state)
{
  g_cfg = cfg;
  g_state = state;
}

bool i2c_take_command(i2c_cmd_t *cmd)
{
  if (!cmd || !g_cmd_pending)
    return false;
  *cmd = (i2c_cmd_t)g_cmd_code;
  g_cmd_pending = 0;
  return true;
}
bool i2c_take_range_request(uint8_t *code)
{
  if (!code || !g_range_req_pending)
    return false;
  *code = (uint8_t)(g_range_req_code & 0x03);
  g_range_req_pending = 0;
  return true;
}

/** \internal onReceive: sets reg_ptr; writes config/range/cmd. */
static void on_receive(int n)
{
  if (n <= 0)
    return;
  reg_ptr = Wire.read();
  n--;

  while (n > 0)
  {
    uint8_t b = Wire.read();
    n--;

    if (reg_ptr == 0x20)
    { // range select (I²C)
      g_range_req_code = (uint8_t)(b & 0x03);
      g_range_req_pending = 1;
      g_range_src = RANGE_SRC_I2C; // I²C takeover
      reg_ptr++;
      continue;
    }
    if (reg_ptr == 0x21)
    { // range_src (0=HW,1=I2C,2=AUTO)
      i2c_set_range_src(b);
      reg_ptr++;
      continue;
    }
    if (reg_ptr == 0x40)
    { // command
      g_cmd_code = b;
      g_cmd_pending = 1;
      reg_ptr++;
      continue;
    }
    if (reg_ptr >= 0x50 && reg_ptr < 0x60)
    { // RO status
      reg_ptr++;
      continue;
    }
    // config image write
    uint8_t *base = (uint8_t *)&g_cfg->ctrl;
    uint16_t sz = sizeof(ControlParams);
    if (reg_ptr < sz)
      base[reg_ptr] = b;
    reg_ptr++;
  }
}

/** \internal onRequest: streams config, or serves reserved regions. */
static void on_request()
{
  uint8_t *base = (uint8_t *)&g_cfg->ctrl;
  uint16_t sz = sizeof(ControlParams);

  if (reg_ptr == 0x20)
  {
    uint8_t c = g_range_req_code & 0x03;
    Wire.write(&c, 1);
    return;
  }
  if (reg_ptr == 0x21)
  {
    uint8_t s = g_range_src;
    Wire.write(&s, 1);
    return;
  }
  if (reg_ptr == 0x40)
  {
    uint8_t ack = 0;
    Wire.write(&ack, 1);
    return;
  }
  if (reg_ptr == 0x50 && g_state)
  {
    uint8_t buf[8];
    buf[0] = 0;
    buf[1] = 0;
    uint16_t fb = (uint16_t)g_state->fb_ema;
    int16_t d = 0;
    buf[2] = fb & 0xFF;
    buf[3] = fb >> 8;
    buf[4] = d & 0xFF;
    buf[5] = d >> 8;
    buf[6] = (uint8_t)(range_get_current_code() & 0x03);
    buf[7] = (uint8_t)(range_is_busy() ? 1 : 0);
    Wire.write(buf, sizeof buf);
    return;
  }
  if (reg_ptr < sz)
  {
    uint16_t remaining = (uint16_t)(sz - reg_ptr);
    uint8_t chunk = (remaining > 16) ? 16 : (uint8_t)remaining;
    Wire.write(base + reg_ptr, chunk);
    reg_ptr = (uint8_t)(reg_ptr + chunk);
    return;
  }
  uint8_t zero = 0;
  Wire.write(&zero, 1);
}

void i2c_init(uint8_t addr)
{
  Wire.begin(addr);
  Wire.onReceive(on_receive);
  Wire.onRequest(on_request);
}
