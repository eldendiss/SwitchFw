// comm/i2c_proto.cpp
#include <Wire.h>
#include "i2c_proto.h"
#include "../drivers/range.h"

/**
 * \file
 * \brief I²C register-pointer protocol glue for config/range control and status.
 * \details
 * Exposes the live \c ControlParams image (inside \c AppConfig), a **range-select**
 * register, a **command** register, and a compact **status** block.
 *
 * Protocol summary (little-endian, byte-addressed):
 * - **Write (Master→Slave)**:
 *   - First byte = \b reg_ptr (register pointer).
 *   - Following bytes → copied byte-wise into the config image starting at \b reg_ptr,
 *     except for reserved regions (commands/status) which are handled specially.
 * - **Read (Master←Slave)**:
 *   - Set \b reg_ptr, then read.
 *   - Config reads stream up to 16 bytes per onRequest and auto-increment \b reg_ptr.
 *   - Special regions return fixed-size payloads.
 *
 * Reserved regions:
 * - \c 0x10 : \b range_sel_req — W: 0..3 to request relay range; R: last requested code.
 * - \c 0x40 : \b cmd — W: command byte (see \c i2c_cmd_t); R: single \c 0x00 ack.
 * - \c 0x50 : \b status — R: 8 bytes
 *   \code
 *   [0] u8  run_state      (placeholder, 0 unless main fills ControlState prior to read)
 *   [1] u8  fault_code     (placeholder, 0 unless main fills)
 *   [2..3] u16 fb_counts   (EMA-filtered FB)
 *   [4..5] i16 dcounts     (last derivative, if main cached it; else 0)
 *   [6] u8  active_range   (0..3; physical relay-made range)
 *   [7] u8  range_busy     (1 while switching; else 0)
 *   \endcode
 *
 * \note The register pointer \b reg_ptr is 8-bit and wraps at 0xFF.
 * \warning Wire callbacks run in ISR context; only set flags / copy bytes.
 */

// ----------------------------------------------------------------------------
// Backing pointers / register pointer
// ----------------------------------------------------------------------------

/// \internal Bound configuration (provides access to ControlParams).
static AppConfig *g_cfg = nullptr;
/// \internal Bound controller state (optional; used by status reply).
static ControlState *g_state = nullptr;
/// \internal 8-bit register pointer (see file description).
static uint8_t reg_ptr = 0;

/// \internal I²C-side pending range request (consumed by main loop).
static volatile uint8_t g_range_req_pending = 0;
static volatile uint8_t g_range_req_code = 0;

/// \internal I²C-side pending command (consumed by main loop).
static volatile uint8_t g_cmd_pending = 0;
static volatile uint8_t g_cmd_code = I2C_CMD_NONE;

/**
 * \brief Bind configuration and state structures for I²C access.
 * \param cfg   Pointer to application config (must outlive this module).
 * \param state Pointer to controller state (may be null if status is unused).
 */
void i2c_bind(AppConfig *cfg, ControlState *state)
{
  g_cfg = cfg;
  g_state = state;
}

/// \internal Byte view of the control parameter block.
static uint8_t *cfg_bytes() { return (uint8_t *)&g_cfg->ctrl; }

/**
 * \brief Fetch and clear a pending command written to \c 0x40.
 * \param[out] cmd Receives the command code (\c i2c_cmd_t).
 * \retval true  A command was pending and was returned.
 * \retval false No command pending.
 */
bool i2c_take_command(i2c_cmd_t *cmd)
{
  if (!cmd)
    return false;
  if (!g_cmd_pending)
    return false;
  *cmd = (i2c_cmd_t)g_cmd_code;
  g_cmd_pending = 0;
  return true;
}

/**
 * \brief Fetch and clear a pending range-select request written to \c 0x10.
 * \param[out] code Receives requested range code (0..3).
 * \retval true  A request was pending and was returned.
 * \retval false No request pending.
 */
bool i2c_take_range_request(uint8_t *code)
{
  if (!code)
    return false;
  if (!g_range_req_pending)
    return false;
  *code = (uint8_t)(g_range_req_code & 0x03);
  g_range_req_pending = 0;
  return true;
}

// ----------------------------------------------------------------------------
// I²C callbacks
// ----------------------------------------------------------------------------

/**
 * \internal
 * \brief onReceive handler: first byte sets \b reg_ptr; remaining bytes write config or reserved regs.
 * \param n Number of bytes received in this transaction.
 * \details
 * - \c 0x10 : range select (1 byte) → sets pending range request flag.
 * - \c 0x40 : command      (1 byte) → sets pending command flag.
 * - \c 0x50..0x5F : ignored (status area is read-only).
 * - Else   : written into ControlParams image up to \c sizeof(ControlParams).
 * \warning Byte-wise writes can tear multi-byte fields; host should stage+commit if needed.
 */
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

    // Range select write at 0x10 (one byte)
    if (reg_ptr == 0x10)
    {
      g_range_req_code = (uint8_t)(b & 0x03);
      g_range_req_pending = 1;
      reg_ptr++;
      continue;
    }

    // Command write at 0x40 (one byte)
    if (reg_ptr == 0x40)
    {
      g_cmd_code = b;
      g_cmd_pending = 1;
      reg_ptr++;
      continue;
    }

    // Block writes into status region 0x50..0x5F (read-only)
    if (reg_ptr >= 0x50 && reg_ptr < 0x60)
    {
      reg_ptr++;
      continue;
    }

    // General config image writes (into ControlParams)
    uint8_t *base = cfg_bytes();
    uint16_t sz = sizeof(ControlParams);
    if (reg_ptr < sz)
    {
      base[reg_ptr] = b;
    }
    reg_ptr++;
  }
}

/**
 * \internal
 * \brief onRequest handler: respond based on \b reg_ptr (config/status/ack/zero).
 * \details
 * - Config region: streams up to 16 bytes per request (auto-incrementing \b reg_ptr).
 * - \c 0x10 : returns the last requested range code (not necessarily made yet).
 * - \c 0x40 : command ack region → returns a single byte \c 0x00.
 * - \c 0x50 : returns 8-byte status block (see file header).
 * - Other   : returns a single zero byte.
 * \note Multi-byte fields are little-endian.
 */
static void on_request()
{
  uint8_t *base = cfg_bytes();
  uint16_t sz = sizeof(ControlParams);

  if (reg_ptr == 0x10)
  {
    // Return last requested range (not necessarily made yet)
    uint8_t code = (uint8_t)(g_range_req_code & 0x03);
    Wire.write(&code, 1);
    return;
  }
  else if (reg_ptr == 0x40)
  {
    // Command ack / placeholder
    uint8_t ack = 0;
    Wire.write(&ack, 1);
    return;
  }
  else if (reg_ptr == 0x50 && g_state)
  {
    // Status block: 8 bytes
    uint8_t buf[8];
    buf[0] = 0; // run_state (caller can mirror into this struct in main if desired)
    buf[1] = 0; // fault_code
    uint16_t fb = (uint16_t)g_state->fb_ema;
    int16_t dcounts = 0; // main can cache derivative for reporting, else 0
    buf[2] = (uint8_t)(fb & 0xFF);
    buf[3] = (uint8_t)(fb >> 8);
    buf[4] = (uint8_t)(dcounts & 0xFF);
    buf[5] = (uint8_t)(dcounts >> 8);
    buf[6] = (uint8_t)(range_get_current_code() & 0x03); // active (made) range
    buf[7] = (uint8_t)(range_is_busy() ? 1 : 0);         // 1 while switching
    Wire.write(buf, sizeof(buf));
    return;
  }
  else
  {
    if (reg_ptr < sz)
    {
      // Stream out config image in <=16B chunks
      uint16_t remaining = (uint16_t)(sz - reg_ptr);
      uint8_t chunk = (remaining > 16) ? 16 : (uint8_t)remaining;
      Wire.write(base + reg_ptr, chunk);
      reg_ptr = (uint8_t)(reg_ptr + chunk);
      return;
    }
  }

  uint8_t zero = 0;
  Wire.write(&zero, 1);
}

/**
 * \brief Initialize the I²C slave and register the handlers.
 * \param addr 7-bit I²C slave address.
 * \note Must be called once during setup; \c i2c_bind() should be called beforehand.
 */
void i2c_init(uint8_t addr)
{
  Wire.begin(addr); // slave
  Wire.onReceive(on_receive);
  Wire.onRequest(on_request);
}
