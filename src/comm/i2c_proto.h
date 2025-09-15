// comm/i2c_proto.h
#pragma once
#include <stdint.h>
#include "../cfg/config.h"
#include "../control/types.h"

/**
 * \file
 * \brief I²C slave protocol for controller configuration, range select, and status.
 * \details
 * Byte-addressed "register pointer" scheme (little-endian multi-byte fields).
 * The config image is the live \c ControlParams in memory; host may R/W fields
 * directly. Reserved registers sit **outside** hot config offsets to avoid
 * collisions.
 *
 * \section regmap Register Map (little-endian)
 * | Addr | Name/Region                 | Type     | Access | Notes                                                  |
 * |:----:|-----------------------------|:--------:|:------:|--------------------------------------------------------|
 * | 0x00 | fb_set_cnt                  | u16      | R/W    | ACTIVE setpoint (used by controller).                  |
 * | 0x02 | fb_ov_trip_cnt              | u16      | R/W    | ACTIVE OV trip threshold.                              |
 * | 0x04 | fb_ov_clear_cnt             | u16      | R/W    | ACTIVE OV clear threshold.                             |
 * | 0x06 | fb_set_cnt_tab[0]           | u16      | R/W    | Per-range setpoint, R0.                                |
 * | 0x08 | fb_set_cnt_tab[1]           | u16      | R/W    | R1.                                                    |
 * | 0x0A | fb_set_cnt_tab[2]           | u16      | R/W    | R2.                                                    |
 * | 0x0C | fb_set_cnt_tab[3]           | u16      | R/W    | R3.                                                    |
 * | 0x0E | fb_ov_trip_cnt_tab[0]       | u16      | R/W    | Per-range OV trip, R0.                                 |
 * | 0x10 | fb_ov_trip_cnt_tab[1]       | u16      | R/W    | R1.                                                    |
 * | 0x12 | fb_ov_trip_cnt_tab[2]       | u16      | R/W    | R2.                                                    |
 * | 0x14 | fb_ov_trip_cnt_tab[3]       | u16      | R/W    | R3.                                                    |
 * | 0x16 | fb_ov_clear_cnt_tab[0]      | u16      | R/W    | Per-range OV clear, R0.                                |
 * | 0x18 | fb_ov_clear_cnt_tab[1]      | u16      | R/W    | R1.                                                    |
 * | 0x1A | fb_ov_clear_cnt_tab[2]      | u16      | R/W    | R2.                                                    |
 * | 0x1C | fb_ov_clear_cnt_tab[3]      | u16      | R/W    | R3.                                                    |
 * | ...  | (rest of ControlParams...)  | bytes    | R/W    | Gains, limits, PFM, etc.                               |
 * | 0x20 | range_sel_req               | u8       | R/W    | Write 0..3 to request range; read returns last request.|
 * | 0x21 | range_src                   | u8       | R/W    | 0=HW, 1=I2C, 2=AUTO (default AUTO)                     |
 * | 0x40 | cmd                         | u8       |  W     | 1=save config (EEPROM), 2=reset controller. Read: 0x00.|
 * | 0x50 | status                      | 8 bytes  |  R     | {u8 run,u8 fault,u16 fb,i16 d,u8 active,u8 busy}.      |
 *
 * \note Multi-byte fields are little-endian. Config reads stream up to 16 bytes
 * per onRequest() and auto-increment the internal register pointer.
 */

void i2c_init(uint8_t addr);
void i2c_bind(AppConfig *cfg, ControlState *state);

/** \brief Host commands for 0x40. */
typedef enum : uint8_t
{
    I2C_CMD_NONE = 0,
    I2C_CMD_SAVE_CONFIG = 1,
    I2C_CMD_RESET_CTRL = 2
} i2c_cmd_t;

/** Who controls the active range. */
typedef enum : uint8_t {
  RANGE_SRC_HW   = 0,  //!< Follow hardware pins only
  RANGE_SRC_I2C  = 1,  //!< Follow I²C only
  RANGE_SRC_AUTO = 2   //!< HW at boot; switch to I²C after first I²C request; optional timeout to fall back
} range_src_t;

/** R/W register (0x21) to select who controls the range. */
uint8_t i2c_get_range_src();
void    i2c_set_range_src(uint8_t src);

/**
 * \brief Fetch and clear a pending range-select request (0x20).
 * \param[out] code 0..3
 * \return true if a request was present.
 */
bool i2c_take_range_request(uint8_t *code);

/**
 * \brief Fetch and clear a pending command (0x40).
 * \param[out] cmd See \c i2c_cmd_t.
 * \return true if a command was present.
 */
bool i2c_take_command(i2c_cmd_t *cmd);
