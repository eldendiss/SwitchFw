// comm/i2c_proto.h
#pragma once
#include <stdint.h>
#include "../cfg/config.h"
#include "../control/types.h"

/**
 * \file
 * \brief I²C slave protocol for controller configuration, range select, and status.
 * \details
 * The device exposes a byte-addressed register space accessed via a simple
 * "register pointer" scheme:
 *  - **Write (Master→Slave):** first byte = \b reg_ptr, subsequent bytes are
 *    written into the configuration image starting at \b reg_ptr, unless the
 *    address is reserved for commands or status.
 *  - **Read (Master←Slave):** set \b reg_ptr with a zero-length write or a
 *    write-then-read; the device returns data depending on \b reg_ptr.
 *
 * Multi-byte fields are **little-endian**. For config-image reads the device
 * returns up to **16 bytes** per onRequest(), then auto-increments \b reg_ptr.
 *
 * \warning The Wire callbacks execute in ISR context on AVR; keep handlers short.
 * Host writes into the live \c ControlParams image are **byte-wise** and can
 * tear multi-byte fields. If you need atomic updates, stage a block in the host
 * and send a "commit" command, or pause the main loop around a memcpy snapshot.
 *
 * \section regmap Register Map (little-endian)
 * | Addr | Name/Region               | Type     | Access | Notes                                                      |
 * |:----:|---------------------------|:--------:|:------:|------------------------------------------------------------|
 * | 0x00 | fb_set_cnt_tab[0]        | u16      | R/W    | Per-range setpoint (counts) for **range 0**.              |
 * | 0x02 | fb_set_cnt_tab[1]        | u16      | R/W    | Per-range setpoint (counts) for **range 1**.              |
 * | 0x04 | fb_set_cnt_tab[2]        | u16      | R/W    | Per-range setpoint (counts) for **range 2**.              |
 * | 0x06 | fb_set_cnt_tab[3]        | u16      | R/W    | Per-range setpoint (counts) for **range 3**.              |
 * | 0x08 | fb_ov_trip_cnt           | u16      | R/W    | OV trip threshold (counts).                               |
 * | 0x0A | fb_ov_clear_cnt          | u16      | R/W    | OV clear threshold (counts).                              |
 * | 0x0C | …ControlParams (rest)    | bytes    | R/W    | Remaining fields of \c ControlParams, contiguous.         |
 * | 0x10 | range_sel_req            | u8       | R/W    | Write 0..3 to request range (one relay selected). Readback returns last requested code. |
 * | 0x40 | cmd                      | u8       |  W     | \c 1=save config to EEPROM, \c 2=reset controller. Read returns 0x00.      |
 * | 0x50 | status                   | 8 bytes  |  R     | {u8 run\_state, u8 fault\_code, u16 fb, i16 dcounts, u8 active\_range, u8 busy}. |
 *
 * \note The **active** working setpoint used by the controller is \c ctrl.fb_set_cnt.
 * It is automatically updated by firmware when the physical range relay finishes
 * switching, using \c fb_set_cnt_tab[active_range].
 *
 * \note To persist configuration changes written to the ControlParams image,
 * issue command \c 1 at \c 0x40 (handled in main context via a pending flag).
 */

/**
 * \brief Initialize the I²C slave interface and register callbacks.
 * \param addr 7-bit I²C slave address.
 * \note Call \c i2c_bind() before this to attach backing storage.
 */
void i2c_init(uint8_t addr);

/**
 * \brief Bind configuration and controller state used by the I²C protocol.
 * \param cfg   Pointer to \c AppConfig (provides \c ControlParams image). Must outlive the binding.
 * \param state Pointer to \c ControlState for status reads (may be nullptr if unused).
 */
void i2c_bind(AppConfig *cfg, ControlState *state);

/**
 * \brief Command codes written to the \c 0x40 command register.
 */
typedef enum : uint8_t
{
    I2C_CMD_NONE = 0,        //!< No command.
    I2C_CMD_SAVE_CONFIG = 1, //!< Persist current \c AppConfig to EEPROM.
    I2C_CMD_RESET_CTRL = 2   //!< Soft-reset controller state (re-enter PRECHARGE).
} i2c_cmd_t;

/**
 * \brief Fetch and clear a pending command written to \c 0x40.
 * \param[out] cmd Receives the command code (see \c i2c_cmd_t).
 * \retval true  A command was pending and was returned.
 * \retval false No command pending.
 * \details
 * Implemented by the ISR write handler setting a flag; call this from the main loop
 * to act on host requests outside ISR context (EEPROM save, controller reset, ...).
 */
bool i2c_take_command(i2c_cmd_t *cmd);

/**
 * \brief Fetch and clear a pending range-select request written to \c 0x10.
 * \param[out] code Receives requested range code (0..3).
 * \retval true  A request was pending and was returned.
 * \retval false No request pending.
 * \details
 * Use in the main loop to start the safe break→wait-stable→make switching sequence.
 */
bool i2c_take_range_request(uint8_t *code);
