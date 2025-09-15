#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "driver/i2c.h"
#include "esp_err.h"

/**
 * \file
 * \brief ESP-IDF I²C client for the AVR flyback PSU (4 ranges + status + commands).
 * \details
 * Exposes helpers to:
 * - Initialize an I²C master port and talk to the PSU
 * - Set per-range setpoints (counts or volts) and select active range (0..3)
 * - Read compact status (fb, derivative, active range, busy)
 * - Update arbitrary bytes in the ControlParams image
 * - Send SAVE/RESET commands
 *
 * Multi-byte fields are little-endian.
 */

#ifdef __cplusplus
extern "C" {
#endif

/** \brief Register addresses (little-endian fields). */
enum {
  FLYBACK_REG_SET0      = 0x00, //!< u16 fb_set_cnt_tab[0]
  FLYBACK_REG_SET1      = 0x02, //!< u16 fb_set_cnt_tab[1]
  FLYBACK_REG_SET2      = 0x04, //!< u16 fb_set_cnt_tab[2]
  FLYBACK_REG_SET3      = 0x06, //!< u16 fb_set_cnt_tab[3]
  FLYBACK_REG_OV_TRIP   = 0x08, //!< u16 fb_ov_trip_cnt
  FLYBACK_REG_OV_CLEAR  = 0x0A, //!< u16 fb_ov_clear_cnt
  FLYBACK_REG_RANGE_REQ = 0x10, //!< u8  range select request (0..3)
  FLYBACK_REG_CMD       = 0x40, //!< u8  command (see \c flyback_psu_cmd_t)
  FLYBACK_REG_STATUS    = 0x50  //!< 8B status block
};

/** \brief Command byte values written to \c FLYBACK_REG_CMD (0x40). */
typedef enum {
  FLYBACK_CMD_NONE        = 0,   //!< No command
  FLYBACK_CMD_SAVE_CONFIG = 1,   //!< Persist current config to EEPROM
  FLYBACK_CMD_RESET_CTRL  = 2    //!< Soft reset controller (re-enter PRECHARGE)
} flyback_psu_cmd_t;

/** \brief 8-byte status block returned from 0x50. */
typedef struct {
  uint8_t  run_state;     //!< See AVR RunState
  uint8_t  fault_code;    //!< See AVR FaultCode
  uint16_t fb_counts;     //!< EMA feedback (counts)
  int16_t  dcounts;       //!< Derivative (counts/step or counts/ms per FW)
  uint8_t  active_range;  //!< 0..3 (relay-made)
  uint8_t  range_busy;    //!< 1 while switching, else 0
} flyback_psu_status_t;

/** \brief Device context (no heap allocations inside). */
typedef struct {
  i2c_port_t port;           //!< I²C master port
  uint8_t    addr;           //!< 7-bit I²C address (default 0x2A)
  uint32_t   bus_hz;         //!< Bus speed (e.g., 400000)
  gpio_num_t sda_io;         //!< SDA GPIO
  gpio_num_t scl_io;         //!< SCL GPIO
  float      fullscale_volts; //!< 1023 counts ≙ this many volts (default 1077.5)
} flyback_psu_t;

/**
 * \brief Initialize I²C master driver and store context.
 * \param dev    Output device context.
 * \param port   I²C port (I2C_NUM_0/1).
 * \param sda    SDA GPIO.
 * \param scl    SCL GPIO.
 * \param bus_hz Bus speed (e.g., 400000).
 * \param addr   7-bit device address (default 0x2A).
 * \return ESP_OK on success.
 * \note Safe to call once per port; if already installed, it will reuse the driver.
 */
esp_err_t flyback_psu_init(flyback_psu_t* dev,
                           i2c_port_t port,
                           gpio_num_t sda,
                           gpio_num_t scl,
                           uint32_t bus_hz,
                           uint8_t addr);

/**
 * \brief Deinitialize the I²C driver for \c dev->port (optional).
 */
esp_err_t flyback_psu_deinit(const flyback_psu_t* dev);

/** \brief Set the volts↔counts full-scale (default 1077.5). */
static inline void flyback_psu_set_fullscale(flyback_psu_t* dev, float vfs) {
  dev->fullscale_volts = vfs;
}

/** \brief Convert volts to counts using \c dev->fullscale_volts. */
static inline uint16_t flyback_psu_volts_to_counts(const flyback_psu_t* dev, float v) {
  if (v < 0) v = 0;
  float fs = (dev->fullscale_volts > 1e-6f) ? dev->fullscale_volts : 1077.5f;
  float c = v * (1023.0f / fs);
  if (c < 0) c = 0;
  if (c > 1023.0f) c = 1023.0f;
  return (uint16_t)(c + 0.5f);
}

/** \brief Convert counts to volts using \c dev->fullscale_volts. */
static inline float flyback_psu_counts_to_volts(const flyback_psu_t* dev, uint16_t c) {
  float fs = (dev->fullscale_volts > 1e-6f) ? dev->fullscale_volts : 1077.5f;
  return ((float)c) * (fs / 1023.0f);
}

/**
 * \brief Read the 8-byte status block (0x50).
 * \return ESP_OK on success.
 */
esp_err_t flyback_psu_read_status(const flyback_psu_t* dev, flyback_psu_status_t* out);

/**
 * \brief Select active range (0..3) by writing to 0x10.
 * \return ESP_OK on success.
 */
esp_err_t flyback_psu_select_range(const flyback_psu_t* dev, uint8_t range_code);

/**
 * \brief Program a single per-range setpoint (counts) into the table at 0x00/02/04/06.
 * \param range_idx 0..3.
 * \param counts    0..1023.
 * \return ESP_OK on success.
 */
esp_err_t flyback_psu_set_table_counts(const flyback_psu_t* dev, uint8_t range_idx, uint16_t counts);

/**
 * \brief Program a single per-range setpoint (volts) into the table.
 * \param range_idx 0..3.
 * \param volts     Desired voltage (converted via \c dev->fullscale_volts).
 */
static inline esp_err_t flyback_psu_set_table_volts(const flyback_psu_t* dev, uint8_t range_idx, float volts) {
  return flyback_psu_set_table_counts(dev, range_idx, flyback_psu_volts_to_counts(dev, volts));
}

/**
 * \brief Program all four table entries at once (counts).
 * \param counts4 4-element array [R0,R1,R2,R3] in counts.
 * \return ESP_OK on success.
 */
esp_err_t flyback_psu_set_table_counts4(const flyback_psu_t* dev, const uint16_t counts4[4]);

/**
 * \brief Issue a command (SAVE/RESET) to 0x40.
 */
esp_err_t flyback_psu_send_command(const flyback_psu_t* dev, flyback_psu_cmd_t cmd);

/**
 * \brief Poll until range switching is idle (status[7]==0) or timeout expires.
 * \param timeout_ms Max time to wait.
 * \param poll_ms    Poll period.
 * \return ESP_OK if idle observed; ESP_ERR_TIMEOUT otherwise.
 */
esp_err_t flyback_psu_wait_idle(const flyback_psu_t* dev, uint32_t timeout_ms, uint32_t poll_ms);

/**
 * \brief Raw read helper starting at \c reg (handles 16-B server chunking).
 */
esp_err_t flyback_psu_read(const flyback_psu_t* dev, uint8_t reg, uint8_t* dst, size_t len);

/**
 * \brief Raw write helper starting at \c reg (sends [reg][payload] chunks).
 * \details Do not write 0x50..0x5F (status); 0x40 and 0x10 are single-byte control regs.
 */
esp_err_t flyback_psu_write(const flyback_psu_t* dev, uint8_t reg, const uint8_t* src, size_t len);

#ifdef __cplusplus
} // extern "C"
#endif
