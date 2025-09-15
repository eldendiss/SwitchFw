#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "driver/i2c.h"
#include "esp_err.h"

/**
 * \file
 * \brief ESP-IDF I²C client for the AVR flyback PSU (4 ranges + status + commands).
 * \details
 * This module provides an interface to communicate with the AVR flyback power supply unit (PSU)
 * over I²C. It supports:
 * - Initialization of an I²C master port to communicate with the PSU.
 * - Setting per-range setpoints in counts or volts and selecting the active range (0..3).
 * - Reading a compact status block including feedback, derivative, active range, and busy state.
 * - Updating arbitrary bytes in the ControlParams image.
 * - Sending commands such as SAVE and RESET.
 *
 * Multi-byte fields are encoded in little-endian format.
 */

#ifdef __cplusplus
extern "C"
{
#endif

    /** \brief Register addresses (multi-byte fields are little-endian). */
    enum
    {
        FLYBACK_REG_SET0 = 0x06,   //!< Setpoint register for range 0
        FLYBACK_REG_SET1 = 0x08,   //!< Setpoint register for range 1
        FLYBACK_REG_SET2 = 0x0A,   //!< Setpoint register for range 2
        FLYBACK_REG_SET3 = 0x0C,   //!< Setpoint register for range 3
        FLYBACK_REG_TRIP0 = 0x0E,  //!< Trip threshold register for range 0
        FLYBACK_REG_TRIP1 = 0x10,  //!< Trip threshold register for range 1
        FLYBACK_REG_TRIP2 = 0x12,  //!< Trip threshold register for range 2
        FLYBACK_REG_TRIP3 = 0x14,  //!< Trip threshold register for range 3
        FLYBACK_REG_CLEAR0 = 0x16, //!< Clear threshold register for range 0
        FLYBACK_REG_CLEAR1 = 0x18, //!< Clear threshold register for range 1
        FLYBACK_REG_CLEAR2 = 0x1A, //!< Clear threshold register for range 2
        FLYBACK_REG_CLEAR3 = 0x1C, //!< Clear threshold register for range 3

        FLYBACK_REG_ACTIVE_SET = 0x00,   //!< Active setpoint register (optional, R/W)
        FLYBACK_REG_ACTIVE_TRIP = 0x02,  //!< Active trip threshold register (optional)
        FLYBACK_REG_ACTIVE_CLEAR = 0x04, //!< Active clear threshold register (optional)

        FLYBACK_REG_RANGE_REQ = 0x20, //!< Requested range register
        FLYBACK_REG_RANGE_SRC = 0x21, //!< Range source register (0=HW, 1=I2C, 2=AUTO)
        FLYBACK_REG_CMD = 0x40,       //!< Command register (write-only)
        FLYBACK_REG_STATUS = 0x50     //!< Status register (read-only, 8 bytes)
    };

    /**
     * \brief Command byte values written to \c FLYBACK_REG_CMD (0x40).
     */
    typedef enum
    {
        FLYBACK_CMD_NONE = 0,        //!< No command (no operation)
        FLYBACK_CMD_SAVE_CONFIG = 1, //!< Persist current configuration to EEPROM
        FLYBACK_CMD_RESET_CTRL = 2   //!< Soft reset controller (re-enters PRECHARGE state)
    } flyback_psu_cmd_t;

    /**
     * \brief Arbitration mode for controlling the active range selection.
     */
    typedef enum
    {
        FLYBACK_RANGE_SRC_HW = 0,  //!< Hardware pins control the range selection
        FLYBACK_RANGE_SRC_I2C = 1, //!< I²C commands control the range selection
        FLYBACK_RANGE_SRC_AUTO = 2 //!< Hardware controls at boot; first I²C select takes over
    } flyback_range_src_t;

    /**
     * \brief 8-byte status block read from register 0x50.
     * \details Provides current run state, fault code, feedback counts, derivative,
     * active range, and range switching busy flag.
     */
    typedef struct
    {
        uint8_t run_state;    //!< Current run state (see AVR RunState enum)
        uint8_t fault_code;   //!< Current fault code (see AVR FaultCode enum)
        uint16_t fb_counts;   //!< Exponential moving average feedback in counts
        int16_t dcounts;      //!< Derivative of counts (counts per step or counts/ms)
        uint8_t active_range; //!< Currently active range (0..3, relay controlled)
        uint8_t range_busy;   //!< 1 if range switching is in progress, else 0
    } flyback_psu_status_t;

    /**
     * \brief Device context structure.
     * \details Holds configuration and state for the I²C communication with the PSU.
     * No dynamic memory allocation is performed inside this structure.
     */
    typedef struct
    {
        i2c_port_t port;       //!< I²C master port number (I2C_NUM_0 or I2C_NUM_1)
        uint8_t addr;          //!< 7-bit I²C device address (default 0x2A)
        uint32_t bus_hz;       //!< I²C bus speed in Hz (e.g., 400000)
        gpio_num_t sda_io;     //!< GPIO number for SDA line
        gpio_num_t scl_io;     //!< GPIO number for SCL line
        float fullscale_volts; //!< Full-scale voltage corresponding to 1023 counts (default 1077.5 V)
    } flyback_psu_t;

    /**
     * \brief Voltage profile for a single range.
     * \details Contains setpoint, trip, and clear voltages for one range.
     */
    typedef struct
    {
        float set_volts;   //!< Setpoint voltage for the range
        float trip_volts;  //!< Trip threshold voltage for the range
        float clear_volts; //!< Clear threshold voltage for the range
    } flyback_range_profile_v_t;

    /**
     * \brief Initialize the I²C master driver and store device context.
     * \param dev    Pointer to device context to initialize.
     * \param port   I²C port number (I2C_NUM_0 or I2C_NUM_1).
     * \param sda    GPIO number for SDA line.
     * \param scl    GPIO number for SCL line.
     * \param bus_hz I²C bus speed in Hz (e.g., 400000).
     * \param addr   7-bit I²C device address (default 0x2A).
     * \return ESP_OK on success, error code otherwise.
     * \note Safe to call once per port; if the driver is already installed, it will reuse it.
     */
    esp_err_t flyback_psu_init(flyback_psu_t *dev,
                               i2c_port_t port,
                               gpio_num_t sda,
                               gpio_num_t scl,
                               uint32_t bus_hz,
                               uint8_t addr);

    /**
     * \brief Set the voltage profile (set, trip, clear) for a specific range.
     * \param dev       Pointer to device context.
     * \param range_idx Range index (0..3).
     * \param p         Pointer to voltage profile structure.
     * \return ESP_OK on success.
     */
    esp_err_t flyback_psu_set_range_profile_v(const flyback_psu_t *dev, uint8_t range_idx,
                                              const flyback_range_profile_v_t *p);

    /**
     * \brief Deinitialize the I²C driver for the device's port.
     * \param dev Pointer to device context.
     * \return ESP_OK on success.
     * \note Optional; frees I²C driver resources.
     */
    esp_err_t flyback_psu_deinit(const flyback_psu_t *dev);

    /**
     * \brief Set the full-scale voltage for counts↔volts conversion.
     * \param dev Pointer to device context.
     * \param vfs Full-scale voltage corresponding to 1023 counts.
     * \note Default is 1077.5 volts.
     */
    static inline void flyback_psu_set_fullscale(flyback_psu_t *dev, float vfs)
    {
        dev->fullscale_volts = vfs;
    }

    /**
     * \brief Set the source controlling the active range selection.
     * \param dev Pointer to device context.
     * \param src Source selection (HW, I2C, or AUTO).
     * \return ESP_OK on success.
     * \details Writes the value to register 0x21.
     */
    esp_err_t flyback_psu_set_range_source(const flyback_psu_t *dev, flyback_range_src_t src);

    /**
     * \brief Read the current range source selection.
     * \param dev Pointer to device context.
     * \param out Pointer to store the read range source.
     * \return ESP_OK on success.
     */
    esp_err_t flyback_psu_get_range_source(const flyback_psu_t *dev, flyback_range_src_t *out);

    /**
     * \brief Convert a voltage value to counts using the device's full-scale voltage.
     * \param dev Pointer to device context.
     * \param v Voltage value to convert (volts).
     * \return Corresponding counts (0..1023).
     * \note Values below 0 are clamped to 0; values above full scale are clamped to 1023.
     */
    static inline uint16_t flyback_psu_volts_to_counts(const flyback_psu_t *dev, float v)
    {
        if (v < 0)
            v = 0;
        float fs = (dev->fullscale_volts > 1e-6f) ? dev->fullscale_volts : 1077.5f;
        float c = v * (1023.0f / fs);
        if (c < 0)
            c = 0;
        if (c > 1023.0f)
            c = 1023.0f;
        return (uint16_t)(c + 0.5f);
    }

    /**
     * \brief Convert counts to voltage using the device's full-scale voltage.
     * \param dev Pointer to device context.
     * \param c Counts value (0..1023).
     * \return Corresponding voltage in volts.
     */
    static inline float flyback_psu_counts_to_volts(const flyback_psu_t *dev, uint16_t c)
    {
        float fs = (dev->fullscale_volts > 1e-6f) ? dev->fullscale_volts : 1077.5f;
        return ((float)c) * (fs / 1023.0f);
    }

    /**
     * \brief Set the trip threshold in counts for a specific range.
     * \param dev       Pointer to device context.
     * \param range_idx Range index (0..3).
     * \param counts    Trip threshold counts (0..1023).
     * \return ESP_OK on success.
     */
    esp_err_t flyback_psu_set_trip_counts(const flyback_psu_t *dev, uint8_t range_idx, uint16_t counts);

    /**
     * \brief Set the clear threshold in counts for a specific range.
     * \param dev       Pointer to device context.
     * \param range_idx Range index (0..3).
     * \param counts    Clear threshold counts (0..1023).
     * \return ESP_OK on success.
     */
    esp_err_t flyback_psu_set_clear_counts(const flyback_psu_t *dev, uint8_t range_idx, uint16_t counts);

    /**
     * \brief Read the 8-byte status block from register 0x50.
     * \param dev Pointer to device context.
     * \param out Pointer to status structure to fill.
     * \return ESP_OK on success.
     */
    esp_err_t flyback_psu_read_status(const flyback_psu_t *dev, flyback_psu_status_t *out);

    /**
     * \brief Select the active range by writing to register 0x10.
     * \param dev Pointer to device context.
     * \param range_code Range code to select (0..3).
     * \return ESP_OK on success.
     */
    esp_err_t flyback_psu_select_range(const flyback_psu_t *dev, uint8_t range_code);

    /**
     * \brief Program a single per-range setpoint in counts into the setpoint table.
     * \param dev       Pointer to device context.
     * \param range_idx Range index (0..3).
     * \param counts    Setpoint counts (0..1023).
     * \return ESP_OK on success.
     */
    esp_err_t flyback_psu_set_table_counts(const flyback_psu_t *dev, uint8_t range_idx, uint16_t counts);

    /**
     * \brief Program a single per-range setpoint in volts into the setpoint table.
     * \param dev       Pointer to device context.
     * \param range_idx Range index (0..3).
     * \param volts     Desired voltage (converted internally to counts).
     * \return ESP_OK on success.
     */
    static inline esp_err_t flyback_psu_set_table_volts(const flyback_psu_t *dev, uint8_t range_idx, float volts)
    {
        return flyback_psu_set_table_counts(dev, range_idx, flyback_psu_volts_to_counts(dev, volts));
    }

    /**
     * \brief Program all four setpoint table entries at once using counts.
     * \param dev     Pointer to device context.
     * \param counts4 Array of 4 counts values [R0, R1, R2, R3].
     * \return ESP_OK on success.
     */
    esp_err_t flyback_psu_set_table_counts4(const flyback_psu_t *dev, const uint16_t counts4[4]);

    /**
     * \brief Issue a command to the PSU by writing to register 0x40.
     * \param dev Pointer to device context.
     * \param cmd Command to send (SAVE_CONFIG, RESET_CTRL, or NONE).
     * \return ESP_OK on success.
     */
    esp_err_t flyback_psu_send_command(const flyback_psu_t *dev, flyback_psu_cmd_t cmd);

    /**
     * \brief Poll the PSU until range switching is idle or timeout expires.
     * \param dev        Pointer to device context.
     * \param timeout_ms Maximum time to wait in milliseconds.
     * \param poll_ms    Polling interval in milliseconds.
     * \return ESP_OK if idle state observed before timeout; ESP_ERR_TIMEOUT otherwise.
     */
    esp_err_t flyback_psu_wait_idle(const flyback_psu_t *dev, uint32_t timeout_ms, uint32_t poll_ms);

    /**
     * \brief Raw I²C read helper starting at a given register.
     * \param dev Pointer to device context.
     * \param reg Starting register address.
     * \param dst Destination buffer to store read data.
     * \param len Number of bytes to read.
     * \return ESP_OK on success.
     * \note Handles 16-byte server chunking internally.
     */
    esp_err_t flyback_psu_read(const flyback_psu_t *dev, uint8_t reg, uint8_t *dst, size_t len);

    /**
     * \brief Raw I²C write helper starting at a given register.
     * \param dev Pointer to device context.
     * \param reg Starting register address.
     * \param src Source buffer containing data to write.
     * \param len Number of bytes to write.
     * \return ESP_OK on success.
     * \note Do not write to status registers (0x50..0x5F).
     *       Registers 0x40 (command) and 0x10 (active range) are single-byte control registers.
     */
    esp_err_t flyback_psu_write(const flyback_psu_t *dev, uint8_t reg, const uint8_t *src, size_t len);

#ifdef __cplusplus
} // extern "C"
#endif
