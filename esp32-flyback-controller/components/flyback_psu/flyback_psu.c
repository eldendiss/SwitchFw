#include "flyback_psu.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>

#ifndef FLYBACK_PSU_I2C_TIMEOUT_MS
#define FLYBACK_PSU_I2C_TIMEOUT_MS 100 //!< Default I2C timeout in milliseconds
#endif

static const char *TAG = "flyback_psu";

/**
 * \brief Initialize the flyback PSU device context and I²C master driver.
 * \param dev   Pointer to device context to initialize.
 * \param port  I²C port number (I2C_NUM_0 or I2C_NUM_1).
 * \param sda   GPIO number for SDA line.
 * \param scl   GPIO number for SCL line.
 * \param bus_hz I²C bus speed in Hz (default 400000 if 0).
 * \param addr  7-bit I²C device address (default 0x2A if 0).
 * \return ESP_OK on success, error code otherwise.
 * \note If the I2C driver is already installed on the port, it will be reused.
 */
esp_err_t flyback_psu_init(flyback_psu_t *dev,
                           i2c_port_t port,
                           gpio_num_t sda,
                           gpio_num_t scl,
                           uint32_t bus_hz,
                           uint8_t addr)
{
    ESP_RETURN_ON_FALSE(dev, ESP_ERR_INVALID_ARG, TAG, "dev null");
    memset(dev, 0, sizeof(*dev));
    dev->port = port;
    dev->addr = addr ? addr : 0x2A;
    dev->sda_io = sda;
    dev->scl_io = scl;
    dev->bus_hz = bus_hz ? bus_hz : 400000;
    dev->fullscale_volts = 1077.5f;

    i2c_config_t cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda,
        .scl_io_num = scl,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
        .clk_flags = 0,
#endif
    };
    cfg.master.clk_speed = dev->bus_hz;
    ESP_RETURN_ON_ERROR(i2c_param_config(port, &cfg), TAG, "param_config");
    esp_err_t e = i2c_driver_install(port, I2C_MODE_MASTER, 0, 0, 0);
    if (e == ESP_ERR_INVALID_STATE)
        e = ESP_OK; // Driver already installed, reuse it
    return e;
}

/**
 * \brief Deinitialize the I²C driver for the device's port.
 * \param dev Pointer to device context.
 * \return ESP_OK on success, error code otherwise.
 */
esp_err_t flyback_psu_deinit(const flyback_psu_t *dev)
{
    if (!dev)
        return ESP_ERR_INVALID_ARG;
    return i2c_driver_delete(dev->port);
}

/**
 * \brief Read bytes from the device starting at a given register.
 * \param dev Pointer to device context.
 * \param reg Starting register address.
 * \param dst Destination buffer to store read data.
 * \param len Number of bytes to read.
 * \return ESP_OK on success, error code otherwise.
 * \note Reads are chunked in 16-byte blocks due to AVR server limitations.
 */
esp_err_t flyback_psu_read(const flyback_psu_t *dev, uint8_t reg, uint8_t *dst, size_t len)
{
    ESP_RETURN_ON_FALSE(dev && dst && len, ESP_ERR_INVALID_ARG, TAG, "bad args");
    size_t off = 0;
    while (off < len)
    {
        size_t chunk = (len - off) > 16 ? 16 : (len - off); // Max 16 bytes per read
        esp_err_t e = i2c_master_write_read_device(
            dev->port, dev->addr,
            &reg, 1,
            dst + off, chunk,
            pdMS_TO_TICKS(FLYBACK_PSU_I2C_TIMEOUT_MS));
        if (e != ESP_OK)
            return e;
        off += chunk;
        reg = (uint8_t)(reg + chunk); // Increment register address for next chunk
    }
    return ESP_OK;
}

/**
 * \brief Write bytes to the device starting at a given register.
 * \param dev Pointer to device context.
 * \param reg Starting register address.
 * \param src Source buffer containing data to write.
 * \param len Number of bytes to write.
 * \return ESP_OK on success, error code otherwise.
 * \note Writes are chunked in 24-byte payloads to keep transactions short.
 *       Do not write to status registers (0x50..0x5F).
 */
esp_err_t flyback_psu_write(const flyback_psu_t *dev, uint8_t reg, const uint8_t *src, size_t len)
{
    ESP_RETURN_ON_FALSE(dev && src && len, ESP_ERR_INVALID_ARG, TAG, "bad args");
    size_t off = 0;
    uint8_t buf[1 + 24]; // 1 byte for reg + up to 24 bytes payload
    while (off < len)
    {
        size_t chunk = (len - off) > 24 ? 24 : (len - off);
        buf[0] = reg;
        memcpy(&buf[1], src + off, chunk);
        esp_err_t e = i2c_master_write_to_device(
            dev->port, dev->addr,
            buf, 1 + chunk,
            pdMS_TO_TICKS(FLYBACK_PSU_I2C_TIMEOUT_MS));
        if (e != ESP_OK)
            return e;
        off += chunk;
        reg = (uint8_t)(reg + chunk); // Increment register address for next chunk
    }
    return ESP_OK;
}

/**
 * \brief Read the 8-byte status block from the device.
 * \param dev Pointer to device context.
 * \param out Pointer to status structure to fill.
 * \return ESP_OK on success, error code otherwise.
 */
esp_err_t flyback_psu_read_status(const flyback_psu_t *dev, flyback_psu_status_t *out)
{
    ESP_RETURN_ON_FALSE(out, ESP_ERR_INVALID_ARG, TAG, "out null");
    uint8_t b[8] = {0};
    ESP_RETURN_ON_ERROR(flyback_psu_read(dev, FLYBACK_REG_STATUS, b, sizeof b), TAG, "read status");
    out->run_state = b[0];
    out->fault_code = b[1];
    out->fb_counts = (uint16_t)b[2] | ((uint16_t)b[3] << 8);
    out->dcounts = (int16_t)((uint16_t)b[4] | ((uint16_t)b[5] << 8));
    out->active_range = b[6] & 0x03;
    out->range_busy = b[7] ? 1 : 0;
    return ESP_OK;
}

/**
 * \brief Helper to write a single byte to a register.
 * \param dev Pointer to device context.
 * \param reg Register address.
 * \param v   Byte value to write.
 * \return ESP_OK on success.
 */
static inline esp_err_t write_u8(const flyback_psu_t *dev, uint8_t reg, uint8_t v)
{
    return flyback_psu_write(dev, reg, &v, 1);
}

/**
 * \brief Helper to write a 16-bit value in little-endian format to a register.
 * \param dev Pointer to device context.
 * \param reg Register address.
 * \param v   16-bit value to write.
 * \return ESP_OK on success.
 */
static inline esp_err_t write_u16le(const flyback_psu_t *dev, uint8_t reg, uint16_t v)
{
    uint8_t b[2] = {(uint8_t)(v & 0xFF), (uint8_t)(v >> 8)};
    return flyback_psu_write(dev, reg, b, 2);
}

/**
 * \brief Select the active range by writing to the range request register.
 * \param dev Pointer to device context.
 * \param range_code Range code to select (0..3).
 * \return ESP_OK on success.
 */
esp_err_t flyback_psu_select_range(const flyback_psu_t *dev, uint8_t range_code)
{
    return write_u8(dev, FLYBACK_REG_RANGE_REQ, (uint8_t)(range_code & 0x03));
}

/**
 * \brief Set the setpoint counts for a specific range in the setpoint table.
 * \param dev Pointer to device context.
 * \param range_idx Range index (0..3).
 * \param counts Setpoint counts (0..1023).
 * \return ESP_OK on success, error code otherwise.
 */
esp_err_t flyback_psu_set_table_counts(const flyback_psu_t *dev, uint8_t range_idx, uint16_t counts)
{
    if (range_idx > 3)
        return ESP_ERR_INVALID_ARG;
    uint8_t reg = (uint8_t)(FLYBACK_REG_SET0 + 2 * range_idx);
    return write_u16le(dev, reg, counts);
}

/**
 * \brief Set all four setpoint counts at once in the setpoint table.
 * \param dev Pointer to device context.
 * \param counts4 Array of 4 counts values [R0, R1, R2, R3].
 * \return ESP_OK on success, error code otherwise.
 */
esp_err_t flyback_psu_set_table_counts4(const flyback_psu_t *dev, const uint16_t counts4[4])
{
    ESP_RETURN_ON_FALSE(counts4, ESP_ERR_INVALID_ARG, TAG, "null counts4");
    // Pack counts into 8-byte buffer in little-endian order
    uint8_t buf[8];
    buf[0] = (uint8_t)(counts4[0] & 0xFF);
    buf[1] = (uint8_t)(counts4[0] >> 8);
    buf[2] = (uint8_t)(counts4[1] & 0xFF);
    buf[3] = (uint8_t)(counts4[1] >> 8);
    buf[4] = (uint8_t)(counts4[2] & 0xFF);
    buf[5] = (uint8_t)(counts4[2] >> 8);
    buf[6] = (uint8_t)(counts4[3] & 0xFF);
    buf[7] = (uint8_t)(counts4[3] >> 8);
    return flyback_psu_write(dev, FLYBACK_REG_SET0, buf, sizeof buf);
}

/**
 * \brief Send a command to the PSU by writing to the command register.
 * \param dev Pointer to device context.
 * \param cmd Command to send (e.g., SAVE_CONFIG, RESET_CTRL).
 * \return ESP_OK on success.
 */
esp_err_t flyback_psu_send_command(const flyback_psu_t *dev, flyback_psu_cmd_t cmd)
{
    return write_u8(dev, FLYBACK_REG_CMD, (uint8_t)cmd);
}

/**
 * \brief Poll the device until range switching is idle or timeout expires.
 * \param dev Pointer to device context.
 * \param timeout_ms Maximum time to wait in milliseconds.
 * \param poll_ms Polling interval in milliseconds (default 10 ms if 0).
 * \return ESP_OK if idle state observed before timeout; ESP_ERR_TIMEOUT otherwise.
 */
esp_err_t flyback_psu_wait_idle(const flyback_psu_t *dev, uint32_t timeout_ms, uint32_t poll_ms)
{
    const int64_t t0 = esp_timer_get_time(); // Start time in microseconds
    flyback_psu_status_t s;
    do
    {
        esp_err_t e = flyback_psu_read_status(dev, &s);
        if (e != ESP_OK)
            return e;
        if (!s.range_busy)
            return ESP_OK; // Range switching idle
        vTaskDelay(pdMS_TO_TICKS(poll_ms ? poll_ms : 10));
    } while ((esp_timer_get_time() - t0) < ((int64_t)timeout_ms * 1000));
    return ESP_ERR_TIMEOUT; // Timeout expired
}

/**
 * \brief Set the trip threshold counts for a specific range.
 * \param dev Pointer to device context.
 * \param idx Range index (0..3).
 * \param cnt Trip threshold counts.
 * \return ESP_OK on success, error code otherwise.
 */
esp_err_t flyback_psu_set_trip_counts(const flyback_psu_t *dev, uint8_t idx, uint16_t cnt)
{
    if (idx > 3)
        return ESP_ERR_INVALID_ARG;
    uint8_t reg = (uint8_t)(FLYBACK_REG_TRIP0 + 2 * idx);
    return write_u16le(dev, reg, cnt);
}

/**
 * \brief Set the clear threshold counts for a specific range.
 * \param dev Pointer to device context.
 * \param idx Range index (0..3).
 * \param cnt Clear threshold counts.
 * \return ESP_OK on success, error code otherwise.
 */
esp_err_t flyback_psu_set_clear_counts(const flyback_psu_t *dev, uint8_t idx, uint16_t cnt)
{
    if (idx > 3)
        return ESP_ERR_INVALID_ARG;
    uint8_t reg = (uint8_t)(FLYBACK_REG_CLEAR0 + 2 * idx);
    return write_u16le(dev, reg, cnt);
}

/**
 * \brief Set the voltage profile (set, trip, clear) for a specific range.
 * \param dev Pointer to device context.
 * \param idx Range index (0..3).
 * \param p Pointer to voltage profile structure.
 * \return ESP_OK on success, error code otherwise.
 */
esp_err_t flyback_psu_set_range_profile_v(const flyback_psu_t *dev, uint8_t idx,
                                          const flyback_range_profile_v_t *p)
{
    if (!p || idx > 3)
        return ESP_ERR_INVALID_ARG;
    uint16_t set = flyback_psu_volts_to_counts(dev, p->set_volts);
    uint16_t trip = flyback_psu_volts_to_counts(dev, p->trip_volts);
    uint16_t clear = flyback_psu_volts_to_counts(dev, p->clear_volts);
    esp_err_t e;
    e = flyback_psu_set_table_counts(dev, idx, set);
    if (e != ESP_OK)
        return e;
    e = flyback_psu_set_trip_counts(dev, idx, trip);
    if (e != ESP_OK)
        return e;
    e = flyback_psu_set_clear_counts(dev, idx, clear);
    if (e != ESP_OK)
        return e;
    return ESP_OK;
}

/**
 * \brief Set the source controlling the active range selection.
 * \param dev Pointer to device context.
 * \param src Source selection (HW, I2C, or AUTO).
 * \return ESP_OK on success.
 * \note Values outside valid range are clamped to maximum valid value.
 */
esp_err_t flyback_psu_set_range_source(const flyback_psu_t *dev, flyback_range_src_t src)
{
    uint8_t v = (uint8_t)src;
    if (v > (uint8_t)FLYBACK_RANGE_SRC_AUTO)
        v = (uint8_t)FLYBACK_RANGE_SRC_AUTO;
    return write_u8(dev, FLYBACK_REG_RANGE_SRC, v);
}

/**
 * \brief Read the current source controlling the active range selection.
 * \param dev Pointer to device context.
 * \param out Pointer to store the read range source.
 * \return ESP_OK on success, error code otherwise.
 * \note Values outside valid range are clamped to maximum valid value.
 */
esp_err_t flyback_psu_get_range_source(const flyback_psu_t *dev, flyback_range_src_t *out)
{
    if (!out)
        return ESP_ERR_INVALID_ARG;
    uint8_t v = 0;
    esp_err_t e = flyback_psu_read(dev, FLYBACK_REG_RANGE_SRC, &v, 1);
    if (e != ESP_OK)
        return e;
    if (v > (uint8_t)FLYBACK_RANGE_SRC_AUTO)
        v = (uint8_t)FLYBACK_RANGE_SRC_AUTO;
    *out = (flyback_range_src_t)v;
    return ESP_OK;
}
