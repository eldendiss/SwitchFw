#include "flyback_psu.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>

#ifndef FLYBACK_PSU_I2C_TIMEOUT_MS
#define FLYBACK_PSU_I2C_TIMEOUT_MS 100
#endif

static const char* TAG = "flyback_psu";

esp_err_t flyback_psu_init(flyback_psu_t* dev,
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
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5,0,0)
    .clk_flags = 0,
#endif
  };
  cfg.master.clk_speed = dev->bus_hz;
  ESP_RETURN_ON_ERROR(i2c_param_config(port, &cfg), TAG, "param_config");
  esp_err_t e = i2c_driver_install(port, I2C_MODE_MASTER, 0, 0, 0);
  if (e == ESP_ERR_INVALID_STATE) e = ESP_OK; // already installed
  return e;
}

esp_err_t flyback_psu_deinit(const flyback_psu_t* dev)
{
  if (!dev) return ESP_ERR_INVALID_ARG;
  return i2c_driver_delete(dev->port);
}

esp_err_t flyback_psu_read(const flyback_psu_t* dev, uint8_t reg, uint8_t* dst, size_t len)
{
  ESP_RETURN_ON_FALSE(dev && dst && len, ESP_ERR_INVALID_ARG, TAG, "bad args");
  size_t off = 0;
  while (off < len) {
    size_t chunk = (len - off) > 16 ? 16 : (len - off); // AVR emits <=16B per request
    esp_err_t e = i2c_master_write_read_device(
      dev->port, dev->addr,
      &reg, 1,
      dst + off, chunk,
      pdMS_TO_TICKS(FLYBACK_PSU_I2C_TIMEOUT_MS));
    if (e != ESP_OK) return e;
    off += chunk;
    reg = (uint8_t)(reg + chunk);
  }
  return ESP_OK;
}

esp_err_t flyback_psu_write(const flyback_psu_t* dev, uint8_t reg, const uint8_t* src, size_t len)
{
  ESP_RETURN_ON_FALSE(dev && src && len, ESP_ERR_INVALID_ARG, TAG, "bad args");
  // Chunk payload to keep transactions short (<= 24B payload each)
  size_t off = 0;
  uint8_t buf[1 + 24];
  while (off < len) {
    size_t chunk = (len - off) > 24 ? 24 : (len - off);
    buf[0] = reg;
    memcpy(&buf[1], src + off, chunk);
    esp_err_t e = i2c_master_write_to_device(
      dev->port, dev->addr,
      buf, 1 + chunk,
      pdMS_TO_TICKS(FLYBACK_PSU_I2C_TIMEOUT_MS));
    if (e != ESP_OK) return e;
    off += chunk;
    reg = (uint8_t)(reg + chunk);
  }
  return ESP_OK;
}

esp_err_t flyback_psu_read_status(const flyback_psu_t* dev, flyback_psu_status_t* out)
{
  ESP_RETURN_ON_FALSE(out, ESP_ERR_INVALID_ARG, TAG, "out null");
  uint8_t b[8] = {0};
  ESP_RETURN_ON_ERROR(flyback_psu_read(dev, FLYBACK_REG_STATUS, b, sizeof b), TAG, "read status");
  out->run_state    = b[0];
  out->fault_code   = b[1];
  out->fb_counts    = (uint16_t)b[2] | ((uint16_t)b[3] << 8);
  out->dcounts      = (int16_t)((uint16_t)b[4] | ((uint16_t)b[5] << 8));
  out->active_range = b[6] & 0x03;
  out->range_busy   = b[7] ? 1 : 0;
  return ESP_OK;
}

static inline esp_err_t write_u8(const flyback_psu_t* dev, uint8_t reg, uint8_t v) {
  return flyback_psu_write(dev, reg, &v, 1);
}
static inline esp_err_t write_u16le(const flyback_psu_t* dev, uint8_t reg, uint16_t v) {
  uint8_t b[2] = { (uint8_t)(v & 0xFF), (uint8_t)(v >> 8) };
  return flyback_psu_write(dev, reg, b, 2);
}

esp_err_t flyback_psu_select_range(const flyback_psu_t* dev, uint8_t range_code)
{
  return write_u8(dev, FLYBACK_REG_RANGE_REQ, (uint8_t)(range_code & 0x03));
}

esp_err_t flyback_psu_set_table_counts(const flyback_psu_t* dev, uint8_t range_idx, uint16_t counts)
{
  if (range_idx > 3) return ESP_ERR_INVALID_ARG;
  uint8_t reg = (uint8_t)(FLYBACK_REG_SET0 + 2 * range_idx);
  return write_u16le(dev, reg, counts);
}

esp_err_t flyback_psu_set_table_counts4(const flyback_psu_t* dev, const uint16_t counts4[4])
{
  ESP_RETURN_ON_FALSE(counts4, ESP_ERR_INVALID_ARG, TAG, "null counts4");
  // write sequentially starting at 0x00; 8 bytes total
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

esp_err_t flyback_psu_send_command(const flyback_psu_t* dev, flyback_psu_cmd_t cmd)
{
  return write_u8(dev, FLYBACK_REG_CMD, (uint8_t)cmd);
}

esp_err_t flyback_psu_wait_idle(const flyback_psu_t* dev, uint32_t timeout_ms, uint32_t poll_ms)
{
  const int64_t t0 = esp_timer_get_time(); // µs
  flyback_psu_status_t s;
  do {
    esp_err_t e = flyback_psu_read_status(dev, &s);
    if (e != ESP_OK) return e;
    if (!s.range_busy) return ESP_OK;
    vTaskDelay(pdMS_TO_TICKS(poll_ms ? poll_ms : 10));
  } while ((esp_timer_get_time() - t0) < ((int64_t)timeout_ms * 1000));
  return ESP_ERR_TIMEOUT;
}
