#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "flyback_psu.h"
#include "esp_timer.h"

static const char* TAG = "example";

void app_main(void)
{
    // Adjust pins for your board if needed
  const i2c_port_t port = I2C_NUM_0;
  const gpio_num_t SDA = 21;
  const gpio_num_t SCL = 22;

  flyback_psu_t psu;
  ESP_ERROR_CHECK(flyback_psu_init(&psu, port, SDA, SCL, 10000, 0x2A));
  flyback_psu_set_fullscale(&psu, 1077.5f); // matches AVR scaling

  // Program per-range table: 200 / 500 / 700 / 900 V
  const float   vtab[4] = {200.0f, 500.0f, 700.0f, 900.0f};
  uint16_t ctab[4];
  for (int i = 0; i < 4; ++i) ctab[i] = flyback_psu_volts_to_counts(&psu, vtab[i]);
  ESP_ERROR_CHECK(flyback_psu_set_table_counts4(&psu, ctab));

  // Optionally persist config to EEPROM (host-initiated)
  ESP_ERROR_CHECK(flyback_psu_send_command(&psu, FLYBACK_CMD_SAVE_CONFIG));

  // Cycle 4 ranges, 60 s each; print status every 1 s
  while (true) {
    for (int r = 0; r < 4; ++r) {
      ESP_LOGI(TAG, "Select range %d -> target ~%.1f V (counts=%u)", r, vtab[r], (unsigned)ctab[r]);
      ESP_ERROR_CHECK(flyback_psu_select_range(&psu, (uint8_t)r));
      // Wait until relay switching is complete
      esp_err_t idle = flyback_psu_wait_idle(&psu, 3000, 10);
      if (idle != ESP_OK) ESP_LOGW(TAG, "range switch wait timeout");

      int64_t t0 = esp_timer_get_time();
      while ((esp_timer_get_time() - t0) < (60LL * 1000 * 1000)) {
        flyback_psu_status_t s;
        if (flyback_psu_read_status(&psu, &s) == ESP_OK) {
          float vfb = flyback_psu_counts_to_volts(&psu, s.fb_counts);
          ESP_LOGI(TAG, "state=%u fault=%u fb=%u (~%.1f V) d=%d r=%u busy=%u",
                   s.run_state, s.fault_code, s.fb_counts, vfb, (int)s.dcounts,
                   s.active_range, s.range_busy);
        } else {
          ESP_LOGW(TAG, "read_status failed");
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
      }
    }
  }
}