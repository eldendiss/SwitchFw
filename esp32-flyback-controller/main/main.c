#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "flyback_psu.h"
#include "esp_timer.h"
#include "geiger_counter.h"

static const char* TAG = "example";

#define TUBE_GPIO        GPIO_NUM_4   // GPIO pin connected to Geiger tube pulse output
#define TUBE_ACTIVE_HIGH true         // Set to false if pulses are active LOW

// Register base addresses for per-range overvoltage (OV) arrays (little-endian u16 x4)
#define REG_TRIP0   0x0E  // fb_ov_trip_cnt_tab[0] register base
#define REG_CLEAR0  0x16  // fb_ov_clear_cnt_tab[0] register base

/**
 * \brief Helper to write a 16-bit little-endian value into a byte buffer.
 * \param b Byte buffer pointer.
 * \param off Offset in bytes where to write the value.
 * \param v  16-bit value to write.
 */
static inline void put_u16le(uint8_t* b, int off, uint16_t v) {
  b[off + 0] = (uint8_t)(v & 0xFF);
  b[off + 1] = (uint8_t)(v >> 8);
}

void app_main(void)
{
  // I2C configuration: adjust pins and bus speed for your hardware
  const i2c_port_t port = I2C_NUM_0;
  const gpio_num_t SDA = 21;
  const gpio_num_t SCL = 22;

  // Initialize flyback PSU device context
  flyback_psu_t psu;
  // Use 10 kHz I2C speed for stability; increase to 400 kHz if your bus is stable
  ESP_ERROR_CHECK(flyback_psu_init(&psu, port, SDA, SCL, 10000, 0x2A));
  // Set full-scale voltage to match AVR firmware scaling (default 1077.5 V)
  flyback_psu_set_fullscale(&psu, 1077.5f);

  // Force I2C to control the active range immediately (overrides hardware pins)
  ESP_ERROR_CHECK(flyback_psu_set_range_source(&psu, FLYBACK_RANGE_SRC_I2C));

  // Define per-range voltage setpoints (in volts)
  const float vtab[4] = { 200.0f, 500.0f, 700.0f, 900.0f };
  uint16_t ctab[4]; // Corresponding counts for each voltage setpoint

  // Convert voltages to counts using device scaling
  for (int i = 0; i < 4; ++i) {
    ctab[i] = flyback_psu_volts_to_counts(&psu, vtab[i]);
  }
  // Program all four setpoints at once into the PSU
  ESP_ERROR_CHECK(flyback_psu_set_table_counts4(&psu, ctab));

  // Define per-range overvoltage thresholds:
  // Trip threshold = +10% above setpoint
  // Clear threshold = +5% above setpoint
  uint8_t trip8[8], clear8[8];
  for (int i = 0; i < 4; ++i) {
    float v_trip  = vtab[i] * 1.10f;
    float v_clear = vtab[i] * 1.05f;
    uint16_t c_trip  = flyback_psu_volts_to_counts(&psu, v_trip);
    uint16_t c_clear = flyback_psu_volts_to_counts(&psu, v_clear);
    put_u16le(trip8,  i * 2, c_trip);
    put_u16le(clear8, i * 2, c_clear);
  }
  // Write trip thresholds to registers 0x0E..0x14
  ESP_ERROR_CHECK(flyback_psu_write(&psu, REG_TRIP0,  trip8,  sizeof(trip8)));
  // Write clear thresholds to registers 0x16..0x1C
  ESP_ERROR_CHECK(flyback_psu_write(&psu, REG_CLEAR0, clear8, sizeof(clear8)));

  // Optionally persist configuration to EEPROM (uncomment if desired)
  // ESP_ERROR_CHECK(flyback_psu_send_command(&psu, FLYBACK_CMD_SAVE_CONFIG));

  // Initialize Geiger counter for pulse counting
  static geiger_counter_t gc;
  // Use 20 MHz RMT resolution (50 ns ticks), accept pulses >= 1000 ns (1 µs),
  // and allocate buffer for 2048 symbols
  ESP_ERROR_CHECK(geiger_counter_start(&gc, TUBE_GPIO, TUBE_ACTIVE_HIGH,
                                       20000000, 1000, 2048));

  // Main loop: cycle through the 4 ranges, holding each for 60 seconds
  // Print PSU status and Geiger counts every 1 second
  while (true) {
    for (int r = 0; r < 4; ++r) {
      ESP_LOGI(TAG, "Selecting range %d -> target voltage ~%.1f V (counts=%u)",
               r, vtab[r], (unsigned)ctab[r]);
      ESP_ERROR_CHECK(flyback_psu_select_range(&psu, (uint8_t)r));

      // Wait for relay switching to complete (timeout 3 seconds)
      esp_err_t idle = flyback_psu_wait_idle(&psu, 3000, 10);
      if (idle != ESP_OK) {
        ESP_LOGW(TAG, "Timeout waiting for range switch to complete");
      }

      int64_t t0 = esp_timer_get_time();
      while ((esp_timer_get_time() - t0) < (60LL * 1000 * 1000)) { // 60 seconds
        flyback_psu_status_t s;
        if (flyback_psu_read_status(&psu, &s) == ESP_OK) {
          float vfb = flyback_psu_counts_to_volts(&psu, s.fb_counts);
          ESP_LOGI(TAG,
                   "PSU status: state=%u fault=%u fb_counts=%u (~%.1f V) dcounts=%d active_range=%u busy=%u",
                   s.run_state, s.fault_code, s.fb_counts, vfb, (int)s.dcounts,
                   s.active_range, s.range_busy);
        } else {
          ESP_LOGW(TAG, "Failed to read PSU status");
        }

        // Read Geiger counter statistics
        float cps = geiger_counter_get_cps(&gc);
        float cpm = geiger_counter_get_cpm(&gc);
        uint64_t total = geiger_counter_get_total(&gc);
        ESP_LOGI("GEIGER",
                 "Counts: CPS=%.1f  CPM=%.1f  Total=%llu",
                 cps, cpm, (unsigned long long)total);

        // Delay 1 second before next status update
        vTaskDelay(pdMS_TO_TICKS(1000));
      }
    }
  }
}
