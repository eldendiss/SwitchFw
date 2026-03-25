#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/portmacro.h"

#include "driver/pulse_cnt.h"   // ESP-IDF v5 PCNT driver

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  pcnt_unit_handle_t unit;
  pcnt_channel_handle_t chan;

  gpio_num_t gpio;
  bool active_high;
  uint32_t min_pulse_ns;

  // ownership / sync
  portMUX_TYPE mux;

  // active selection
  uint8_t active_range; // 0..3

  // per-range totals and rolling window
  uint64_t total[4];
  uint32_t sec_counts[4][60];
  uint32_t sec_sum[4];
  uint32_t cur_sec[4];
  uint8_t  sec_head;          // shared head index 0..59 (global time)
  uint8_t  filled_secs[4];    // per-range fill depth 0..60

  // per-range instantaneous CPS (optional)
  uint64_t last_total[4];
  int64_t  last_time_us[4];
  float    last_cps[4];

  // timebase
  int64_t last_sec_us;

  // conversion
  float conversion_factors[4];

  // sampler
  esp_timer_handle_t sampler;
  uint32_t sample_period_us;

  bool running;
} geiger_counter_pcnt4_t;

/**
 * Start PCNT-based Geiger counter.
 * @param dev zero-initialized struct
 * @param gpio input GPIO
 * @param active_high true: count rising, false: count falling
 * @param min_pulse_ns glitch filter (typ. 1000–3000 ns)
 */
esp_err_t geiger_counter_pcnt_start(geiger_counter_pcnt4_t* dev,
                                    gpio_num_t gpio,
                                    bool active_high,
                                    uint32_t min_pulse_ns);

/** Stop and free resources */
esp_err_t geiger_counter_pcnt_stop(geiger_counter_pcnt4_t* dev);

/** 64-bit total pulses */
uint64_t geiger_counter_pcnt_get_total(geiger_counter_pcnt4_t* dev, uint8_t range);

esp_err_t geiger_counter_pcnt_pause(geiger_counter_pcnt4_t* dev);

esp_err_t geiger_counter_pcnt_resume(geiger_counter_pcnt4_t* dev);

esp_err_t geiger_counter_pcnt_reset(geiger_counter_pcnt4_t* dev);

/** CPS since prior call (first call returns 0) */
float geiger_counter_pcnt_get_cps(geiger_counter_pcnt4_t* dev, uint8_t range);

/** CPM rolling 60s window (scaled until filled) */
float geiger_counter_pcnt_get_cpm(geiger_counter_pcnt4_t* dev, uint8_t range);

/** Convert CPM to µSv/h using given conversion factor */
float geiger_counter_pcnt_get_sieverts_per_hour(geiger_counter_pcnt4_t* dev, uint8_t range);

esp_err_t geiger_counter_set_active_range(geiger_counter_pcnt4_t* dev, uint8_t range);

esp_err_t geiger_counter_set_conversion_factor(geiger_counter_pcnt4_t* dev, uint8_t range, float factor);

esp_err_t geiger_counter_pcnt_get_cr_dr(geiger_counter_pcnt4_t *dev,
                                        uint8_t range,
                                        float *cr_out,
                                        float *dr_out);

#ifdef __cplusplus
}
#endif
