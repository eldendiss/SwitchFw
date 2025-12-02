#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  // config
  gpio_num_t gpio;          // pulse input GPIO
  bool active_high;         // true = rising edges are pulses, false = falling
  uint32_t min_pulse_ns;    // glitch filter threshold

  // pcnt handles (opaque in header)
  void* unit;               // pcnt_unit_handle_t
  void* chan;               // pcnt_channel_handle_t

  // stats
  volatile bool running;
  uint64_t total;           // 64-bit total (extends PCNT 16/24-bit)
  int64_t last_time_us;
  uint64_t last_total;
  float    last_cps;

  // CPM rolling window
  uint32_t sec_counts[60];
  uint32_t sec_sum;
  uint32_t filled_secs;
  uint32_t cur_sec_count;
  size_t   sec_head;
  int64_t  last_sec_us;

} geiger_counter_pcnt_t;

/**
 * Start PCNT-based Geiger counter.
 * @param dev zero-initialized struct
 * @param gpio input GPIO
 * @param active_high true: count rising, false: count falling
 * @param min_pulse_ns glitch filter (typ. 1000–3000 ns)
 */
esp_err_t geiger_counter_pcnt_start(geiger_counter_pcnt_t* dev,
                                    gpio_num_t gpio,
                                    bool active_high,
                                    uint32_t min_pulse_ns);

/** Stop and free resources */
esp_err_t geiger_counter_pcnt_stop(geiger_counter_pcnt_t* dev);

/** 64-bit total pulses */
uint64_t geiger_counter_pcnt_get_total(geiger_counter_pcnt_t* dev);

/** CPS since prior call (first call returns 0) */
float geiger_counter_pcnt_get_cps(geiger_counter_pcnt_t* dev);

/** CPM rolling 60s window (scaled until filled) */
float geiger_counter_pcnt_get_cpm(geiger_counter_pcnt_t* dev);

#ifdef __cplusplus
}
#endif
