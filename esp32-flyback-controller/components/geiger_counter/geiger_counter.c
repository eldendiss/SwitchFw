#include "geiger_counter.h"
#include "esp_timer.h"
#include "esp_check.h"
#include "esp_log.h"
#include <string.h>

#include "driver/gpio.h"
#include "driver/pulse_cnt.h"   // ESP-IDF v5 PCNT driver

static const char* TAG = "geiger_pcnt";

typedef pcnt_unit_handle_t   pcnt_unit_t;
typedef pcnt_channel_handle_t pcnt_chan_t;

static void roll_seconds(geiger_counter_pcnt_t* dev, int64_t now_us) {
  while (now_us - dev->last_sec_us >= 1000000) {
    dev->sec_head = (dev->sec_head + 1) % 60;
    dev->sec_sum -= dev->sec_counts[dev->sec_head];
    dev->sec_counts[dev->sec_head] = dev->cur_sec_count;
    dev->sec_sum += dev->cur_sec_count;

    if (dev->filled_secs < 60) dev->filled_secs++;
    dev->cur_sec_count = 0;
    dev->last_sec_us += 1000000;
  }
}

esp_err_t geiger_counter_pcnt_start(geiger_counter_pcnt_t* dev,
                                    gpio_num_t gpio,
                                    bool active_high,
                                    uint32_t min_pulse_ns)
{
  ESP_RETURN_ON_FALSE(dev, ESP_ERR_INVALID_ARG, TAG, "dev null");
  memset(dev, 0, sizeof(*dev));
  dev->gpio = gpio;
  dev->active_high = active_high;
  dev->min_pulse_ns = (min_pulse_ns ? min_pulse_ns : 1000);

  // 1) Create PCNT unit
  pcnt_unit_config_t ucfg = {
    .high_limit = INT16_MAX,  // wrap range; we’ll read & clear often
    .low_limit  = INT16_MIN,
    .flags = { .accum_count = false } // we extend to 64-bit in software
  };
  pcnt_unit_t unit;
  ESP_RETURN_ON_ERROR(pcnt_new_unit(&ucfg, &unit), TAG, "new_unit");
  dev->unit = unit;

  // 2) Optional glitch filter (hardware)
  pcnt_glitch_filter_config_t gcfg = {
    .max_glitch_ns = dev->min_pulse_ns   // reject narrower pulses
  };
  ESP_RETURN_ON_ERROR(pcnt_unit_set_glitch_filter(unit, &gcfg), TAG, "glitch");

  // 3) Create channel on the pulse GPIO
  pcnt_chan_config_t ccfg = {
    .edge_gpio_num  = dev->gpio,  // pulse input
    .level_gpio_num = -1          // not used
  };
  pcnt_chan_t chan;
  ESP_RETURN_ON_ERROR(pcnt_new_channel(unit, &ccfg, &chan), TAG, "new_channel");
  dev->chan = chan;

  // 4) Count only the desired edge, ignore the other
  pcnt_channel_edge_action_t pos = dev->active_high
      ? PCNT_CHANNEL_EDGE_ACTION_INCREASE
      : PCNT_CHANNEL_EDGE_ACTION_HOLD;
  pcnt_channel_edge_action_t neg = dev->active_high
      ? PCNT_CHANNEL_EDGE_ACTION_HOLD
      : PCNT_CHANNEL_EDGE_ACTION_INCREASE;
  ESP_RETURN_ON_ERROR(pcnt_channel_set_edge_action(chan, pos, neg), TAG, "edge_act");

  // 5) Level action (don’t gate by level)
  ESP_RETURN_ON_ERROR(pcnt_channel_set_level_action(chan,
      PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_KEEP), TAG, "lvl_act");

  // 6) Enable & start unit
  ESP_RETURN_ON_ERROR(pcnt_unit_enable(unit), TAG, "enable");
  ESP_RETURN_ON_ERROR(pcnt_unit_clear_count(unit), TAG, "clear");
  ESP_RETURN_ON_ERROR(pcnt_unit_start(unit), TAG, "start");

  // 7) Init stats time bases
  int64_t now = esp_timer_get_time();
  dev->last_time_us = now;
  dev->last_sec_us = now;
  dev->sec_head = 59;
  dev->filled_secs = 0;
  dev->running = true;

  ESP_LOGI(TAG, "PCNT started on GPIO %d, edge=%s, glitch>=%u ns",
           (int)dev->gpio, dev->active_high ? "rising" : "falling",
           (unsigned)dev->min_pulse_ns);
  return ESP_OK;
}

esp_err_t geiger_counter_pcnt_stop(geiger_counter_pcnt_t* dev)
{
  if (!dev) return ESP_ERR_INVALID_ARG;
  dev->running = false;

  if (dev->unit) {
    pcnt_unit_t unit = (pcnt_unit_t)dev->unit;
    pcnt_unit_stop(unit);
    pcnt_unit_disable(unit);
    pcnt_del_unit(unit);
  }
  dev->unit = NULL;
  dev->chan = NULL;
  return ESP_OK;
}

// Read & extend 16-bit counter to 64-bit total; also update the rolling CPM window
static void sample_counter(geiger_counter_pcnt_t* dev)
{
  if (!dev || !dev->unit) return;
  int value = 0;
  pcnt_unit_t unit = (pcnt_unit_t)dev->unit;

  // Read and clear to get delta since last read
  pcnt_unit_get_count(unit, &value);
  pcnt_unit_clear_count(unit);

  if (value > 0) {
    dev->total += (uint32_t)value;
    dev->cur_sec_count += (uint32_t)value;
  }

  int64_t now = esp_timer_get_time();
  roll_seconds(dev, now);
}

uint64_t geiger_counter_pcnt_get_total(geiger_counter_pcnt_t* dev)
{
  if (!dev) return 0;
  sample_counter(dev);
  return dev->total;
}

float geiger_counter_pcnt_get_cps(geiger_counter_pcnt_t* dev)
{
  if (!dev) return 0.0f;
  int64_t now = esp_timer_get_time();
  sample_counter(dev);

  uint64_t dtc = dev->total - dev->last_total;
  int64_t dtu = now - dev->last_time_us;
  dev->last_total = dev->total;
  dev->last_time_us = now;

  if (dtu <= 0) return dev->last_cps;
  float cps = (float)dtc * (1000000.0f / (float)dtu);
  dev->last_cps = cps;
  return cps;
}

float geiger_counter_pcnt_get_cpm(geiger_counter_pcnt_t* dev)
{
  if (!dev) return 0.0f;
  sample_counter(dev);
  uint32_t sum = dev->sec_sum;
  uint32_t secs = dev->filled_secs ? dev->filled_secs : 1;
  return (float)sum * (60.0f / (float)secs);
}
