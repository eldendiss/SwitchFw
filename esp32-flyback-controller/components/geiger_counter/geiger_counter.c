#include "geiger_counter.h"
#include "esp_timer.h"
#include "esp_check.h"
#include "esp_log.h"
#include <string.h>

#include "driver/gpio.h"
#include "driver/pulse_cnt.h" // ESP-IDF v5 PCNT driver

#include "freertos/portmacro.h"

static const char *TAG = "geiger_pcnt";

typedef pcnt_unit_handle_t pcnt_unit_t;
typedef pcnt_channel_handle_t pcnt_chan_t;

static inline void roll_seconds_locked(geiger_counter_pcnt4_t *d, int64_t now_us)
{
  while (now_us - d->last_sec_us >= 1000000)
  {
    d->sec_head = (d->sec_head + 1) % 60;

    for (int r = 0; r < 4; r++)
    {
      d->sec_sum[r] -= d->sec_counts[r][d->sec_head];
      d->sec_counts[r][d->sec_head] = d->cur_sec[r];
      d->sec_sum[r] += d->cur_sec[r];
      d->cur_sec[r] = 0;

      if (d->filled_secs[r] < 60)
        d->filled_secs[r]++;
    }

    d->last_sec_us += 1000000;
  }
}

static void sampler_cb(void *arg)
{
  geiger_counter_pcnt4_t *d = (geiger_counter_pcnt4_t *)arg;

  int pcnt_val = 0;
  int64_t now = esp_timer_get_time();

  // Read+clear outside lock is OK if only this callback touches PCNT.
  // If you also pause/stop elsewhere, put these inside the lock.
  pcnt_unit_get_count(d->unit, &pcnt_val);
  pcnt_unit_clear_count(d->unit);

  portENTER_CRITICAL(&d->mux);

  roll_seconds_locked(d, now);

  if (pcnt_val > 0)
  {
    uint8_t ar = d->active_range;
    d->total[ar] += (uint32_t)pcnt_val;
    d->cur_sec[ar] += (uint32_t)pcnt_val;
  }

  portEXIT_CRITICAL(&d->mux);
}

static inline void init_timebases_locked(geiger_counter_pcnt4_t *dev, int64_t now_us)
{
  dev->last_sec_us = now_us;
  dev->sec_head = 59;

  for (int r = 0; r < 4; r++)
  {
    dev->total[r] = 0;
    dev->sec_sum[r] = 0;
    dev->cur_sec[r] = 0;
    dev->filled_secs[r] = 0;
    dev->last_total[r] = 0;
    dev->last_time_us[r] = now_us;
    dev->last_cps[r] = 0.0f;

    memset(dev->sec_counts[r], 0, sizeof(dev->sec_counts[r]));
  }
}

esp_err_t geiger_counter_pcnt_start(geiger_counter_pcnt4_t *dev,
                                    gpio_num_t gpio,
                                    bool active_high,
                                    uint32_t min_pulse_ns)
{
  ESP_RETURN_ON_FALSE(dev, ESP_ERR_INVALID_ARG, TAG, "dev null");

  if (dev->running)
  {
    ESP_LOGW(TAG, "Device already running; stop it first");
    return ESP_ERR_INVALID_STATE;
  }

  memset(dev, 0, sizeof(*dev));

  dev->gpio = gpio;
  dev->active_high = active_high;
  dev->min_pulse_ns = (min_pulse_ns ? min_pulse_ns : 1000);

  // Sampler period: choose something that prevents 16-bit saturation.
  // 100 ms is usually safe; if you expect high pulse rates/noise, reduce to 10 ms.
  dev->sample_period_us = 100000;

  dev->active_range = 0;

  // Default conversion factors
  for (int r = 0; r < 4; r++)
  {
    dev->conversion_factors[r] = 1.0f;
  }

  // Initialize lock
  dev->mux = (portMUX_TYPE)portMUX_INITIALIZER_UNLOCKED;

  // 1) Create PCNT unit
  pcnt_unit_config_t ucfg = {
      .high_limit = INT16_MAX,
      .low_limit = INT16_MIN, // we only count up; keep it simple
      .flags = {
          .accum_count = false // we extend in software by periodic sampling
      }};

  ESP_RETURN_ON_ERROR(pcnt_new_unit(&ucfg, &dev->unit), TAG, "pcnt_new_unit");

  // 2) Glitch filter
  pcnt_glitch_filter_config_t gcfg = {
      .max_glitch_ns = dev->min_pulse_ns};
  ESP_RETURN_ON_ERROR(pcnt_unit_set_glitch_filter(dev->unit, &gcfg), TAG, "pcnt_glitch");

  // 3) Create PCNT channel
  pcnt_chan_config_t ccfg = {
      .edge_gpio_num = (int)dev->gpio,
      .level_gpio_num = -1};
  ESP_RETURN_ON_ERROR(pcnt_new_channel(dev->unit, &ccfg, &dev->chan), TAG, "pcnt_new_channel");

  // 4) Count only one edge depending on polarity
  pcnt_channel_edge_action_t pos = active_high
                                       ? PCNT_CHANNEL_EDGE_ACTION_INCREASE
                                       : PCNT_CHANNEL_EDGE_ACTION_HOLD;
  pcnt_channel_edge_action_t neg = active_high
                                       ? PCNT_CHANNEL_EDGE_ACTION_HOLD
                                       : PCNT_CHANNEL_EDGE_ACTION_INCREASE;

  ESP_RETURN_ON_ERROR(pcnt_channel_set_edge_action(dev->chan, pos, neg), TAG, "pcnt_edge_action");

  // 5) Level action: do not gate counting
  ESP_RETURN_ON_ERROR(
      pcnt_channel_set_level_action(dev->chan,
                                    PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                    PCNT_CHANNEL_LEVEL_ACTION_KEEP),
      TAG, "pcnt_level_action");

  // 6) Enable & start PCNT
  ESP_RETURN_ON_ERROR(pcnt_unit_enable(dev->unit), TAG, "pcnt_enable");
  ESP_RETURN_ON_ERROR(pcnt_unit_clear_count(dev->unit), TAG, "pcnt_clear");
  ESP_RETURN_ON_ERROR(pcnt_unit_start(dev->unit), TAG, "pcnt_start");

  // 7) Initialize statistics timebases
  int64_t now = esp_timer_get_time();
  portENTER_CRITICAL(&dev->mux);
  init_timebases_locked(dev, now);
  portEXIT_CRITICAL(&dev->mux);

  // 8) Create periodic sampler timer (attributes PCNT deltas to active_range)
  esp_timer_create_args_t tcfg = {
      .callback = &sampler_cb,
      .arg = dev,
      .dispatch_method = ESP_TIMER_TASK, // runs in esp_timer task context
      .name = "geiger_pcnt4"};
  ESP_RETURN_ON_ERROR(esp_timer_create(&tcfg, &dev->sampler), TAG, "timer_create");
  ESP_RETURN_ON_ERROR(esp_timer_start_periodic(dev->sampler, dev->sample_period_us),
                      TAG, "timer_start");

  dev->running = true;

  ESP_LOGI(TAG, "PCNT4 started: GPIO=%d edge=%s glitch>=%u ns sample=%u us",
           (int)dev->gpio,
           dev->active_high ? "rising" : "falling",
           (unsigned)dev->min_pulse_ns,
           (unsigned)dev->sample_period_us);

  return ESP_OK;
}

esp_err_t geiger_counter_pcnt_stop(geiger_counter_pcnt4_t *dev)
{
  ESP_RETURN_ON_FALSE(dev, ESP_ERR_INVALID_ARG, TAG, "dev null");

  // Mark not running early to stop sampler activity quickly
  dev->running = false;

  // 1) Stop and delete periodic sampler timer (if created)
  if (dev->sampler)
  {
    // Ignore "not started" errors; we only care that it is not running afterwards
    esp_err_t e;

    e = esp_timer_stop(dev->sampler);
    if (e != ESP_OK && e != ESP_ERR_INVALID_STATE)
    {
      ESP_LOGW(TAG, "esp_timer_stop: %s", esp_err_to_name(e));
    }

    e = esp_timer_delete(dev->sampler);
    if (e != ESP_OK)
    {
      ESP_LOGW(TAG, "esp_timer_delete: %s", esp_err_to_name(e));
      // Continue anyway; we'll still try to release PCNT resources.
    }
    dev->sampler = NULL;
  }

  // 2) Stop/disable PCNT unit
  if (dev->unit)
  {
    esp_err_t e;

    e = pcnt_unit_stop(dev->unit);
    if (e != ESP_OK)
    {
      ESP_LOGW(TAG, "pcnt_unit_stop: %s", esp_err_to_name(e));
    }

    e = pcnt_unit_disable(dev->unit);
    if (e != ESP_OK)
    {
      ESP_LOGW(TAG, "pcnt_unit_disable: %s", esp_err_to_name(e));
    }

    // 3) Delete channel first then unit
    if (dev->chan)
    {
      e = pcnt_del_channel(dev->chan);
      if (e != ESP_OK)
      {
        ESP_LOGW(TAG, "pcnt_del_channel: %s", esp_err_to_name(e));
      }
      dev->chan = NULL;
    }

    e = pcnt_del_unit(dev->unit);
    if (e != ESP_OK)
    {
      ESP_LOGW(TAG, "pcnt_del_unit: %s", esp_err_to_name(e));
      // If this fails, caller may retry stop or leak resources; report failure.
      dev->unit = NULL;
      return e;
    }

    dev->unit = NULL;
  }

  // 4) Clear state (avoid stale reads)
  portENTER_CRITICAL(&dev->mux);
  dev->active_range = 0;
  for (int r = 0; r < 4; r++)
  {
    dev->total[r] = 0;
    dev->sec_sum[r] = 0;
    dev->cur_sec[r] = 0;
    dev->filled_secs[r] = 0;
    dev->last_total[r] = 0;
    dev->last_time_us[r] = 0;
    dev->last_cps[r] = 0.0f;
    memset(dev->sec_counts[r], 0, sizeof(dev->sec_counts[r]));
    dev->conversion_factors[r] = 1.0f;
  }
  dev->sec_head = 59;
  dev->last_sec_us = 0;
  dev->sample_period_us = 0;
  portEXIT_CRITICAL(&dev->mux);

  return ESP_OK;
}

esp_err_t geiger_counter_pcnt_pause(geiger_counter_pcnt4_t *dev)
{
  ESP_RETURN_ON_FALSE(dev, ESP_ERR_INVALID_ARG, TAG, "dev null");
  ESP_RETURN_ON_FALSE(dev->unit, ESP_ERR_INVALID_STATE, TAG, "pcnt not started");

  // If already paused, treat as success
  if (!dev->running)
  {
    return ESP_OK;
  }

  // 1) Stop sampler timer first so it cannot touch PCNT while we stop the unit
  if (dev->sampler)
  {
    esp_err_t e = esp_timer_stop(dev->sampler);
    if (e != ESP_OK && e != ESP_ERR_INVALID_STATE)
    {
      ESP_LOGW(TAG, "esp_timer_stop: %s", esp_err_to_name(e));
      // continue; we still attempt to stop PCNT
    }
  }

  // 2) Stop PCNT unit
  esp_err_t e = pcnt_unit_stop(dev->unit);
  if (e != ESP_OK)
  {
    ESP_LOGW(TAG, "pcnt_unit_stop: %s", esp_err_to_name(e));
    // Keep going; we can still mark paused.
  }

  // 3) Clear the instantaneous counter so resume starts clean
  // (We do not touch per-range totals/windows here.)
  (void)pcnt_unit_clear_count(dev->unit);

  // 4) Mark paused
  portENTER_CRITICAL(&dev->mux);
  dev->running = false;
  portEXIT_CRITICAL(&dev->mux);

  return ESP_OK;
}

esp_err_t geiger_counter_pcnt_resume(geiger_counter_pcnt4_t *dev)
{
  ESP_RETURN_ON_FALSE(dev, ESP_ERR_INVALID_ARG, TAG, "dev null");
  ESP_RETURN_ON_FALSE(dev->unit, ESP_ERR_INVALID_STATE, TAG, "pcnt not started");
  ESP_RETURN_ON_FALSE(dev->sampler, ESP_ERR_INVALID_STATE, TAG, "sampler not created");

  // If already running, treat as success
  if (dev->running)
  {
    return ESP_OK;
  }

  // 1) Clear instantaneous PCNT count to avoid carrying stale pulses into the first sample
  esp_err_t e = pcnt_unit_clear_count(dev->unit);
  if (e != ESP_OK)
  {
    ESP_LOGW(TAG, "pcnt_unit_clear_count: %s", esp_err_to_name(e));
  }

  // 2) Re-base the rolling time window to "now" so we don't roll a huge gap at once.
  // This prevents injecting a large number of zero-seconds if you were paused for minutes/hours.
  int64_t now = esp_timer_get_time();
  portENTER_CRITICAL(&dev->mux);
  dev->last_sec_us = now;
  dev->sec_head = 59;

  // Reset per-range fill depth so CPM ramps in again after resume.
  for (int r = 0; r < 4; r++)
  {
    dev->filled_secs[r] = 0;
    dev->sec_sum[r] = 0;
    dev->cur_sec[r] = 0;
    memset(dev->sec_counts[r], 0, sizeof(dev->sec_counts[r]));

    // Re-base CPS time for each range so CPS doesn't spike after long pause
    dev->last_total[r] = dev->total[r];
    dev->last_time_us[r] = now;
    dev->last_cps[r] = 0.0f;
  }
  dev->running = true;
  portEXIT_CRITICAL(&dev->mux);

  // 3) Start PCNT unit
  e = pcnt_unit_start(dev->unit);
  if (e != ESP_OK)
  {
    ESP_LOGE(TAG, "pcnt_unit_start: %s", esp_err_to_name(e));
    // Roll back running state
    portENTER_CRITICAL(&dev->mux);
    dev->running = false;
    portEXIT_CRITICAL(&dev->mux);
    return e;
  }

  // 4) Restart periodic sampler
  e = esp_timer_start_periodic(dev->sampler, dev->sample_period_us);
  if (e != ESP_OK && e != ESP_ERR_INVALID_STATE)
  {
    ESP_LOGE(TAG, "esp_timer_start_periodic: %s", esp_err_to_name(e));
    // If sampler can't start, stop PCNT so we don't run "blind"
    (void)pcnt_unit_stop(dev->unit);
    portENTER_CRITICAL(&dev->mux);
    dev->running = false;
    portEXIT_CRITICAL(&dev->mux);
    return e;
  }

  return ESP_OK;
}

esp_err_t geiger_counter_pcnt_reset(geiger_counter_pcnt4_t *dev)
{
  ESP_RETURN_ON_FALSE(dev, ESP_ERR_INVALID_ARG, TAG, "dev null");
  ESP_RETURN_ON_FALSE(dev->unit, ESP_ERR_INVALID_STATE, TAG, "pcnt not started");

  // 1) Clear the hardware counter so we don't carry pending pulses into new epoch
  esp_err_t e = pcnt_unit_clear_count(dev->unit);
  ESP_RETURN_ON_ERROR(e, TAG, "pcnt_clear_count");

  // 2) Reset software state (keep configuration like conversion_factors and active_range)
  int64_t now = esp_timer_get_time();

  portENTER_CRITICAL(&dev->mux);

  // Per-range totals and rolling windows
  for (int r = 0; r < 4; r++)
  {
    dev->total[r] = 0;

    dev->sec_sum[r] = 0;
    dev->cur_sec[r] = 0;
    dev->filled_secs[r] = 0;
    memset(dev->sec_counts[r], 0, sizeof(dev->sec_counts[r]));

    // CPS timebases
    dev->last_total[r] = 0;
    dev->last_time_us[r] = now;
    dev->last_cps[r] = 0.0f;
  }

  // Global timebase for the rolling second bins
  dev->last_sec_us = now;
  dev->sec_head = 59;

  portEXIT_CRITICAL(&dev->mux);

  return ESP_OK;
}

uint64_t geiger_counter_pcnt_get_total(geiger_counter_pcnt4_t *dev, uint8_t range)
{
  if (!dev || range >= 4)
    return 0;

  uint64_t total;

  portENTER_CRITICAL(&dev->mux);
  total = dev->total[range];
  portEXIT_CRITICAL(&dev->mux);

  return total;
}

float geiger_counter_pcnt_get_cps(geiger_counter_pcnt4_t *dev, uint8_t range)
{
  if (!dev || range >= 4)
    return 0.0f;

  int64_t now = esp_timer_get_time();
  float cps = 0.0f;

  portENTER_CRITICAL(&dev->mux);

  // If the range is not currently active, CPS is defined as 0 (no pulses can arrive).
  // This avoids misleading "stale CPS" from the last time the tube was active.
  if (range != dev->active_range || !dev->running)
  {
    dev->last_cps[range] = 0.0f;
    dev->last_total[range] = dev->total[range];
    dev->last_time_us[range] = now;
    portEXIT_CRITICAL(&dev->mux);
    return 0.0f;
  }

  uint64_t total_now = dev->total[range];
  uint64_t total_prev = dev->last_total[range];
  int64_t time_prev = dev->last_time_us[range];

  int64_t dt_us = now - time_prev;
  uint64_t dcnt = total_now - total_prev;

  if (dt_us > 0)
  {
    cps = (float)dcnt * (1000000.0f / (float)dt_us);
    dev->last_cps[range] = cps;
  }
  else
  {
    cps = dev->last_cps[range];
  }

  dev->last_total[range] = total_now;
  dev->last_time_us[range] = now;

  portEXIT_CRITICAL(&dev->mux);
  return cps;
}

float geiger_counter_pcnt_get_cpm(geiger_counter_pcnt4_t *dev, uint8_t range)
{
  if (!dev || range >= 4)
    return 0.0f;

  float cpm;

  portENTER_CRITICAL(&dev->mux);

  uint8_t secs = dev->filled_secs[range];
  if (secs == 0)
  {
    cpm = 0.0f;
  }
  else
  {
    cpm = (float)dev->sec_sum[range] * (60.0f / (float)secs);
  }

  portEXIT_CRITICAL(&dev->mux);
  return cpm;
}

float geiger_counter_pcnt_get_sieverts_per_hour(geiger_counter_pcnt4_t *dev, uint8_t range)
{
  if (!dev || range >= 4)
    return 0.0f;

  float usvh;

  portENTER_CRITICAL(&dev->mux);

  uint8_t secs = dev->filled_secs[range];
  if (secs == 0)
  {
    usvh = 0.0f;
  }
  else
  {
    float cpm = (float)dev->sec_sum[range] * (60.0f / (float)secs);
    usvh = cpm * dev->conversion_factors[range];
  }

  portEXIT_CRITICAL(&dev->mux);
  return usvh;
}

esp_err_t geiger_counter_pcnt_get_cr_dr(geiger_counter_pcnt4_t *dev,
                                        uint8_t range,
                                        float *cr_out,
                                        float *dr_out)
{
    if (!dev || range >= 4 || !cr_out || !dr_out)
        return ESP_ERR_INVALID_ARG;

    int64_t now = esp_timer_get_time();
    float cr = 0.0f;
    float dr = 0.0f;

    portENTER_CRITICAL(&dev->mux);

    uint64_t total_now = dev->total[range];
    uint64_t total_prev = dev->last_total[range];
    int64_t time_prev = dev->last_time_us[range];
    float factor = dev->conversion_factors[range];

    int64_t dt_us = now - time_prev;
    uint64_t dcnt = (total_now >= total_prev) ? (total_now - total_prev) : total_now;

    if (dt_us > 0)
    {
        float dt_s = (float)dt_us / 1000000.0f;
        cr = ((float)dcnt / dt_s) * 60.0f;
        dr = cr * factor;
    }

    dev->last_total[range] = total_now;
    dev->last_time_us[range] = now;

    portEXIT_CRITICAL(&dev->mux);

    *cr_out = cr;
    *dr_out = dr;

    return ESP_OK;
}

esp_err_t geiger_counter_set_active_range(geiger_counter_pcnt4_t *dev, uint8_t range)
{
  ESP_RETURN_ON_FALSE(dev, ESP_ERR_INVALID_ARG, "geiger", "dev null");
  ESP_RETURN_ON_FALSE(range < 4, ESP_ERR_INVALID_ARG, "geiger", "range out of bounds");
  ESP_RETURN_ON_FALSE(dev->unit, ESP_ERR_INVALID_STATE, "geiger", "pcnt not started");

  // If no change, nothing to do
  if (range == dev->active_range)
  {
    return ESP_OK;
  }

  // 1) Finalize any pending PCNT counts to the OLD range
  int pcnt_val = 0;
  pcnt_unit_get_count(dev->unit, &pcnt_val);
  pcnt_unit_clear_count(dev->unit);

  int64_t now = esp_timer_get_time();

  portENTER_CRITICAL(&dev->mux);

  // Roll timebase so counts are attributed to the correct second buckets
  while (now - dev->last_sec_us >= 1000000)
  {
    dev->sec_head = (dev->sec_head + 1) % 60;

    for (int r = 0; r < 4; r++)
    {
      dev->sec_sum[r] -= dev->sec_counts[r][dev->sec_head];
      dev->sec_counts[r][dev->sec_head] = dev->cur_sec[r];
      dev->sec_sum[r] += dev->cur_sec[r];
      dev->cur_sec[r] = 0;

      if (dev->filled_secs[r] < 60)
        dev->filled_secs[r]++;
    }

    dev->last_sec_us += 1000000;
  }

  if (pcnt_val > 0)
  {
    uint8_t old = dev->active_range;
    dev->total[old] += (uint32_t)pcnt_val;
    dev->cur_sec[old] += (uint32_t)pcnt_val;
  }

  // 2) Switch active range
  dev->active_range = range;

  // 3) Re-base CPS state for the new range (prevents CPS spikes)
  dev->last_total[range] = dev->total[range];
  dev->last_time_us[range] = now;
  dev->last_cps[range] = 0.0f;

  portEXIT_CRITICAL(&dev->mux);

  return ESP_OK;
}

esp_err_t geiger_counter_set_conversion_factor(geiger_counter_pcnt4_t *dev, uint8_t range, float factor)
{
  if (range > 3 || factor < 0)
  {
    ESP_LOGE(TAG, "Invalid range or factor");
    return ESP_ERR_INVALID_ARG;
  }

  dev->conversion_factors[range] = factor;
  return ESP_OK;
}