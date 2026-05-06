#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "flyback_psu.h"
#include "esp_timer.h"
#include "geiger_counter.h"
#include "driver/gpio.h"
#include "wifi.h"
#include "storage.h"
#include "iot_is.h"
#include "job_manager.h"
#include "ota.h"
#include "sntp.h"
#include "flyback.h"
#include "commands/commands.h"
#include "ble.h"
#include "provisioning.h"
#include "hv_mon.h"
#include "LAN.h"
#include "esp_mac.h"

static const char *TAG = "main";

// Persists across soft resets; cleared on power cycle.
// Used to detect crash loops and break rollback purgatory.
static RTC_DATA_ATTR uint32_t s_consecutive_crashes = 0;

geiger_counter_pcnt4_t gc;
device_config_data_t devCfg;

static void avg_u(void *p)
{
  const TickType_t period = pdMS_TO_TICKS(500); // 2 Hz sampling
  TickType_t last = xTaskGetTickCount();

  while (true)
  {
    vTaskDelayUntil(&last, period);

    flyback_psu_status_t s;
    if (flyback_get_status(&s) == ESP_OK)
    {
      float vfb = flyback_counts_to_volts(s.fb_counts);
      hv_avg_add(vfb);

      // optional: only log anomalies
      if (s.run_state != 0 || s.fault_code != 0)
      {
        ESP_LOGW(TAG, "PSU state=%u fault=%u V=%.1f busy=%u",
                 s.run_state, s.fault_code, vfb, s.range_busy);
      }
    }
    else
    {
      ESP_LOGW(TAG, "Failed to read PSU status");
    }
  }
}

extern "C" void app_main(void)
{
  gpio_config_t io_conf = {};
  io_conf.pin_bit_mask = ((1ULL << CONFIG_RASENS_ENABLE_PIN) | (1ULL << CONFIG_RASENS_SLEEP_PIN));
  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.intr_type = GPIO_INTR_DISABLE;
  io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  gpio_config(&io_conf);
  // disable flyback by default on boot for safety
  gpio_set_level((gpio_num_t)CONFIG_RASENS_SLEEP_PIN, 0);
  gpio_set_level((gpio_num_t)CONFIG_RASENS_ENABLE_PIN, 0);

  ESP_LOGI(TAG, "Boot...");
  // initialize storage
  storage_init();

  // Crash-loop detection: count consecutive panics/WDT resets via RTC memory.
  // RTC memory survives software resets but clears on power loss.
  esp_reset_reason_t reset_reason = esp_reset_reason();
  if (reset_reason == ESP_RST_PANIC  || reset_reason == ESP_RST_INT_WDT ||
      reset_reason == ESP_RST_TASK_WDT || reset_reason == ESP_RST_WDT)
  {
    s_consecutive_crashes++;
    ESP_LOGE(TAG, "Recovered from crash (reason=%d). Consecutive crashes: %lu",
             (int)reset_reason, (unsigned long)s_consecutive_crashes);
  }
  else
  {
    s_consecutive_crashes = 0;
  }

  // Escape hatch: after 5 consecutive crashes mark firmware valid regardless
  // of what fails below. This prevents rollback purgatory if both firmware
  // images have the same hardware issue.
  if (s_consecutive_crashes >= 5)
  {
    ESP_LOGW(TAG, "Crash loop detected (%lu), marking firmware valid early",
             (unsigned long)s_consecutive_crashes);
    mark_app_valid_cancel_rollback();
  }

  // NOTE: mark_app_valid_cancel_rollback() is called again after all hardware
  // init succeeds. That is the normal path. The call above is only the escape hatch.

  register_got_ip_callback(eth_connected_override);
  register_down_callback(eth_disconnected_override);

  ble_init();
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_BT);
  LAN_init();

  // initialize wifi
  wifi_init();

  provisioning_manager_init();

  // try to get provisioning data
  provisioning_data_t cfg = {0};
  bool have_nvs = (storage_load_provisioning_data(&cfg) == ESP_OK) && validate_prov(&cfg); // if validate_prov visible, copy same logic here
  if (!have_nvs)
  {
    memset(&cfg, 0, sizeof(cfg));
    strncpy(cfg.ssid, CONFIG_RASENS_DEFAULT_SSID, sizeof(cfg.ssid) - 1);
    strncpy(cfg.password, CONFIG_RASENS_DEFAULT_WIFI_PASSWORD, sizeof(cfg.password) - 1);
    strncpy(cfg.mqtt_host, CONFIG_RASENS_MQTT_HOST, sizeof(cfg.mqtt_host) - 1);
    cfg.mqtt_port = CONFIG_RASENS_MQTT_PORT;
    strncpy(cfg.access_token, CONFIG_RASENS_ACCESS_TOKEN, sizeof(cfg.access_token) - 1);
  }

  // Set supervisor active BEFORE starting supervisor task
  connection_supervisor_set_active(&cfg, have_nvs);

  // Start supervisor: it will retry indefinitely
  ESP_ERROR_CHECK(connection_supervisor_init());

  // try to load device configuration data

  if (storage_load_device_config_data(&devCfg) != ESP_OK)
  {
    ESP_LOGW(TAG, "No device configuration data found in storage, using defaults");
    devCfg.interval = 60; // 60 seconds
    devCfg.active_range = 0;
    devCfg.set_voltage[0] = 200; // R1
    devCfg.set_voltage[1] = 250; // R2
    devCfg.set_voltage[2] = 300; // R3
    devCfg.set_voltage[3] = 350; // R4
    devCfg.coeff[0] = 93;        // R1
    devCfg.coeff[1] = 62;        // R2
    devCfg.coeff[2] = 1111;      // R3
    devCfg.coeff[3] = 11111;     // R4

    // store defaults back to storage
    storage_save_device_config_data(&devCfg);
  }
  else
  {
    ESP_LOGI(TAG, "Loaded device configuration data: interval=%u s, active_range = %u", devCfg.interval, devCfg.active_range);
  }

  // Sanitize loaded config — corrupt/zero values can cause infinite tight loops
  // or invalid hardware state in the field.
  if (devCfg.interval == 0 || devCfg.interval > 3600)
  {
    ESP_LOGW(TAG, "Interval %u out of range, clamping to 60s", devCfg.interval);
    devCfg.interval = 60;
  }
  if (devCfg.active_range > 3)
  {
    ESP_LOGW(TAG, "Invalid active_range %u, resetting to 0", devCfg.active_range);
    devCfg.active_range = 0;
  }

  // wait for provisioning to complete
  ESP_LOGI(TAG, "Waiting for network (Wi-Fi or Ethernet)...");
  esp_err_t net_ready = wifi_wait_connected(pdMS_TO_TICKS(60000));

  if (net_ready == ESP_OK)
  {
    ESP_LOGI(TAG, "Network ready, starting online services");

    init_sntp(CONFIG_RASENS_SNTP_SYNC_INTERVAL_MS);

    updateFirmwareVersion(CONFIG_RASENS_HTTP_BACKEND_URL, cfg.access_token);
    perform_ota_update(CONFIG_RASENS_HTTP_BACKEND_URL, cfg.access_token);
  }
  else
  {
    ESP_LOGW(TAG, "No network within timeout, continuing in offline mode");
  }

  job_manager.init();
  job_manager.register_command("setTube", setTube_command);
  job_manager.register_command("setInterval", setIntervalCommand);
  job_manager.register_command("setVoltage_r1", setVoltage_r1_command);
  job_manager.register_command("setVoltage_r2", setVoltage_r2_command);
  job_manager.register_command("setVoltage_r3", setVoltage_r3_command);
  job_manager.register_command("setVoltage_r4", setVoltage_r4_command);
  job_manager.register_command("setConversion_r1", setConversion_r1_command);
  job_manager.register_command("setConversion_r2", setConversion_r2_command);
  job_manager.register_command("setConversion_r3", setConversion_r3_command);
  job_manager.register_command("setConversion_r4", setConversion_r4_command);
  job_manager.register_command("reset", resetCommand);
  job_manager.register_command("factoryReset", factoryResetCommand);


  // Initialize flyback PSU device context
  esp_err_t ret = ESP_FAIL;
  for (int i = 0; i < 3; i++)
  {
    ret = flyback_init();
    if (ret == ESP_OK)
    {
      break;
    }
    ESP_LOGE(TAG, "Flyback board failed to initialize: %s, trying again after 5s.", esp_err_to_name(ret));
    i2c_driver_delete(I2C_NUM_0);
    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }

  // On PSU failure: do NOT abort(). That causes a bootloop in field-deployed devices.
  // Continue without HV — WiFi/MQTT/OTA/BLE remain operational so recovery is possible.
  bool psu_ok = (ret == ESP_OK);
  if (!psu_ok)
  {
    ESP_LOGE(TAG, "Flyback PSU init failed permanently — running without HV. Network remains active.");
  }
  else
  {
    flyback_set_voltage(0, static_cast<float>(devCfg.set_voltage[0]));
    flyback_set_voltage(1, static_cast<float>(devCfg.set_voltage[1]));
    flyback_set_voltage(2, static_cast<float>(devCfg.set_voltage[2]));
    flyback_set_voltage(3, static_cast<float>(devCfg.set_voltage[3]));
  }

  // Geiger counter PCNT is independent of the PSU — start it regardless
  esp_err_t err = geiger_counter_pcnt_start(&gc, (gpio_num_t)CONFIG_RASENS_INTERRUPT_PIN, CONFIG_RASENS_INTERRUPT_PIN_ACTIVE_HIGH, 500);
  if (err != ESP_OK)
  {
    ESP_LOGE(TAG, "Failed to start counter: %s", esp_err_to_name(err));
  }

  geiger_counter_set_conversion_factor(&gc, 0, static_cast<float>(devCfg.coeff[0]) / 10000.0f);
  geiger_counter_set_conversion_factor(&gc, 1, static_cast<float>(devCfg.coeff[1]) / 10000.0f);
  geiger_counter_set_conversion_factor(&gc, 2, static_cast<float>(devCfg.coeff[2]) / 10000.0f);
  geiger_counter_set_conversion_factor(&gc, 3, static_cast<float>(devCfg.coeff[3]) / 10000.0f);

  if (psu_ok)
  {
    flyback_set_geiger_counter_device(&gc);

    flyback_enable();

    esp_err_t idle = flyback_wait_for_idle(3000, 10);
    if (idle != ESP_OK)
    {
      ESP_LOGW(TAG, "Timeout waiting for range switch to complete");
    }

    flyback_set_channel(devCfg.active_range);
    geiger_counter_set_active_range(&gc, devCfg.active_range);

    idle = flyback_wait_for_idle(3000, 10);
    if (idle != ESP_OK)
    {
      ESP_LOGW(TAG, "Timeout waiting for range switch to complete");
    }
  }

  // All hardware init done — safe to mark firmware valid and clear crash counter.
  // This is the normal (non-escape-hatch) call. OTA rollback will not occur from here.
  s_consecutive_crashes = 0;
  mark_app_valid_cancel_rollback();

  hv_avg_init();
  if (psu_ok)
  {
    xTaskCreate(avg_u, "average U", 4096, NULL, 6, NULL);
  }

  while (true)
  {
    // wait before next measurement
    vTaskDelay(pdMS_TO_TICKS((uint32_t)devCfg.interval * 1000UL));

    // If the PSU wasn't available at boot (or was lost), try to bring it up.
    // i2c_driver_delete() is required between attempts because flyback_psu_init()
    // installs the driver and leaves it installed even on partial failures.
    if (!psu_ok)
    {
      ESP_LOGI(TAG, "Attempting PSU recovery...");
      if (flyback_init() == ESP_OK)
      {
        ESP_LOGI(TAG, "Flyback PSU recovered — applying config and enabling output");
        psu_ok = true;
        flyback_set_voltage(0, static_cast<float>(devCfg.set_voltage[0]));
        flyback_set_voltage(1, static_cast<float>(devCfg.set_voltage[1]));
        flyback_set_voltage(2, static_cast<float>(devCfg.set_voltage[2]));
        flyback_set_voltage(3, static_cast<float>(devCfg.set_voltage[3]));
        flyback_set_geiger_counter_device(&gc);
        flyback_enable();
        flyback_wait_for_idle(3000, 10);
        flyback_set_channel(devCfg.active_range);
        geiger_counter_set_active_range(&gc, devCfg.active_range);
        flyback_wait_for_idle(3000, 10);
        hv_avg_reset();
        xTaskCreate(avg_u, "average U", 4096, NULL, 6, NULL);
      }
      else
      {
        ESP_LOGW(TAG, "PSU recovery failed, will retry next interval");
        i2c_driver_delete(I2C_NUM_0);
      }
    }

    // Read Geiger counter statistics
    float cpm = geiger_counter_pcnt_get_cpm(&gc, devCfg.active_range);
    float usvh = geiger_counter_pcnt_get_sieverts_per_hour(&gc, devCfg.active_range);
    uint64_t total = geiger_counter_pcnt_get_total(&gc, devCfg.active_range);
    float cr = 0;
    float dr = 0;
    geiger_counter_pcnt_get_cr_dr(&gc, devCfg.active_range, &cr, &dr);
    ESP_LOGI("GEIGER", "Counts: CPM=%.1f uSv/h=%.3f Total=%llu", cpm, usvh, (unsigned long long)total);

    // get current ts
    time_t now;
    time(&now);
    // iotIs.send_data("cps", cps, now);
    
    uint8_t curCh = devCfg.active_range;
    if (psu_ok)
    {
      flyback_get_channel(&curCh);
    }
    int iface = 0;
    if (eth_is_connected()) {
        iface = 2;
    } else if (wifi_sta_is_connected()) {
        iface = 1;
    }

    if (iotIs.can_publish())
    {
      if (!iotIs.send_data("cr60", cpm, now))
        goto mqtt_publish_fail;
      if (!iotIs.send_data("dr60", usvh, now))
        goto mqtt_publish_fail;
      if (!iotIs.send_data("cr", cr, now))
        goto mqtt_publish_fail;
      if (!iotIs.send_data("dr", dr, now))
        goto mqtt_publish_fail;
      if (psu_ok)
      {
        if (!iotIs.send_data("voltage", hv_avg_get(), now))
          goto mqtt_publish_fail;
      }
      if (!iotIs.send_data("samplerate", devCfg.interval, now))
        goto mqtt_publish_fail;
      if (!iotIs.send_data("range", curCh + 1, now))
        goto mqtt_publish_fail;
      if (!iotIs.send_data("iface", iface, now))
        goto mqtt_publish_fail;
    }
    else
    {
      ESP_LOGW(TAG, "MQTT not connected, dropping telemetry batch");
    }
    mqtt_publish_fail:
    ;
  }
}
