#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
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

static const char *TAG = "main";

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

  // initialize storage
  storage_init();

  // mark app as valid to avoid rollback
  mark_app_valid_cancel_rollback();

  ble_init();

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

  // wait for provisioning to complete
  ESP_LOGI(TAG, "Waiting for provisioning to complete...");
  wifi_wait_connected(pdMS_TO_TICKS(60000)); // wait up to 60 seconds

  // initialize sntp for time synchronization
  init_sntp(CONFIG_RASENS_SNTP_SYNC_INTERVAL_MS); // sync interval 5 minutes

  // publish current firmware version
  updateFirmwareVersion(CONFIG_RASENS_HTTP_BACKEND_URL, cfg.access_token);

  // perform ota update if available
  perform_ota_update(CONFIG_RASENS_HTTP_BACKEND_URL, cfg.access_token);

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

  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "Flyback init failed permanently, restarting");
    abort();
  }

  flyback_set_voltage(0, static_cast<float>(devCfg.set_voltage[0]));
  flyback_set_voltage(1, static_cast<float>(devCfg.set_voltage[1]));
  flyback_set_voltage(2, static_cast<float>(devCfg.set_voltage[2]));
  flyback_set_voltage(3, static_cast<float>(devCfg.set_voltage[3]));

  // Optionally persist configuration to EEPROM (uncomment if desired)
  // ESP_ERROR_CHECK(flyback_psu_send_command(&psu, FLYBACK_CMD_SAVE_CONFIG));

  // accept pulses >= 1000 ns (1 µs),
  esp_err_t err = geiger_counter_pcnt_start(&gc, (gpio_num_t)CONFIG_RASENS_INTERRUPT_PIN, CONFIG_RASENS_INTERRUPT_PIN_ACTIVE_HIGH, 10000);
  if (err != ESP_OK)
  {
    ESP_LOGE(TAG, "Failed to start counter: %s", esp_err_to_name(err));
  }

  geiger_counter_set_conversion_factor(&gc, 0, static_cast<float>(devCfg.coeff[0]) / 10000.0f); // R1
  geiger_counter_set_conversion_factor(&gc, 1, static_cast<float>(devCfg.coeff[1]) / 10000.0f); // R2
  geiger_counter_set_conversion_factor(&gc, 2, static_cast<float>(devCfg.coeff[2]) / 10000.0f); // R3
  geiger_counter_set_conversion_factor(&gc, 3, static_cast<float>(devCfg.coeff[3]) / 10000.0f); // R4

  // link Geiger counter device to flyback controller for range sync
  flyback_set_geiger_counter_device(&gc);

  // Enable flyback PSU output
  flyback_enable();

  // Wait for relay switching to complete (timeout 3 seconds)
  esp_err_t idle = flyback_wait_for_idle(3000, 10);
  if (idle != ESP_OK)
  {
    ESP_LOGW(TAG, "Timeout waiting for range switch to complete");
  }

  flyback_set_channel(devCfg.active_range);
  geiger_counter_set_active_range(&gc, devCfg.active_range);

  // Wait for relay switching to complete (timeout 3 seconds)
  idle = flyback_wait_for_idle(3000, 10);
  if (idle != ESP_OK)
  {
    ESP_LOGW(TAG, "Timeout waiting for range switch to complete");
  }

  hv_avg_init();
  xTaskCreate(avg_u, "average U", 4096, NULL, 6, NULL);

  while (true)
  {
    // wait 1 minute before next measurement
    vTaskDelay(pdMS_TO_TICKS((uint32_t)devCfg.interval * 1000UL));
    /*flyback_wake();
    flyback_enable();
    geiger_counter_pcnt_resume(&gc);*/

    // Measure for 1 minute */
    // vTaskDelay(pdMS_TO_TICKS(60000));
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
    //iotIs.send_data("cps", cps, now);
    iotIs.send_data("cr60", cpm, now);
    iotIs.send_data("dr60", usvh, now);
    iotIs.send_data("cr", cr, now);
    iotIs.send_data("dr", dr, now);
    iotIs.send_data("voltage", hv_avg_get(), now);
    iotIs.send_data("samplerate", devCfg.interval, now);
    uint8_t curCh = 0;
    flyback_get_channel(&curCh);
    iotIs.send_data("range", curCh+1, now);

    /*char tag[8];
    snprintf(tag,sizeof(tag),"r%d", curCh + 1);
    iotIs.send_data(tag, usvh, now);*/

    /*geiger_counter_pcnt_pause(&gc);
    flyback_disable();
    flyback_sleep();*/
  }
}
