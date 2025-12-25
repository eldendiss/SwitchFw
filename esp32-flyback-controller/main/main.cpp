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

static const char *TAG = "main";

geiger_counter_pcnt4_t gc;
device_config_data_t devCfg;

extern "C" void app_main(void)
{

  // initialize storage
  storage_init();

  // mark app as valid to avoid rollback
  mark_app_valid_cancel_rollback();

  // initialize wifi
  wifi_init();

  // try to get provisioning data
  provisioning_data_t provData;
  if (storage_load_provisioning_data(&provData) != ESP_OK)
  {
    ESP_LOGE(TAG, "Failed to load provisioning data from storage, using defaults");
    memccpy(provData.ssid, CONFIG_RASENS_DEFAULT_SSID, 0, sizeof(provData.ssid));
    memccpy(provData.password, CONFIG_RASENS_DEFAULT_WIFI_PASSWORD, 0, sizeof(provData.password));
    memccpy(provData.mqtt_host, CONFIG_RASENS_MQTT_HOST, 0, sizeof(provData.mqtt_host));
    provData.mqtt_port = CONFIG_RASENS_MQTT_PORT;
    memccpy(provData.access_token, CONFIG_RASENS_ACCESS_TOKEN, 0, sizeof(provData.access_token));

    // store defaults back to storage
    storage_save_provisioning_data(&provData);
  }
  else
  {
    ESP_LOGI(TAG, "Loaded provisioning data: SSID=%s, MQTT host=%s, MQTT port=%u",
             provData.ssid, provData.mqtt_host, provData.mqtt_port);
  }

  // try to load device configuration data
 
  if (storage_load_device_config_data(&devCfg) != ESP_OK)
  {
    ESP_LOGW(TAG, "No device configuration data found in storage, using defaults");
    devCfg.interval = 60000; // 60 seconds
    devCfg.active_range = 0;
    devCfg.set_voltage[0] = 200; // R1
    devCfg.set_voltage[1] = 250; // R2
    devCfg.set_voltage[2] = 300; // R3
    devCfg.set_voltage[3] = 350; // R4
    devCfg.coeff[0] = 93;      // R1
    devCfg.coeff[1] = 62;     // R2
    devCfg.coeff[2] = 1111;     // R3
    devCfg.coeff[3] = 11111;     // R4

    // store defaults back to storage
    storage_save_device_config_data(&devCfg);
  }
  else
  {
    ESP_LOGI(TAG, "Loaded device configuration data: interval=%u ms, active_range = %u", devCfg.interval,devCfg.active_range);
  }

  // connect to wifi using defaults
  wifi_connect(provData.ssid, provData.password);

  // publish current firmware version
  updateFirmwareVersion(CONFIG_RASENS_HTTP_BACKEND_URL, provData.access_token);

  // perform ota update if available
  perform_ota_update(CONFIG_RASENS_HTTP_BACKEND_URL, provData.access_token);

  // initialize sntp for time synchronization
  init_sntp(CONFIG_RASENS_SNTP_SYNC_INTERVAL_MS); // sync interval 5 minutes

  // initialize IoT IS platform connection and wait for connection
  iotIs.connect(provData.access_token, provData.mqtt_host, provData.mqtt_port);
  while (!iotIs.isConnected)
  {
    ESP_LOGI(TAG, "Waiting for MQTT connection...");
    vTaskDelay(pdMS_TO_TICKS(1000));
  }

  job_manager.init();
  job_manager.register_command("setTube", setTube_command);
  job_manager.register_command("setVoltage_r1", setVoltage_r1_command);
  job_manager.register_command("setVoltage_r2", setVoltage_r2_command);
  job_manager.register_command("setVoltage_r3", setVoltage_r3_command);
  job_manager.register_command("setVoltage_r4", setVoltage_r4_command);
  job_manager.register_command("setConversion_r1", setConversion_r1_command);
  job_manager.register_command("setConversion_r2", setConversion_r2_command);
  job_manager.register_command("setConversion_r3", setConversion_r3_command);
  job_manager.register_command("setConversion_r4", setConversion_r4_command);

  // Initialize flyback PSU device context
  flyback_init();

  flyback_set_voltage(0, static_cast<float>(devCfg.set_voltage[0]));
  flyback_set_voltage(1, static_cast<float>(devCfg.set_voltage[1]));
  flyback_set_voltage(2, static_cast<float>(devCfg.set_voltage[2]));
  flyback_set_voltage(3, static_cast<float>(devCfg.set_voltage[3]));

  // Optionally persist configuration to EEPROM (uncomment if desired)
  // ESP_ERROR_CHECK(flyback_psu_send_command(&psu, FLYBACK_CMD_SAVE_CONFIG));

  // accept pulses >= 1000 ns (1 µs),
  esp_err_t err = geiger_counter_pcnt_start(&gc, (gpio_num_t)CONFIG_RASENS_INTERRUPT_PIN, CONFIG_RASENS_INTERRUPT_PIN_ACTIVE_HIGH, 1000);
  if (err != ESP_OK)
  {
    ESP_LOGE(TAG, "Failed to start counter: %s", esp_err_to_name(err));
  }
  
  geiger_counter_set_conversion_factor(&gc, 0, static_cast<float>(devCfg.coeff[0]) / 10000.0f);  // R1
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

  flyback_set_channel(1);
  geiger_counter_set_active_range(&gc, 1);

  // Wait for relay switching to complete (timeout 3 seconds)
  idle = flyback_wait_for_idle(3000, 10);
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

  while (true)
  {
    /*flyback_wake();
    flyback_enable();
    vTaskDelay(pdMS_TO_TICKS(10000)); // Wait 3 seconds after enabling
    geiger_counter_pcnt_resume(&gc);*/
    flyback_psu_status_t s;
    if (flyback_get_status(&s) == ESP_OK)
    {
      float vfb = flyback_counts_to_volts(s.fb_counts); // Convert counts to volts
      ESP_LOGI(TAG,
               "PSU status: state=%u fault=%u fb_counts=%u (~%.1f V) dcounts=%d active_range=%u busy=%u",
               s.run_state, s.fault_code, s.fb_counts, vfb, (int)s.dcounts,
               s.active_range, s.range_busy);
    }
    else
    {
      ESP_LOGW(TAG, "Failed to read PSU status");
    }
    // Measure for 1 minute */
    // vTaskDelay(pdMS_TO_TICKS(60000));
    // Read Geiger counter statistics
    float cps = geiger_counter_pcnt_get_cps(&gc, devCfg.active_range);
    float cpm = geiger_counter_pcnt_get_cpm(&gc, devCfg.active_range);
    float usvh = geiger_counter_pcnt_get_sieverts_per_hour(&gc, devCfg.active_range);
    uint64_t total = geiger_counter_pcnt_get_total(&gc, devCfg.active_range);
    ESP_LOGI("GEIGER", "Counts: CPS=%.1f  CPM=%.1f uSv/h=%.3f Total=%llu", cps, cpm, usvh, (unsigned long long)total);

    iotIs.send_data("cps", cps);
    iotIs.send_data("cpm", cpm);
    iotIs.send_data("rl", usvh);
    iotIs.send_data(("voltage"), flyback_counts_to_volts(s.fb_counts));
    /*geiger_counter_pcnt_pause(&gc);
    flyback_disable();
    flyback_sleep();*/
    // wait 1 minute before next measurement
    vTaskDelay(pdMS_TO_TICKS(devCfg.interval));
  }
}
