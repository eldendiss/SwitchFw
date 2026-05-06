#include "commands.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "flyback.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hv_mon.h"
#include "storage.h"

#include "../main.h"

static void restartTask(void*){
    vTaskDelay(4000 / portTICK_PERIOD_MS);
    esp_restart();
}

bool resetCommand(const std::vector<double> &params)
{
    xTaskCreate(restartTask, "restart", 4096, NULL, 6, NULL);
    return true;
}

bool delayCommand(const std::vector<double> &params)
{
    if (params.empty())
        return false;

    int delayMs = static_cast<int>(params[0]);
    vTaskDelay(delayMs / portTICK_PERIOD_MS);
    return true;
}

bool setIntervalCommand(const std::vector<double> &params)
{
    if (params.empty())
        return false;

    int interval = static_cast<int>(params[0]);
    devCfg.interval = interval;
    storage_save_device_config_data(&devCfg);
    return true;
}

bool setTube_command(const std::vector<double> &params)
{
    if (params.empty())
        return false;

    int rPinHelp = static_cast<int>(params[0]);
    ESP_LOGI("COMMAND", "Setting tube range to %.2f", params[0]);

    if (rPinHelp == 4)
    {
        flyback_disable();
        ESP_LOGI("COMMAND", "Voltage switched off");
        hv_avg_reset();
        return true;
    }

    if (rPinHelp <= 3)
    {
        flyback_enable();
        flyback_set_channel((uint8_t)rPinHelp);
        esp_err_t idle = flyback_wait_for_idle(3000, 10);
        if (idle != ESP_OK)
        {
            ESP_LOGW("COMMAND", "Timeout waiting for range switch to complete");
        }

        devCfg.active_range = (uint8_t)rPinHelp;
        storage_save_device_config_data(&devCfg);
        ESP_LOGI("COMMAND", "Range %d applied", rPinHelp);
        hv_avg_reset();

        return true;
    }

    return true;
}

bool setVoltage_r1_command(const std::vector<double> &params)
{
    if (params.empty())
        return false;

    flyback_set_voltage(0, static_cast<float>(params[0]));
    ESP_LOGI("COMMAND", "Voltage R1 applied: %.2f V", static_cast<float>(params[0]));

    devCfg.set_voltage[0] = static_cast<uint16_t>(params[0]);
    storage_save_device_config_data(&devCfg);

    // switch ranges to apply voltage
    flyback_disable();
    // select suitable range
    uint8_t tRange = 0;
    if (devCfg.active_range == tRange)
    {
        tRange++;
    }
    flyback_set_channel(tRange);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    flyback_set_channel(devCfg.active_range);
    flyback_enable();
    hv_avg_reset();

    return true;
}

bool setVoltage_r2_command(const std::vector<double> &params)
{
    if (params.empty())
        return false;

    flyback_set_voltage(1, static_cast<float>(params[0]));
    ESP_LOGI("COMMAND", "Voltage R1 applied: %.2f V", static_cast<float>(params[0]));

    devCfg.set_voltage[1] = static_cast<uint16_t>(params[0]);
    storage_save_device_config_data(&devCfg);

    // switch ranges to apply voltage
    flyback_disable();
    // select suitable range
    uint8_t tRange = 0;
    if (devCfg.active_range == tRange)
    {
        tRange++;
    }
    flyback_set_channel(tRange);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    flyback_set_channel(devCfg.active_range);
    flyback_enable();
    hv_avg_reset();

    return true;
}

bool setVoltage_r3_command(const std::vector<double> &params)
{
    if (params.empty())
        return false;

    flyback_set_voltage(2, static_cast<float>(params[0]));
    ESP_LOGI("COMMAND", "Voltage R1 applied: %.2f V", static_cast<float>(params[0]));

    devCfg.set_voltage[2] = static_cast<uint16_t>(params[0]);
    storage_save_device_config_data(&devCfg);

    // switch ranges to apply voltage
    flyback_disable();
    // select suitable range
    uint8_t tRange = 0;
    if (devCfg.active_range == tRange)
    {
        tRange++;
    }
    flyback_set_channel(tRange);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    flyback_set_channel(devCfg.active_range);
    flyback_enable();
    hv_avg_reset();

    return true;
}

bool setVoltage_r4_command(const std::vector<double> &params)
{
    if (params.empty())
        return false;

    flyback_set_voltage(3, static_cast<float>(params[0]));
    ESP_LOGI("COMMAND", "Voltage R1 applied: %.2f V", static_cast<float>(params[0]));

    devCfg.set_voltage[3] = static_cast<uint16_t>(params[0]);
    storage_save_device_config_data(&devCfg);

    // switch ranges to apply voltage
    flyback_disable();
    // select suitable range
    uint8_t tRange = 0;
    if (devCfg.active_range == tRange)
    {
        tRange++;
    }
    flyback_set_channel(tRange);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    flyback_set_channel(devCfg.active_range);
    flyback_enable();
    hv_avg_reset();

    return true;
}

bool setConversion_r1_command(const std::vector<double> &params)
{
    if (params.empty())
        return false;

    geiger_counter_set_conversion_factor(&gc, 0, static_cast<float>(params[0]));
    ESP_LOGI("COMMAND", "Conversion factor R1 applied: %.2f", static_cast<float>(params[0]));

    devCfg.coeff[0] = static_cast<uint32_t>(params[0] * 10000.0f);
    storage_save_device_config_data(&devCfg);
    return true;
}

bool setConversion_r2_command(const std::vector<double> &params)
{
    if (params.empty())
        return false;

    geiger_counter_set_conversion_factor(&gc, 1, static_cast<float>(params[0]));
    ESP_LOGI("COMMAND", "Conversion factor R1 applied: %.2f", static_cast<float>(params[0]));

    devCfg.coeff[1] = static_cast<uint32_t>(params[0] * 10000.0f);
    storage_save_device_config_data(&devCfg);
    return true;
}

bool setConversion_r3_command(const std::vector<double> &params)
{
    if (params.empty())
        return false;

    geiger_counter_set_conversion_factor(&gc, 2, static_cast<float>(params[0]));
    ESP_LOGI("COMMAND", "Conversion factor R1 applied: %.2f", static_cast<float>(params[0]));

    devCfg.coeff[2] = static_cast<uint32_t>(params[0] * 10000.0f);
    storage_save_device_config_data(&devCfg);
    return true;
}

bool setConversion_r4_command(const std::vector<double> &params)
{
    if (params.empty())
        return false;

    geiger_counter_set_conversion_factor(&gc, 3, static_cast<float>(params[0]));
    ESP_LOGI("COMMAND", "Conversion factor R1 applied: %.2f", static_cast<float>(params[0]));

    devCfg.coeff[3] = static_cast<uint32_t>(params[0] * 10000.0f);
    storage_save_device_config_data(&devCfg);
    return true;
}

bool factoryResetCommand(const std::vector<double> &params)
{
    (void)params;

    // Clear both NVS namespaces so the device boots with menuconfig defaults.
    storage_clear_provisioning_data();
    storage_clear_device_config_data();

    xTaskCreate(restartTask, "restart", 4096, NULL, 6, NULL);
    return true;
}