#include "commands.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "flyback.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "../main.h"

bool delayCommand(const std::vector<double> &params){
    if (params.empty())
        return false;

    int delayMs = static_cast<int>(params[0]);
    vTaskDelay(delayMs/portTICK_PERIOD_MS);
    return true;
}

bool setTube_command(const std::vector<double> &params){
    if (params.empty())
        return false;

    int rPinHelp = static_cast<int>(params[0]);
    ESP_LOGI("COMMAND", "Setting tube range to %.2f", params[0]);

    if (rPinHelp == 4) {
        flyback_disable();
        ESP_LOGI("COMMAND", "Voltage switched off");
        return true;
    }

    if (rPinHelp <= 3) {
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

        return true;
    }

    return true;
}

bool setVoltage_r1_command(const std::vector<double> &params){
    if (params.empty())
        return false;

    flyback_set_voltage(0, static_cast<float>(params[0]));
    ESP_LOGI("COMMAND", "Voltage R1 applied: %.2f V", static_cast<float>(params[0]));

    devCfg.set_voltage[0] = static_cast<uint16_t>(params[0]);
    storage_save_device_config_data(&devCfg);

    return true;
}

bool setVoltage_r2_command(const std::vector<double> &params){
    if (params.empty())
        return false;

    flyback_set_voltage(1, static_cast<float>(params[0]));
    ESP_LOGI("COMMAND", "Voltage R1 applied: %.2f V", static_cast<float>(params[0]));

    devCfg.set_voltage[1] = static_cast<uint16_t>(params[0]);
    storage_save_device_config_data(&devCfg);

    return true;
}

bool setVoltage_r3_command(const std::vector<double> &params){
    if (params.empty())
        return false;

    flyback_set_voltage(2, static_cast<float>(params[0]));
    ESP_LOGI("COMMAND", "Voltage R1 applied: %.2f V", static_cast<float>(params[0]));

    devCfg.set_voltage[2] = static_cast<uint16_t>(params[0]);
    storage_save_device_config_data(&devCfg);

    return true;
}

bool setVoltage_r4_command(const std::vector<double> &params){
    if (params.empty())
        return false;

    flyback_set_voltage(3, static_cast<float>(params[0]));
    ESP_LOGI("COMMAND", "Voltage R1 applied: %.2f V", static_cast<float>(params[0]));

    devCfg.set_voltage[3] = static_cast<uint16_t>(params[0]);
    storage_save_device_config_data(&devCfg);

    return true;
}

bool setConversion_r1_command(const std::vector<double> &params){
    if (params.empty())
        return false;

    geiger_counter_set_conversion_factor(&gc, 0, static_cast<float>(params[0]));
    ESP_LOGI("COMMAND", "Conversion factor R1 applied: %.2f", static_cast<float>(params[0]));

    devCfg.coeff[0] = static_cast<uint32_t>(params[0] * 10000.0f);
    storage_save_device_config_data(&devCfg);
    return true;
}

bool setConversion_r2_command(const std::vector<double> &params){
    if (params.empty())
        return false;

    geiger_counter_set_conversion_factor(&gc, 1, static_cast<float>(params[0]));
    ESP_LOGI("COMMAND", "Conversion factor R1 applied: %.2f", static_cast<float>(params[0]));

    devCfg.coeff[1] = static_cast<uint32_t>(params[0] * 10000.0f);
    storage_save_device_config_data(&devCfg);
    return true;
}

bool setConversion_r3_command(const std::vector<double> &params){
    if (params.empty())
        return false;

    geiger_counter_set_conversion_factor(&gc, 2, static_cast<float>(params[0]));
    ESP_LOGI("COMMAND", "Conversion factor R1 applied: %.2f", static_cast<float>(params[0]));

    devCfg.coeff[2] = static_cast<uint32_t>(params[0] * 10000.0f);
    storage_save_device_config_data(&devCfg);
    return true;
}

bool setConversion_r4_command(const std::vector<double> &params){
    if (params.empty())
        return false;

    geiger_counter_set_conversion_factor(&gc, 3, static_cast<float>(params[0]));
    ESP_LOGI("COMMAND", "Conversion factor R1 applied: %.2f", static_cast<float>(params[0]));

    devCfg.coeff[3] = static_cast<uint32_t>(params[0] * 10000.0f);
    storage_save_device_config_data(&devCfg);
    return true;
}