#include "commands.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "flyback.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

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

    if (rPinHelp == 4) {
        flyback_disable();
        ESP_LOGI("COMMAND", "Voltage switched off");
        return true;
    }

    if (rPinHelp <= 3) {

        flyback_set_channel((uint8_t)rPinHelp);
        esp_err_t idle = flyback_wait_for_idle(3000, 10);
        if (idle != ESP_OK)
        {
            ESP_LOGW("COMMAND", "Timeout waiting for range switch to complete");
        }
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

    return true;
}
bool setVoltage_r2_command(const std::vector<double> &params){
    if (params.empty())
        return false;

    flyback_set_voltage(1, static_cast<float>(params[0]));
    ESP_LOGI("COMMAND", "Voltage R2 applied: %.2f V", static_cast<float>(params[0]));

    return true;
}
bool setVoltage_r3_command(const std::vector<double> &params){
    if (params.empty())
        return false;

    flyback_set_voltage(2, static_cast<float>(params[0]));
    ESP_LOGI("COMMAND", "Voltage R3 applied: %.2f V", static_cast<float>(params[0]));

    return true;
}
bool setVoltage_r4_command(const std::vector<double> &params){
    if (params.empty())
        return false;

    flyback_set_voltage(3, static_cast<float>(params[0]));
    ESP_LOGI("COMMAND", "Voltage R4 applied: %.2f V", static_cast<float>(params[0]));

    return true;
}
bool setConversion_r1_command(const std::vector<double> &params){
    return true;
}
bool setConversion_r2_command(const std::vector<double> &params){
    return true;
}
bool setConversion_r3_command(const std::vector<double> &params){
    return true;
}
bool setConversion_r4_command(const std::vector<double> &params){
    return true;
}