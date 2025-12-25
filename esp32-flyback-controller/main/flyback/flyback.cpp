#include "flyback.h"
#include "esp_log.h"
#include "flyback_psu.h"
#include "geiger_counter.h"

#define TAG "flyback"

#define I2C_FREQ_HZ 10000 // 10 kHz I2C frequency
#define I2C_PORT I2C_NUM_0
#define FULLSCALE_VOLTAGE 1077.5f

// Register base addresses for per-range overvoltage (OV) arrays (little-endian u16 x4)
#define REG_TRIP0 0x0E  // fb_ov_trip_cnt_tab[0] register base
#define REG_CLEAR0 0x16 // fb_ov_clear_cnt_tab[0] register base

static flyback_psu_t psu;
static bool is_enabled = false;
static bool is_awake = false;
static bool is_initialized = false;

static float voltage_table[4] = {0.0f, 0.0f, 0.0f, 0.0f};

static geiger_counter_pcnt4_t *gc_dev = nullptr;

esp_err_t flyback_init()
{
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = ((1ULL << CONFIG_RASENS_ENABLE_PIN) | (1ULL << CONFIG_RASENS_SLEEP_PIN));
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
    
    // initialize enable pin
    gpio_set_level((gpio_num_t)CONFIG_RASENS_ENABLE_PIN, 0); // Disable PSU by setting pin LOW

    // initialize sleep pin
    gpio_set_level((gpio_num_t)CONFIG_RASENS_SLEEP_PIN, 1); // Exit sleep mode by setting pin HIGH - this is required for I2C operation

    // Use 10 kHz I2C speed for stability; increase to 400 kHz if your bus is stable
    esp_err_t err = flyback_psu_init(&psu, I2C_PORT, (gpio_num_t)CONFIG_RASENS_SDA_PIN, (gpio_num_t)CONFIG_RASENS_SCL_PIN, I2C_FREQ_HZ, CONFIG_RASENS_I2C_ADDRESS);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize flyback PSU: %s", esp_err_to_name(err));
        return err;
    }

    // Set full-scale voltage to match AVR firmware scaling (default 1077.5 V)
    flyback_psu_set_fullscale(&psu, FULLSCALE_VOLTAGE);

    // Force I2C to control the active range immediately (overrides hardware pins)
    err = flyback_psu_set_range_source(&psu, FLYBACK_RANGE_SRC_I2C);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set range source: %s", esp_err_to_name(err));
        return err;
    }
    is_initialized = true;
    is_awake = true;
    ESP_LOGI(TAG, "Flyback PSU initialized successfully");
    return ESP_OK;
}

esp_err_t flyback_sleep()
{
    if (!is_initialized)
        return ESP_ERR_INVALID_STATE;

    geiger_counter_pcnt_pause(gc_dev);
    gpio_set_level((gpio_num_t)CONFIG_RASENS_SLEEP_PIN, 0); // Set sleep pin LOW to enter sleep mode
    is_awake = false;
    return ESP_OK;
}

esp_err_t flyback_wake()
{
    if (!is_initialized)
        return ESP_ERR_INVALID_STATE;

    gpio_set_level((gpio_num_t)CONFIG_RASENS_SLEEP_PIN, 1); // Set sleep pin HIGH to exit sleep mode

    // Small delay to allow PSU to stabilize after waking up
    vTaskDelay(pdMS_TO_TICKS(5000));
    geiger_counter_pcnt_resume(gc_dev);
    is_awake = true;
    return ESP_OK;
}

bool flyback_is_awake()
{
    return is_awake;
}

esp_err_t flyback_enable()
{
    if (!is_initialized)
        return ESP_ERR_INVALID_STATE;

    gpio_set_level((gpio_num_t)CONFIG_RASENS_ENABLE_PIN, 1); // Set enable pin HIGH to enable PSU
    vTaskDelay(pdMS_TO_TICKS(3000));
    geiger_counter_pcnt_resume(gc_dev);
    is_enabled = true;
    return ESP_OK;
}

esp_err_t flyback_disable()
{
    if (!is_initialized)
        return ESP_ERR_INVALID_STATE;
    
    geiger_counter_pcnt_pause(gc_dev);
    gpio_set_level((gpio_num_t)CONFIG_RASENS_ENABLE_PIN, 0); // Set enable pin LOW to disable PSU
    is_enabled = false;
    return ESP_OK;
}

bool flyback_is_enabled()
{
    return is_enabled;
}

esp_err_t flyback_set_voltage(uint8_t channel, float voltage)
{
    if (!is_initialized)
        return ESP_ERR_INVALID_STATE;

    if (!is_awake)
    {
        ESP_LOGE(TAG, "Cannot set voltage: Flyback PSU is in sleep mode");
        return ESP_ERR_INVALID_STATE;
    }

    if (channel > 3)
        return ESP_ERR_INVALID_ARG;

    // Convert voltage to counts using device scaling
    uint16_t counts = flyback_psu_volts_to_counts(&psu, voltage);
    esp_err_t err = flyback_psu_set_table_counts(&psu, channel, counts);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set voltage for channel %d: %s", channel, esp_err_to_name(err));
        return err;
    }
    voltage_table[channel] = voltage;

    // Set under and overvoltage thresholds (±10% and ±5%)
    float v_trip = voltage * 1.10f;
    float v_clear = voltage * 1.05f;
    uint16_t c_trip = flyback_psu_volts_to_counts(&psu, v_trip);
    uint16_t c_clear = flyback_psu_volts_to_counts(&psu, v_clear);
    uint8_t trip_reg = REG_TRIP0 + channel * 2;
    uint8_t clear_reg = REG_CLEAR0 + channel * 2;
    uint8_t trip_data[2] = {(uint8_t)(c_trip & 0xFF), (uint8_t)(c_trip >> 8)};
    uint8_t clear_data[2] = {(uint8_t)(c_clear & 0xFF), (uint8_t)(c_clear >> 8)};

    err = flyback_psu_write(&psu, trip_reg, trip_data, sizeof(trip_data));
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set trip threshold for channel %d: %s", channel, esp_err_to_name(err));
        return err;
    }
    err = flyback_psu_write(&psu, clear_reg, clear_data, sizeof(clear_data));
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set clear threshold for channel %d: %s", channel, esp_err_to_name(err));
        return err;
    }

    return ESP_OK;
}

esp_err_t flyback_get_voltage(float *voltage)
{
    if (!is_initialized)
        return ESP_ERR_INVALID_STATE;
    if (!is_awake)
    {
        ESP_LOGE(TAG, "Cannot get voltage: Flyback PSU is in sleep mode");
        return ESP_ERR_INVALID_STATE;
    }

    flyback_psu_status_t status;
    flyback_psu_read_status(&psu, &status);

    *voltage = (status.fb_counts * psu.fullscale_volts) / 1023.0f;

    return ESP_OK;
}

esp_err_t flyback_get_status(flyback_psu_status_t *status)
{
    if (!is_initialized)
        return ESP_ERR_INVALID_STATE;
    if (!is_awake)
    {
        ESP_LOGE(TAG, "Cannot get status: Flyback PSU is in sleep mode");
        return ESP_ERR_INVALID_STATE;
    }

    return flyback_psu_read_status(&psu, status);
}

esp_err_t flyback_set_channel(uint8_t channel)
{
    if (!is_initialized)
        return ESP_ERR_INVALID_STATE;
    if (!is_awake)
    {
        ESP_LOGE(TAG, "Cannot set channel: Flyback PSU is in sleep mode");
        return ESP_ERR_INVALID_STATE;
    }

    if (channel > 3)
        return ESP_ERR_INVALID_ARG;

    ESP_LOGI(TAG, "Selecting range %d -> target voltage ~%.1f V (counts=%u)",
             channel, voltage_table[channel], (unsigned int)(voltage_table[channel] * (1023.0f / psu.fullscale_volts)));
    esp_err_t err = flyback_psu_select_range(&psu, channel);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set channel %d: %s", channel, esp_err_to_name(err));
        return err;
    }
    if (gc_dev)
        geiger_counter_set_active_range(gc_dev, channel);
    return err;
}

esp_err_t flyback_get_channel(uint8_t *channel)
{
    if (!is_initialized)
        return ESP_ERR_INVALID_STATE;
    if (!is_awake)
    {
        ESP_LOGE(TAG, "Cannot get channel: Flyback PSU is in sleep mode");
        return ESP_ERR_INVALID_STATE;
    }

    flyback_psu_status_t status;
    esp_err_t err = flyback_psu_read_status(&psu, &status);
    if (err != ESP_OK)
    {
        return err;
    }

    *channel = status.active_range;
    return ESP_OK;
}

esp_err_t flyback_wait_for_idle(uint32_t timeout_ms, uint32_t poll_interval_ms)
{
    if (!is_initialized)
        return ESP_ERR_INVALID_STATE;
    if (!is_awake)
    {
        ESP_LOGE(TAG, "Cannot wait for idle: Flyback PSU is in sleep mode");
        return ESP_ERR_INVALID_STATE;
    }
    
    return flyback_psu_wait_idle(&psu, timeout_ms, poll_interval_ms);
}

float flyback_counts_to_volts(uint16_t counts)
{
    if (!is_initialized)
        return -1.0f;
    return (counts * psu.fullscale_volts) / 1023.0f;
}

uint16_t flyback_volts_to_counts(float volts)
{
    if (!is_initialized)
        return -1;
    return (uint16_t)((volts * 1023.0f) / psu.fullscale_volts);
}

void flyback_set_geiger_counter_device(geiger_counter_pcnt4_t* dev)
{
    gc_dev = dev;
}