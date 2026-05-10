#include "storage.h"

#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include <string.h>

static uint32_t calculate_crc32(const void* data, size_t length)
{
    //use CRC32-CCIIT polynomial 0x04C11DB7
    uint32_t crc = 0xFFFFFFFF;
    const uint8_t* bytes = (const uint8_t*)data;
    for (size_t i = 0; i < length; i++) {
        crc ^= (uint32_t)(bytes[i]) << 24;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x80000000) {
                crc = (crc << 1) ^ 0x04C11DB7;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc ^ 0xFFFFFFFF;
}

static uint32_t verify_crc32(const void* data, size_t length, uint32_t expected_crc)
{
    uint32_t crc = calculate_crc32(data, length);
    return crc == expected_crc;
}

esp_err_t storage_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW("storage", "nvs partition corrupted, erasing");
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    return err;
}

esp_err_t storage_save_provisioning_data(provisioning_data_t* data)
{
    nvs_handle_t handle;

    //calculate crc32
    uint32_t crc = calculate_crc32(data, sizeof(provisioning_data_t) - sizeof(uint32_t));
    data->crc32 = crc;

    esp_err_t err = nvs_open("prov", NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE("storage", "Failed to open NVS namespace 'prov': %d", err);
        return err;
    }

    err = nvs_set_blob(handle, "prov_d", data, sizeof(provisioning_data_t));
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    } else {
        ESP_LOGE("storage", "Failed to set provisioning data in NVS: %d", err);
    }

    nvs_close(handle);
    return err;
}

esp_err_t storage_load_provisioning_data(provisioning_data_t* data)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open("prov", NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE("storage", "Failed to open NVS namespace 'prov': %d", err);
        return err;
    }

    size_t required_size = sizeof(provisioning_data_t);
    err = nvs_get_blob(handle, "prov_d", data, &required_size);
    if (err != ESP_OK) {
        ESP_LOGE("storage", "Failed to get provisioning data from NVS: %d", err);
    }
    nvs_close(handle);

    //verify crc32
    if (err == ESP_OK) {
        if (!verify_crc32(data, sizeof(provisioning_data_t) - sizeof(uint32_t), data->crc32)) {
            ESP_LOGE("storage", "Provisioning data CRC32 mismatch");
            return ESP_ERR_INVALID_CRC;
        }
    }
    return err;
}

esp_err_t storage_clear_provisioning_data(void)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open("prov", NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return err;
    }

    err = nvs_erase_key(handle, "prov_d");
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }

    nvs_close(handle);
    return err;
}

esp_err_t storage_save_device_config_data(device_config_data_t* data)
{
    nvs_handle_t handle;

    uint32_t crc = calculate_crc32(data, sizeof(device_config_data_t) - sizeof(uint32_t));
    data->crc32 = crc;

    esp_err_t err = nvs_open("devcfg", NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return err;
    }

    err = nvs_set_blob(handle, "devcfg_d", data, sizeof(device_config_data_t));
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }

    nvs_close(handle);
    return err;
}

esp_err_t storage_load_device_config_data(device_config_data_t* data)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open("devcfg", NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return err;
    }

    size_t required_size = sizeof(device_config_data_t);
    err = nvs_get_blob(handle, "devcfg_d", data, &required_size);

    nvs_close(handle);

    if (err == ESP_OK) {
        if (!verify_crc32(data, sizeof(device_config_data_t) - sizeof(uint32_t), data->crc32)) {
            ESP_LOGE("storage", "Device config data CRC32 mismatch");
            return ESP_ERR_INVALID_CRC;
        }
    }
    return err;
}

esp_err_t storage_clear_device_config_data(void)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open("devcfg", NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return err;
    }

    err = nvs_erase_key(handle, "devcfg_d");
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }

    nvs_close(handle);
    return err;
}
