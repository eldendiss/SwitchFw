#include "storage.h"

#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include <string.h>

esp_err_t storage_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    return err;
}

esp_err_t storage_save_provisioning_data(const provisioning_data_t* data)
{
    nvs_handle_t handle;
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
    return err;
}

esp_err_t storage_save_device_config_data(const device_config_data_t* data)
{
    nvs_handle_t handle;
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
    return err;
}

