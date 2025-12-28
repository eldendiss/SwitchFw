#pragma once
#ifndef RS_STORAGE_H
#define RS_STORAGE_H

#include "esp_err.h"
#include "stdint.h"

typedef struct 
{
    char ssid[32];
    char password[64];
    char mqtt_host[64];
    uint16_t mqtt_port;
    char access_token[25];
} provisioning_data_t;

typedef struct {
    uint16_t interval;
    uint8_t active_range;
    uint16_t set_voltage[4];
    uint32_t coeff[4];
} device_config_data_t;


esp_err_t storage_init(void);

esp_err_t storage_save_provisioning_data(const provisioning_data_t* data);
esp_err_t storage_load_provisioning_data(provisioning_data_t* data);

esp_err_t storage_save_device_config_data(const device_config_data_t* data);
esp_err_t storage_load_device_config_data(device_config_data_t* data);



#endif // RS_STORAGE_H
