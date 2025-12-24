#pragma once
#ifndef BLE_H
#define BLE_H

#include "esp_err.h"
#include "stdint.h"

esp_err_t ble_init(void);
esp_err_t ble_start_advertising(const char* device_name);
esp_err_t ble_stop_advertising(void);

#endif // BLE_H