#pragma once
#ifndef STORAGE_H
#define STORAGE_H

#include "esp_err.h"
#include "stdint.h"
#include "esp_wifi.h"

esp_err_t wifi_init(void);

esp_err_t wifi_connect(const char* ssid, const char* password);

esp_err_t wifi_get_status(wifi_ap_record_t* ap_info);

esp_err_t wifi_disconnect(void);

esp_err_t wifi_scan(wifi_ap_record_t* ap_info, uint16_t* ap_count);




#endif // STORAGE_H