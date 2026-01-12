#pragma once
#ifndef STORAGE_H
#define STORAGE_H

#include "esp_err.h"
#include "stdint.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#define WIFI_EVT_CONNECTED_BIT  BIT0
#define WIFI_EVT_FAIL_BIT       BIT1

esp_err_t wifi_init(void);


// Start a connection attempt (non-blocking). Does NOT wait.
esp_err_t wifi_connect_async(const char *ssid, const char *password);

// Wait for connection result with timeout.
// Returns:
//  - ESP_OK if got IP
//  - ESP_FAIL if failed (after retries)
//  - ESP_ERR_TIMEOUT if timed out
esp_err_t wifi_wait_connected(TickType_t ticks_to_wait);

bool wifi_is_connected();

esp_err_t wifi_get_status(wifi_ap_record_t* ap_info);

esp_err_t wifi_disconnect(void);

esp_err_t wifi_scan(wifi_ap_record_t* ap_info, uint16_t* ap_count);

// Clear event bits and retry counter before a fresh attempt
void wifi_reset_state(void);

// Helpers
esp_err_t wifi_get_status(wifi_ap_record_t *ap_info);
EventGroupHandle_t wifi_get_event_group(void);


#endif // STORAGE_H