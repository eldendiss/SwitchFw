#pragma once
#ifndef PROVISIONING_H
#define PROVISIONING_H

#include "ble.h"
#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>
#include "storage.h"   // provisioning_data_t

typedef enum {
    DEV_IDLE,
    DEV_PROVISIONED,
    DEV_APPLYING,
    DEV_ERROR
} device_status_t;

typedef enum {
    WIFI_DISCONNECTED = 0,
    WIFI_CONNECTING   = 1,
    WIFI_CONNECTED    = 2,  // got IP
    WIFI_FAILED       = 3,
} wifi_status_t;

typedef enum {
    MQTT_DISCONNECTED = 0,
    MQTT_CONNECTING   = 1,
    MQTT_CONNECTED    = 2,
    MQTT_FAILED       = 3,
} mqtt_status_t;

typedef enum {
    ERR_NONE            = 0,
    ERR_VALIDATION      = 1,
    ERR_WIFI_TIMEOUT    = 2,
    ERR_WIFI_AUTH       = 3,
    ERR_WIFI_NO_IP      = 4,
    ERR_MQTT_FAILED     = 5,
    ERR_STORAGE         = 6,
} prov_err_t;

typedef struct __attribute__((packed)) {
    uint8_t device_status;
    uint8_t wifi_status;
    uint8_t mqtt_status;
    uint8_t err_code;
    int8_t  wifi_rssi;      // store signed RSSI (dBm)
    uint8_t reserved[3];
} prov_status8_t;


esp_err_t provisioning_manager_init(void);

esp_err_t connection_supervisor_init(void);

// Call when CMD characteristic is written:
void provisioning_manager_on_command(uint8_t cmd);

//wait until device is provisioned (blocking)
void wifi_wait_provisioned(void);

// Start (or restart) connection attempts using new active config
void connection_supervisor_set_active(const provisioning_data_t *cfg, bool cfg_is_provisioned);

// Get current active config
void connection_supervisor_get_active(provisioning_data_t *out, bool *out_is_provisioned);

bool validate_prov(const provisioning_data_t *p);

#endif // PROVISIONING_H