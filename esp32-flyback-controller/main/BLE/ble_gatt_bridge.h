#pragma once
#ifndef BLE_GATT_BRIDGE_H
#define BLE_GATT_BRIDGE_H

#include "storage.h"
#include "provisioning.h"
#include <stdbool.h>

void ble_gatt_get_prov_snapshot(provisioning_data_t *out);
void ble_gatt_set_status(const prov_status8_t *st);
void ble_gatt_notify_status(void);

// Optionally clear buffers when CMD=Clear
void ble_gatt_clear_prov_buffers(void);

#endif // BLE_GATT_BRIDGE_H