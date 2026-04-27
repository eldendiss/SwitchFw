#ifndef PROVISIONING_H
#define PROVISIONING_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "storage.h"


/* ------------------------------------------------------------------------- */
/* Status enums (BLE-facing — values are wire format, do NOT renumber)       */
/* ------------------------------------------------------------------------- */

typedef enum {
    DEV_IDLE        = 0,
    DEV_APPLYING    = 1,
    DEV_PROVISIONED = 2,
    DEV_ERROR       = 3,
} device_status_t;

typedef enum {
    WIFI_DISCONNECTED = 0,
    WIFI_CONNECTING   = 1,
    WIFI_CONNECTED    = 2,
    WIFI_FAILED       = 3,
} wifi_status_t;

typedef enum {
    MQTT_DISCONNECTED = 0,
    MQTT_CONNECTING   = 1,
    MQTT_CONNECTED    = 2,
    MQTT_FAILED       = 3,
} mqtt_status_t;

typedef enum {
    ERR_NONE         = 0,
    ERR_VALIDATION   = 1,
    ERR_WIFI_AUTH    = 2,
    ERR_WIFI_TIMEOUT = 3,
    ERR_MQTT_FAILED  = 4,
    ERR_STORAGE      = 5,
} prov_err_t;

/* ------------------------------------------------------------------------- */
/* BLE status payload                                                        */
/*                                                                           */
/* Packed 8-byte status block published over the BLE GATT status             */
/* characteristic. Layout matches the wire format expected by the app.       */
/* The trailing reserved bytes pad to 8 and are available for future use.    */
/* ------------------------------------------------------------------------- */

typedef struct __attribute__((packed)) {
    uint8_t device_status;   /* device_status_t */
    uint8_t wifi_status;     /* wifi_status_t   */
    uint8_t mqtt_status;     /* mqtt_status_t   */
    uint8_t err_code;        /* prov_err_t      */
    int8_t  wifi_rssi;       /* dBm, 0 if unknown */
    uint8_t reserved[3];
} prov_status8_t;

/* ------------------------------------------------------------------------- */
/* Lifecycle                                                                 */
/* ------------------------------------------------------------------------- */

/**
 * @brief Initialize internal state, locks, and request queue.
 *
 * Must be called exactly once at startup, before connection_supervisor_init()
 * and before any provisioning_manager_on_command() calls.
 */
esp_err_t provisioning_manager_init(void);

/**
 * @brief Start the connection supervisor task.
 *
 * The supervisor is the sole owner of Wi-Fi and MQTT state changes — it
 * handles Apply/Test requests from apply_task and runs a low-rate maintain
 * loop that reconnects when either link drops.
 */
esp_err_t connection_supervisor_init(void);

/* ------------------------------------------------------------------------- */
/* Active config                                                             */
/* ------------------------------------------------------------------------- */

/**
 * @brief Set the configuration the supervisor will maintain.
 *
 * Typically called at boot with NVS-loaded credentials, and again by
 * apply_task once a candidate has been accepted. The supervisor only
 * attempts to maintain a connection when ssid is non-empty.
 *
 * @param cfg                Config to copy in. Must not be NULL.
 * @param cfg_is_provisioned True if cfg came from NVS (vs. menuconfig defaults).
 */
void connection_supervisor_set_active(const provisioning_data_t *cfg,
                                      bool cfg_is_provisioned);

/**
 * @brief Read a snapshot of the active config.
 *
 * @param out                Filled with the current active config.
 * @param out_is_provisioned Optional. Filled with the provisioned flag.
 */
void connection_supervisor_get_active(provisioning_data_t *out,
                                      bool *out_is_provisioned);

/* ------------------------------------------------------------------------- */
/* Command entrypoint (called from BLE write handler)                        */
/* ------------------------------------------------------------------------- */

/* Command codes (wire format) */
#define PROV_CMD_APPLY      0x01  /* Validate, test connection, save on success */
#define PROV_CMD_CLEAR      0x02  /* Wipe NVS, revert to menuconfig defaults    */
#define PROV_CMD_SAVE_ONLY  0x03  /* Validate and save, no connection attempt   */

/**
 * @brief Dispatch a command from the BLE control characteristic.
 *
 * Spawns a short-lived worker task; returns immediately. Status updates
 * are published via the BLE status characteristic asynchronously.
 *
 * @param cmd One of PROV_CMD_APPLY, PROV_CMD_CLEAR, PROV_CMD_SAVE_ONLY.
 *            Anything else is reported as ERR_VALIDATION.
 */
void provisioning_manager_on_command(uint8_t cmd);

/* ------------------------------------------------------------------------- */
/* Helpers                                                                   */
/* ------------------------------------------------------------------------- */

/**
 * @brief Validate a candidate provisioning record.
 *
 * Checks for non-empty SSID, non-empty MQTT host, sensible port, and
 * non-empty access token. Does not check semantic validity (e.g. whether
 * the host actually resolves).
 */
bool validate_prov(const provisioning_data_t *p);

#ifdef __cplusplus
}
#endif

#endif /* PROVISIONING_H */