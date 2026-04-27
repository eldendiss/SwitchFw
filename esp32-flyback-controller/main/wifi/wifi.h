#ifndef WIFI_H
#define WIFI_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_wifi.h"
#include "esp_netif.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

/* Bits used in the event group returned by wifi_get_event_group() */
#define WIFI_EVT_CONNECTED_BIT  BIT0
#define WIFI_EVT_FAIL_BIT       BIT1

/**
 * @brief Initialize Wi-Fi in STA mode and start the driver.
 *
 * Idempotent — safe to call more than once. On first call it creates the
 * default STA netif, registers event handlers, sets STA mode and starts
 * the driver. Subsequent calls just re-sync the event-group bits with
 * the current connection state.
 *
 * NOTE: NVS and the default event loop must already be initialized before
 *       calling this (i.e. nvs_flash_init() and esp_event_loop_create_default()).
 *       esp_netif_init() must also have been called.
 *
 * @return ESP_OK on success, or an esp_err_t from the underlying driver.
 */
esp_err_t wifi_init(void);

/**
 * @brief Begin associating with the given SSID asynchronously.
 *
 * If the STA is currently associated with a different AP, it is disconnected
 * cleanly first so the new credentials are honored. The internal retry
 * counter and the FAIL event bit are reset before the new attempt.
 *
 * Returns as soon as the connect request has been issued — use
 * wifi_wait_connected() to block until association completes (or fails).
 *
 * @param ssid     Null-terminated SSID. Must not be NULL or empty.
 * @param password Null-terminated passphrase, or NULL for an open network.
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG, or a driver error.
 */
esp_err_t wifi_connect_async(const char *ssid, const char *password);

/**
 * @brief Block until the device has network connectivity (Wi-Fi or Ethernet).
 *
 * Returns immediately if either link is already up. Otherwise waits on the
 * event group. A Wi-Fi FAIL during the wait is non-fatal — Ethernet may
 * still come up — so this only returns ESP_ERR_TIMEOUT when the full
 * timeout elapses without connectivity.
 *
 * @param ticks_to_wait FreeRTOS ticks to wait. Use portMAX_DELAY for forever.
 * @return ESP_OK on connectivity, ESP_ERR_TIMEOUT on timeout,
 *         ESP_ERR_INVALID_STATE if wifi_init() has not run.
 */
esp_err_t wifi_wait_connected(TickType_t ticks_to_wait);

/**
 * @brief Disconnect the STA without triggering auto-reconnect.
 *
 * Suppresses the internal retry mechanism for this intentional disconnect
 * and clears the event-group bits.
 *
 * @return ESP_OK on success, or a driver error.
 */
esp_err_t wifi_disconnect(void);

/**
 * @brief True if the STA is currently associated with an AP and has an IP.
 */
bool wifi_sta_is_connected(void);

/**
 * @brief True if Ethernet has been marked connected via eth_connected_override().
 */
bool eth_is_connected(void);

/**
 * @brief True if either Wi-Fi STA or Ethernet currently has connectivity.
 */
bool wifi_is_connected(void);

/**
 * @brief Get info about the AP the STA is currently associated with.
 *
 * @param ap_info Output AP record. Must not be NULL.
 * @return ESP_OK on success, ESP_ERR_WIFI_NOT_CONNECT if not associated,
 *         or a driver error.
 */
esp_err_t wifi_get_status(wifi_ap_record_t *ap_info);

/**
 * @brief Perform a blocking active scan and return up to *ap_count records.
 *
 * On entry, *ap_count must be the capacity of the ap_info buffer.
 * On return, *ap_count holds the number of records actually written.
 *
 * @param ap_info  Caller-allocated buffer for AP records.
 * @param ap_count In: capacity of ap_info. Out: records written.
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG, or a driver error.
 */
esp_err_t wifi_scan(wifi_ap_record_t *ap_info, uint16_t *ap_count);

/**
 * @brief Reset the retry counter and clear the FAIL event bit.
 *
 * Called automatically by wifi_connect_async(). Exposed so other modules
 * can clear a stale FAIL state before triggering a fresh connect through
 * a different path.
 */
void wifi_reset_state(void);

/**
 * @brief Get the underlying event group handle.
 *
 * Returns NULL before wifi_init() has run.
 */
EventGroupHandle_t wifi_get_event_group(void);

/**
 * @brief Notify the Wi-Fi module that Ethernet has come up.
 *
 * Updates the connectivity event-group bits so wifi_wait_connected() and
 * wifi_is_connected() see Ethernet as a valid uplink. Call from your
 * Ethernet "got IP" handler.
 */
void eth_connected_override(void);

/**
 * @brief Notify the Wi-Fi module that Ethernet has gone down.
 *
 * Call from your Ethernet "disconnected" handler.
 */
void eth_disconnected_override(void);

#ifdef __cplusplus
}
#endif

#endif /* WIFI_H */