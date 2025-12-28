#include "ble.h"

#include "esp_log.h"

#define TAG "BLE"

static void on_stack_reset(int reason);
static void on_stack_sync(void);
static void nimble_host_config_init(void);
static void nimble_host_task(void *param);

/* Stack event callbacks 
 *      - on_stack_reset is called when host resets BLE stack due to errors
 *      - on_stack_sync is called when host has synced with controller
 *
 */

static void on_stack_reset(int reason) {
    /* On reset, print reset reason to console */
    ESP_LOGW(TAG, "nimble stack reset, reset reason: %d", reason);
}

static void on_stack_sync(void) {
    /* On stack sync, do advertising initialization */
    adv_init();
}

static void nimble_host_config_init(void) {
    /* Set host callbacks */
    ble_hs_cfg.reset_cb = on_stack_reset;
    ble_hs_cfg.sync_cb = on_stack_sync;
    ble_hs_cfg.gatts_register_cb = gatt_svr_register_cb;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

}

static void nimble_host_task(void *param) {
    /* Task entry log */
    ESP_LOGI(TAG, "nimble host task has been started!");

    /* This function won't return until nimble_port_stop() is executed */
    nimble_port_run();

    /* Clean up at exit */
    vTaskDelete(NULL);
}

esp_err_t ble_init(void){
    /* Local variables */
    esp_err_t ret = ESP_OK;

    /* Initialize NimBLE port */
    ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to initialize nimble port, error code: %d", ret);
        return ret;
    }

    ret = gap_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to initialize GAP service, error code: %d", ret);
        return ret;
    }

    ret = gatt_svc_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to initialize GATT service, error code: %d", ret);
        return ret;
    }

    /* Initialize NimBLE host configuration */
    nimble_host_config_init();

    /* Start the NimBLE host task */
    nimble_port_freertos_init(nimble_host_task);

    return ret;
}