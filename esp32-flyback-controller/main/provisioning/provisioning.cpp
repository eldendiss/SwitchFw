#include "provisioning.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_wifi.h"

#include "storage.h"
#include "ble_gatt_bridge.h"
#include "wifi.h"
#include "iot_is.h"

#define TAG "prov_mgr"

#define WIFI_TRY_MS              15000
#define MQTT_TRY_MS               8000
#define SUPERVISOR_LOOP_MS        2000

// =======================
// Status handling
// =======================

static prov_status8_t s_status;
static SemaphoreHandle_t s_status_lock;

static void status_publish_locked(void)
{
    ble_gatt_set_status(&s_status);
    ble_gatt_notify_status();
}

static void status_set(device_status_t dev, wifi_status_t wifi, mqtt_status_t mqtt, prov_err_t err)
{
    xSemaphoreTake(s_status_lock, portMAX_DELAY);
    s_status.device_status = (uint8_t)dev;
    s_status.wifi_status   = (uint8_t)wifi;
    s_status.mqtt_status   = (uint8_t)mqtt;
    s_status.err_code      = (uint8_t)err;

    // Update RSSI opportunistically
    wifi_ap_record_t ap = {0};
    if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
        s_status.wifi_rssi = (int8_t)ap.rssi;
    } else {
        s_status.wifi_rssi = 0;
    }

    status_publish_locked();
    xSemaphoreGive(s_status_lock);
}

bool validate_prov(const provisioning_data_t *p)
{
    if (!p) return false;
    if (p->ssid[0] == '\0') return false;
    if (p->mqtt_host[0] == '\0') return false;
    if (p->mqtt_port == 0 || p->mqtt_port > 65535) return false;
    if (p->access_token[0] == '\0') return false;
    return true;
}

// =======================
// Active config (supervisor-owned state)
// =======================

static provisioning_data_t s_active;
static bool s_active_is_provisioned = false;
static SemaphoreHandle_t s_active_lock;

// =======================
// Supervisor request/response (Apply/Test)
// =======================

typedef enum {
    REQ_TEST_CANDIDATE = 1,
} req_type_t;

typedef struct {
    req_type_t type;
    provisioning_data_t candidate;
    TaskHandle_t requester;
} sup_req_t;

static QueueHandle_t s_sup_req_q;

// Forward
static bool supervisor_connect_with(const provisioning_data_t *cfg, bool publish_status);

// =======================
// Supervisor connect logic (single owner of Wi-Fi + MQTT)
// =======================

static bool net_is_connected_now(void)
{
    return wifi_is_connected();   // your wifi.c now reports Wi-Fi OR Ethernet
}

static bool supervisor_connect_with(const provisioning_data_t *cfg, bool publish_status)
{
    if (!cfg || cfg->ssid[0] == '\0') {
        if (publish_status) {
            status_set(DEV_ERROR, WIFI_FAILED, MQTT_DISCONNECTED, ERR_VALIDATION);
        }
        return false;
    }

    bool net_ok = net_is_connected_now();

    // Step 1: bring up network only if neither ETH nor Wi-Fi is available
    if (!net_ok) {
        if (publish_status) {
            status_set(DEV_APPLYING, WIFI_CONNECTING, MQTT_DISCONNECTED, ERR_NONE);
        }

        ESP_LOGI(TAG, "Connecting Wi-Fi: SSID=%s", cfg->ssid);
        esp_err_t rc = wifi_connect_async(cfg->ssid, cfg->password);
        if (rc != ESP_OK) {
            ESP_LOGW(TAG, "wifi_connect_async failed: %s", esp_err_to_name(rc));
            if (publish_status) {
                status_set(DEV_ERROR, WIFI_FAILED, MQTT_DISCONNECTED, ERR_WIFI_AUTH);
            }
            return false;
        }

        esp_err_t w = wifi_wait_connected(pdMS_TO_TICKS(WIFI_TRY_MS));
        if (w != ESP_OK) {
            ESP_LOGW(TAG, "Network bring-up failed: %s", esp_err_to_name(w));
            if (publish_status) {
                prov_err_t e = (w == ESP_ERR_TIMEOUT) ? ERR_WIFI_TIMEOUT : ERR_WIFI_AUTH;
                status_set(DEV_ERROR, WIFI_FAILED, MQTT_DISCONNECTED, e);
            }
            return false;
        }

        net_ok = true;
    } else {
        ESP_LOGI(TAG, "Network already available, skipping Wi-Fi connect");
    }

    if (publish_status) {
        status_set(DEV_APPLYING, WIFI_CONNECTED, MQTT_CONNECTING, ERR_NONE);
    }

    // Step 2: MQTT only if not already connected
    if (!iotIs.isConnected) {
        ESP_LOGI(TAG, "Connecting MQTT: %s:%u", cfg->mqtt_host, cfg->mqtt_port);
        iotIs.connect(cfg->access_token, cfg->mqtt_host, cfg->mqtt_port);

        uint32_t t = MQTT_TRY_MS;
        while (t > 0 && !iotIs.isConnected) {
            vTaskDelay(pdMS_TO_TICKS(100));
            t -= 100;
        }

        if (!iotIs.isConnected) {
            ESP_LOGW(TAG, "MQTT connect failed");
            if (publish_status) {
                status_set(DEV_ERROR, WIFI_CONNECTED, MQTT_FAILED, ERR_MQTT_FAILED);
            }
            return false;
        }
    } else {
        ESP_LOGI(TAG, "MQTT already connected, skipping reconnect");
    }

    if (publish_status) {
        status_set(DEV_PROVISIONED, WIFI_CONNECTED, MQTT_CONNECTED, ERR_NONE);
    }

    return true;
}

// =======================
// Supervisor task
// =======================

static void supervisor_task(void *arg)
{
    (void)arg;

    for (;;) {
        // 1) Handle Apply/Test requests (non-blocking poll)
        sup_req_t req;
        while (xQueueReceive(s_sup_req_q, &req, 0) == pdTRUE) {
            if (req.type == REQ_TEST_CANDIDATE) {
                // Test candidate WITHOUT committing to s_active and WITHOUT changing BLE "device status" permanently.
                // We do publish status so the app sees what's happening.
                provisioning_data_t old;
                bool old_is_prov;
                connection_supervisor_get_active(&old, &old_is_prov);

                bool ok = supervisor_connect_with(&req.candidate, true);

                // Revert runtime to old active if test failed OR even if ok (policy choice):
                // For your requirements, if ok we'll keep it connected (nice UX), and apply_task will set active+save.
                // If fail, revert immediately.
                if (!ok) {
                    ESP_LOGI(TAG, "Candidate failed, reverting to previous active config");
                    supervisor_connect_with(&old, true);
                }

                if (req.requester) {
                    if (ok) xTaskNotifyGive(req.requester);
                }
            }
        }

        // 2) Background maintain: if Wi-Fi or MQTT is down, reconnect using current s_active.
        provisioning_data_t cfg;
        xSemaphoreTake(s_active_lock, portMAX_DELAY);
        cfg = s_active;
        xSemaphoreGive(s_active_lock);

        bool net_ok  = net_is_connected_now();
        bool mqtt_ok = iotIs.isConnected;

        if (!net_ok  || !mqtt_ok) {
            // publish status according to actual state before reconnect
            if (!net_ok ) {
                status_set(DEV_IDLE, WIFI_DISCONNECTED, MQTT_DISCONNECTED, ERR_NONE);
            } else {
                status_set(DEV_IDLE, WIFI_CONNECTED, mqtt_ok ? MQTT_CONNECTED : MQTT_DISCONNECTED, ERR_NONE);
            }
            supervisor_connect_with(&cfg, true);
        }

        vTaskDelay(pdMS_TO_TICKS(SUPERVISOR_LOOP_MS));
    }
}

// =======================
// Public init / active config APIs
// =======================

esp_err_t provisioning_manager_init(void)
{
    s_status_lock = xSemaphoreCreateMutex();
    if (!s_status_lock) return ESP_ERR_NO_MEM;

    s_active_lock = xSemaphoreCreateMutex();
    if (!s_active_lock) return ESP_ERR_NO_MEM;

    memset(&s_status, 0, sizeof(s_status));
    s_status.device_status = DEV_IDLE;
    s_status.wifi_status   = WIFI_DISCONNECTED;
    s_status.mqtt_status   = MQTT_DISCONNECTED;
    s_status.err_code      = ERR_NONE;
    s_status.wifi_rssi     = 0;

    xSemaphoreTake(s_status_lock, portMAX_DELAY);
    status_publish_locked();
    xSemaphoreGive(s_status_lock);

    // Request queue (Apply/Test)
    s_sup_req_q = xQueueCreate(4, sizeof(sup_req_t));
    if (!s_sup_req_q) return ESP_ERR_NO_MEM;

    return ESP_OK;
}

esp_err_t connection_supervisor_init(void)
{
    if (!s_sup_req_q) return ESP_ERR_INVALID_STATE;
    BaseType_t ok = xTaskCreate(supervisor_task, "conn_sup", 4096, NULL, 6, NULL);
    return ok == pdPASS ? ESP_OK : ESP_FAIL;
}

void connection_supervisor_set_active(const provisioning_data_t *cfg, bool cfg_is_provisioned)
{
    if (!cfg) return;
    xSemaphoreTake(s_active_lock, portMAX_DELAY);
    s_active = *cfg;
    s_active_is_provisioned = cfg_is_provisioned;
    xSemaphoreGive(s_active_lock);
}

void connection_supervisor_get_active(provisioning_data_t *out, bool *out_is_provisioned)
{
    if (!out) return;
    xSemaphoreTake(s_active_lock, portMAX_DELAY);
    *out = s_active;
    if (out_is_provisioned) *out_is_provisioned = s_active_is_provisioned;
    xSemaphoreGive(s_active_lock);
}

// =======================
// Apply / Clear command handling
// =======================

static void clear_task(void *param)
{
    (void)param;

    // Clear NVS provisioning
    storage_clear_provisioning_data();
    ble_gatt_clear_prov_buffers();

    // After clear, go back to menuconfig defaults as "active" (scenario 1 behavior).
    provisioning_data_t def = {0};
    strncpy(def.ssid,        CONFIG_RASENS_DEFAULT_SSID,           sizeof(def.ssid) - 1);
    strncpy(def.password,    CONFIG_RASENS_DEFAULT_WIFI_PASSWORD,  sizeof(def.password) - 1);
    strncpy(def.mqtt_host,   CONFIG_RASENS_MQTT_HOST,              sizeof(def.mqtt_host) - 1);
    def.mqtt_port = CONFIG_RASENS_MQTT_PORT;
    strncpy(def.access_token, CONFIG_RASENS_ACCESS_TOKEN,          sizeof(def.access_token) - 1);

    connection_supervisor_set_active(&def, false);

    // Force reconnect via supervisor (it will notice disconnected and reconnect)
    iotIs.disconnect();
    wifi_disconnect();

    status_set(DEV_IDLE, WIFI_DISCONNECTED, MQTT_DISCONNECTED, ERR_NONE);
    vTaskDelete(NULL);
}

static void apply_task(void *param)
{
    uint8_t cmd = (uint8_t)(uintptr_t)param;

    provisioning_data_t candidate = {0};
    ble_gatt_get_prov_snapshot(&candidate);

    if (!validate_prov(&candidate)) {
        status_set(DEV_ERROR, WIFI_DISCONNECTED, MQTT_DISCONNECTED, ERR_VALIDATION);
        vTaskDelete(NULL);
        return;
    }

    if (cmd == 0x03) {
        // SAVE ONLY: validate and store without connecting (per your doc)
        if (storage_save_provisioning_data(&candidate) != ESP_OK) {
            status_set(DEV_ERROR, (wifi_status_t)s_status.wifi_status, (mqtt_status_t)s_status.mqtt_status, ERR_STORAGE);
            vTaskDelete(NULL);
            return;
        }
        connection_supervisor_set_active(&candidate, true);
        status_set(DEV_PROVISIONED, (wifi_status_t)s_status.wifi_status, (mqtt_status_t)s_status.mqtt_status, ERR_NONE);
        vTaskDelete(NULL);
        return;
    }

    // APPLY: ask supervisor to test, then wait for notify
    sup_req_t req = {
        .type = REQ_TEST_CANDIDATE,
        .candidate = candidate,
        .requester = xTaskGetCurrentTaskHandle(),
    };

    // Clear previous notify value
    (void)ulTaskNotifyTake(pdTRUE, 0);

    if (xQueueSend(s_sup_req_q, &req, pdMS_TO_TICKS(500)) != pdTRUE) {
        status_set(DEV_ERROR, WIFI_DISCONNECTED, MQTT_DISCONNECTED, ERR_STORAGE);
        vTaskDelete(NULL);
        return;
    }

    uint32_t ok = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(WIFI_TRY_MS + MQTT_TRY_MS + 5000));

    if (!ok) {
        // IMPORTANT: do not overwrite NVS, do not change active
        // Supervisor already reverted runtime to old active on failure.
        status_set(DEV_ERROR, WIFI_FAILED, MQTT_FAILED, ERR_MQTT_FAILED);
        vTaskDelete(NULL);
        return;
    }

    // Candidate succeeded -> commit NVS and set active
    if (storage_save_provisioning_data(&candidate) != ESP_OK) {
        // Policy: do not brick runtime; keep connection that is already up, but signal storage error
        status_set(DEV_ERROR, WIFI_CONNECTED, MQTT_CONNECTED, ERR_STORAGE);
        vTaskDelete(NULL);
        return;
    }

    connection_supervisor_set_active(&candidate, true);
    status_set(DEV_PROVISIONED, WIFI_CONNECTED, MQTT_CONNECTED, ERR_NONE);
    vTaskDelete(NULL);
}

void provisioning_manager_on_command(uint8_t cmd)
{
    ESP_LOGI(TAG, "Command received: 0x%02X", cmd);

    if (cmd == 0x01 || cmd == 0x03) {
        xTaskCreate(apply_task, "prov_apply", 4096, (void *)(uintptr_t)cmd, 5, NULL);
        return;
    }

    if (cmd == 0x02) {
        xTaskCreate(clear_task, "prov_clear", 3072, NULL, 5, NULL);
        return;
    }

    status_set(DEV_ERROR, (wifi_status_t)s_status.wifi_status, (mqtt_status_t)s_status.mqtt_status, ERR_VALIDATION);
}
