#include "provisioning.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_wifi.h"

#include <string.h>

#include "storage.h"
#include "ble_gatt_bridge.h"
#include "wifi.h"
#include "iot_is.h"

#define TAG "prov_mgr"

#define WIFI_TRY_MS         15000
#define MQTT_TRY_MS         8000
#define SUPERVISOR_LOOP_MS  2000

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
    /* Read RSSI BEFORE taking the lock — esp_wifi_sta_get_ap_info can block briefly
     * and we don't want to stall BLE notifications behind the driver. */
    int8_t rssi = 0;
    wifi_ap_record_t ap = {0};
    if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK)
    {
        rssi = (int8_t)ap.rssi;
    }

    xSemaphoreTake(s_status_lock, portMAX_DELAY);
    s_status.device_status = (uint8_t)dev;
    s_status.wifi_status   = (uint8_t)wifi;
    s_status.mqtt_status   = (uint8_t)mqtt;
    s_status.err_code      = (uint8_t)err;
    s_status.wifi_rssi     = rssi;
    status_publish_locked();
    xSemaphoreGive(s_status_lock);
}

/* Read current status fields without racing — used when we need to preserve
 * one field while updating others. */
static void status_get_snapshot(prov_status8_t *out)
{
    xSemaphoreTake(s_status_lock, portMAX_DELAY);
    *out = s_status;
    xSemaphoreGive(s_status_lock);
}

bool validate_prov(const provisioning_data_t *p)
{
    if (!p)
        return false;
    if (p->ssid[0] == '\0')
        return false;
    if (p->mqtt_host[0] == '\0')
        return false;
    if (p->mqtt_port == 0 || p->mqtt_port > 65535)
        return false;
    if (p->access_token[0] == '\0')
        return false;
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

typedef enum
{
    REQ_TEST_CANDIDATE = 1,
} req_type_t;

typedef struct
{
    req_type_t type;
    provisioning_data_t candidate;
    TaskHandle_t requester;
    bool result_ok;        /* Filled by supervisor before notifying */
    prov_err_t result_err; /* Filled by supervisor before notifying */
} sup_req_t;

static QueueHandle_t s_sup_req_q;

/* When non-zero, the supervisor's background "maintain" branch is suspended.
 * This prevents the supervisor from fighting itself during Apply/Test. */
static volatile bool s_test_in_progress = false;

// Forward
static bool supervisor_connect_with(const provisioning_data_t *cfg, bool publish_status, prov_err_t *out_err);

// =======================
// Supervisor connect logic (single owner of Wi-Fi + MQTT)
// =======================

static bool net_is_connected_now(void)
{
    return wifi_is_connected(); /* Wi-Fi OR Ethernet */
}

/* True if the currently associated AP matches the requested SSID.
 * If we're not on Wi-Fi at all (e.g., Ethernet only), returns false so that
 * a Wi-Fi switch will still be attempted when requested. */
static bool wifi_already_on_ssid(const char *ssid)
{
    if (!ssid || ssid[0] == '\0')
        return false;
    if (!wifi_sta_is_connected())
        return false;

    wifi_ap_record_t ap = {0};
    if (esp_wifi_sta_get_ap_info(&ap) != ESP_OK)
        return false;

    return strncmp((const char *)ap.ssid, ssid, sizeof(ap.ssid)) == 0;
}

static bool supervisor_connect_with(const provisioning_data_t *cfg, bool publish_status, prov_err_t *out_err)
{
    if (out_err) *out_err = ERR_NONE;

    if (!cfg || cfg->ssid[0] == '\0')
    {
        if (publish_status)
            status_set(DEV_ERROR, WIFI_FAILED, MQTT_DISCONNECTED, ERR_VALIDATION);
        if (out_err) *out_err = ERR_VALIDATION;
        return false;
    }

    /* ---- Step 1: Wi-Fi ----
     * Bring up Wi-Fi if:
     *   - we have no network at all, OR
     *   - we ARE on Wi-Fi but it's a different SSID than requested.
     * If Ethernet is up and we've never been asked to use Wi-Fi, leave it alone.
     * If Wi-Fi is on and already on the right SSID, don't churn it. */
    bool net_ok       = net_is_connected_now();
    bool on_wifi      = wifi_sta_is_connected();
    bool right_ssid   = wifi_already_on_ssid(cfg->ssid);
    bool need_wifi_up = !net_ok || (on_wifi && !right_ssid);

    if (need_wifi_up)
    {
        if (publish_status)
            status_set(DEV_APPLYING, WIFI_CONNECTING, MQTT_DISCONNECTED, ERR_NONE);

        ESP_LOGI(TAG, "Testing candidate Wi-Fi: SSID=%s", cfg->ssid);

        iotIs.disconnect();

        wifi_disconnect();
        vTaskDelay(pdMS_TO_TICKS(500));

        esp_err_t rc = wifi_connect_async(cfg->ssid, cfg->password);
        if (rc != ESP_OK)
        {
            ESP_LOGW(TAG, "candidate wifi_connect_async failed: %s", esp_err_to_name(rc));

            if (publish_status)
                status_set(DEV_ERROR, WIFI_FAILED, MQTT_DISCONNECTED, ERR_WIFI_AUTH);
            if (out_err) *out_err = ERR_WIFI_AUTH;
            return false;
        }

        esp_err_t w = wifi_wait_connected(pdMS_TO_TICKS(WIFI_TRY_MS));
        if (w != ESP_OK)
        {
            ESP_LOGW(TAG, "Network bring-up failed: %s", esp_err_to_name(w));
            prov_err_t e = (w == ESP_ERR_TIMEOUT) ? ERR_WIFI_TIMEOUT : ERR_WIFI_AUTH;
            if (publish_status)
                status_set(DEV_ERROR, WIFI_FAILED, MQTT_DISCONNECTED, e);
            if (out_err) *out_err = e;
            return false;
        }
    }
    else
    {
        ESP_LOGI(TAG, "Network already available (Wi-Fi=%s, ETH=%s), skipping Wi-Fi connect",
                 on_wifi ? "up" : "down", eth_is_connected() ? "up" : "down");
    }

    if (publish_status)
        status_set(DEV_APPLYING, WIFI_CONNECTED, MQTT_CONNECTING, ERR_NONE);

    /* ---- Step 2: MQTT ----
     * If the broker host/port/token differs from what iot_is currently has,
     * a force-reconnect is required. We can't introspect iot_is here, so the
     * cleanest contract is: on Apply we always disconnect MQTT first.
     * That happens in apply_task before queueing the request, see below.
     * Here we just connect if not already connected. */
    if (!iotIs.is_connected())
    {
        if (!iotIs.is_connecting())
        {
            ESP_LOGI(TAG, "Connecting MQTT: %s:%u", cfg->mqtt_host, cfg->mqtt_port);
            iotIs.connect(cfg->access_token, cfg->mqtt_host, cfg->mqtt_port);
        }
        else
        {
            ESP_LOGI(TAG, "MQTT connection already in progress");
        }

        int t = MQTT_TRY_MS;
        while (t > 0 && !iotIs.is_connected())
        {
            if (!net_is_connected_now())
            {
                ESP_LOGW(TAG, "Network lost while waiting for MQTT");
                break;
            }

            vTaskDelay(pdMS_TO_TICKS(100));
            t -= 100;
        }

        if (!iotIs.is_connected())
        {
            ESP_LOGW(TAG, "MQTT connect failed");

            if (publish_status)
                status_set(DEV_ERROR, WIFI_CONNECTED, MQTT_FAILED, ERR_MQTT_FAILED);
            if (out_err) *out_err = ERR_MQTT_FAILED;
            return false;
        }
    }
    else
    {
        ESP_LOGI(TAG, "MQTT already connected, skipping reconnect");
    }

    if (publish_status)
        status_set(DEV_PROVISIONED, WIFI_CONNECTED, MQTT_CONNECTED, ERR_NONE);

    return true;
}

// =======================
// Supervisor task
// =======================

static void supervisor_task(void *arg)
{
    (void)arg;

    for (;;)
    {
        // 1) Handle Apply/Test requests (non-blocking poll)
        sup_req_t req;
        while (xQueueReceive(s_sup_req_q, &req, 0) == pdTRUE)
        {
            if (req.type == REQ_TEST_CANDIDATE)
            {
                /* Block the maintain branch until we're done. */
                s_test_in_progress = true;

                provisioning_data_t old;
                bool old_is_prov;
                connection_supervisor_get_active(&old, &old_is_prov);

                prov_err_t cand_err = ERR_NONE;
                bool ok = supervisor_connect_with(&req.candidate, true, &cand_err);

                if (!ok)
                {
                    ESP_LOGI(TAG, "Candidate failed (err=%d), reverting to previous active config", cand_err);
                    /* Best-effort revert. We don't surface the revert's outcome to the requester;
                     * the maintain branch will keep trying anyway. */
                    prov_err_t revert_err = ERR_NONE;
                    (void)supervisor_connect_with(&old, true, &revert_err);
                }

                /* Always notify the requester (success or failure) so apply_task
                 * doesn't need to wait the full timeout on failure. */
                if (req.requester)
                {
                    /* Stash result on a small static slot keyed by requester.
                     * Simpler approach: piggyback through queue is overkill;
                     * we use a notify value to encode ok/err. */
                    uint32_t notify_val = ok
                        ? (uint32_t)1
                        : (uint32_t)(0x80000000u | (uint32_t)cand_err);
                    xTaskNotify(req.requester, notify_val, eSetValueWithOverwrite);
                }

                s_test_in_progress = false;
            }
        }

        // 2) Background maintain — but never while a test is in progress.
        if (!s_test_in_progress)
        {
            provisioning_data_t cfg;
            xSemaphoreTake(s_active_lock, portMAX_DELAY);
            cfg = s_active;
            xSemaphoreGive(s_active_lock);

            /* Only maintain if we have something usable to maintain. */
            if (cfg.ssid[0] != '\0')
            {
                bool net_ok    = net_is_connected_now();
                bool mqtt_ok   = iotIs.is_connected();
                bool mqtt_busy = iotIs.is_connecting();

                if (!net_ok)
                {
                    status_set(DEV_IDLE, WIFI_DISCONNECTED, MQTT_DISCONNECTED, ERR_NONE);
                    (void)supervisor_connect_with(&cfg, true, NULL);
                }
                else if (!mqtt_ok && !mqtt_busy)
                {
                    status_set(DEV_IDLE, WIFI_CONNECTED, MQTT_DISCONNECTED, ERR_NONE);
                    (void)supervisor_connect_with(&cfg, true, NULL);
                }
            }
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
    if (!s_status_lock)
        return ESP_ERR_NO_MEM;

    s_active_lock = xSemaphoreCreateMutex();
    if (!s_active_lock)
        return ESP_ERR_NO_MEM;

    memset(&s_status, 0, sizeof(s_status));
    s_status.device_status = DEV_IDLE;
    s_status.wifi_status   = WIFI_DISCONNECTED;
    s_status.mqtt_status   = MQTT_DISCONNECTED;
    s_status.err_code      = ERR_NONE;
    s_status.wifi_rssi     = 0;

    xSemaphoreTake(s_status_lock, portMAX_DELAY);
    status_publish_locked();
    xSemaphoreGive(s_status_lock);

    s_sup_req_q = xQueueCreate(4, sizeof(sup_req_t));
    if (!s_sup_req_q)
        return ESP_ERR_NO_MEM;

    return ESP_OK;
}

esp_err_t connection_supervisor_init(void)
{
    if (!s_sup_req_q)
        return ESP_ERR_INVALID_STATE;
    BaseType_t ok = xTaskCreate(supervisor_task, "conn_sup", 4096, NULL, 6, NULL);
    return ok == pdPASS ? ESP_OK : ESP_FAIL;
}

void connection_supervisor_set_active(const provisioning_data_t *cfg, bool cfg_is_provisioned)
{
    if (!cfg)
        return;
    xSemaphoreTake(s_active_lock, portMAX_DELAY);
    s_active = *cfg;
    s_active_is_provisioned = cfg_is_provisioned;
    xSemaphoreGive(s_active_lock);
}

void connection_supervisor_get_active(provisioning_data_t *out, bool *out_is_provisioned)
{
    if (!out)
        return;
    xSemaphoreTake(s_active_lock, portMAX_DELAY);
    *out = s_active;
    if (out_is_provisioned)
        *out_is_provisioned = s_active_is_provisioned;
    xSemaphoreGive(s_active_lock);
}

// =======================
// Apply / Clear command handling
// =======================

static void clear_task(void *param)
{
    (void)param;

    storage_clear_provisioning_data();
    ble_gatt_clear_prov_buffers();

    provisioning_data_t def = {0};
    strncpy(def.ssid,         CONFIG_RASENS_DEFAULT_SSID,          sizeof(def.ssid) - 1);
    strncpy(def.password,     CONFIG_RASENS_DEFAULT_WIFI_PASSWORD, sizeof(def.password) - 1);
    strncpy(def.mqtt_host,    CONFIG_RASENS_MQTT_HOST,             sizeof(def.mqtt_host) - 1);
    def.mqtt_port = CONFIG_RASENS_MQTT_PORT;
    strncpy(def.access_token, CONFIG_RASENS_ACCESS_TOKEN,          sizeof(def.access_token) - 1);

    connection_supervisor_set_active(&def, false);

    /* Tear down current connections so the supervisor's maintain branch
     * (or a subsequent Apply) brings up the defaults cleanly. */
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

    if (!validate_prov(&candidate))
    {
        status_set(DEV_ERROR, WIFI_DISCONNECTED, MQTT_DISCONNECTED, ERR_VALIDATION);
        vTaskDelete(NULL);
        return;
    }

    if (cmd == 0x03)
    {
        /* SAVE ONLY — preserve current wifi/mqtt status fields */
        prov_status8_t snap;
        status_get_snapshot(&snap);

        if (storage_save_provisioning_data(&candidate) != ESP_OK)
        {
            status_set(DEV_ERROR,
                       (wifi_status_t)snap.wifi_status,
                       (mqtt_status_t)snap.mqtt_status,
                       ERR_STORAGE);
            vTaskDelete(NULL);
            return;
        }
        connection_supervisor_set_active(&candidate, true);
        status_set(DEV_PROVISIONED,
                   (wifi_status_t)snap.wifi_status,
                   (mqtt_status_t)snap.mqtt_status,
                   ERR_NONE);
        vTaskDelete(NULL);
        return;
    }

    /* APPLY (cmd == 0x01)
     *
     * Critical: force MQTT down so the supervisor will use the candidate's broker.
     * If the candidate's SSID differs from the current AP, wifi_connect_async()
     * inside the supervisor will switch APs. If it's the same SSID, Wi-Fi is left
     * alone (no needless churn).
     *
     * We do NOT pre-disconnect Wi-Fi here — supervisor_connect_with() decides that
     * based on SSID match, which avoids dropping Wi-Fi unnecessarily. */
    iotIs.disconnect();

    sup_req_t req = {
        .type      = REQ_TEST_CANDIDATE,
        .candidate = candidate,
        .requester = xTaskGetCurrentTaskHandle(),
        .result_ok = false,
        .result_err = ERR_NONE,
    };

    /* Drain any stale notify value */
    uint32_t dummy;
    while (xTaskNotifyWait(0, ULONG_MAX, &dummy, 0) == pdTRUE) { }

    if (xQueueSend(s_sup_req_q, &req, pdMS_TO_TICKS(500)) != pdTRUE)
    {
        status_set(DEV_ERROR, WIFI_DISCONNECTED, MQTT_DISCONNECTED, ERR_STORAGE);
        vTaskDelete(NULL);
        return;
    }

    /* Wait long enough to cover supervisor poll latency + Wi-Fi try + MQTT try + slack */
    uint32_t notify_val = 0;
    BaseType_t got = xTaskNotifyWait(0, ULONG_MAX, &notify_val,
                                     pdMS_TO_TICKS(SUPERVISOR_LOOP_MS + WIFI_TRY_MS + MQTT_TRY_MS + 5000));

    if (got != pdTRUE)
    {
        /* Supervisor never replied — should not happen, but be safe. */
        ESP_LOGE(TAG, "Apply timed out waiting for supervisor");
        status_set(DEV_ERROR, WIFI_FAILED, MQTT_FAILED, ERR_MQTT_FAILED);
        vTaskDelete(NULL);
        return;
    }

    bool ok = (notify_val == 1);

    if (!ok)
    {
        /* Decode error from notify high bit */
        prov_err_t err = (prov_err_t)(notify_val & 0x7FFFFFFFu);
        ESP_LOGW(TAG, "Apply failed: err=%d (supervisor reverted to previous config)", err);

        /* Map err to wifi/mqtt status fields for the BLE update */
        wifi_status_t ws = WIFI_FAILED;
        mqtt_status_t ms = MQTT_DISCONNECTED;
        if (err == ERR_MQTT_FAILED)
        {
            ws = WIFI_CONNECTED;
            ms = MQTT_FAILED;
        }
        status_set(DEV_ERROR, ws, ms, err);
        vTaskDelete(NULL);
        return;
    }

    /* Candidate succeeded -> commit NVS and set active */
    if (storage_save_provisioning_data(&candidate) != ESP_OK)
    {
        /* Connection is up; flag storage error but don't tear down. */
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

    if (cmd == 0x01 || cmd == 0x03)
    {
        xTaskCreate(apply_task, "prov_apply", 4096, (void *)(uintptr_t)cmd, 5, NULL);
        return;
    }

    if (cmd == 0x02)
    {
        xTaskCreate(clear_task, "prov_clear", 3072, NULL, 5, NULL);
        return;
    }

    prov_status8_t snap;
    status_get_snapshot(&snap);
    status_set(DEV_ERROR,
               (wifi_status_t)snap.wifi_status,
               (mqtt_status_t)snap.mqtt_status,
               ERR_VALIDATION);
}