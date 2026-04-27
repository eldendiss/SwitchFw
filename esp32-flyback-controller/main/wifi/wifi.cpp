#include "wifi.h"
#include "esp_log.h"
#include "esp_event.h"
#include "string.h"

#define TAG "wifi"
#define MAXIMUM_RETRY 5

static EventGroupHandle_t s_wifi_event_group = NULL;
static int s_retry_num = 0;

/* Persistent link state */
static volatile bool s_wifi_connected = false;
static volatile bool s_wifi_connecting = false;
static volatile bool s_eth_connected = false;

/* Track whether we've already registered handlers / created the netif so wifi_init is idempotent */
static bool s_inited = false;
static esp_event_handler_instance_t s_inst_wifi = NULL;
static esp_event_handler_instance_t s_inst_ip   = NULL;

static void set_connected_bit_if_needed(void)
{
    if (!s_wifi_event_group)
    {
        return;
    }

    if (s_wifi_connected || s_eth_connected)
    {
        xEventGroupSetBits(s_wifi_event_group, WIFI_EVT_CONNECTED_BIT);
        xEventGroupClearBits(s_wifi_event_group, WIFI_EVT_FAIL_BIT);
    }
    else
    {
        xEventGroupClearBits(s_wifi_event_group, WIFI_EVT_CONNECTED_BIT);
    }
}

static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        if (!s_wifi_connected && !s_wifi_connecting)
        {
            s_wifi_connecting = true;
            esp_err_t err = esp_wifi_connect();
            if (err != ESP_OK)
            {
                s_wifi_connecting = false;
                ESP_LOGE(TAG, "esp_wifi_connect on STA_START failed: %s", esp_err_to_name(err));
            }
        }
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        wifi_event_sta_disconnected_t *disc = (wifi_event_sta_disconnected_t *)event_data;

        s_wifi_connected = false;
        s_wifi_connecting = false;

        ESP_LOGW(TAG, "Wi-Fi disconnected, reason=%d", disc ? disc->reason : -1);

        set_connected_bit_if_needed();

        if (s_retry_num < MAXIMUM_RETRY)
        {
            s_retry_num++;

            ESP_LOGW(TAG, "retry to connect to the AP (%d/%d)", s_retry_num, MAXIMUM_RETRY);

            s_wifi_connecting = true;
            esp_err_t err = esp_wifi_connect();
            if (err != ESP_OK)
            {
                s_wifi_connecting = false;
                ESP_LOGE(TAG, "esp_wifi_connect retry failed: %s", esp_err_to_name(err));
            }
        }
        else
        {
            if (!s_eth_connected && s_wifi_event_group)
            {
                xEventGroupSetBits(s_wifi_event_group, WIFI_EVT_FAIL_BIT);
            }
            ESP_LOGW(TAG, "Wi-Fi connect failed after %d retries, Ethernet state=%s",
                     MAXIMUM_RETRY, s_eth_connected ? "up" : "down");
        }
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Wi-Fi got ip:" IPSTR, IP2STR(&event->ip_info.ip));

        s_retry_num = 0;
        s_wifi_connected = true;
        s_wifi_connecting = false;

        set_connected_bit_if_needed();
    }
}

esp_err_t wifi_init(void)
{
    if (s_inited)
    {
        /* Idempotent: just make sure event bits reflect current state */
        set_connected_bit_if_needed();
        return ESP_OK;
    }

    if (!s_wifi_event_group)
    {
        s_wifi_event_group = xEventGroupCreate();
        if (!s_wifi_event_group)
        {
            return ESP_ERR_NO_MEM;
        }
    }

    /* Create STA netif once */
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_err_t err = esp_wifi_init(&cfg);
    if (err != ESP_OK)
    {
        return err;
    }

    err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err != ESP_OK)
    {
        return err;
    }

    err = esp_event_handler_instance_register(WIFI_EVENT,
                                              ESP_EVENT_ANY_ID,
                                              &event_handler,
                                              NULL,
                                              &s_inst_wifi);
    if (err != ESP_OK)
    {
        return err;
    }

    err = esp_event_handler_instance_register(IP_EVENT,
                                              IP_EVENT_STA_GOT_IP,
                                              &event_handler,
                                              NULL,
                                              &s_inst_ip);
    if (err != ESP_OK)
    {
        return err;
    }

    /* Sync event bits with any ETH state that may have happened earlier */
    set_connected_bit_if_needed();

    err = esp_wifi_start();
    if (err == ESP_OK)
    {
        s_inited = true;
    }
    return err;
}

void eth_connected_override(void)
{
    s_eth_connected = true;
    ESP_LOGI(TAG, "Ethernet marked as connected");
    set_connected_bit_if_needed();
}

void eth_disconnected_override(void)
{
    s_eth_connected = false;
    ESP_LOGW(TAG, "Ethernet marked as disconnected");
    set_connected_bit_if_needed();
}

void wifi_reset_state(void)
{
    s_retry_num = 0;

    if (!s_wifi_event_group)
    {
        return;
    }

    /* Always clear FAIL on a fresh attempt; CONNECTED bit will be re-evaluated. */
    xEventGroupClearBits(s_wifi_event_group, WIFI_EVT_FAIL_BIT);
    set_connected_bit_if_needed();
}

bool wifi_sta_is_connected(void)
{
    return s_wifi_connected;
}

bool eth_is_connected(void)
{
    return s_eth_connected;
}

esp_err_t wifi_connect_async(const char *ssid, const char *password)
{
    if (!ssid || ssid[0] == '\0')
    {
        return ESP_ERR_INVALID_ARG;
    }

    /* If we're currently associated, drop it so the new config is honored cleanly.
     * Without this, esp_wifi_set_config + esp_wifi_connect on a different SSID
     * races with the existing association and sometimes silently keeps the old AP. */
    wifi_ap_record_t cur = {0};
    if (esp_wifi_sta_get_ap_info(&cur) == ESP_OK)
    {
        ESP_LOGI(TAG, "Disconnecting from current AP \"%s\" before switching", (const char *)cur.ssid);
        /* Suppress retry storm during the intentional disconnect by pre-arming retry counter.
         * We then reset it below before issuing the new connect. */
        s_retry_num = MAXIMUM_RETRY;
        esp_wifi_disconnect();
        /* Give the driver a moment to process the disconnect event */
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    wifi_config_t wifi_config = {0};
    strncpy((char *)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid) - 1);
    if (password)
    {
        strncpy((char *)wifi_config.sta.password,
                password,
                sizeof(wifi_config.sta.password) - 1);
    }

    esp_err_t err = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_wifi_set_config failed: %s", esp_err_to_name(err));
        return err;
    }

    /* Reset retry counter and event bits AFTER the intentional disconnect above,
     * so the upcoming connect attempt gets a fresh budget of MAXIMUM_RETRY tries. */
    wifi_reset_state();

    s_wifi_connecting = true;

    err = esp_wifi_connect();
    if (err != ESP_OK)
    {
        s_wifi_connecting = false;

        if (err == ESP_ERR_WIFI_CONN)
        {
            ESP_LOGW(TAG, "esp_wifi_connect ignored: already connecting");
            return ESP_OK;
        }

        ESP_LOGE(TAG, "esp_wifi_connect failed: %s", esp_err_to_name(err));
        return err;
    }

    return ESP_OK;
}

esp_err_t wifi_wait_connected(TickType_t ticks_to_wait)
{
    if (s_wifi_connected || s_eth_connected)
    {
        return ESP_OK;
    }

    if (!s_wifi_event_group)
    {
        return ESP_ERR_INVALID_STATE;
    }

    TickType_t start = xTaskGetTickCount();

    while (true)
    {
        if (s_wifi_connected || s_eth_connected)
        {
            return ESP_OK;
        }

        TickType_t now = xTaskGetTickCount();
        TickType_t elapsed = now - start;
        if (elapsed >= ticks_to_wait)
        {
            return ESP_ERR_TIMEOUT;
        }

        TickType_t remaining = ticks_to_wait - elapsed;

        EventBits_t bits = xEventGroupWaitBits(
            s_wifi_event_group,
            WIFI_EVT_CONNECTED_BIT | WIFI_EVT_FAIL_BIT,
            pdFALSE, // do not clear on exit
            pdFALSE, // wait for any bit
            remaining);

        if ((bits & WIFI_EVT_CONNECTED_BIT) || s_wifi_connected || s_eth_connected)
        {
            return ESP_OK;
        }

        /* If Wi-Fi failed, keep waiting until timeout because Ethernet may still come up.
         * Clear the FAIL bit so we actually block on the next iteration instead of spinning. */
        if (bits & WIFI_EVT_FAIL_BIT)
        {
            ESP_LOGW(TAG, "Wi-Fi reported failure, still waiting for Ethernet or recovery");
            xEventGroupClearBits(s_wifi_event_group, WIFI_EVT_FAIL_BIT);
        }
    }
}

bool wifi_is_connected(void)
{
    return s_wifi_connected || s_eth_connected;
}

esp_err_t wifi_get_status(wifi_ap_record_t *ap_info)
{
    if (s_wifi_connected)
    {
        return esp_wifi_sta_get_ap_info(ap_info);
    }
    return ESP_ERR_WIFI_NOT_CONNECT;
}

EventGroupHandle_t wifi_get_event_group(void)
{
    return s_wifi_event_group;
}

esp_err_t wifi_disconnect(void)
{
    s_wifi_connected = false;
    /* Suppress auto-reconnect by exhausting retry budget for this intentional disconnect. */
    s_retry_num = MAXIMUM_RETRY;
    esp_err_t err = esp_wifi_disconnect();
    /* Reset bits so a subsequent connect attempt starts clean */
    if (s_wifi_event_group)
    {
        xEventGroupClearBits(s_wifi_event_group, WIFI_EVT_CONNECTED_BIT | WIFI_EVT_FAIL_BIT);
    }
    return err;
}

esp_err_t wifi_scan(wifi_ap_record_t *ap_info, uint16_t *ap_count)
{
    if (!ap_info || !ap_count || *ap_count == 0)
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t caller_capacity = *ap_count;

    esp_err_t err = esp_wifi_scan_start(NULL, true);
    if (err != ESP_OK)
    {
        return err;
    }

    uint16_t found = 0;
    err = esp_wifi_scan_get_ap_num(&found);
    if (err != ESP_OK)
    {
        return err;
    }

    /* Cap to caller's buffer size to avoid overflow */
    uint16_t to_copy = (found < caller_capacity) ? found : caller_capacity;
    *ap_count = to_copy;

    if (to_copy == 0)
    {
        return ESP_OK;
    }

    return esp_wifi_scan_get_ap_records(ap_count, ap_info);
}