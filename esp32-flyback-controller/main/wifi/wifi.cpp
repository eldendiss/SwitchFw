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
static volatile bool s_eth_connected = false;

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
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        s_wifi_connected = false;
        set_connected_bit_if_needed();

        if (s_retry_num < MAXIMUM_RETRY)
        {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGW(TAG, "retry to connect to the AP (%d/%d)", s_retry_num, MAXIMUM_RETRY);
        }
        else
        {
            /* Wi-Fi failed, but that must not be fatal if Ethernet is up */
            if (!s_eth_connected && s_wifi_event_group)
            {
                xEventGroupSetBits(s_wifi_event_group, WIFI_EVT_FAIL_BIT);
            }
            ESP_LOGW(TAG, "Wi-Fi connect failed, Ethernet state=%s",
                     s_eth_connected ? "up" : "down");
        }
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Wi-Fi got ip:" IPSTR, IP2STR(&event->ip_info.ip));

        s_retry_num = 0;
        s_wifi_connected = true;
        set_connected_bit_if_needed();
    }
}

esp_err_t wifi_init(void)
{
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

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;

    err = esp_event_handler_instance_register(WIFI_EVENT,
                                              ESP_EVENT_ANY_ID,
                                              &event_handler,
                                              NULL,
                                              &instance_any_id);
    if (err != ESP_OK)
    {
        return err;
    }

    err = esp_event_handler_instance_register(IP_EVENT,
                                              IP_EVENT_STA_GOT_IP,
                                              &event_handler,
                                              NULL,
                                              &instance_got_ip);
    if (err != ESP_OK)
    {
        return err;
    }

    /* Sync event bits with any ETH state that may have happened earlier */
    set_connected_bit_if_needed();

    return esp_wifi_start();
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
    if (!ssid)
    {
        return ESP_ERR_INVALID_ARG;
    }

    wifi_config_t wifi_config = {0};
    strncpy((char *)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid) - 1);

    if (password)
    {
        strncpy((char *)wifi_config.sta.password, password, sizeof(wifi_config.sta.password) - 1);
    }

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));

    wifi_reset_state();

    return esp_wifi_connect();
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

        /* If Wi-Fi failed, keep waiting until timeout because Ethernet may still come up */
        if (bits & WIFI_EVT_FAIL_BIT)
        {
            ESP_LOGW(TAG, "Wi-Fi reported failure, still waiting for Ethernet or later recovery");
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
    wifi_reset_state();
    return esp_wifi_disconnect();
}

esp_err_t wifi_scan(wifi_ap_record_t *ap_info, uint16_t *ap_count)
{
    esp_err_t err;

    err = esp_wifi_scan_start(NULL, true);
    if (err != ESP_OK)
    {
        return err;
    }

    err = esp_wifi_scan_get_ap_num(ap_count);
    if (err != ESP_OK)
    {
        return err;
    }

    err = esp_wifi_scan_get_ap_records(ap_count, ap_info);
    return err;
}