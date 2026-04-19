#include "wifi.h"
#include "esp_log.h"
#include "esp_event.h"
#include "string.h"

#define TAG "wifi"

#define MAXIMUM_RETRY 5

static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;
static bool eth_connected = false;

static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    // Station started - initiate connection
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    // Disconnection event - retry if under maximum attempts
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        if (s_retry_num < MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGW(TAG, "retry to connect to the AP (%d/%d)", s_retry_num, MAXIMUM_RETRY);
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_EVT_FAIL_BIT);
        }
        ESP_LOGE(TAG, "connect to the AP fail");
    }
    // Successfully obtained IP - update event group
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_EVT_CONNECTED_BIT);
        return;
    }
}

esp_err_t wifi_init(void)
{
    // Create FreeRTOS event group to signal Wi-Fi events
    s_wifi_event_group = xEventGroupCreate();
    if (!s_wifi_event_group) return ESP_ERR_NO_MEM;

    // Create default network interface for station mode
    esp_netif_create_default_wifi_sta();

    // Initialize WiFi with default configuration
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_err_t err = esp_wifi_init(&cfg);
    if (err != ESP_OK)
    {
        return err;
    }

    // Set WiFi mode to STA (station)
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

    
    // Start WiFi
    err = esp_wifi_start();
    return err;
}

void eth_connected_override(void)
{
    // Set connected bit in event group to bypass WiFi connection logic
    xEventGroupSetBits(s_wifi_event_group, WIFI_EVT_CONNECTED_BIT);
    eth_connected = true;
}

void eth_disconnected_override(void)
{
    eth_connected = false;
}


void wifi_reset_state(void)
{
    s_retry_num = 0;
    xEventGroupClearBits(s_wifi_event_group, WIFI_EVT_CONNECTED_BIT | WIFI_EVT_FAIL_BIT);
}

esp_err_t wifi_connect_async(const char *ssid, const char *password)
{
    if (!ssid) return ESP_ERR_INVALID_ARG;

    wifi_config_t wifi_config = {0};
    strncpy((char *)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid) - 1);
    if (password) {
        strncpy((char *)wifi_config.sta.password, password, sizeof(wifi_config.sta.password) - 1);
    }

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));

    wifi_reset_state();

    // Trigger connect
    return esp_wifi_connect();
}

esp_err_t wifi_wait_connected(TickType_t ticks_to_wait)
{
    EventBits_t bits = xEventGroupWaitBits(
        s_wifi_event_group,
        WIFI_EVT_CONNECTED_BIT | WIFI_EVT_FAIL_BIT,
        pdTRUE,   // clear on exit
        pdFALSE,  // wait any bit
        ticks_to_wait
    );

    if (bits & WIFI_EVT_CONNECTED_BIT) return ESP_OK;
    if (bits & WIFI_EVT_FAIL_BIT) return ESP_FAIL;
    return ESP_ERR_TIMEOUT;
}

bool wifi_is_connected()
{
    if (eth_connected) {
        return true;
    }
    EventBits_t bits = xEventGroupGetBits(s_wifi_event_group);
    return (bits & WIFI_EVT_CONNECTED_BIT) != 0;
}

esp_err_t wifi_get_status(wifi_ap_record_t *ap_info)
{
    return esp_wifi_sta_get_ap_info(ap_info);
}

EventGroupHandle_t wifi_get_event_group(void)
{
    return s_wifi_event_group;
}


esp_err_t wifi_disconnect(void)
{
    wifi_reset_state();
    return esp_wifi_disconnect();
}

esp_err_t wifi_scan(wifi_ap_record_t *ap_info, uint16_t *ap_count)
{
    esp_err_t err;

    // Start WiFi scan
    err = esp_wifi_scan_start(NULL, true);
    if (err != ESP_OK)
    {
        return err;
    }

    // Get number of APs found
    err = esp_wifi_scan_get_ap_num(ap_count);
    if (err != ESP_OK)
    {
        return err;
    }

    // Get list of APs found
    err = esp_wifi_scan_get_ap_records(ap_count, ap_info);
    return err;
}