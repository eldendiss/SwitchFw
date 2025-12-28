#include "wifi.h"
#include "esp_log.h"
#include "esp_event.h"
#include "string.h"

#define TAG "wifi"

#define MAXIMUM_RETRY 5
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1
static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;

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
        if (s_retry_num < MAXIMUM_RETRY)
        {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGW(TAG, "retry to connect to the AP");
        }
        else
        {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGE(TAG, "connect to the AP fail");
    }
    // Successfully obtained IP - update event group
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

esp_err_t wifi_init(void)
{
    // Create FreeRTOS event group to signal Wi-Fi events
    s_wifi_event_group = xEventGroupCreate();

    // Initialize the TCP/IP stack
    ESP_ERROR_CHECK(esp_netif_init());

    // Create default event loop
    ESP_ERROR_CHECK(esp_event_loop_create_default());

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

    // Start WiFi
    err = esp_wifi_start();

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
    return err;
}

esp_err_t wifi_connect(const char *ssid, const char *password)
{
    wifi_config_t wifi_config = {};
    strncpy((char *)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char *)wifi_config.sta.password, password, sizeof(wifi_config.sta.password) - 1);
    // Set WiFi configuration
    esp_err_t err = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    if (err != ESP_OK)
    {
        return err;
    }

    // Connect to WiFi
    err = esp_wifi_connect();

    xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);
    return err;
}

esp_err_t wifi_get_status(wifi_ap_record_t *ap_info)
{
    return esp_wifi_sta_get_ap_info(ap_info);
}

esp_err_t wifi_disconnect(void)
{
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