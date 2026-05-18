#include "LAN.h"
#include "esp_check.h"

static const char *TAG = "LAN";
static void eth_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
static void got_ip_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
static got_ip_callback_t got_ip_callback = NULL;
static got_ip_callback_t down_callback = NULL;

static esp_err_t parse_ipv4_string(const char *str, esp_ip4_addr_t *out)
{
    unsigned int a, b, c, d;
    if (!str || !out) {
        return ESP_ERR_INVALID_ARG;
    }

    if (sscanf(str, "%u.%u.%u.%u", &a, &b, &c, &d) != 4) {
        return ESP_ERR_INVALID_ARG;
    }

    if (a > 255 || b > 255 || c > 255 || d > 255) {
        return ESP_ERR_INVALID_ARG;
    }

    out->addr = ESP_IP4TOADDR(a, b, c, d);
    return ESP_OK;
}

static esp_err_t build_static_ip_from_base(const char *base, uint8_t last_octet, esp_ip4_addr_t *out)
{
    char buf[32];

    if (!base || !out) {
        return ESP_ERR_INVALID_ARG;
    }

    int n = snprintf(buf, sizeof(buf), "%s%u", base, last_octet);
    if (n <= 0 || n >= (int)sizeof(buf)) {
        return ESP_ERR_INVALID_ARG;
    }

    return parse_ipv4_string(buf, out);
}

esp_err_t LAN_init(){

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_inherent_config_t esp_netif_config = ESP_NETIF_INHERENT_DEFAULT_ETH();
    esp_netif_config_t cfg_spi = {
        .base = &esp_netif_config,
        .stack = ESP_NETIF_NETSTACK_DEFAULT_ETH
    };
    esp_netif_t *eth_netif_spi =  NULL;

    esp_netif_config.if_key = "ETH_SPI_0";
    esp_netif_config.if_desc = "eth0";
    esp_netif_config.route_prio = 30;
    eth_netif_spi = esp_netif_new(&cfg_spi);
    ESP_RETURN_ON_FALSE(eth_netif_spi != NULL, ESP_FAIL, TAG, "esp_netif_new failed");

#ifdef CONFIG_RASENS_DHCP
    ESP_LOGI(TAG, "Ethernet set to DHCP");
#else
    ESP_LOGI(TAG, "Ethernet set to static IP");

    esp_netif_ip_info_t ip_info = {0};

    uint8_t base_mac[6];
    ESP_ERROR_CHECK(esp_read_mac(base_mac, ESP_MAC_BT));

    uint8_t last_octet = base_mac[5];
    if (last_octet == 0 || last_octet == 255) {
        last_octet = 100;
    }

    ESP_ERROR_CHECK(build_static_ip_from_base(CONFIG_RASENS_STATIC_IP_BASE, last_octet, &ip_info.ip));
    ESP_ERROR_CHECK(parse_ipv4_string(CONFIG_RASENS_GATEWAY, &ip_info.gw));
    ESP_ERROR_CHECK(parse_ipv4_string(CONFIG_RASENS_NETMASK, &ip_info.netmask));

    ESP_LOGI(TAG, "Static IP config:");
    ESP_LOGI(TAG, "  IP: " IPSTR, IP2STR(&ip_info.ip));
    ESP_LOGI(TAG, "  GW: " IPSTR, IP2STR(&ip_info.gw));
    ESP_LOGI(TAG, "  NM: " IPSTR, IP2STR(&ip_info.netmask));

    ESP_ERROR_CHECK(esp_netif_dhcpc_stop(eth_netif_spi));
    ESP_ERROR_CHECK(esp_netif_set_ip_info(eth_netif_spi, &ip_info));
#endif

    // Init MAC and PHY configs to default
    eth_mac_config_t mac_config_spi = ETH_MAC_DEFAULT_CONFIG();
    eth_phy_config_t phy_config_spi = ETH_PHY_DEFAULT_CONFIG();

    // Install GPIO ISR handler to be able to service SPI Eth modlues interrupts
    gpio_install_isr_service(0);

    // Init SPI bus
    spi_device_handle_t spi_handle = NULL;
    spi_bus_config_t buscfg = {
        .mosi_io_num = MOSI,
        .miso_io_num = MISO,
        .sclk_io_num = SCK
        //.max_transfer_sz = 9192
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH1));
   

    // Configure SPI interface and Ethernet driver for specific SPI module
    esp_eth_mac_t *mac_spi;
    esp_eth_phy_t *phy_spi;
    esp_eth_handle_t eth_handle_spi = NULL;
    spi_device_interface_config_t devcfg = {
        .command_bits = 16, // Actually it's the address phase in W5500 SPI frame
        .address_bits = 8,  // Actually it's the control phase in W5500 SPI frame
        .mode = 0,
        .clock_speed_hz = 20 * 1000 * 1000,
        .queue_size = 20
    };

    // Set SPI module Chip Select GPIO
    devcfg.spics_io_num = CS;

    ESP_ERROR_CHECK(spi_bus_add_device(SPI3_HOST, &devcfg, &spi_handle));
    // w5500 ethernet driver is based on spi driver
    eth_w5500_config_t w5500_config = ETH_W5500_DEFAULT_CONFIG(SPI3_HOST,&devcfg);

    // Set remaining GPIO numbers and configuration used by the SPI module
    w5500_config.int_gpio_num = INT;
    phy_config_spi.phy_addr = -1;
    phy_config_spi.reset_gpio_num = RST;

    mac_spi = esp_eth_mac_new_w5500(&w5500_config, &mac_config_spi);
    phy_spi = esp_eth_phy_new_w5500(&phy_config_spi);

    esp_eth_config_t eth_config_spi = ETH_DEFAULT_CONFIG(mac_spi, phy_spi);
    ESP_RETURN_ON_ERROR(esp_eth_driver_install(&eth_config_spi, &eth_handle_spi), TAG, "esp_eth_driver_install failed");

    /* The SPI Ethernet module might not have a burned factory MAC address, we cat to set it manually.
    02:00:00 is a Locally Administered OUI range so should not be used except when testing on a LAN under your control.
     */
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_ETH);
    if (mac[5] == 0xFF) {
        mac[5] = 0x00;
    } else {
        mac[5] += 1;
    }
    ESP_RETURN_ON_ERROR(esp_eth_ioctl(eth_handle_spi, ETH_CMD_S_MAC_ADDR, mac),TAG, "esp_eth_ioctl failed");

    // attach Ethernet driver to TCP/IP stack
    ESP_RETURN_ON_ERROR(esp_netif_attach(eth_netif_spi, esp_eth_new_netif_glue(eth_handle_spi)), TAG, "esp_netif_attach failed");

    // Register user defined event handers
    ESP_RETURN_ON_ERROR(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL), TAG, "esp_event_handler_register failed");
    ESP_RETURN_ON_ERROR(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &got_ip_event_handler, NULL), TAG, "esp_event_handler_register failed");


    return esp_eth_start(eth_handle_spi);
}

void register_got_ip_callback(got_ip_callback_t callback){
    got_ip_callback = callback;
}

void register_down_callback(got_ip_callback_t callback){
    down_callback = callback;
}

void eth_event_handler(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data)
{
    uint8_t mac_addr[6] = {0};
    /* we can get the ethernet driver handle from event data */
    esp_eth_handle_t eth_handle = *(esp_eth_handle_t *)event_data;

    switch (event_id) {
    case ETHERNET_EVENT_CONNECTED:
        esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR, mac_addr);
        ESP_LOGI(TAG, "Ethernet Link Up");
        ESP_LOGD(TAG, "Ethernet HW Addr %02x:%02x:%02x:%02x:%02x:%02x",
                 mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
        break;
    case ETHERNET_EVENT_DISCONNECTED:
        ESP_LOGE(TAG, "Ethernet Link Down");
        if (down_callback) {
            down_callback();
        }
        break;
    case ETHERNET_EVENT_START:
        ESP_LOGI(TAG, "Ethernet Started");
        break;
    case ETHERNET_EVENT_STOP:
        ESP_LOGE(TAG, "Ethernet Stopped");
        break;
    default:
        break;
    }
}

void got_ip_event_handler(void *arg, esp_event_base_t event_base,
                                 int32_t event_id, void *event_data)
{
    ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
    const esp_netif_ip_info_t *ip_info = &event->ip_info;

    ESP_LOGI(TAG, "Ethernet Got IP Address");
    ESP_LOGI(TAG, "~~~~~~~~~~~");
    ESP_LOGI(TAG, "ETHIP:" IPSTR, IP2STR(&ip_info->ip));
    ESP_LOGI(TAG, "ETHMASK:" IPSTR, IP2STR(&ip_info->netmask));
    ESP_LOGI(TAG, "ETHGW:" IPSTR, IP2STR(&ip_info->gw));
    ESP_LOGI(TAG, "~~~~~~~~~~~");

    
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_BT);
    //create string "Rasens-XXXXXX" where XXXXXX are last 3 bytes of MAC
    char hostname[32];
    snprintf(hostname, sizeof(hostname), "RaSens-%02X%02X%02X", mac[3], mac[4], mac[5]);
    ESP_LOGI(TAG, "Setting mDNS hostname %s.local", hostname);
    static bool s_mdns_initialized = false;
    if (!s_mdns_initialized) {
        ESP_ERROR_CHECK(mdns_init());
        s_mdns_initialized = true;
    }
    mdns_hostname_set(hostname);
    mdns_instance_name_set(hostname);

    if (got_ip_callback) {
        got_ip_callback();
    }
}