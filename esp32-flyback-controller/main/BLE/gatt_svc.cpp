#include "ble.h"
#include "esp_log.h"
#include "storage.h"
#include "ble_gatt_bridge.h"

#define TAG "GATT_SVC"

/* Private function declarations */
static int chr_access(uint16_t conn_handle, uint16_t attr_handle,
                      struct ble_gatt_access_ctxt *ctxt, void *arg);

/* Private variables */
/**************************
 * Provisioning service
 **************************/
static const ble_uuid128_t provisioning_svc_uuid = BLE_UUID128_INIT(0x10, 0x00, 0x4e, 0x36, 0xa1, 0x8c, 0x18, 0x98, 0xf1, 0x49, 0x07, 0xdd, 0xa7, 0x43, 0x4d, 0x70);
/* SSID characteristic */
static char ssid_chr_val[32] = {0};
static uint16_t ssid_chr_val_handle;
static const ble_uuid128_t ssid_chr_uuid = BLE_UUID128_INIT(0x11, 0x00, 0x4e, 0x36, 0xa1, 0x8c, 0x18, 0x98, 0xf1, 0x49, 0x07, 0xdd, 0xa7, 0x43, 0x4d, 0x70);

/* Password characteristic */
static char password_chr_val[64] = {0};
static uint16_t password_chr_val_handle;
static const ble_uuid128_t password_chr_uuid = BLE_UUID128_INIT(0x12, 0x00, 0x4e, 0x36, 0xa1, 0x8c, 0x18, 0x98, 0xf1, 0x49, 0x07, 0xdd, 0xa7, 0x43, 0x4d, 0x70);
/* MQTT server characteristic */
static char mqtt_server_chr_val[64] = {0};
static uint16_t mqtt_server_chr_val_handle;
static const ble_uuid128_t mqtt_server_chr_uuid = BLE_UUID128_INIT(0x13, 0x00, 0x4e, 0x36, 0xa1, 0x8c, 0x18, 0x98, 0xf1, 0x49, 0x07, 0xdd, 0xa7, 0x43, 0x4d, 0x70);
/* MQTT port characteristic */
static uint8_t mqtt_port_chr_val[2] = {0};
static uint16_t mqtt_port_chr_val_handle;
static const ble_uuid128_t mqtt_port_chr_uuid = BLE_UUID128_INIT(0x14, 0x00, 0x4e, 0x36, 0xa1, 0x8c, 0x18, 0x98, 0xf1, 0x49, 0x07, 0xdd, 0xa7, 0x43, 0x4d, 0x70);

/* Access token characteristic */
static uint8_t access_token_chr_val[25] = {0};
static uint16_t access_token_chr_val_handle;
static const ble_uuid128_t access_token_chr_uuid = BLE_UUID128_INIT(0x15, 0x00, 0x4e, 0x36, 0xa1, 0x8c, 0x18, 0x98, 0xf1, 0x49, 0x07, 0xdd, 0xa7, 0x43, 0x4d, 0x70);

/* Command characteristic */
static uint8_t command_chr_val[1] = {0};
static uint16_t command_chr_val_handle;
static const ble_uuid128_t command_chr_uuid = BLE_UUID128_INIT(0x16, 0x00, 0x4e, 0x36, 0xa1, 0x8c, 0x18, 0x98, 0xf1, 0x49, 0x07, 0xdd, 0xa7, 0x43, 0x4d, 0x70);
/* Status characteristic */
static uint8_t status_chr_val[8] = {0};
static uint16_t status_chr_val_handle;
static uint16_t status_chr_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static bool status_chr_conn_handle_inited = false;
static uint8_t status_ind_status = false;
static const ble_uuid128_t status_chr_uuid = BLE_UUID128_INIT(0x17, 0x00, 0x4e, 0x36, 0xa1, 0x8c, 0x18, 0x98, 0xf1, 0x49, 0x07, 0xdd, 0xa7, 0x43, 0x4d, 0x70);

/************************
 * Configuration service
 ************************/
static const ble_uuid128_t config_svc_uuid = BLE_UUID128_INIT(0x20, 0x00, 0x4e, 0x36, 0xa1, 0x8c, 0x18, 0x98, 0xf1, 0x49, 0x07, 0xdd, 0xa7, 0x43, 0x4d, 0x70);
/* Interval characteristic */
static uint8_t interval_chr_val[4] = {0};
static uint16_t interval_chr_val_handle;
static const ble_uuid128_t interval_chr_uuid = BLE_UUID128_INIT(0x21, 0x00, 0x4e, 0x36, 0xa1, 0x8c, 0x18, 0x98, 0xf1, 0x49, 0x07, 0xdd, 0xa7, 0x43, 0x4d, 0x70);
/* Range characteristic */
static uint8_t range_chr_val[1] = {0};
static uint16_t range_chr_val_handle;
static const ble_uuid128_t range_chr_uuid = BLE_UUID128_INIT(0x22, 0x00, 0x4e, 0x36, 0xa1, 0x8c, 0x18, 0x98, 0xf1, 0x49, 0x07, 0xdd, 0xa7, 0x43, 0x4d, 0x70);
/* Voltage characteristic */
static uint8_t voltage_chr_val[8] = {0};
static uint16_t voltage_chr_val_handle;
static const ble_uuid128_t voltage_chr_uuid = BLE_UUID128_INIT(0x23, 0x00, 0x4e, 0x36, 0xa1, 0x8c, 0x18, 0x98, 0xf1, 0x49, 0x07, 0xdd, 0xa7, 0x43, 0x4d, 0x70);

/* Coefficient characteristic */
static uint8_t coeff_chr_val[16] = {0};
static uint16_t coeff_chr_val_handle;
static const ble_uuid128_t coeff_chr_uuid = BLE_UUID128_INIT(0x24, 0x00, 0x4e, 0x36, 0xa1, 0x8c, 0x18, 0x98, 0xf1, 0x49, 0x07, 0xdd, 0xa7, 0x43, 0x4d, 0x70);

/* GATT services table */
static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    /* Provisioning service */
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &provisioning_svc_uuid.u,
        .characteristics =
            (struct ble_gatt_chr_def[]){
                {/* SSID characteristic */
                 .uuid = &ssid_chr_uuid.u,
                 .access_cb = chr_access,
                 .flags = BLE_GATT_CHR_F_WRITE,
                 .val_handle = &ssid_chr_val_handle},
                {/* Password characteristic */
                 .uuid = &password_chr_uuid.u,
                 .access_cb = chr_access,
                 .flags = BLE_GATT_CHR_F_WRITE,
                 .val_handle = &password_chr_val_handle},
                {/* MQTT server characteristic */
                 .uuid = &mqtt_server_chr_uuid.u,
                 .access_cb = chr_access,
                 .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_READ,
                 .val_handle = &mqtt_server_chr_val_handle},
                {/* MQTT port characteristic */
                 .uuid = &mqtt_port_chr_uuid.u,
                 .access_cb = chr_access,
                 .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_READ,
                 .val_handle = &mqtt_port_chr_val_handle},
                {/* Access Token characteristic */
                 .uuid = &access_token_chr_uuid.u,
                 .access_cb = chr_access,
                 .flags = BLE_GATT_CHR_F_WRITE,
                 .val_handle = &access_token_chr_val_handle},
                {/* Command characteristic */
                 .uuid = &command_chr_uuid.u,
                 .access_cb = chr_access,
                 .flags = BLE_GATT_CHR_F_WRITE,
                 .val_handle = &command_chr_val_handle},
                {/* Status characteristic */
                 .uuid = &status_chr_uuid.u,
                 .access_cb = chr_access,
                 .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                 .val_handle = &status_chr_val_handle},
                {
                    0,
                }}},

    /* Config service */
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &config_svc_uuid.u,
        .characteristics =
            (struct ble_gatt_chr_def[]){
                /* Interval characteristic */
                {.uuid = &interval_chr_uuid.u,
                 .access_cb = chr_access,
                 .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_READ,
                 .val_handle = &interval_chr_val_handle},
                /* Range characteristic */
                {.uuid = &range_chr_uuid.u,
                 .access_cb = chr_access,
                 .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_READ,
                 .val_handle = &range_chr_val_handle},
                /* Voltage characteristic */
                {.uuid = &voltage_chr_uuid.u,
                 .access_cb = chr_access,
                 .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_READ,
                 .val_handle = &voltage_chr_val_handle},
                /* Coeff characteristic */
                {.uuid = &coeff_chr_uuid.u,
                 .access_cb = chr_access,
                 .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_READ,
                 .val_handle = &coeff_chr_val_handle},
                {0}},
    },

    {
        0, /* No more services. */
    },
};

/* Private functions */
static int chr_access(uint16_t conn_handle, uint16_t attr_handle,
                      struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    /* Local variables */
    int rc = 0;

    /* Handle access events */
    /* Note: Heart rate characteristic is read only */
    switch (ctxt->op)
    {

    /* Read characteristic event */
    case BLE_GATT_ACCESS_OP_READ_CHR:
        if (conn_handle != BLE_HS_CONN_HANDLE_NONE)
        {
            ESP_LOGI(TAG, "characteristic read; conn_handle=%d attr_handle=%d",
                     conn_handle, attr_handle);
        }
        else
        {
            ESP_LOGI(TAG, "characteristic read by nimble stack; attr_handle=%d",
                     attr_handle);
        }

        if (attr_handle == mqtt_port_chr_val_handle)
        {
            rc = os_mbuf_append(ctxt->om, mqtt_port_chr_val,
                                sizeof(mqtt_port_chr_val));
            ESP_LOG_BUFFER_HEX(TAG, mqtt_port_chr_val, sizeof(mqtt_port_chr_val));
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }

        if (attr_handle == mqtt_server_chr_val_handle)
        {
            rc = os_mbuf_append(ctxt->om, mqtt_server_chr_val,
                                strlen(mqtt_server_chr_val) + 1);
            ESP_LOG_BUFFER_HEX(TAG, mqtt_server_chr_val, strlen(mqtt_server_chr_val) + 1);
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }

        if (attr_handle == status_chr_val_handle)
        {            
            rc = os_mbuf_append(ctxt->om, status_chr_val,
                                sizeof(status_chr_val));
            ESP_LOG_BUFFER_HEX(TAG, status_chr_val, sizeof(status_chr_val));
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }

        if (attr_handle == interval_chr_val_handle)
        {
            rc = os_mbuf_append(ctxt->om, interval_chr_val,
                                sizeof(interval_chr_val));
            ESP_LOG_BUFFER_HEX(TAG, interval_chr_val, sizeof(interval_chr_val));
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }

        if (attr_handle == range_chr_val_handle)
        {
            rc = os_mbuf_append(ctxt->om, range_chr_val,
                                sizeof(range_chr_val));
            ESP_LOG_BUFFER_HEX(TAG, range_chr_val, sizeof(range_chr_val));
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }

        if (attr_handle == voltage_chr_val_handle)
        {
            rc = os_mbuf_append(ctxt->om, voltage_chr_val,
                                sizeof(voltage_chr_val));
            ESP_LOG_BUFFER_HEX(TAG, voltage_chr_val, sizeof(voltage_chr_val));
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }

        if (attr_handle == coeff_chr_val_handle)
        {
            rc = os_mbuf_append(ctxt->om, coeff_chr_val,
                                sizeof(coeff_chr_val));
            ESP_LOG_BUFFER_HEX(TAG, coeff_chr_val, sizeof(coeff_chr_val));
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }
        goto error;
    case BLE_GATT_ACCESS_OP_WRITE_CHR:
        /* Verify connection handle */
        if (conn_handle != BLE_HS_CONN_HANDLE_NONE)
        {
            ESP_LOGI(TAG, "characteristic write; conn_handle=%d attr_handle=%d",
                     conn_handle, attr_handle);
        }
        else
        {
            ESP_LOGI(TAG, "characteristic write by nimble stack; attr_handle=%d",
                     attr_handle);
        }
        /* Verify attribute handle */
        if (attr_handle == ssid_chr_val_handle)
        {
            size_t len = ctxt->om->om_len;
            if (len >= sizeof(ssid_chr_val))
                return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
            rc = os_mbuf_copydata(ctxt->om, 0, len, ssid_chr_val);
            if (rc != 0)
                return BLE_ATT_ERR_UNLIKELY;
            ssid_chr_val[len] = '\0';
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }

        if (attr_handle == password_chr_val_handle)
        {
            size_t len = ctxt->om->om_len;
            if (len >= sizeof(password_chr_val))
                return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
            rc = os_mbuf_copydata(ctxt->om, 0, len, password_chr_val);
            if (rc != 0)
                return BLE_ATT_ERR_UNLIKELY;
            password_chr_val[len] = '\0';
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }

        if (attr_handle == mqtt_server_chr_val_handle)
        {
            size_t len = ctxt->om->om_len;
            if (len >= sizeof(mqtt_server_chr_val))
                return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
            rc = os_mbuf_copydata(ctxt->om, 0, len, mqtt_server_chr_val);
            if (rc != 0)
                return BLE_ATT_ERR_UNLIKELY;
            mqtt_server_chr_val[len] = '\0';
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }

        if (attr_handle == mqtt_port_chr_val_handle)
        {
            if (ctxt->om->om_len > sizeof(mqtt_port_chr_val))
            {
                ESP_LOGE(TAG, "MQTT port characteristic write overflow; length=%d",
                         ctxt->om->om_len);
                return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
            }
            rc = os_mbuf_copydata(ctxt->om, 0, ctxt->om->om_len, mqtt_port_chr_val);
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }

        if (attr_handle == access_token_chr_val_handle)
        {
            size_t len = ctxt->om->om_len;
            if (len >= sizeof(access_token_chr_val))
                return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
            rc = os_mbuf_copydata(ctxt->om, 0, len, access_token_chr_val);
            if (rc != 0)
                return BLE_ATT_ERR_UNLIKELY;
            access_token_chr_val[len] = '\0';
            ESP_LOG_BUFFER_CHAR(TAG, access_token_chr_val, len);
            ESP_LOG_BUFFER_HEX(TAG, access_token_chr_val, len);
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }

        if (attr_handle == command_chr_val_handle)
        {
            /* Append data to Command characteristic value */
            if (ctxt->om->om_len > sizeof(command_chr_val))
            {
                ESP_LOGE(TAG, "Command characteristic write overflow; length=%d",
                         ctxt->om->om_len);
                return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
            }
            rc = os_mbuf_copydata(ctxt->om, 0, ctxt->om->om_len, command_chr_val);
            ESP_LOGI(TAG, "Command characteristic written, value=0x%02X",
                     command_chr_val[0]);

            provisioning_manager_on_command(command_chr_val[0]);
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }

        if (attr_handle == interval_chr_val_handle)
        {
            /* Append data to Interval characteristic value */
            if (ctxt->om->om_len > sizeof(interval_chr_val))
            {
                ESP_LOGE(TAG, "Interval characteristic write overflow; length=%d",
                         ctxt->om->om_len);
                return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
            }
            rc = os_mbuf_copydata(ctxt->om, 0, ctxt->om->om_len, interval_chr_val);
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }

        if (attr_handle == range_chr_val_handle)
        {
            /* Append data to Range characteristic value */
            if (ctxt->om->om_len > sizeof(range_chr_val))
            {
                ESP_LOGE(TAG, "Range characteristic write overflow; length=%d",
                         ctxt->om->om_len);
                return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
            }
            rc = os_mbuf_copydata(ctxt->om, 0, ctxt->om->om_len, range_chr_val);
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }

        if (attr_handle == voltage_chr_val_handle)
        {
            /* Append data to Voltage characteristic value */
            if (ctxt->om->om_len > sizeof(voltage_chr_val))
            {
                ESP_LOGE(TAG, "Voltage characteristic write overflow; length=%d",
                         ctxt->om->om_len);
                return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
            }
            rc = os_mbuf_copydata(ctxt->om, 0, ctxt->om->om_len, voltage_chr_val);
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }

        if (attr_handle == coeff_chr_val_handle)
        {
            /* Append data to Coefficient characteristic value */
            if (ctxt->om->om_len > sizeof(coeff_chr_val))
            {
                ESP_LOGE(TAG, "Coefficient characteristic write overflow; length=%d",
                         ctxt->om->om_len);
                return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
            }
            rc = os_mbuf_copydata(ctxt->om, 0, ctxt->om->om_len, coeff_chr_val);
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }
        goto error;

    /* Unknown event */
    default:
        goto error;
    }

error:
    ESP_LOGE(
        TAG,
        "unexpected access operation to ssid characteristic, opcode: %d",
        ctxt->op);
    return BLE_ATT_ERR_UNLIKELY;
}

/*
 *  Handle GATT attribute register events
 *      - Service register event
 *      - Characteristic register event
 *      - Descriptor register event
 */
void gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg)
{
    /* Local variables */
    char buf[BLE_UUID_STR_LEN];

    /* Handle GATT attributes register events */
    switch (ctxt->op)
    {

    /* Service register event */
    case BLE_GATT_REGISTER_OP_SVC:
        ESP_LOGD(TAG, "registered service %s with handle=%d",
                 ble_uuid_to_str(ctxt->svc.svc_def->uuid, buf),
                 ctxt->svc.handle);
        break;

    /* Characteristic register event */
    case BLE_GATT_REGISTER_OP_CHR:
        ESP_LOGD(TAG,
                 "registering characteristic %s with "
                 "def_handle=%d val_handle=%d",
                 ble_uuid_to_str(ctxt->chr.chr_def->uuid, buf),
                 ctxt->chr.def_handle, ctxt->chr.val_handle);
        break;

    /* Descriptor register event */
    case BLE_GATT_REGISTER_OP_DSC:
        ESP_LOGD(TAG, "registering descriptor %s with handle=%d",
                 ble_uuid_to_str(ctxt->dsc.dsc_def->uuid, buf),
                 ctxt->dsc.handle);
        break;

    /* Unknown event */
    default:
        assert(0);
        break;
    }
}

/*
 *  GATT server subscribe event callback
 *      1. Update heart rate subscription status
 */

void gatt_svr_subscribe_cb(struct ble_gap_event *event)
{
    /* Check connection handle */
    if (event->subscribe.conn_handle != BLE_HS_CONN_HANDLE_NONE)
    {
        ESP_LOGI(TAG, "subscribe event; conn_handle=%d attr_handle=%d",
                 event->subscribe.conn_handle, event->subscribe.attr_handle);
    }
    else
    {
        ESP_LOGI(TAG, "subscribe by nimble stack; attr_handle=%d",
                 event->subscribe.attr_handle);
    }

    /* Check attribute handle */
    if (event->subscribe.attr_handle == status_chr_val_handle)
    {
        /* Update heart rate subscription status */
        status_chr_conn_handle = event->subscribe.conn_handle;
        status_chr_conn_handle_inited = true;
        status_ind_status = event->subscribe.cur_notify;
    }
}

/** Get provisioning data snapshot */
void ble_gatt_get_prov_snapshot(provisioning_data_t *out)
{
    if (!out) return;
    memset(out, 0, sizeof(*out));

    strncpy(out->ssid, ssid_chr_val, sizeof(out->ssid) - 1);
    strncpy(out->password, password_chr_val, sizeof(out->password) - 1);
    strncpy(out->mqtt_host, mqtt_server_chr_val, sizeof(out->mqtt_host) - 1);

    uint16_t port = ((uint16_t)mqtt_port_chr_val[1] << 8) | mqtt_port_chr_val[0];
    out->mqtt_port = port;

    strncpy(out->access_token, (char*)access_token_chr_val, sizeof(out->access_token) - 1);
}

void ble_gatt_set_status(const prov_status8_t *st)
{
    if (!st) return;
    memcpy(status_chr_val, st, sizeof(status_chr_val));
}


void ble_gatt_notify_status(void)
{
    // Only if a client subscribed to notifications/indications
    if (!status_chr_conn_handle_inited) return;
    if (!status_ind_status) return;

    // Notify/indicate updated value
    // For NimBLE: use ble_gatts_chr_updated(val_handle) to notify subscribers
    ble_gatts_chr_updated(status_chr_val_handle);
}

void ble_gatt_clear_prov_buffers(void)
{
    memset(ssid_chr_val, 0, sizeof(ssid_chr_val));
    memset(password_chr_val, 0, sizeof(password_chr_val));
    memset(mqtt_server_chr_val, 0, sizeof(mqtt_server_chr_val));
    memset(mqtt_port_chr_val, 0, sizeof(mqtt_port_chr_val));
    memset(access_token_chr_val, 0, sizeof(access_token_chr_val));
}


/*
 *  GATT server initialization
 *      1. Initialize GATT service
 *      2. Update NimBLE host GATT services counter
 *      3. Add GATT services to server
 */
int gatt_svc_init(void)
{
    /* Local variables */
    int rc = 0;

    /* 1. GATT service initialization */
    ble_svc_gatt_init();

    /* 2. Update GATT services counter */
    rc = ble_gatts_count_cfg(gatt_svr_svcs);
    if (rc != 0)
    {
        return rc;
    }

    /* 3. Add GATT services */
    rc = ble_gatts_add_svcs(gatt_svr_svcs);
    if (rc != 0)
    {
        return rc;
    }

    //load initial values from storage
    provisioning_data_t prov_data;
    esp_err_t err = storage_load_provisioning_data(&prov_data);
    if (err == ESP_OK)
    {
        strncpy(ssid_chr_val, prov_data.ssid, sizeof(ssid_chr_val) - 1);
        strncpy(password_chr_val, prov_data.password, sizeof(password_chr_val) - 1);
        strncpy(mqtt_server_chr_val, prov_data.mqtt_host, sizeof(mqtt_server_chr_val) - 1);
        mqtt_port_chr_val[1] = (prov_data.mqtt_port >> 8) & 0xFF;
        mqtt_port_chr_val[0] = prov_data.mqtt_port & 0xFF;
        strncpy((char*)access_token_chr_val, prov_data.access_token, sizeof(access_token_chr_val) - 1);
        ESP_LOGI(TAG, "Loaded provisioning data from storage");
    }
    else
    {
        ESP_LOGI(TAG, "No provisioning data in storage");
    }
    //log loaded provisioning data
    ESP_LOGI(TAG, "SSID: %s", ssid_chr_val);
    ESP_LOGI(TAG, "Password: %s", password_chr_val);
    ESP_LOGI(TAG, "MQTT server: %s", mqtt_server_chr_val);
    uint16_t mqtt_port = ((uint16_t)mqtt_port_chr_val[1] << 8)
                            | mqtt_port_chr_val[0];
    ESP_LOGI(TAG, "MQTT port: %d", mqtt_port);
    ESP_LOGI(TAG, "Access token: %s", access_token_chr_val);

    device_config_data_t config_data;
    err = storage_load_device_config_data(&config_data);
    if (err == ESP_OK)
    {
        interval_chr_val[3] = (config_data.interval >> 24) & 0xFF;
        interval_chr_val[2] = (config_data.interval >> 16) & 0xFF;
        interval_chr_val[1] = (config_data.interval >> 8) & 0xFF;
        interval_chr_val[0] = config_data.interval & 0xFF;
        range_chr_val[0] = config_data.active_range+1;

        for (int i = 0; i < 4; i++)
        {
            voltage_chr_val[i * 2 + 1] = (config_data.set_voltage[i] >> 8) & 0xFF;
            voltage_chr_val[i * 2] = config_data.set_voltage[i] & 0xFF;
        }

        for (int i = 0; i < 4; i++)
        {
            float coeff = (float)config_data.coeff[i]/10000.0;
            //convert to 4 bytes little endian
            memcpy(&coeff_chr_val[i * 4], &coeff, 4);

        }

        ESP_LOGI(TAG, "Loaded device config data from storage");
    }
    else
    {
        ESP_LOGI(TAG, "No device config data in storage");
    }

    //log loaded device config data
    uint32_t interval = (interval_chr_val[3] << 24)
                        | (interval_chr_val[2] << 16)
                        | (interval_chr_val[1] << 8)
                        | interval_chr_val[0];
    ESP_LOGI(TAG, "Interval: %lu ms", interval);
    ESP_LOGI(TAG, "Active range: %u", range_chr_val[0]);
    ESP_LOGI(TAG, "Set voltages:");
    for (int i = 0; i < 4; i++)
    {
        uint16_t voltage = (voltage_chr_val[i * 2 + 1] << 8)
                            | voltage_chr_val[i * 2];
        ESP_LOGI(TAG, "  R%d: %u V", i + 1, voltage);
    }
    ESP_LOGI(TAG, "Coefficients:");
    for (int i = 0; i < 4; i++)
    {
        uint32_t coeff = (coeff_chr_val[i * 4 + 3] << 24)
                         | (coeff_chr_val[i * 4 + 2] << 16)
                         | (coeff_chr_val[i * 4 + 1] << 8)
                         | coeff_chr_val[i * 4];
        ESP_LOGI(TAG, "  R%d: %lu", i + 1, coeff);
    }

    return 0;
}