# RaSens BLE Documentation

## UUID

We use 128-bit Vendor UUID - 16 bit UUIDs are reserved for SIG approved devices.

Device base UUID is assigned at random per device type, except for last 2 bytes. For example:
- Base: ```704d43a7-dd07-49f1-9818-8ca1364e0000```

Each service characteristic uses the last 2 bytes for identification. For simplicity in this documetation, we will only specify the last 2 bytes appended to base UUID, e.g. ```...0001``` translates to ```704d43a7-dd07-49f1-9818-8ca1364e0001```

```
0000 - Base UUID
    0010 - Provisioning Service
        0011 - SSID Characteristics
        0012 - Password Characteristics
        0013 - MQTT Server Characteristics
        0014 - MQTT Port Characteristics
        0015 - Access Token Characteristics
        0016 - Command Characteristics
        0017 - Status Characteristics
    0020 - Configuration Service
        0021 - Interval Characteristics    
        0022 - Range Characteristics 
        0023 - Set Voltage Characteristics      
        0024 - Coefficients Characteristics 
```

Each characteristics also has permissions and properties, see BLE documentation for more info about these. There is also an option of encrypted read/write, which allow operation only after devices are bonded.
Test

Defined services (later will be extended further):

---
### Provisioning Service

- Provision wifi credentials / MQTT server
- Provisioning is confirmed by writing a command
- Allows notifications on status
- UUID: ```0010```

#### SSID
- UUID: ```0011```
- Properties: Write
- Permissions: currently only W, in the future consider encrypted W
- Do not expose as Read in production
- Data: UTF-8, max 32 bytes

#### Password
- UUID: ```0012```
- Properties: Write
- Permissions: currently only W, in the future consider encrypted W
- Do not expose as Read in production
- Data: UTF-8, max 63 bytes

#### MQTT Server
- UUID: ```0013```
- Properties: Read/Write
- Permissions: R/W
- Data: UTF-8, max. 63 bytes

#### MQTT Port
- UUID: ```0014```
- Properties: Read/Write
- Permissions: R/W
- Data: uint16

#### Access Token
- UUID: ```0015```
- Properties: Write
- Permissions: currently only W, in the future consider encrypted W
- Data: UTF-8, max. 24 bytes

#### Apply/Command
- UUID: ```0016```
- Properties: Write
- Permissions: W
- Data: 1 byte
    - ```0x01``` - Apply (attempt Wifi+MQTT connect)
    - ```0x02``` - Disconnect / Clear
    - ```0x03``` - Save (do not connect instantly)

#### Status
- UUID: ```0017```
- Properties: Read + Notify
- Permissions: R
- Data: 8 byte array

| Byte | Field | Values |
|------|-------|--------|
| 0 | `device_status` | `0` IDLE, `1` APPLYING, `2` PROVISIONED, `3` ERROR |
| 1 | `wifi_status` | `0` DISCONNECTED, `1` CONNECTING, `2` CONNECTED, `3` FAILED |
| 2 | `mqtt_status` | `0` DISCONNECTED, `1` CONNECTING, `2` CONNECTED, `3` FAILED |
| 3 | `err_code` | `0` NONE, `1` VALIDATION, `2` WIFI_AUTH, `3` WIFI_TIMEOUT, `4` MQTT_FAILED, `5` STORAGE |
| 4 | `wifi_rssi` | signed dBm, `0` if not on WiFi |
| 5 | `eth_status` | `0` DISCONNECTED, `1` CONNECTED |
| 6–7 | `reserved[2]` | — |

---
### Configuration Service
- Set up device parameters
- UUID: ```0020```

#### Measurement Interval
- UUID: ```0021```
- Properties: Read/Write
- Permissions: R/W
- Data: unsigned word (4 bytes) - interval

#### Selected Range
- UUID: ```0022```
- Properties: Read/Write
- Permissions: R/W
- Data: 1 byte - range

#### Set Voltage
- UUID: ```0023```
- Properties: Read/Write
- Permissions: R/W
- Data: 8 bytes
    - 2 bytes - V0
    - 2 bytes - V1
    - 2 bytes - V2
    - 2 bytes - V3

#### Coefficients
- UUID: ```0024```
- Properties: Read/Write
- Permissions: R/W
- Data: 16 bytes
    - 4 bytes - c0
    - 4 bytes - c1
    - 4 bytes - c2
    - 4 bytes - c3