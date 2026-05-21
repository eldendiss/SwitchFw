# RaSens — ESP32 Radiation Sensor Firmware

ESP-IDF firmware for a field-deployed radiation sensor based on a Geiger-Müller tube.
Drives a 4-range flyback HV PSU, counts tube pulses, and streams telemetry over MQTT via Wi-Fi or Ethernet.

**ESP-IDF v5.4.1 · ESP32 · 8 MB flash**

---

## Hardware

| Signal | GPIO (sdkconfig) |
|---|---|
| I²C SDA (flyback PSU) | 21 |
| I²C SCL (flyback PSU) | 22 |
| PSU enable | 15 |
| PSU sleep | 16 |
| Geiger pulse input | 17 |
| Battery ADC (ADC1_CH5) | 33 |

- **Flyback HV PSU**: AVR-based I²C board at address 0x2A, 4 switchable HV ranges, mechanical relay per range.
- **Geiger tube**: Pulse output connected to GPIO17. PCNT peripheral counts pulses; glitch filter ≥ 500 ns.
- **Ethernet**: W5500 SPI module for wired connectivity (runs alongside Wi-Fi).
- **Battery**: Resistor divider (R1=100 kΩ, R2=22 kΩ) on GPIO33. Supports up to ~12.6 V packs.

---

## Features

- **4-range HV control** — configurable setpoints and ±10 %/±5 % OV trip/clear thresholds per range.
- **MQTT telemetry** — publishes radiation and system metrics at a configurable interval.
- **BLE provisioning** — configure Wi-Fi credentials and MQTT server over Bluetooth without USB access.
- **Dual network** — Wi-Fi STA + W5500 Ethernet; either can carry MQTT. Telemetry reports active interface.
- **OTA updates** — HTTPS pull from backend server; dual OTA partitions with automatic rollback on crash.
- **Job commands** — MQTT-delivered commands to change HV range, voltage, conversion factor, interval, and more.
- **Power management** — automatic light sleep (CPU 80–240 MHz), Wi-Fi modem sleep (DTIM), Wi-Fi config kept in RAM only.
- **Crash loop protection** — RTC crash counter; after 5 consecutive panics marks firmware valid to break rollback purgatory.
- **Task watchdog** — 30 s TWDT with panic; catches hung tasks.

---

## Partition Table

| Name | Type | Offset | Size |
|---|---|---|---|
| nvs | NVS | 0x9000 | 16 KB |
| otadata | OTA data | 0xd000 | 8 KB |
| phy_init | RF data | 0xf000 | 4 KB |
| ota_0 | App | 0x10000 | 2 MB |
| ota_1 | App | 0x210000 | 2 MB |

> **Do not change the partition table on deployed devices.** OTA flashes app partitions only; the partition table itself requires a full re-flash.

---

## Getting Started

### Prerequisites

- ESP-IDF v5.4.1
- `idf.py` toolchain

### Build and flash

```sh
idf.py menuconfig   # set RaSens options (Wi-Fi defaults, MQTT host, pins, etc.)
idf.py build
idf.py flash monitor
```

### Menuconfig options (`RaSens` menu)

| Option | Default | Description |
|---|---|---|
| `RASENS_DEFAULT_SSID` | `your_ssid` | Fallback Wi-Fi SSID (used when no BLE provisioning data exists) |
| `RASENS_DEFAULT_WIFI_PASSWORD` | `your_password` | Fallback Wi-Fi password |
| `RASENS_MQTT_HOST` | `isdg.fei.stuba.sk` | MQTT broker hostname |
| `RASENS_MQTT_PORT` | `1883` | MQTT broker port |
| `RASENS_ACCESS_TOKEN` | _(empty)_ | Device access token |
| `RASENS_HTTP_BACKEND_URL` | `https://…/backend/` | OTA/version backend URL |
| `RASENS_SDA_PIN` | `22` | I²C SDA |
| `RASENS_SCL_PIN` | `21` | I²C SCL |
| `RASENS_ENABLE_PIN` | `17` | PSU enable GPIO |
| `RASENS_SLEEP_PIN` | `18` | PSU sleep GPIO |
| `RASENS_INTERRUPT_PIN` | `4` | Geiger pulse GPIO |
| `RASENS_I2C_ADDRESS` | `42` | PSU I²C address (0x2A) |

---

## Provisioning (BLE)

On first boot (or after factory reset) the device advertises over BLE. Use the companion app or a BLE terminal to write provisioning data and issue control commands.

### Control characteristic commands (write 1 byte)

| Value | Command | Description |
|---|---|---|
| `0x01` | Apply | Validate credentials, test Wi-Fi + MQTT, save to NVS on success |
| `0x02` | Clear | Erase NVS provisioning data, revert to menuconfig defaults |
| `0x03` | Save Only | Validate and save, skip connection test |

### Status characteristic (notify / read, 8 bytes)

| Byte | Field | Values |
|---|---|---|
| 0 | `device_status` | 0=Idle, 1=Applying, 2=Provisioned, 3=Error |
| 1 | `wifi_status` | 0=Disconnected, 1=Connecting, 2=Connected, 3=Failed |
| 2 | `mqtt_status` | 0=Disconnected, 1=Connecting, 2=Connected, 3=Failed |
| 3 | `err_code` | 0=None, 1=Validation, 2=WiFi auth, 3=WiFi timeout, 4=MQTT failed, 5=Storage |
| 4 | `wifi_rssi` | Signed dBm (0 if unknown) |
| 5 | `eth_status` | 0=Disconnected, 1=Connected |
| 6–7 | reserved | — |

---

## MQTT Telemetry

Published at the configured interval (default 60 s, range 1–3600 s).

| Key | Type | Description |
|---|---|---|
| `cr60` | float | Counts per minute — 60 s rolling window |
| `dr60` | float | Dose rate µSv/h — 60 s rolling window |
| `cr` | float | Counts per interval (short-term rate) |
| `dr` | float | Dose rate µSv/h (short-term) |
| `voltage` | float | HV feedback average (V) |
| `samplerate` | int | Measurement interval (s) |
| `range` | int | Active HV range (1–4) |
| `iface` | int | Network interface: 0=none, 1=Wi-Fi, 2=Ethernet |
| `battery` | float | Battery voltage (V) |

---

## Job Commands

Commands arrive over MQTT via the job manager. Each command is a JSON object with a `method` key and optional `params` array.

| Method | Params | Description |
|---|---|---|
| `setTube` | `[0–3]` | Select HV range (0-indexed). `4` disables HV output. |
| `setInterval` | `[s]` | Measurement interval in seconds (1–3600) |
| `setVoltage_r1` | `[V]` | HV setpoint for range 1 |
| `setVoltage_r2` | `[V]` | HV setpoint for range 2 |
| `setVoltage_r3` | `[V]` | HV setpoint for range 3 |
| `setVoltage_r4` | `[V]` | HV setpoint for range 4 |
| `setConversion_r1` | `[factor]` | µSv/h conversion factor for range 1 |
| `setConversion_r2` | `[factor]` | µSv/h conversion factor for range 2 |
| `setConversion_r3` | `[factor]` | µSv/h conversion factor for range 3 |
| `setConversion_r4` | `[factor]` | µSv/h conversion factor for range 4 |
| `reset` | — | Restart device (4 s delay) |
| `factoryReset` | — | Erase all NVS data and restart |

All settings (voltage, conversion factor, interval, active range) are persisted to NVS immediately after each command.

---

## Device Configuration (NVS)

Stored in the `devcfg` namespace under the `nvs` partition.

| Field | Default | Description |
|---|---|---|
| `interval` | 60 s | Telemetry publish interval |
| `active_range` | 0 | HV range index (0–3) |
| `set_voltage[4]` | 200, 250, 300, 350 V | HV target per range |
| `coeff[4]` | 93, 62, 1111, 11111 (×10⁻⁴) | µSv/h conversion factor per range |

Config is validated on load: `interval` is clamped to 1–3600 s, `active_range` to 0–3. Corrupt or absent data falls back to defaults and is re-saved.

---

## Power Management

- **Automatic light sleep** — FreeRTOS tickless idle; CPU sleeps when all tasks blocked.
- **CPU frequency** — scales dynamically 80–240 MHz.
- **Wi-Fi modem sleep** — `WIFI_PS_MIN_MODEM`; radio wakes only for DTIM beacons.
- **Wi-Fi NVS writes suppressed** — `WIFI_STORAGE_RAM` prevents `esp_wifi_set_config()` from writing to flash on every reconnect attempt.

---

## OTA

On each boot (when network is available), the device:
1. Reports current firmware version to the backend.
2. Queries the backend for the active firmware version.
3. Downloads and flashes if versions differ.
4. Reboots; on success marks the new image valid; on crash rolls back automatically.

---

## Troubleshooting

**Device boots to default voltage instead of saved setpoint**
The flyback PSU powers on at its own default voltage. The firmware forces a temporary range switch after boot to make the PSU pick up the configured voltage table.

**NVS data lost after battery discharge**
Fixed in current firmware: `esp_wifi_set_storage(WIFI_STORAGE_RAM)` prevents Wi-Fi reconnect logic from hammering NVS and triggering compaction during power loss.

**Coefficient values look like large integers after firmware update**
The `coeff` field storage format changed between versions (float → uint32_t ×10000). Issue a `factoryReset` command to clear stale data, then re-apply settings via job commands.

**I²C timeouts to flyback PSU**
Check SDA/SCL pull-ups and address (0x2A). Bus speed is 10 kHz for stability on long harnesses.

**Relay never settles (`busy=1`)**
`flyback_wait_for_idle()` uses 10 ms poll, 3 s timeout. If the relay takes longer, check PSU board power supply.

---

## License

**Non-Commercial License**
- Free for personal, educational, and research use.
- Commercial use, redistribution for profit, or inclusion in commercial products requires written permission.

See [LICENSE](./LICENSE) for details.
