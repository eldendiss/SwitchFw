# Flyback PSU & Geiger Counter ESP-IDF Driver

This repository provides ESP-IDF drivers and example code for interfacing with an AVR flyback power supply unit (PSU) and a Geiger counter pulse detector using the ESP32's RMT peripheral.

---

## Overview

- **Flyback PSU driver**: Communicates with the AVR flyback PSU over I²C to configure voltage ranges, setpoints, thresholds, and read status.
- **Geiger counter driver**: Uses the ESP32 RMT RX peripheral to count pulses from a Geiger tube, providing counts per second (CPS) and counts per minute (CPM).
- **Example application**: Demonstrates how to initialize and use both drivers together, cycling through PSU voltage ranges and logging Geiger counts.

---

## Features

### Flyback PSU

- Initialize I²C master and communicate with PSU device.
- Set per-range voltage setpoints and overvoltage trip/clear thresholds.
- Select active voltage range (0..3) via I²C.
- Read compact status including feedback voltage, fault codes, and range switching state.
- Send commands to save configuration or reset controller.
- Convert between volts and internal counts with configurable full-scale voltage.

### Geiger Counter

- Count pulses on a GPIO using ESP32 RMT RX peripheral.
- Filter out glitches shorter than a configurable minimum pulse width.
- Provide total pulse count, counts per second (CPS), and rolling counts per minute (CPM).
- Thread-safe API with internal FreeRTOS task handling pulse processing.

---

## Hardware Requirements

- ESP32 development board with ESP-IDF v5 or later.
- AVR flyback PSU connected via I²C (default address 0x2A).
- Geiger tube connected to a GPIO pin capable of input capture.
- Pull-up resistors on I²C lines (SDA, SCL) as required.

---

## Getting Started

### 1. Clone and Setup ESP-IDF

Follow the official ESP-IDF setup guide:

[ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/)

### 2. Add Flyback PSU and Geiger Counter Drivers

Include the provided driver source files (`flyback_psu.h/c`, `geiger_counter.h/c`) in your project.

### 3. Example Application

The example demonstrates:

- Initializing the flyback PSU over I²C.
- Setting voltage setpoints and overvoltage thresholds.
- Forcing I²C control of the active voltage range.
- Initializing the Geiger counter on a GPIO pin.
- Cycling through the four voltage ranges, holding each for 60 seconds.
- Logging PSU status and Geiger counts every second.

---

## API Usage

### Flyback PSU

#### Initialization

```c
flyback_psu_t psu;
ESP_ERROR_CHECK(flyback_psu_init(&psu, I2C_NUM_0, SDA_GPIO, SCL_GPIO, 10000, 0x2A));
flyback_psu_set_fullscale(&psu, 1077.5f);
```

#### Set Range Source to I²C

```c
ESP_ERROR_CHECK(flyback_psu_set_range_source(&psu,  FLYBACK_RANGE_SRC_I2C));
```

#### Set Voltage Setpoints (volts)

```c
float voltages[4] = {200.0f, 500.0f, 700.0f, 900.0f};
uint16_t counts[4];
for (int i = 0; i < 4; ++i) {
    counts[i] = flyback_psu_volts_to_counts(&psu, voltages[i]);
}
ESP_ERROR_CHECK(flyback_psu_set_table_counts4(&psu, counts));
```

#### Set Overvoltage Trip and Clear Thresholds

```c
uint8_t trip_buf[8], clear_buf[8];
for (int i = 0; i < 4; ++i) {
    uint16_t trip = flyback_psu_volts_to_counts(&psu, voltages[i] * 1.10f);
    uint16_t clear = flyback_psu_volts_to_counts(&psu, voltages[i] * 1.05f);
    trip_buf[i*2] = trip & 0xFF;
    trip_buf[i*2+1] = trip >> 8;
    clear_buf[i*2] = clear & 0xFF;
    clear_buf[i*2+1] = clear >> 8;
}
ESP_ERROR_CHECK(flyback_psu_write(&psu, REG_TRIP0, trip_buf, sizeof(trip_buf)));
ESP_ERROR_CHECK(flyback_psu_write(&psu, REG_CLEAR0, clear_buf, sizeof(clear_buf)));
```

#### Select Active Range

```c
ESP_ERROR_CHECK(flyback_psu_select_range(&psu, range_index));
ESP_ERROR_CHECK(flyback_psu_wait_idle(&psu, 3000, 10)); // wait for relay switch
```

#### Read Status

```c
flyback_psu_status_t status;
if (flyback_psu_read_status(&psu, &status) == ESP_OK) {
    float feedback_volts = flyback_psu_counts_to_volts(&psu, status.fb_counts);
    // Use status fields as needed
}
```

### Geiger Counter

#### Initialization
```c
static geiger_counter_t gc;
ESP_ERROR_CHECK(geiger_counter_start(&gc, TUBE_GPIO, TUBE_ACTIVE_HIGH, 20000000, 1000, 2048));
```

#### Get Counts

```c
float cps = geiger_counter_get_cps(&gc);   // Counts per second (delta)
float cpm = geiger_counter_get_cpm(&gc);   // Counts per minute (rolling 60s)
uint64_t total = geiger_counter_get_total(&gc); // Total counts since start
```

#### Stop Counter
```c
ESP_ERROR_CHECK(geiger_counter_stop(&gc));
```

## Example output
```
I (1234) example: Selecting range 0 -> target voltage ~200.0 V (counts=190)
I (1235) example: PSU status: state=1 fault=0 fb_counts=185 (~198.5 V) dcounts=0 active_range=0 busy=0
I (1235) GEIGER: Counts: CPS=5.0  CPM=300.0  Total=1500
...
```

## Notes
- Adjust I²C bus speed and GPIO pins according to your hardware.
- The Geiger counter uses a dedicated FreeRTOS task and RMT peripheral; ensure sufficient stack size and priority.
- The flyback PSU driver assumes the AVR firmware uses little-endian multi-byte registers.
- Use flyback_psu_send_command() to save configuration to EEPROM or reset the controller if needed.
- The example cycles through all four voltage ranges, holding each for 60 seconds, printing status and counts every second.

## License
See LICENSE file for details.

## References
- [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/)
- [ESP32 RMT Peripheral](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/rmt.html)
- AVR flyback PSU firmware documentation