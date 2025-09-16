# Flyback PSU & Geiger Counter ESP-IDF Driver

ESP-IDF drivers for:
- an **AVR flyback HV PSU** (I²C, 4 ranges, status/commands), and
- a **Geiger pulse counter** using the ESP32 RMT RX peripheral.

Works on ESP-IDF v5+.

## Overview

- **Flyback PSU driver (`flyback_psu`)**: Configure per-range setpoints/OV thresholds, select ranges (HW/I²C/AUTO), read 8-byte status, and issue SAVE/RESET commands. Counts↔volts helpers included.
- **Geiger counter driver (`geiger_counter`**: Counts tube pulses with RMT RX. Filters glitches by minimum pulse width. Provides **CPS, CPM (rolling 60 s)**, and **total**. Thread-safe getters.
- **Example application (`main`)**: Demonstrates how to initialize and use both drivers together, cycling through PSU voltage ranges and logging Geiger counts.

---

## Features

### Flyback PSU

- I²C master init/reuse, configurable clock.
- Range source control: HW / I²C / AUTO.
- Program per-range set, OV trip, OV clear (counts or volts).
- Read compact STATUS: run_state, fault_code, fb_counts, dcounts, active_range, busy.
- SAVE_CONFIG, RESET_CTRL commands.
- Helpers: volts↔counts, set profiles, wait-until-idle.

### Geiger Counter

- RMT RX capture with **min pulse width** filter (ns).
- Configurable **active level** (high/low), **resolution** (Hz), buffer size.
- Thread-safe **get_total(), get_cps(), get_cpm()**.
- Internal task processes RX buffers (zero-copy on symbols).

---

## Hardware Requirements

- ESP32/ESP32-Sx/Cx with ESP-IDF v5+
- I²C wiring to AVR PSU at 0x2A (pull-ups required)
- Tube pulse output routed to an ESP32 GPIO
- CMake build (component-based)

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
ESP_ERROR_CHECK(flyback_psu_init(&psu, I2C_NUM_0, SDA_GPIO, SCL_GPIO, 10000 /* 10 kHz */, 0x2A));
flyback_psu_set_fullscale(&psu, 1077.5f); // 1023 counts == 1077.5 V
```

#### Set Range Source to I²C

```c
ESP_ERROR_CHECK(flyback_psu_set_range_source(&psu,  FLYBACK_RANGE_SRC_I2C));
```

#### Set Voltage Setpoints (volts)

```c
const float V[4] = {200, 500, 700, 900};
uint16_t C[4];
for (int i=0;i<4;i++) C[i] = flyback_psu_volts_to_counts(&psu, V[i]);
ESP_ERROR_CHECK(flyback_psu_set_table_counts4(&psu, C));

// Trip/Clear @ +10% / +5%
for (int i=0;i<4;i++){
  ESP_ERROR_CHECK(flyback_psu_set_trip_counts(&psu,  i,
     flyback_psu_volts_to_counts(&psu, V[i]*1.10f)));
  ESP_ERROR_CHECK(flyback_psu_set_clear_counts(&psu, i,
     flyback_psu_volts_to_counts(&psu, V[i]*1.05f)));
}
// Optional: ESP_ERROR_CHECK(flyback_psu_send_command(&psu, FLYBACK_CMD_SAVE_CONFIG));
```

#### Start the Geiger counter
```c
geiger_counter_t gc = {0};
ESP_ERROR_CHECK(geiger_counter_start(&gc,
    GPIO_NUM_4,          // pulse input GPIO
    true,                // active-high pulses
    20000000,            // 20 MHz resolution (50 ns tick)
    1000,                // ≥1 µs min pulse width
    2048));              // RX symbol buffer

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
    ESP_LOGI("PSU","st=%u fault=%u fb=%u(%.1fV) d=%d r=%u busy=%u",
             s.run_state, s.fault_code, s.fb_counts, v, (int)s.dcounts,
             s.active_range, s.range_busy);
}
ESP_LOGI("GEIGER","CPS=%.1f CPM=%.1f Total=%llu",
           geiger_counter_get_cps(&gc),
           geiger_counter_get_cpm(&gc),
           (unsigned long long)geiger_counter_get_total(&gc));
  vTaskDelay(pdMS_TO_TICKS(1000));
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

## Notes & Best Practices
- **I²C speed:** Start at **10 kHz** (robust on long harnesses). If wiring is short/clean, 100–400 kHz is fine.
- **Range arbitration:**
    - `AUTO`: hardware pins control until the first I²C request, then I²C takes over.
    - If your front panel uses the HW pins, prefer AUTO so remote control can still take over.
- **STATUS → volts:** `V ≈ fb_counts * (fullscale / 1023.0)`, default fullscale **1077.5 V**.
- **RMT sizing:**
    - `resolution_hz = 20 MHz` gives 50 ns ticks; good for 2–5 µs pulses.
    - `min_pulse_ns`: start with 1000 ns (rejects narrow noise).
    - `buf_symbols`: 1024–2048 typical; increase if you expect high CPM or bursts.
- **Threading**:
    - RMT RX runs a task; getters use a mutex; safe from multiple tasks.
    - Avoid calling `geiger_counter_stop` from ISR context.

- The Geiger counter uses a dedicated FreeRTOS task and RMT peripheral; ensure sufficient stack size and priority.
- Use flyback_psu_send_command() to save configuration to EEPROM or reset the controller if needed.
- The example cycles through all four voltage ranges, holding each for 60 seconds, printing status and counts every second.

## Troubleshooting
- **I²C timeouts**
    - Check address `0x2A`, pull-ups, and grounds. Drop bus to **10 kHz** first.
- **STATUS never idle (`busy=1`)**
    - Mechanical relays need time; ensure `flyback_psu_wait_idle()` poll interval ≥ 10 ms and timeout ≥ 3 s.
- **OV faults**
    - Trip/clear thresholds too tight for the measured divider scaling. Re-measure HV and adjust counts.
- **Few/no counts**
    - Pulse polarity mismatch (`active_high`). Increase `min_pulse_ns` if noise, decrease if valid pulses are ~1–2 µs but get filtered.

## License
This project is licensed under a **Non-Commercial License**.  
- ✅ Free for personal, educational, and research use.  
- ❌ Commercial use, redistribution for profit, or inclusion in commercial products is prohibited without written permission.  
See [LICENSE](./LICENSE) for details.

## References
- [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/)
- [ESP32 RMT Peripheral](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/rmt.html)
- AVR flyback PSU firmware documentation