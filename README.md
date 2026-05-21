# SwitchFw — AVR HV Flyback PSU Controller

AVR firmware for the high-voltage flyback power supply that drives the Geiger-Müller tube.
It is a subsystem: the ESP32 (see `esp32-flyback-controller/`) is the system master and controls this AVR via I²C.

## Role in the system

```
ESP32 (master)  ──I²C──►  AVR SwitchFw (slave 0x2A)
                                │
                                ├─ PWM → MOSFET → flyback transformer → HV output
                                ├─ 4× relay range switches (R_SW network)
                                └─ Analog comparator OC protection
```

The ESP32 sets the target voltage (per-range setpoints), enables the supply via the EN line, and reads back feedback and fault status.

## Hardware

| Component | Part |
|-----------|------|
| MCU | ATmega328P (8 MHz) |
| Power switch | IRF540N + TC4420 gate driver |
| Voltage reference | LM4040 2.5 V shunt |
| Signal conditioning | LM321 op-amps |
| HV clamp | SMAJ150A TVS |
| Logic supply | LM317 → 5 V rail |

## Pin Map

| Pin | Dir | Signal |
|-----|-----|--------|
| PB1 | OUT | Gate drive (PWM, OC1A) |
| PC0 | IN  | HV feedback (ADC0, external AREF) |
| PC2 | OUT | OK LED |
| PC3 | OUT | Fault LED |
| PD3 | IN  | EN (active-high, external pull-down) |
| PD6 | IN  | OC comparator input (AIN0) |
| PB6 | OUT | R_SW0 (range relay 0) |
| PB7 | OUT | R_SW1 (range relay 1) |
| PD0 | OUT | R_SW2 (range relay 2) |
| PD1 | OUT | R_SW3 (range relay 3) |
| PC4 | I²C | SDA |
| PC5 | I²C | SCL |
| PD4 | —   | Free — connected to ESP32 |
| PD5 | —   | Free — connected to ESP32 |
| PB3/4/5, PC6 | ISP | MOSI/MISO/SCK/RESET |

> PD4 and PD5 were previously hardware range-select inputs. They are now free;
> range control is I²C-only.

## Firmware Overview

### PWM / control loop
- **Frequency**: 31.25 kHz, prescaler = 1, ICR1 = 255 (Fast PWM, TOP = ICR1)
- **Duty ceiling**: 20% hard cap (software and hardware)
- **Loop rate**: 1 kHz PI+D controller
- **Feedback**: ADC0, 8× boxcar average, ~125 kHz conversion rate
- **Timebase**: Timer2, prescaler /8 → 1 µs tick, 32-bit `us_now32()`

### Control phases

| Phase | Description |
|-------|-------------|
| `PH_PRECHARGE` | Fixed small duty soft-start until time elapsed and FB threshold reached |
| `PH_PI` | Closed-loop PI + filtered derivative on measured FB |

### Regulation modes

| Mode | Description |
|------|-------------|
| `REG_PWM` | Continuous PWM regulation |
| `REG_PFM` | Hysteretic microburst: fires a short burst when FB drops below the lower threshold, then coasts; quieter at light load |

### Fault protection
- **OV**: ADC-based, per-range trip/clear thresholds; latches until FB drops below clear level
- **OC**: Analog comparator (AIN0 vs 1.1 V bandgap), cycle-by-cycle hard gate kill; blanking window at each period start avoids switch-edge noise

### Range switching
A safe open → wait-stable → make sequence prevents false OC/OV trips during relay transitions:
1. Open all R_SW outputs
2. Wait ≥ 5 ms and `|dFB/dt|` ≤ threshold
3. Close requested R_SW

### Power saving when EN is deasserted
When EN is low the firmware:
- Forces gate off (PWM disabled)
- Stops the ADC free-run (saves ~350 µA + eliminates 125 kHz ISR load)
- Disables Timer1 OVF/COMPB ISRs (was 31 k interrupts/s, only needed when switching)
- Disarms the analog comparator
- Puts the CPU into `SLEEP_MODE_IDLE`

Timer2 OVF (every 256 µs) and I²C events wake the CPU for housekeeping. Estimated MCU current drops from ~10 mA to ~2–3 mA.

### EEPROM configuration
- Slot size: 128 B, CRC-16 protected
- Wear leveling: automatic slot rotation
- On boot: loads valid slot or writes defaults and continues

## I²C Protocol (slave 0x2A)

Register-pointer scheme: first byte of a write sets the pointer, subsequent bytes are data.

### Register map

| Address | Name | Access | Description |
|---------|------|--------|-------------|
| 0x00–0x01 | `fb_set_cnt` | R/W | Active setpoint (ADC counts) |
| 0x02–0x03 | `fb_ov_trip_cnt` | R/W | Active OV trip threshold |
| 0x04–0x05 | `fb_ov_clear_cnt` | R/W | Active OV clear threshold |
| 0x06–0x0D | `fb_set_cnt_tab[4]` | R/W | Per-range setpoints (R0–R3, u16 LE) |
| 0x0E–0x15 | `fb_ov_trip_cnt_tab[4]` | R/W | Per-range OV trip thresholds |
| 0x16–0x1D | `fb_ov_clear_cnt_tab[4]` | R/W | Per-range OV clear thresholds |
| 0x1E–… | *(rest of ControlParams)* | R/W | Gains, PFM settings, timing, etc. |
| 0x20 | `range_req` | R/W | Write 0–3 to request range switch; read = last request |
| 0x21 | `range_src` | R/W | 0=legacy HW (treated as I²C), **1=I²C (default)**, 2=AUTO |
| 0x40 | `cmd` | W | Command (see below); read returns 0x00 |
| 0x50–0x57 | `status` | R | 8-byte status block (see below) |

All multi-byte fields are **little-endian**. Config reads auto-increment the pointer (up to 16 bytes per `onRequest`).

### Commands (0x40)
| Code | Action |
|------|--------|
| 0x01 | Save current config to EEPROM |
| 0x02 | Reset controller to `PH_PRECHARGE` |

### Status block (0x50, 8 bytes)

| Bytes | Field | Description |
|-------|-------|-------------|
| 0 | *(reserved)* | Always 0 |
| 1 | *(reserved)* | Always 0 |
| 2–3 | `fb_ema` | EMA-filtered feedback (ADC counts, u16 LE) |
| 4–5 | *(reserved)* | Always 0 |
| 6 | `active_range` | Currently engaged range (0–3) |
| 7 | `range_busy` | 1 while relay transition is in progress |

## LED patterns (OK LED, PC2)

| State | Pattern |
|-------|---------|
| Waiting for EN | Short 120 ms blip every 1 s |
| Precharge | Fast blink ~5 Hz |
| Ramping | Medium blink ~2.5 Hz |
| Regulating (PWM) | Steady ON |
| PFM hold | 1 Hz blink |
| Range switching | Double-blink |

Fault LED (PC3) blinks N times (pause, repeat) where N = fault code: 1 = OC, 2 = OV.

## Build

```
PlatformIO, AVR-GCC toolchain.
Programmer: ISP (USBasp, AVR Dragon, etc.) via ISP header.
```

---

## License

Non-Commercial. Free for personal, educational, and research use.
Commercial use or redistribution for profit requires written permission. See [LICENSE](./LICENSE).

> ⚠️ This device generates potentially lethal high voltages. Only qualified personnel should work on it.
