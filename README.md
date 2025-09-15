# AVR HV Flyback & Geiger-Müller Tube Detector

A high-voltage flyback power supply controller with integrated Geiger-Müller tube detector, featuring 4 selectable voltage ranges and I²C communication interface.

## Overview
This project implements an AVR-based controller for a high-voltage flyback power supply system designed for Geiger-Müller tube applications. The system provides precise voltage regulation across 4 selectable ranges, pulse counting capabilities, and remote control via I²C communication.

### Key Features
- High-Voltage Flyback Control: PWM-controlled flyback transformer with feedback regulation
- 4 Voltage Ranges: Relay-switched voltage ranges for different GM tube requirements
- Geiger Pulse Detection: Hardware pulse counting with debouncing and filtering
- I²C Slave Interface: Remote configuration and monitoring via I²C protocol
- Fault Protection: Overvoltage detection, current limiting, and safety shutdowns
- EEPROM Storage: Persistent configuration storage with wear leveling
- Status Indication: LED indicators for system status and fault conditions

## Hardware Architecture
### Core Components
- MCU: ATmega328P (TQFP-32) running at up to 20MHz
- Power Stage: IRF540N N-channel MOSFET with TC4420 gate driver
- Voltage Reference: LM4040 2.5V precision shunt regulator
- Amplifiers: LM321 operational amplifiers for signal conditioning
- Protection: SMAJ150A transient voltage suppressor
- Power Supply: LM317 adjustable linear regulator for 5V rail

### Voltage Range Switching
Four relay-controlled voltage ranges provide flexibility for different GM tube types.

## Pin Configuration
### I²C Communication
- **SDA**: PC4
- **SCL**: PC5
- **Default address**: 0x2A

### Control Outputs
- **PWM**: PB1 - Flyback MOSFET
- **Range Relays**: PD4, PD5, PB6, PB7 - Voltage range selection
- **Status LEDs**: PC2, PC3 - System status indication

### Analog Inputs
- **Feedback**: PC0 - Voltage feedback from HV output
- **Current Sense**: PC1 - Shunt current measurement

### Digital Inputs
- **Enable**: External enable signal
- **Range Select**: Hardware range selection inputs

### Programming interface
- **ISP header**: MISO (PB4), MOSI (PB3), SCK (PB5), RESET (PC6)

## Firmware Features

### PWM Control System
- **Frequency**: 31.25 kHz (configurable)
- **Resolution**: 8-bit (256 steps)
- **Duty Cycle Range**: 0-20% (safety limited)
- **Control Loop**: 1 kHz PI controller with derivative feedback

### Voltage Regulation
- **Feedback**: 10-bit ADC with 8× averaging
- **Setpoint Resolution**: ~1V steps (depending on range)
- **Regulation Accuracy**: ±1% typical
- **Response Time**: <100ms to 90% of setpoint
### Fault Protection
- **Overvoltage**: Programmable trip and clear thresholds per range
- **Overcurrent**: Hardware comparator with cycle-by-cycle limiting
- **Watchdog**: Hardware and software watchdog protection
### Geiger Pulse Detection
- **Input Conditioning**: Hardware debouncing and noise filtering
- **Count Resolution**: 32-bit counters
- **Rate Calculation**: CPS (counts per second) and CPM (counts per minute)
- **Dead Time Compensation:** Configurable dead time correction

## I²C Protocol
### Register Map
|Address    |Name               |Type   |Decsription                        |
|-----------|-------------------|-------|-----------------------------------|
|0x00-0x01  |FB_SET_CNT         |R/W    |Active feedback setpoint (counts)  |
|0x02-0x03  |FB_OV_TRIP_CNT     |R/W    |Active overvoltage trip threshold  |
|0x04-0x05  |FB_OV_CLEAR_CNT    |R/W    |Active overvoltage clear threshold |
|0x06-0x0D  |FB_SET_CNT_TAB     |R/W    |Per-range setpoint table (4×u16)   |
|0x0E-0x15  |FB_OV_TRIP_TAB     |R/W    |Per-range OV trip table (4×u16)    |
|0x16-0x1D  |FB_OV_CLEAR_TAB    |R/W    |Per-range OV clear table (4×u16)   |
|0x20       |RANGE_REQ          |W      |Range selection request (0-3)      |
|0x21       |RANGE_SRC          |R/W    |Range source (0=HW, 1=I2C, 2=AUTO) |
|0x40       |CMD                |W      |Command register                   |
|0x50-0x57  |STATUS             |R      |8-byte status block                |

### Commands (0x40)
- **0x01**: Save configuration to EEPROM
- **0x02**: Reset controller to PRECHARGE state

### Status Block (0x50-0x57)

```c
struct {
    uint8_t  run_state;     // Current controller state
    uint8_t  fault_code;    // Active fault conditions
    uint16_t fb_counts;     // Current feedback (ADC counts)
    int16_t  dcounts;       // Feedback derivative
    uint8_t  active_range;  // Currently selected range (0-3)
    uint8_t  range_busy;    // Range switching in progress
} status;
```

## Software Architecture
### Control States
- **PRECHARGE**: Soft-start phase with fixed low duty cycle
- **PI_CONTROL**: Closed-loop voltage regulation
- **PFM_HOLD**: Closed-loop voltage regulation
- **FAULT**: Safe shutdown with fault indication

### Main Control Loop (1 kHz)
1. **ADC Sampling**: Read feedback voltage with 8× averaging
2. **Fault Detection**: Check overvoltage and overcurrent conditions
3. **PI Controller**: Calculate required PWM duty cycle
4. **Range Management**: Handle relay switching with stability checks
5. **Pulse Counting**: Process Geiger tube pulses
6. **I²C Service**: Handle configuration updates and status requests

### Safety Features
- **Duty Cycle Limiting**: Hardware and software duty cycle caps
- **Fault Latching**: Persistent fault states requiring explicit reset
- **Graceful Degradation**: Automatic power reduction under fault conditions
- **Watchdog Protection**: System reset on firmware lockup

## Configuration and Calibration
### Voltage Scaling
The system uses a configurable voltage divider to scale HV output to ADC range:
- **Full Scale**: 1077.5V = 1023 ADC counts (default)
- **Resolution**: ~1.05V per ADC count
- **Calibration**: Adjustable via I²C configuration

### Range-Specific Parameters
Each voltage range has independent configuration:

- **Setpoint**: Target voltage in ADC counts
- **OV Trip**: Overvoltage fault threshold
- **OV Clear**: Overvoltage recovery threshold

### EEPROM Layout
Configuration is stored in EEPROM with wear leveling:

- **Slot Size**: 128 bytes per configuration
- **Wear Leveling**: Automatic slot rotation
- **CRC Protection**: 16-bit CRC for data integrity
- **Default Recovery**: Automatic fallback to safe defaults

## Safety Considerations
### Electrical Safety
- **High Voltage Warning**: Output voltages up to 1000V present electrocution hazard
- **Isolation**: Maintain proper isolation between HV and control circuits
- **Interlocks**: Implement appropriate safety interlocks for enclosure access
- ****:

### Operational Safety
- **Fault Monitoring**: Always monitor fault status during operation
- **Emergency Shutdown**: Implement external emergency stop capability
- **Thermal Management**: Ensure adequate cooling for power components

## Troubleshooting

### Common Issues
#### No HV Output
- Check enable signal
- Verify PWM generation on PB1
- Check MOSFET gate drive
- Verify transformer connections

#### I²C Communication Errors
- Verify bus pull-up resistors
- Check address conflicts
- Monitor bus timing
- Verify power supply stability

## Development and Programming
### Build Environment
- **Toolchain**: AVR-GCC with AVR-LibC
- **IDE**: PlatformIO
- **Programmer**: ISP programmer (USBasp, AVR Dragon, etc.)

## License
See LICENSE file for details.

⚠️ WARNING: This device generates potentially lethal high voltages. Only qualified personnel should operate, maintain, or modify this equipment. Always follow proper safety procedures when working with high-voltage systems.