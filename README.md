# Ant Hexapod

Firmware for a 6-legged ant-style robot built around a WeAct STM32H7 controller and an ESP32 companion board. This is an enhanced implementation based on [Dejan's excellent Arduino Ant Hexapod](https://howtomechatronics.com/projects/arduino-ant-hexapod-robot/).

## Table of Contents

- [Overview](#overview)
- [Getting Started](#getting-started)
- [Hardware Design](#hardware-design)
- [Firmware Architecture](#firmware-architecture)
- [Control Modes](#control-modes)
- [Building and Uploading](#building-and-uploading)
- [Runtime Behavior](#runtime-behavior)
- [Troubleshooting](#troubleshooting)
- [Documentation](#documentation)

## Overview

The Ant Hexapod is a 6-legged robot with comprehensive motion control and sensing capabilities.

**Robot Actuators (22 total):**
- 18 leg servos (3 per leg × 6 legs)
- 3 head servos
- 1 tail servo

**Key Features:**
- Real-time gait control with idle pose safety
- Flexible control modes: PC-based, wireless transmitter, manual tuning, or legacy single-byte commands
- Battery monitoring and power management
- Obstacle detection via sonar (default) or ToF (VL53L0x) sensor
- ESP32-C3 BLE/UART bridge for wireless telemetry and remote control
- High-frequency servo control (50 Hz default via PC mode)
 - Optional nRF24L01+ radio link (`NRF24_LINK`) for low-latency RF transmitter support

## Getting Started

### Prerequisites

**Hardware:**
- WeAct STM32H743 board (or equivalent STM32H7 series)
- ESP32-C3 SuperMini board (BLE bridge)
- 18 servo motors (leg)
- 4 servo motors (head + tail)
- Power supply: **15A recommended** (minimum 10A; 8A is insufficient)
- Sonar module (HC-SR04) or ToF sensor (VL53L0x)

**Software:**
- PlatformIO CLI or VS Code with PlatformIO extension
- Python 3.7+
- STM32 Arduino framework (auto-installed by PlatformIO)

### Installation

1. **Clone the repository:**
   ```bash
   git clone https://github.com/titolini72/Ant-Hexapod.git
   cd Ant-Hexapod
   ```

2. **Update STM32 Servo Library:**
   The default STM32 Servo library supports only 12 servos; this project uses 22. Update the servo limit:
   ```bash
   # Find and edit the Servo library file:
   ~/.platformio/packages/framework-arduinoststm32/libraries/Servo/src/Servo.h
   # Change: static const uint8_t nbServoMax = 12;
   # To:     static const uint8_t nbServoMax = 24;
   ```

3. **Build the firmware:**
   ```bash
   platformio run --environment weact_mini_h743vitx
   platformio run --environment esp32c3_supermini
   ```

4. **Upload to boards:**
   ```bash
   platformio run --target upload --environment weact_mini_h743vitx
   platformio run --target upload --environment esp32c3_supermini
   ```

## Hardware Design

### Design Choices

| Component | Choice | Rationale |
|-----------|--------|-----------|
| **Main Controller** | STM32H743 WeAct | More powerful than STM32F4; supports 22 software PWM servos |
| **Wireless** | ESP32-C3 | iPhone-compatible BLE; replaces HC-05 (HC-05 lacks BLE support) |
| **Obstacle Sensor** | Sonar (HC-SR04) default; ToF optional | Sonar matches ant aesthetic; ToF more reliable for detection |
| **Power** | 15A supply | Typical current draw during motion exceeds 10A; 8A insufficient |

### Servo Management

Servo control uses **software PWM (bit banging)** rather than hardware PWM, allowing 22 simultaneous servo signals. This requires the STM32 Servo library update (see Installation).

### Power Considerations

- **Idle draw:** ~2-3A
- **Active motion:** ~8-12A
- **Peak draw:** Can exceed 15A during aggressive motion
- Recommend 15A+ supply for reliable operation

## Firmware Architecture

### Current Firmware Layout

The repository contains two distinct firmware families:

**STM32 / WeAct firmware** (`src/Weact/`)
- Robot model and kinematics
- Gait control and idle pose
- Power management and sensor handling
- Default control mode: framed servo streaming (`CMD_PC`)

**ESP32 firmware** (`src/ESP32/`)
- `esp32c3_supermini`: BLE ↔ UART bridge for STM32 communication
- `espwroom32`: Experimental PS4 controller and telemetry code

### Repository Structure

```
include/           Public interfaces for robot parts and managers
src/Weact/         STM32 firmware (robot logic, gait, sensors)
src/ESP32/         ESP32 firmware (BLE bridge, transmitter experiment)
test/              Test area placeholder
config.h           Shared compile-time configuration
platformio.ini     PlatformIO build environments and compiler flags
CODE_DOCUMENTATION.md  Detailed architecture and implementation notes
```

## Control Modes

The STM32 firmware selects its runtime control protocol in `config.h` using the `COMMANDER` parameter.

| Mode | Use Case | Protocol | Notes |
|------|----------|----------|-------|
| **`CMD_PC`** | Host-based control (default) | Framed servo angles (22 values) | Best for motion planning; 50 Hz max rate; CSV-based orchestration |
| **`CMD_TRANSMITTER`** | Wireless joystick/button input | Framed joystick payloads | Requires paired ESP32-C3 transmitter |
| **`CMD_MIT_APPINVENTOR`** | Mobile app control (legacy) | Single-byte commands | Limited; kept for backward compatibility |
| **`CMD_MANUAL_TEST`** | Tuning and calibration | Direct serial commands | Tune individual servo angles and offsets |

### Default Configuration

```
TRANSMISSION_BLE     = enabled    (STM32 ↔ ESP32 UART bridge)
COMMANDER         = CMD_PC     (host-based framed control)
SONAR_DETECTION   = enabled    (HC-SR04 obstacle detection)
TOF_DETECTION     = disabled   (VL53L0x; set to 1 to enable)
```

### Communication Frame Format (CMD_PC)

```
[Wait (ms, 2 bytes)] [Servo 1 angle] ... [Servo 22 angle]
↓                     ↓                    ↓
Timing/interpolation  0-180°, 1 byte each (22 total)
```

**Response:** ACK packet sent back to host after frame processing.

**Timeout Behavior:** If no valid frame arrives within 500 ms, servos progressively return to idle pose (prevents runaway behavior).

## Building and Uploading

### Build Targets

PlatformIO environments are defined in `platformio.ini`:

| Environment | Target | Description |
|-------------|--------|-------------|
| `weact_mini_h743vitx` | STM32H743 | Main robot firmware |
| `esp32c3_supermini` | ESP32-C3 | BLE/UART bridge |
| `espwroom32` | ESP32 WROOM | PS4 controller experiment |

### Typical Build Commands

```bash
# Build STM32 firmware
platformio run --environment weact_mini_h743vitx

# Build and upload STM32 firmware
platformio run --target upload --environment weact_mini_h743vitx

# Build ESP32-C3 BLE bridge
platformio run --environment esp32c3_supermini

# Build and upload ESP32-C3
platformio run --target upload --environment esp32c3_supermini
```

## Runtime Behavior

### STM32 Startup Sequence

On power-up, the STM32 firmware executes the following initialization:

1. **Hardware initialization** – GPIO, timers, UART, SPI
2. **Servo attachment verification** – `Hexapod::check()` confirms all 22 servos respond
3. **LED and speed initialization** – Set default motion speed and indicator LEDs
4. **Idle pose** – Move servos to safe resting position
5. **Sensor startup** – Initialize sonar or ToF obstacle detection
6. **ESP UART bridge** – Start BLE/UART relay (if `TRANSMISSION_BLE` enabled)
7. **Commander registration** – Install handler for selected control mode

### CMD_PC Mode Operation

- Accepts framed payloads: 1 timing field (ms) + 22 servo angles
- Processes frames at up to 50 Hz (20 ms minimum between frames)
- Returns ACK after each frame
- **Watchdog:** If no valid frame within 500 ms, gradually returns to idle pose

### ESP32-C3 Bridge Operation

- Relays framed packets between BLE peer and STM32 UART
- Validates CRC where applicable
- Forwards ACK and status information bidirectionally
- Maintains BLE connection state and packet queuing

## Troubleshooting

### Build Issues

**"Servo.h: nbServoMax is 12"**
- **Cause:** STM32 Servo library has hardcoded 12-servo limit
- **Solution:** Edit `~/.platformio/packages/framework-arduinoststm32/libraries/Servo/src/Servo.h`, change `nbServoMax` from 12 to 24

**Upload fails with "Port not found"**
- **Cause:** STM32 device not in DFU mode or COM port not recognized
- **Solution:** 
  - Hold BOOT0 button while plugging in USB (DFU mode)
  - Check Device Manager (Windows) or `lsusb` (Linux) for STM device
  - Reinstall STM32 USB drivers if needed

### Runtime Issues

**Servos don't respond or twitch randomly**
- **Cause:** Power supply insufficient; servos brownout
- **Solution:** Use 15A+ power supply; verify all servo power connectors
- **Workaround:** Reduce motion speed temporarily to limit current draw

**"Servo attachment verification failed" at startup**
- **Cause:** One or more servos not powered, disconnected, or defective
- **Solution:** Check all servo connectors; test individual servos with manual commands

**Robot doesn't return to idle after 500 ms without commands**
- **Cause:** `CMD_PC` watchdog disabled or servo timing saturated
- **Solution:** Verify `COMMANDER=CMD_PC` in `config.h`; ensure no other processes sending frames

**BLE connection drops frequently**
- **Cause:** ESP32-C3 UART buffer overflow or clock mismatch
- **Solution:** 
  - Reduce frame rate to 20 Hz (increase inter-frame delay)
  - Check UART baud rate matches on both boards
  - Verify antenna placement on ESP32-C3

### Manual Servo Tuning (CMD_MANUAL_TEST)

Before tuning a robot for a specific model, first switch to `CMD_MANUAL_TEST` in `config.h` and use the serial console to adjust each servo individually. This is the recommended step for model-specific calibration, because each ant build can have slightly different mechanical offsets, leg lengths, and neutral angles. Tune one servo at a time, verify the movement, and only then return to the normal `CMD_PC` control mode.

Manual Test command format:
- First letter: `L` / `R` / `H` / `T` = Left / Right / Head / Tail
- Second letter: `F` / `M` / `B` / `P` / `R` / `G` / ` ` = Front / Middle / Back / Pitch / Roll / Grip / space
- Third field: `F` / `T` / `E` / ` ` = Femur / Tibia / Feet / space for Tail
- Last 3 digits: angle in degrees, with a leading `0` when needed

Examples:
- `LFE020` = Left Front Foot at 20°
- `RMT080` = Right Middle Tibia at 80°
- `HP 050` = Head Pitch at 50°
- `T  095` = Tail at 95°

If servos don't move smoothly or exhibit jerky behavior:

1. Enable `CMD_MANUAL_TEST` in `config.h`
2. Send direct serial commands to adjust per-servo calibration
3. Update `Hexapod::Hexapod()` constructor with corrected offsets:
   - `min`, `max` (safe range per servo)
   - `idle` (neutral position)
   - `offset` (calibration trim)

## Documentation

### Detailed Reference

- [CODE_DOCUMENTATION.md](CODE_DOCUMENTATION.md) – Comprehensive architecture, class hierarchy, and implementation details

### Key Configuration Files

- [config.h](config.h) – Compile-time control mode, sensor, and feature flags
- [platformio.ini](platformio.ini) – Build environments, optimization flags, and library dependencies
- [src/Weact/main.cpp](src/Weact/main.cpp) – STM32 firmware entry point and main loop
- [src/ESP32/main.cpp](src/ESP32/main.cpp) – ESP32-C3 bridge firmware entry point

### References

- [Dejan's Arduino Ant Hexapod](https://howtomechatronics.com/projects/arduino-ant-hexapod-robot/) – Original design inspiration
- [Dejan's DIY Arduino RC Transmitter](https://howtomechatronics.com/projects/diy-arduino-rc-transmitter/) – Transmitter reference
- [WeAct STM32H7 Repo](https://github.com/weactstudio/weactstudio.ministm32f4x1) – Hardware documentation
- [Espressif ESP32-C3 Docs](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c3/) – ESP32-C3 reference

## Validation Status

The current firmware has been built and uploaded successfully:

- ✅ `weact_mini_h743vitx` (STM32H743)
- ✅ `esp32c3_supermini` (ESP32-C3)

Both targets passed compilation and on-device verification in the current session.

---

**Questions or issues?** Check [CODE_DOCUMENTATION.md](CODE_DOCUMENTATION.md) for detailed architecture, or open an issue in the repository.
