# Ant Hexapod Code Documentation

## 1. Project Overview

This firmware controls a 6-legged ant-style robot with:
- 18 leg servos (3 per leg)
- 3 head servos (roll, pitch, grip)
- 1 tail servo
- power monitoring
- obstacle detection (ultrasonic or ToF, compile-time selected)
- a serial/BLE bridge via ESP32 for remote control and telemetry

Main control firmware targets STM32H7 boards (WeAct), while a separate ESP32 firmware handles BLE transport and UART bridging.

Primary files:
- [src/Weact/main.cpp](src/Weact/main.cpp)
- [src/ESP32/main.cpp](src/ESP32/main.cpp)
- [config.h](config.h)
- [platformio.ini](platformio.ini)

## 2. Build and Targets

PlatformIO environments are defined in [platformio.ini](platformio.ini):

- weact_mini_h743vitx
  - Platform: ststm32
  - Framework: Arduino
  - Source filter: WeAct code only
  - Libraries: ArduinoJson, VL53L0X

- esp32c3_supermini
  - Platform: espressif32
  - Framework: Arduino
  - Source filter: ESP32 code only
  - Libraries: NimBLE-Arduino

- espwroom32
  - Platform: espressif32
  - Framework: Arduino
  - Source filter: ESP32 code only
  - Libraries: NimBLE-Arduino, PS4 controller host

## 3. Compile-Time Configuration

Configuration is centralized in [config.h](config.h).

### 3.1 Logging and Modes

- SERIAL_DEBUG enables DEBUG_PRINT macros
- TRANSMISSION_BLE enables the ESP UART link on the STM32 side
- COMMANDER selects the active control protocol:
  - CMD_MIT_APPINVENTOR = single-byte command mode
  - CMD_PC = framed servo-angle streaming mode
  - CMD_TRANSMITTER = framed joystick/button mode
  - CMD_MANUAL_TEST = direct serial bone test mode

Current default in [config.h](config.h) is `#define COMMANDER CMD_PC` with `TRANSMISSION_BLE` enabled.

### 3.2 Sensor Selection

Two obstacle sensors are supported, but only one must be enabled:
- SONAR_DETECTION
- TOF_DETECTION

Current STM32H7 config enables `SONAR_DETECTION` and leaves `TOF_DETECTION` disabled.

### 3.3 STM32 Pins and IDs

[config.h](config.h) defines:
- servo channels S1..S24
- LED pins
- ADC constants
- sonar pins and optional ToF I2C/IRQ pins
- leg and bone index IDs used by manual test parser
- UART selections for the STM32/ESP32 bridge (`ESP` on STM32, `WEACT` on ESP32)

## 4. Runtime Architecture

### 4.1 STM32 Main Loop Flow

Implemented in [src/Weact/main.cpp](src/Weact/main.cpp):

1. Initialize serial and robot hardware
2. Validate actuator attachment using Hexapod.check()
3. Put LEDs into running mode and set the default robot speed
4. Move the robot into its idle pose
5. Initialize the selected sensor manager (sonar or ToF)
6. Initialize the ESP UART link when `TRANSMISSION_BLE` is enabled
7. Register the serial frame handler for the selected `COMMANDER` mode
8. In the main loop:
  - poll inbound serial/BLE data through `SerialProtocolHandler`
  - run commander-specific control flow
  - update power alert and LED patterns
  - run all periodic loops:
   - powerManager.loop()
   - sensorManager.loop()
   - ledON.loop()
   - ledLOW.loop()

Current default behavior (`CMD_PC`) does not use the legacy high-level command switch. It accepts framed servo-angle payloads and drives all 22 actuators directly.

### 4.2 Control Modes

The STM32 side supports multiple compile-time control paths in [src/Weact/main.cpp](src/Weact/main.cpp):

- `CMD_PC`
  - Expected payload: 2-byte wait field + 22 servo angles
  - `handleServoPayload()` applies one full body frame through `Hexapod::servoApplyAngles()`
  - if no valid frame is received for 500 ms, `processServoFrameTimeout()` returns the robot progressively to `idlePose()`

- `CMD_TRANSMITTER`
  - Expected payload: 23 bytes carrying joystick, potentiometer, button, and switch data
  - the right stick drives leg heading and gait speed
  - the left stick drives head roll/pitch
  - one potentiometer controls grip unless a button triggers `bite()`

- `CMD_MIT_APPINVENTOR`
  - Expected payload: one command byte, optionally overloaded with speed when byte > 100
  - command values map to the legacy discrete behaviors listed below

- `CMD_MANUAL_TEST`
  - No framed control logic is used
  - newline-terminated ASCII commands directly move one selected bone

Legacy command values used only in `CMD_MIT_APPINVENTOR` mode:

- 0: autonomous behavior (idle unless obstacle detected, then attack sequence)
- 1: move diagonal forward-left
- 2: move forward
- 3: move diagonal forward-right
- 4: strafe left
- 5: strafe right
- 6: move diagonal back-left
- 7: move backward
- 8: move diagonal back-right
- 9: fold pose
- 10: head idle
- 11: drop (grip positive)
- 12: grab (grip negative)
- 13: bite
- 14: head motion demo
- 20: tail idle
- 21: tail wag
- 30: explicit attack sequence

Speed protocol:
- if received byte > 100, speed becomes byte - 100
- else byte is interpreted as command

## 5. Core Robotics Model

### 5.1 Hardware, Motion Profile, Bone

Defined in [include/Bone.hpp](include/Bone.hpp), implemented in [src/Weact/Bone.cpp](src/Weact/Bone.cpp).

- Hardware
  - Wraps one Servo instance
  - setup() attaches servo with pulse range
  - move(angle) writes angle and stores current value

- Mouvement
  - Parses movement profile from JSON text into StaticJsonDocument
  - get(tag) retrieves named values such as idle, fold, forward, backward, attack
  - Throws runtime_error if tag is missing

- Bone
  - Binds one Hardware and one Mouvement profile
  - Provides move, angle, attached, and get(tag)

### 5.2 Leg

Defined in [include/Leg.hpp](include/Leg.hpp), implemented in [src/Weact/Leg.cpp](src/Weact/Leg.cpp).

State machine:
- UNKNOWN, IDLE, FOLD, UNFOLD, MOVE, PREPARE, ATTACK

Key behavior:
- setup()
  - attaches all three servos
  - moves leg to fold pose
- up_and_move(directionDegrees, phase)
  - computes direction vector from global heading
  - uses side sign to mirror left/right behavior
  - runs smooth swing and stance interpolation over CYCLE_LEN
  - updates femur, tibia, foot positions per phase
- fold(), unfold(), idle()
  - gradual interpolation toward target postures
- prepare_attack(), attack()
  - fast strike preparation and attack transitions

Gait constants:
- CYCLE_LEN = 80
- SWING_LEN = 40

### 5.3 Head

Defined in [include/Head.hpp](include/Head.hpp), implemented in [src/Weact/Head.cpp](src/Weact/Head.cpp).

Actuators:
- roll
- pitch
- grip

Capabilities:
- idle centering
- smooth percentage-based roll/pitch/grip control
- scripted demo routine
- bite sequence

Notes:
- Percentage APIs map range -100..100 into profile limits from JSON
- Motion is time-stepped with speed-controlled intervals

### 5.4 Tail

Defined in [include/Tail.hpp](include/Tail.hpp), implemented in [src/Weact/Tail.cpp](src/Weact/Tail.cpp).

Capabilities:
- idle
- wag (oscillation between left and right)
- smooth percentage move -100..100

### 5.5 Hexapod Aggregator

Defined in [include/Hexapod.hpp](include/Hexapod.hpp), implemented in [src/Weact/Hexapod.cpp](src/Weact/Hexapod.cpp).

Responsibilities:
- owns all servo hardware objects and movement profiles
- constructs 6 legs + head + tail
- initializes leg arrays and phase offsets for tripod gait
- exposes explicit constants for array sizing (`kLegCount = 6`, `kBoneCount = 22`)
- provides high-level APIs:
  - idlePose
  - foldPose
  - move
  - servoApplyAngles
  - attack_sequence

Tripod gait setup:
- Tripod A: LF, RM, LB at phase 0
- Tripod B: RF, LM, RB at phase 40 (half-cycle shift)

## 6. Managers

### 6.1 LEDManager

Defined in [include/LEDManager.hpp](include/LEDManager.hpp), implemented in [src/Weact/LEDManager.cpp](src/Weact/LEDManager.cpp).

- Supports pattern-based non-blocking blinking via loop()
- setMode() updates pattern only when changed
- Built-in patterns in header:
  - BLINK_RUNNING
  - BLINK_SOS
  - BLINK_ERROR

Usage in current firmware:
- `ledON` indicates general running/error state
- `ledLOW` switches to SOS when `PowerManager::alert()` reports low battery

### 6.2 PowerManager

Defined in [include/PowerManager.hpp](include/PowerManager.hpp), implemented in [src/Weact/PowerManager.cpp](src/Weact/PowerManager.cpp).

- Reads battery voltage from ADC
- Converts ADC reading to volts:
  voltage = raw * (ADC_MAX_VOLTAGE / ADC_RANGE) * ADC_RATIO
- Sets alert flag when below threshold
- Exposes last_voltage() and alert()

### 6.3 SensorManager (Ultrasonic)

Defined in [include/SensorManager.hpp](include/SensorManager.hpp), implemented in [src/Weact/SensorManager.cpp](src/Weact/SensorManager.cpp).

- Designed for HC-SR04 style trig/echo sensor
- Uses CHANGE interrupt on echo pin
- ISR captures pulse start/end timestamps
- loop() periodically triggers pulse, computes distance in cm
- timeout path sets distance to 0xFFFF sentinel

Obstacle logic:
- isObstacleAhead() returns true when distance is below configured threshold

Current default build uses this manager.

### 6.4 VL53L0XManager (ToF)

Defined in [include/VL53L0XManager.hpp](include/VL53L0XManager.hpp), implemented in [src/Weact/VL53L0XManager.cpp](src/Weact/VL53L0XManager.cpp).

Design:
- Interrupt-driven data-ready handling via VL53L0X GPIO1
- ISR is minimal: increments pending interrupt count
- I2C reads happen in loop() (main context), not inside ISR

Initialization steps:
- configure Wire pins (SDA/SCL)
- optional XSHUT reset
- sensor init and optional address change
- set timing budget
- configure interrupt registers for active-low data-ready
- attach MCU interrupt and start continuous ranging

Data path:
- on interrupt, pending count increases
- loop consumes pending events
- read continuous range in mm
- timeout sets 0xFFFF invalid value
- clear sensor interrupt register to re-arm GPIO1

API highlights:
- setup()
- loop()
- isObstacleAhead()
- lastDistanceMm()
- hasNewSample() and clearNewSampleFlag()
- restartContinuous()

This manager is compiled only when `TOF_DETECTION` is enabled.

### 6.5 NRF24Manager (nRF24L01+ radio link)

Defined in [include/NRF24Manager.hpp](include/NRF24Manager.hpp), implemented in [src/Weact/NRF24Manager.cpp](src/Weact/NRF24Manager.cpp).

Purpose:
- Optional nRF24L01+ radio link for low-latency RF transmitter/receiver support.
- Compiled only when `NRF24_LINK` is defined in `config.h`.

Key APIs:
- `NRF24Manager(SPIClass& spi, uint32_t cePin, uint32_t csnPin, uint32_t irqPin)` — constructor
- `bool begin()` — initialize SPI and radio, configure channel, addresses, PA/data rate/retries, enable dynamic payloads, start listening
- `using FrameHandler = void (*)(const uint8_t* payload, uint16_t len)` and `void setFrameHandler(FrameHandler)` — register callback for received frames
- `void poll()` — check radio for incoming frames and invoke handler
- `bool isReady() const` — returns initialization state

Behavior notes:
- Uses the RF24 library; supports dynamic payloads up to 32 bytes. Addresses, channel, PA level, data rate and retry settings are read from `config.h` constants (e.g. `NRF24_RX_ADDRESS`, `NRF24_TX_ADDRESS`, `NRF24_CHANNEL`).
- `poll()` reads available dynamic payloads and forwards them to the registered frame handler; invalid payload sizes are discarded.
- The IRQ pin is used to reduce SPI polling when supported by the hardware.

File map additions:
- [include/NRF24Manager.hpp](include/NRF24Manager.hpp)
- [src/Weact/NRF24Manager.cpp](src/Weact/NRF24Manager.cpp)

## 7. BLE Bridge Firmware (ESP32)

Implemented in [src/ESP32/main.cpp](src/ESP32/main.cpp).

### 7.1 ESP32-C3 Variant

- Uses `BLEManager`, `SerialManager`, and `LEDManager`
- Supports server or client role through `BLE_FLAVOR`
- Bridges complete framed packets between BLE and the STM32 UART
- In server mode:
  - accepts BLE writes on the RX characteristic
  - validates CRC for framed modes
  - forwards valid frames to STM32 over UART
  - waits for STM32 ACK and notifies the BLE peer with a compact ACK payload
- Frames coming back from STM32 are forwarded to the BLE peer through `onUartFrame()`
- Trace frames from STM32 are printed to the ESP32 USB console through `onUartTrace()`

### 7.2 ESP-WROOM Variant

- Uses PS4 controller library
- prints joystick and event telemetry
- includes paired-device cleanup utility
- does not currently bridge PS4 data into the STM32 protocol

## 8. Manual Test Mode

When `COMMANDER == CMD_MANUAL_TEST` is selected in [config.h](config.h), [src/Weact/main.cpp](src/Weact/main.cpp) parses serial text commands to directly move specific bones.

Input parsing maps:
- Legs
  - side: L(eft) or R(ight)
  - segment: F(ront), M(iddle), B(Back) or head selectors P, R, G
  - bone: F(emur), T(ibia), (f)E(et)
  - angle: 3 digits (always add leading 0 <100)
  Ex LFE (Left Front Feet at 20 Deg)
- Head
  - H(ead)
  - P(itch), R(oll), G(rip)
  - Space
  - angle: 3 digits (always add leading 0 <100)
  Ex HP 050 (Head Pitch 50 Deg)
- Tail
  - T(Tail)
  - 2 Space
  - angle: 3 digits (always add leading 0 <100)
  Ex T  095 (Tail at 95 Deg)

On valid command, selected bone index receives direct move(angle).

Before tuning the robot for a specific model, first switch to `COMMANDER == CMD_MANUAL_TEST` in [config.h](config.h) and use the serial console to adjust each servo individually. This is the safest way to calibrate the servos for the actual robot geometry, because every ant model can have slightly different leg offsets, joint limits, and neutral angles. Start with one leg or one head axis at a time, then validate the full motion before moving to the final `CMD_PC` control profile.

Manual Test command format:
- First letter: `L`/`R`/`H`/`T` = Left / Right / Head / Tail
- Second letter: `F`/`M`/`B`/`P`/`R`/`G`/` ` = Front / Middle / Back / Pitch / Roll / Grip / space
- Third field: `F`/`T`/`E`/` ` = Femur / Tibia / Feet / space for Tail
- Last 3 digits: servo angle in degrees, with a leading `0` when needed

Examples:
- `LFE020` = Left Front Foot at 20°
- `RMT080` = Right Middle Tibia at 80°
- `HP 050` = Head Pitch at 50°
- `T  095` = Tail at 95°

Parsing behavior (current):
- command parsing is triggered on newline/carriage return, or when the buffer reaches max length
- parser rejects short frames (`< 6` chars) before indexing command fields
- buffer remains null-terminated while reading
- newline characters are discarded before field decoding

## 9. Error and Safety Behavior

- Actuator attachment failures in startup halt execution in an infinite error blink loop
- Power alert switches low-battery LED to SOS pattern
- Sensor timeout values use sentinel 0xFFFF
- Debug macros compile out unless SERIAL_DEBUG is set
- In `CMD_PC`, loss of incoming servo frames for 500 ms causes a controlled fallback to the idle pose

## 10. Known Implementation Notes

- The default checked-in build path is `CMD_PC` on STM32H7 with sonar detection enabled.
- The codebase still contains alternate commander paths (`CMD_MIT_APPINVENTOR`, `CMD_TRANSMITTER`, `CMD_MANUAL_TEST`) behind compile-time switches in [config.h](config.h).
- The ToF manager constructor takes both measurement period and obstacle threshold in millimeters, but the current call site in [src/Weact/main.cpp](src/Weact/main.cpp) passes `TOF_DETECTION_RANGE` as the fifth argument only. If ToF is re-enabled, review that constructor call so the intended threshold and sampling period are explicit.

## 11. File Map

Headers:
- [include/Bone.hpp](include/Bone.hpp)
- [include/Leg.hpp](include/Leg.hpp)
- [include/Head.hpp](include/Head.hpp)
- [include/Tail.hpp](include/Tail.hpp)
- [include/Hexapod.hpp](include/Hexapod.hpp)
- [include/LEDManager.hpp](include/LEDManager.hpp)
- [include/PowerManager.hpp](include/PowerManager.hpp)
- [include/SensorManager.hpp](include/SensorManager.hpp)
- [include/VL53L0XManager.hpp](include/VL53L0XManager.hpp)

WeAct sources:
- [src/Weact/main.cpp](src/Weact/main.cpp)
- [src/Weact/Bone.cpp](src/Weact/Bone.cpp)
- [src/Weact/Leg.cpp](src/Weact/Leg.cpp)
- [src/Weact/Head.cpp](src/Weact/Head.cpp)
- [src/Weact/Tail.cpp](src/Weact/Tail.cpp)
- [src/Weact/Hexapod.cpp](src/Weact/Hexapod.cpp)
- [src/Weact/LEDManager.cpp](src/Weact/LEDManager.cpp)
- [src/Weact/PowerManager.cpp](src/Weact/PowerManager.cpp)
- [src/Weact/SensorManager.cpp](src/Weact/SensorManager.cpp)
- [src/Weact/VL53L0XManager.cpp](src/Weact/VL53L0XManager.cpp)

ESP32 source:
- [src/ESP32/main.cpp](src/ESP32/main.cpp)
