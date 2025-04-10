/**
 * @file        main.cpp
 * @author      Pierre-Yves Mordret <titolini72@gmail.com>
 * @brief       Main entry point for the Arduino Ant Hexapod robot firmware.
 * @version     1.0
 * @date        2026-01-02
 * @copyright   Copyright (c) 2026 Pierre-Yves Mordret
 */

/*
    === Arduino Ant Robot / Hexapod ===
    by Dejan, https://howtomechatronics.com
*/
#include <Arduino.h>
#include "LEDManager.hpp"
#include "Bone.hpp"
#include "Hexapod.hpp"
#include "PowerManager.hpp"
#if defined(BLUETOOTH_BLE)
#include "SerialProtocolHandler.hpp"
#endif
#if defined(NRF24_LINK)
#include "NRF24Manager.hpp"
#endif

#if defined(SONAR_DETECTION)
#include "SensorManager.hpp"
#endif

#if defined(TOF_DETECTION)
#include "VL53L0XManager.hpp"
#endif

LEDManager ledON(LED_ON_PIN);
LEDManager ledLOW(LED_LOW_PIN);

Hexapod myAnt;

namespace {
constexpr uint8_t SPEED_COMMAND_OFFSET = 100;
} // namespace

unsigned long lastManualTimestamp = millis();

static constexpr uint8_t WAIT_FIELD_SIZE = 2;
static constexpr uint8_t SERVO_ANGLE_COUNT = static_cast<uint8_t>(Hexapod::kBoneCount);
#if (COMMANDER == CMD_PC)
static constexpr uint16_t DEFAULT_SERVO_FRAME_PAYLOAD_SIZE = WAIT_FIELD_SIZE + SERVO_ANGLE_COUNT;
// When no servo frame is received for this long, fall back to the idle pose.
static constexpr unsigned long SERVO_FRAME_IDLE_TIMEOUT_MS = 500;
static unsigned long lastServoFrameTimestamp = 0;
#elif (COMMANDER == CMD_TRANSMITTER)
// 4 joysticks * 2 bytes each + 2 potentiometers * 2 bytes each + 6 buttons * 1 byte each 
// + 1 switch byte + 4 timestamp bytes = 4*2 + 2*2 + 6*1 + 1 + 4 = 8 + 4 + 6 + 1 + 4 = 23 bytes
static constexpr uint16_t DEFAULT_TRANSMITTER_PAYLOAD_SIZE = NUM_JOYSTICKS * 2 + NUM_POTENTIOMETERS * 2 + NUM_BUTTONS + 1 + 4;
#elif (COMMANDER == MIT_APPINVENTOR)
static constexpr uint16_t DEFAULT_MIT_PAYLOAD_SIZE = 2; 
#endif

int command = 0;
unsigned char speed = DEFAULT_SPEED;

#if defined(BLUETOOTH_BLE)
const uint8_t* s_buffer = nullptr;
#endif

#if (COMMANDER == CMD_MANUAL_TEST)
char manualBuffer[MAX_BUFFER];
int manualCount = 0; // for incoming serial data
unsigned char manualRotation = 0;
Bone* antBones;
#endif

// Analog pin A0, delay 1000ms, threshold 11V, ratio 4.0
PowerManager powerManager(ADC_PIN, ADC_POLL_DELAY, ADC_VOLTAGE_THRESHOLD, ADC_RATIO);
#if defined(SONAR_DETECTION)
SensorManager sensorManager(SONAR_TRIG, SONAR_ECHO, SONAR_MEASURE_DELAY, SONAR_DETECTION_RANGE); // Ultrasonic sensor
#endif
#if defined(TOF_DETECTION)
VL53L0XManager sensorManager(TOF_I2C_SCL, TOF_I2C_SDA, TOF_IRQ, -1, TOF_DETECTION_RANGE); // ToF sensor
#endif

#if defined(NRF24_LINK)
#if NRF24_SPI_USE_CUSTOM_PINS
SPIClass nrf24Spi(NRF24_MOSI_PIN, NRF24_MISO_PIN, NRF24_SCK_PIN);
#else
#ifndef NRF24_SPI_INSTANCE
#error "Define NRF24_SPI_INSTANCE when NRF24_SPI_USE_CUSTOM_PINS is 0."
#endif
SPIClass& nrf24Spi = NRF24_SPI_INSTANCE;
#endif

NRF24Manager radioLink(nrf24Spi, NRF24_CE_PIN, NRF24_CSN_PIN, NRF24_IRQ_PIN);
#endif

#if (COMMANDER == CMD_PC)
SerialProtocolHandler serialReader(ESP, DEFAULT_SERVO_FRAME_PAYLOAD_SIZE); // Frames arrive from the ESP32 over the UART link
#elif (COMMANDER == CMD_TRANSMITTER)
#if defined(BLUETOOTH_BLE)
SerialProtocolHandler serialReader(ESP, DEFAULT_TRANSMITTER_PAYLOAD_SIZE);
#endif
#elif (COMMANDER == MIT_APPINVENTOR)
SerialProtocolHandler serialReader(ESP, DEFAULT_MIT_PAYLOAD_SIZE, '#');
#endif

struct MoveConfig {
  float direction;
  int roll;
  int pitch;
  bool headIdle;
};

void move(const MoveConfig& cfg) {
  myAnt.direction = cfg.direction;
  myAnt.move();

  if (cfg.headIdle) {
    myAnt.getHead().idle();
  } else {
    myAnt.getHead().roll_head(cfg.roll);
    myAnt.getHead().pitch_head(cfg.pitch);
  }

  myAnt.getTail().wag();
}

#if (COMMANDER == CMD_PC)
void handleServoPayload(const uint8_t* payload, uint16_t len) {
  if (len != serialReader.FramePayloadSize()) {
    return;
  }

  const uint16_t wait_ms = (uint16_t)payload[0] | ((uint16_t)payload[1] << 8);

  uint8_t angles[Hexapod::kBoneCount];
  for (uint8_t i = 0; i < Hexapod::kBoneCount; i++) {
    angles[i] = payload[2 + i];
  }

  DEBUG_PRINT("SEQ wait_ms=");
  DEBUG_PRINT(wait_ms);
  DEBUG_PRINT(" |");
  for (uint8_t i = 0; i < Hexapod::kBoneCount; i++) {
    DEBUG_PRINT(" ");
    DEBUG_PRINT(angles[i]);
  }
  DEBUG_PRINTLN();

  myAnt.servoApplyAngles(angles);
  lastServoFrameTimestamp = millis();
}

// Returns the hexapod to its idle pose when no servo frame has been received
// within SERVO_FRAME_IDLE_TIMEOUT_MS. idlePose() animates progressively, so it
// must be driven every loop until the legs reach the idle pose.
void processServoFrameTimeout() {
  if ((millis() - lastServoFrameTimestamp) >= SERVO_FRAME_IDLE_TIMEOUT_MS) {
    myAnt.idlePose();
  }
}
#elif (COMMANDER == CMD_TRANSMITTER)

// Convert raw joystick value (0-1024) to degrees (-60 to 60)
float rawToHeadDegrees(int16_t raw) {
    return (raw - 512) * 120.0f / 1024.0f;
}

float clampf(float d, float min, float max) {
  const float t = d < min ? min : d;
  return t > max ? max : t;
}
// Inputs: rawX, rawY from stick
// Output: angleDeg (0..360), speed (0..1)
void processLegStick(uint16_t rawX, uint16_t rawY)
{
    const float CENTER_X = 512.0f;
    const float CENTER_Y = 512.0f;
    const float MAX_DELTA_X = 512.0f;
    const float MAX_DELTA_Y = 512.0f;
    const float DEADZONE = 0.15f;   // 15%
    const float GAMMA    = 1.6f;   // 1.7f

    // 1. Center and normalize to [-1, +1]
    float x = ((float)rawX - CENTER_X) / MAX_DELTA_X;
    float y = ((float)rawY - CENTER_Y) / MAX_DELTA_Y;

    x = clampf(x, -1.0f, 1.0f);
    y = clampf(y, -1.0f, 1.0f);

    // 2. Magnitude
    float r = fmaxf(fabs(x), fabs(y));

    // 3. Deadzone
    if (r < DEADZONE)
    {
        Leg** legs = myAnt.getLegArray();

        for (size_t i = 0; i < myAnt.getLegCount(); i++)
          legs[i]->idle();
        myAnt.getTail().idle();
        return;
    }

    // 4. Normalize speed after deadzone
    float speed = (r - DEADZONE) / (1.0f - DEADZONE);
    speed = powf(speed, GAMMA); // Optional: apply gamma curve for finer control at low speeds

    // 5. Compute heading
    // 0° = forward
    // 90° = right
    // 180° = backward
    // 270° = left
    float angleDeg = atan2f(x, y) * 180.0f / PI;

    if (angleDeg < 0.0f)
        angleDeg += 360.0f;

    DEBUG_PRINT("[Legs] angleDeg=");
    DEBUG_PRINT(angleDeg);
    DEBUG_PRINT(", speed=");
    DEBUG_PRINTLN(speed);

    Leg** legs = myAnt.getLegArray();

    int servoSpeed = map((int)(speed * 100), 0, 100, 50, 10);
    myAnt.direction = angleDeg;

    for (size_t i = 0; i < myAnt.getLegCount(); i++)
        legs[i]->set_speed(servoSpeed);
    myAnt.move();

    myAnt.getTail().set_speed(servoSpeed);
    myAnt.getTail().wag();
}

void processHeadStick(uint16_t rawX, uint16_t rawY) {
    const float DEADZONE = 0.15f;
    const float CENTER = 512.0f;
    const float MAX_DELTA = 512.0f;

    // 1. Convert to [-1, +1]
    float roll = (rawX - CENTER) / MAX_DELTA;
    float pitch = (rawY - CENTER) / MAX_DELTA;

    // 2. Make UP = forward (positive Y)
    // Remove this line if your gait system expects the opposite.
    // y is already positive when stick is pushed up.

    // 3. Compute magnitude
    float r = sqrtf(roll * roll + pitch * pitch);

    // 4. Clamp to 1.0 (corners exceed 1.0)
    if (r > 1.0f)
        r = 1.0f;

    // 5. Deadzone
    if (r < DEADZONE)
    {
        myAnt.getHead().idle();
        myAnt.getTail().idle();
        return;
    }
    
    // 6. Normalize speed after deadzone
    float speed = (r - DEADZONE) / (1.0f - DEADZONE);

    DEBUG_PRINT("[Head] rollDeg=");
    DEBUG_PRINT(roll * 100.0f);
    DEBUG_PRINT(", pitchDeg=");
    DEBUG_PRINT(pitch * 100.0f);
    DEBUG_PRINT(", speed=");
    DEBUG_PRINTLN(map((int)(speed * 100), 0, 100, 50, 10));

    // Speed: 150ms (slow) -> 100ms (fast)
    myAnt.getHead().set_speed(
        map((int)(speed * 100), 0, 100, 50, 10));

    myAnt.getHead().roll_head(roll * 100.0f);
    myAnt.getHead().pitch_head(-pitch* 100.0f);
}

void handleTransmitterPayload(const uint8_t* payload, uint16_t len) {

  if (len != DEFAULT_TRANSMITTER_PAYLOAD_SIZE) {
    DEBUG_PRINTLN("ERROR: Unexpected len");
  }
  // Parse joystick data from buffer (assuming little-endian 16-bit values)
  uint16_t joy1_x = (uint16_t)payload[0] | ((uint16_t)payload[1] << 8);
  uint16_t joy1_y = (uint16_t)payload[2] | ((uint16_t)payload[3] << 8);
  uint16_t joy2_x = (uint16_t)payload[4] | ((uint16_t)payload[5] << 8);
  uint16_t joy2_y = (uint16_t)payload[6] | ((uint16_t)payload[7] << 8);

  // Offset for potentiometers and buttons (adjust based on actual struct layout)
  const uint8_t POT_OFFSET = NUM_JOYSTICKS * 2; // 4 joysticks * 2 bytes each
  const uint8_t BUTTONS_OFFSET = POT_OFFSET + (NUM_POTENTIOMETERS * 2);
  //const uint8_t SW_OFFSET = BUTTONS_OFFSET + NUM_BUTTONS;

  // Parse potentiometers (assuming uint16_t each)
  uint16_t potentiometers[NUM_POTENTIOMETERS];
  for (uint8_t i = 0; i < NUM_POTENTIOMETERS; i++) {
    uint8_t offset = POT_OFFSET + (i * 2);
    potentiometers[i] = (uint16_t)payload[offset] | ((uint16_t)payload[offset + 1] << 8);
  }

  // Parse buttons
  uint8_t buttons[NUM_BUTTONS];
  for (uint8_t i = 0; i < NUM_BUTTONS; i++) {
    buttons[i] = payload[BUTTONS_OFFSET + i];
  }
  if (buttons[3] == 1) {
    myAnt.getHead().bite();
  }
  else {
    myAnt.getHead().grip(map(potentiometers[0], 0, 1024, -100, 100));
  }
  // Parse switch

  // Process joystick inputs
  processLegStick(joy2_x, joy2_y);      // Joy2 controls leg movement
  processHeadStick(joy1_x, joy1_y);     // Joy1 controls head (roll/pitch)

  // Process buttons for commands
  for (uint8_t i = 0; i < NUM_BUTTONS; i++) {
    if (buttons[i]) {
      // Map button index to command
      // Adjust this mapping based on your hardware layout
      command = i;
      DEBUG_PRINT("Button ");
      DEBUG_PRINT(i);
      DEBUG_PRINTLN(" pressed");
    }
  }

  //uint8_t sw = payload[SW_OFFSET];
}
#endif

void setup() {
  Serial.begin(SERIAL_BAUDRATE);

  if (!myAnt.check()) {
    while (1) {
      ledON.setMode(BLINK_ERROR);
      ledON.loop();
      ledLOW.loop();
      delay(100);
    }
  }

  ledON.setMode(BLINK_RUNNING);
  ledLOW.setMode(BLINK_RUNNING);

#if (COMMANDER == CMD_MANUAL_TEST)
  antBones = myAnt.getBoneArray();
#endif
  myAnt.set_speed(DEFAULT_ANT_SPEED);
  myAnt.idlePose();

  sensorManager.setup();

#ifdef BLUETOOTH_BLE
  ESP.begin(UART_BAUDRATE);
#endif

#if defined(NRF24_LINK)
  radioLink.begin();
#endif

#if (COMMANDER == CMD_PC)
  serialReader.setFrameHandler(handleServoPayload);
#elif (COMMANDER == CMD_TRANSMITTER)
#if defined(BLUETOOTH_BLE)
  serialReader.setFrameHandler(handleTransmitterPayload);
#elif defined(NRF24_LINK)
  radioLink.setFrameHandler(handleTransmitterPayload);
#endif
#endif
}

#if (COMMANDER == CMD_MIT_APPINVENTOR)
void executeCommand() {
  usinged long now = millis();
  switch (command) {
    case 0:
      if (!sensorManager.isObstacleAhead()) {
        myAnt.idlePose();
        myAnt.getHead().idle();
        myAnt.getTail().idle();
      } else {
        myAnt.attack_sequence(now);
      }
      break;
    case 1:
      move({315.0f, -30, 0, false});
      break;
    case 2:
      move({0.0f, 0, 0, true});
      break;
    case 3:
      move({45.0f, 30, 0, false});
      break;
    case 4:
      move({270.0f, -60, 0, false});
      break;
    case 5:
      move({90.0f, 60, 0, false});
      break;
    case 6:
      move({225.0f, -30, 0, false});
      break;
    case 7:
      move({180.0f, 0, 0, true});
      break;
    case 8:
      move({125.0f, 30, 0, false});
      break;
    case 9:
      myAnt.foldPose();
      break;
    case 10:
      myAnt.getHead().idle();
      break;
    case 11:
      // Drop
      myAnt.getHead().grip(100);
      break;
    case 12:
      // Grab
      myAnt.getHead().grip(-100);
      break;
    case 13:
      // Bite
      myAnt.getHead().bite();
      break;
    case 14:
      myAnt.getHead().move_demo();
      break;
    case 20:
      myAnt.getTail().idle();
      break;
    case 21:
      // Tail
      myAnt.getTail().wag();
      break;
    case 30:
      myAnt.attack_sequence(now);
      break;
    default:
      DEBUG_PRINT("Unknown command: ");
      DEBUG_PRINTLN(command);
      break;
  }
}
#endif

void processLinkInput() {
#if defined(BLUETOOTH_BLE)
  if (!serialReader.update()) {
    return;
  }

  s_buffer = serialReader.getBuffer();

#if (COMMANDER == CMD_MIT_APPINVENTOR)
  DEBUG_PRINT("Received command: ");
  DEBUG_PRINTLN(s_buffer[0]);
  if (s_buffer[0] > SPEED_COMMAND_OFFSET) {
    speed = s_buffer[0] - SPEED_COMMAND_OFFSET;
    myAnt.set_speed(speed);
  } else {
    command = s_buffer[0];
  }
#else
#endif

  serialReader.clear();

#elif defined(NRF24_LINK)
  radioLink.poll();
#endif
}

void updateLowBatteryLed() {
  if (powerManager.alert()) {
    ledLOW.setMode(BLINK_SOS);
  } else {
    ledLOW.setMode(BLINK_RUNNING);
  }
}

#if (COMMANDER == CMD_MANUAL_TEST)
void parseAndApplyManualCommand() {
  bool isValid = true;
  int bone = 0;
  int localAngle = 0;

  // Expected format uses at least 6 characters (e.g. "LFF090").
  if (manualCount < 6) {
    manualCount = 0;
    manualBuffer[0] = '\0';
    return;
  }

  DEBUG_PRINTLN(manualBuffer[manualCount - 1]);

  bool foundNewline = false;
  for (int i = 0; i < manualCount; i++) {
    if (manualBuffer[i] == '\n' || manualBuffer[i] == '\r') {
      foundNewline = true;
      break;
    }
  }

  if (foundNewline) {
    manualCount = 0;
    return;
  }

  if (manualBuffer[0] == 'L') {
    DEBUG_PRINT("Left  - ");
    bone = LEFT_ID;
  }
  else if (manualBuffer[0] == 'R') {
    DEBUG_PRINT("Right - ");
    bone = RIGHT_ID;
  }
  else if (manualBuffer[0] == 'H') {
    DEBUG_PRINT("Head - ");
    bone = HEAD_PITCH;
  }
  else if (manualBuffer[0] == 'T') {
    DEBUG_PRINT("Tail - ");
    bone = TAIL_ID;
  }
  else {
    DEBUG_PRINT("None  - ");
    isValid = false;
  }

  if (manualBuffer[1] == 'F') {
    DEBUG_PRINT("Front  - ");
    bone += FRONT_ID;
  }
  else if (manualBuffer[1] == 'M') {
    DEBUG_PRINT("Middle - ");
    bone += MIDDLE_ID;
  }
  else if (manualBuffer[1] == 'B') {
    DEBUG_PRINT("Back   - ");
    bone += BACK_ID;
  }
  else if (manualBuffer[1] == 'P') {
    DEBUG_PRINT("Pitch   - ");
    bone = HEAD_PITCH;
  }
  else if (manualBuffer[1] == 'R') {
    DEBUG_PRINT("Roll   - ");
    bone = HEAD_ROLL;
  }
  else if (manualBuffer[1] == 'G') {
    DEBUG_PRINT("Grip  - ");
    bone = HEAD_GRIP;
  }
  else if (manualBuffer[1] == ' ') {
    DEBUG_PRINT("     - ");
  }
  else {
    DEBUG_PRINT("None   - ");
    isValid = false;
  }

  if (manualBuffer[2] == 'F') {
    DEBUG_PRINT("Femur - ");
    bone += FEMUR_ID;
  }
  else if (manualBuffer[2] == 'T') {
    DEBUG_PRINT("Tibia - ");
    bone += TIBIA_ID;
  }
  else if (manualBuffer[2] == 'E') {
    DEBUG_PRINT("Feet  - ");
    bone += FEET_ID;
  }
  else if (manualBuffer[2] == ' ') {
    DEBUG_PRINT("     - ");
  }
  else {
    DEBUG_PRINT("None  - ");
    isValid = false;
  }

  if ((manualBuffer[3] >= '0' && manualBuffer[3] <= '9') &&
      (manualBuffer[4] >= '0' && manualBuffer[4] <= '9') &&
      (manualBuffer[5] >= '0' && manualBuffer[5] <= '9')) {
    localAngle = (manualBuffer[3] - '0') * 100 + (manualBuffer[4] - '0') * 10 + manualBuffer[5] - '0';
  } else {
    isValid = false;
  }

  if (isValid) {
    antBones[bone].move(localAngle);
    DEBUG_PRINTF("%d - Angle: %d\r\n", bone, localAngle);
  }

  manualCount = 0;
}

void processManualTestInput() {
  if (Serial.available() <= 0) {
    return;
  }

  char inChar = Serial.read();

  const bool isLineEnd = (inChar == '\n' || inChar == '\r');

  if (manualCount < MAX_BUFFER - 1) {
    manualBuffer[manualCount++] = inChar;
    manualBuffer[manualCount] = '\0'; // Always null-terminate for safety
  } else if (isLineEnd) {
    manualCount = 0;
    manualBuffer[0] = '\0';
    return;
  }

  if (manualBuffer[0] == 'I') {
    DEBUG_PRINT("Time: ");
    DEBUG_PRINTLN(millis() - lastManualTimestamp);
    manualCount = 0;
    return;
  }

  if (manualBuffer[0] == 'O') {
    manualRotation += 10;
    DEBUG_PRINT("Rotation: ");
    DEBUG_PRINTLN(manualRotation);
    manualCount = 0;
    return;
  }

  if (manualCount >= MAX_BUFFER - 1) {
    parseAndApplyManualCommand();
    return;
  }

  if (isLineEnd) {
    parseAndApplyManualCommand();
    return;
  }

  DEBUG_PRINT(manualBuffer[manualCount - 1]);
  if (manualBuffer[manualCount - 1] == '\n' || manualBuffer[manualCount - 1] == '\r') {
    manualCount = 0;
  }
}
#endif

void loop() {
  processLinkInput();
  updateLowBatteryLed();

#if (COMMANDER == MIT_APPINVENTOR)
  executeCommand(now);
#elif (COMMANDER == CMD_MANUAL_TEST)
  processManualTestInput();
#elif (COMMANDER == CMD_PC)
  processServoFrameTimeout();
#endif

  powerManager.loop();
  sensorManager.loop();
  ledON.loop();
  ledLOW.loop();
}