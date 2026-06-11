#if defined(ARDUINO_uPesy_WROOM) || defined(ARDUINO_ESP32C3_DEV)
/**
 * @file main.cpp
 * @brief ESP-side firmware entry points for PS4 input, BLE server, or BLE client.
 */
#endif

#include <Arduino.h>

#include "../config.h"
#include "LEDManager.hpp"
#include "SerialManager.hpp"
#include "BLEManager.hpp"

#if defined(ESP32) && defined(TRANSMISSION_BLE)

#ifdef ARDUINO_uPesy_WROOM
#include <PS4Controller.h>
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_err.h"

#warning "ESP32S3"

unsigned long lastTimeStamp = 0;

#define EVENTS 0
#define BUTTONS 0
#define JOYSTICKS 1
#define SENSORS 0

void removePairedDevices() {
  uint8_t pairedDeviceBtAddr[20][6];
  int count = esp_bt_gap_get_bond_device_num();
  esp_bt_gap_get_bond_device_list(&count, pairedDeviceBtAddr);
  for (int i = 0; i < count; i++) {
    esp_bt_gap_remove_bond_device(pairedDeviceBtAddr[i]);
  }
}

void printDeviceAddress() {
  const uint8_t* point = esp_bt_dev_get_address();
  for (int i = 0; i < 6; i++) {
    char str[3];
    snprintf(str, sizeof(str), "%02x", (int)point[i]);
    Serial.print(str);
    if (i < 5) {
      Serial.print(":");
    }
  }
}

void onConnect() {
  Serial.println("Connected!");
}

void notify() {
#if EVENTS
  bool sqd = PS4.event.button_down.square,
       squ = PS4.event.button_up.square,
       trd = PS4.event.button_down.triangle,
       tru = PS4.event.button_up.triangle;
  if (sqd)
    Serial.println("SQUARE down");
  else if (squ)
    Serial.println("SQUARE up");
  else if (trd)
    Serial.println("TRIANGLE down");
  else if (tru)
    Serial.println("TRIANGLE up");
#endif

#if BUTTONS
  bool sq = PS4.Square(),
       tr = PS4.Triangle();
  if (sq)
    Serial.print(" SQUARE pressed");
  if (tr)
    Serial.print(" TRIANGLE pressed");
  if (sq || tr)
    Serial.println();
#endif

  //Only needed to print the message properly on serial monitor. Else we dont need it.
  if (millis() - lastTimeStamp > 50) {
#if JOYSTICKS
    Serial.printf("lx:%4d,ly:%4d,rx:%4d,ry:%4d\n",
                  PS4.LStickX(),
                  PS4.LStickY(),
                  PS4.RStickX(),
                  PS4.RStickY());
#endif
#if SENSORS
    Serial.printf("gx:%5d,gy:%5d,gz:%5d,ax:%5d,ay:%5d,az:%5d\n",
                  PS4.GyrX(),
                  PS4.GyrY(),
                  PS4.GyrZ(),
                  PS4.AccX(),
                  PS4.AccY(),
                  PS4.AccZ());
#endif
    lastTimeStamp = millis();
  }
}

void onDisconnect() {
  Serial.println("Disconnected!");
}

void setup() {
  Serial.begin(115200);
  PS4.attach(notify);
  PS4.attachOnConnect(onConnect);
  PS4.attachOnDisconnect(onDisconnect);
  PS4.begin();
  removePairedDevices(); // This helps to solve connection issues
  Serial.print("This device MAC is: ");
  printDeviceAddress();
  Serial.println();
}

void loop() {
  delay(100);
}

#endif // ARDUINO_uPesy_WROOM

#ifdef ARDUINO_ESP32C3_DEV
#include <Arduino.h>

#include "../config.h"
#include "LEDManager.hpp"
#include "SerialManager.hpp"
#include "BLEManager.hpp"

namespace {
LEDManager status_led(STATUS_LED_PIN);
SerialManager serial_link;

#if (COMMANDER == CMD_MIT_APPINVENTOR)
constexpr BLEManager::BridgeMode BLE_BRIDGE_MODE = BLEManager::BridgeMode::RawPassthrough;
#elif (COMMANDER == CMD_TRANSMITTER)
constexpr BLEManager::BridgeMode BLE_BRIDGE_MODE = BLEManager::BridgeMode::ValidatedPassthrough;
#else
constexpr BLEManager::BridgeMode BLE_BRIDGE_MODE = BLEManager::BridgeMode::AckedBridge;
#endif

BLEManager ble(status_led, serial_link, BLE_BRIDGE_MODE);
} // namespace

/**
 * @brief Forward a complete UART frame from the STM32 to the BLE peer.
 */
static void onUartFrame(const uint8_t* frame, size_t len) {
#if BLE_FLAVOR == BLE_FLAVOR_CLIENT
  if (ble.sendToPeer(frame, len)) {
    DEBUG_PRINTF("Sent frame seq=%u\r\n", frame[2]);
  } else {
    DEBUG_PRINTLN("Send failed");
  }
#else
  ble.sendToPeer(frame, len);
#endif
}

/**
 * @brief Print a trace payload coming from the STM32 to the console.
 */
static void onUartTrace(const uint8_t* payload, uint16_t len) {
  for (uint16_t i = 0; i < len; i++) {
    Serial.write(payload[i]);
  }
}

void setup() {
  Serial.begin(CONSOLE_BAUDRATE);
  delay(2000);

  serial_link.begin();
  serial_link.setFrameHandler(onUartFrame);
  serial_link.setTraceHandler(onUartTrace);

  ble.begin();
}

void loop() {
  status_led.loop();
  ble.loop();
  delay(1);
}

#endif // ARDUINO_ESP32C3_DEV
#else
#include <Arduino.h>

void setup() {
  Serial.begin(115200);
}
void loop() {
  Serial.println("No Bluetooth mode defined. Please check your config.h.");
  delay(1000);
}
#endif // ESP32 && TRANSMISSION_BLE