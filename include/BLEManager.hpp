/**
 * @file BLEManager.hpp
 * @brief NimBLE transport for the ESP32-C3 bridge (server or client per BLE_FLAVOR).
 *
 * A single class encapsulates the BLE side. The Server/Client role is selected at
 * compile time through the BLE_FLAVOR switch (see config.h):
 *   - BLE_FLAVOR_SERVER: advertises, accepts a phone/peer, forwards writes to UART
 *                        and ACKs them back over BLE.
 *   - BLE_FLAVOR_CLIENT: scans/connects to a server and forwards UART frames to it.
 */

#ifndef BLE_MANAGER_HPP
#define BLE_MANAGER_HPP

#ifdef ARDUINO_ESP32C3_DEV

#include <Arduino.h>
#include <NimBLEDevice.h>

#include "../config.h"
#include "LEDManager.hpp"
#include "SerialManager.hpp"

class BLEManager {
public:
  BLEManager(LEDManager& led, SerialManager& serial);

  /// Initialise NimBLE in the selected role.
  void begin();

  /// Periodic housekeeping (client reconnection + UART->BLE forwarding).
  void loop();

  /// True when a BLE peer is connected.
  bool isConnected() const;

  /// Forward a complete frame to the BLE peer (notify on server, write on client).
  bool sendToPeer(const uint8_t* data, size_t len);

#if BLE_FLAVOR == BLE_FLAVOR_SERVER
  /// Notify the BLE client about the result of a received command.
  void sendAck(uint8_t seq, uint8_t status);

  /// Handle an inbound BLE write (RX characteristic) and bridge it to UART.
  void handleBleWrite(const uint8_t* data, size_t len);

  void onPeerConnect(NimBLEServer* server, ble_gap_conn_desc* desc);
  void onPeerDisconnect();
#else
  void onClientConnect();
  void onClientDisconnect();
  void onDeviceFound(NimBLEAdvertisedDevice* advertisedDevice);
#endif

private:
  LEDManager& _led;
  SerialManager& _serial;

#if BLE_FLAVOR == BLE_FLAVOR_SERVER
  bool _deviceConnected = false;
  NimBLECharacteristic* _rx = nullptr;
  NimBLECharacteristic* _tx = nullptr;
#else
  bool _connectToServer();
  void _startScan();

  NimBLEAdvertisedDevice* _targetDevice = nullptr;
  NimBLEClient* _client = nullptr;
  NimBLERemoteCharacteristic* _remoteWriteChar = nullptr;
  bool _clientConnected = false;
#endif
};

#endif // ARDUINO_ESP32C3_DEV
#endif // BLE_MANAGER_HPP
