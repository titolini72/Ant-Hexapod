/**
 * @file BLEManager.cpp
 * @brief NimBLE server/client implementation for the ESP32-C3 bridge.
 */

#include "BLEManager.hpp"

#if defined(ESP32) && defined(TRANSMISSION_BLE)

#ifdef ARDUINO_ESP32C3_DEV
namespace {
// NimBLE callbacks need to reach the owning manager; a single instance exists.
BLEManager* g_instance = nullptr;
} // namespace

BLEManager::BLEManager(LEDManager& led, SerialManager& serial, BridgeMode bridge_mode)
  : _led(led), _serial(serial), _bridgeMode(bridge_mode) {
  g_instance = this;
}

#if BLE_FLAVOR == BLE_FLAVOR_SERVER

// ================================
//       SERVER CALLBACKS
// ================================
namespace {
class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* pServer, ble_gap_conn_desc* desc) override {
    if (g_instance) g_instance->onPeerConnect(pServer, desc);
  }

  void onDisconnect(NimBLEServer* /*pServer*/) override {
    if (g_instance) g_instance->onPeerDisconnect();
  }
};

class RXCallback : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* pChar) override {
    if (!g_instance) return;
    const std::string v = pChar->getValue();
    g_instance->handleBleWrite(
      reinterpret_cast<const uint8_t*>(v.data()), v.length());
  }
};
} // namespace

void BLEManager::onPeerConnect(NimBLEServer* server, ble_gap_conn_desc* desc) {
  // Request low latency parameters.
  server->updateConnParams(
    desc->conn_handle,
    6,   // min interval (7.5 ms)
    6,   // max interval
    0,   // latency
    400  // supervision timeout (ms)
  );
  NimBLEDevice::setMTU(247);
  DEBUG_PRINTLN(">> BLE CONNECTED");

  _deviceConnected = true;
  _led.setMode(BLINK_RUNNING);
}

void BLEManager::onPeerDisconnect() {
  DEBUG_PRINTLN(">> BLE DISCONNECTED - restarting advertising");
  _deviceConnected = false;
  // Restart advertising so phones can reconnect.
  NimBLEDevice::startAdvertising();
  _led.setMode(BLINK_SOS);
}

void BLEManager::handleBleWrite(const uint8_t* data, size_t len) {
  DEBUG_PRINTF("onWrite: %u bytes\r\n", (unsigned)len);

  switch (_bridgeMode) {
    case BridgeMode::RawPassthrough:
      forwardRawWrite(data, len);
      break;

    case BridgeMode::ValidatedPassthrough:
      forwardValidatedWrite(data, len);
      break;

    case BridgeMode::AckedBridge:
      forwardAckedWrite(data, len);
      break;

    default:
      DEBUG_PRINTLN("Unsupported BLE bridge mode");
      break;
  }
}

void BLEManager::forwardRawWrite(const uint8_t* data, size_t len) {
  _serial.writeFrame(data, len);
}

bool BLEManager::validateIncomingFrame(const uint8_t* data, size_t len, uint8_t* seq) const {
  if (len < (SerialManager::FRAME_HEADER_SIZE + SerialManager::CRC_SIZE)) {
    DEBUG_PRINTLN("Frame too short");
    return false;
  }

  if (seq != nullptr) {
    *seq = data[2];
  }

  if (!SerialManager::checkFrame(data + 2, len - 2)) {
    DEBUG_PRINTLN("Invalid frame received (CRC error)");
    return false;
  }

  return true;
}

void BLEManager::forwardValidatedWrite(const uint8_t* data, size_t len) {
  if (!validateIncomingFrame(data, len)) {
    return;
  }

  _serial.writeFrame(data, len);
}

void BLEManager::forwardAckedWrite(const uint8_t* data, size_t len) {
  uint8_t seq = 0;

  if (!validateIncomingFrame(data, len, &seq)) {
    sendAck(seq, SerialManager::ACK_STATUS_CRC_ERROR);
    return;
  }

  const bool ok = _serial.sendFrameAndWaitAck(
    data, len, seq, SerialManager::STM32_ACK_TIMEOUT_MS);
  if (ok) {
    DEBUG_PRINTLN("ACK received from STM32: OK");
    sendAck(seq, SerialManager::ACK_STATUS_OK);
  } else {
    DEBUG_PRINTLN("ACK received from STM32: ERROR or TIMEOUT");
    sendAck(seq, SerialManager::ACK_STATUS_TIMEOUT);
  }
}

void BLEManager::sendAck(uint8_t seq, uint8_t status) {
  if (_tx == nullptr) return;

  uint8_t ack[3] = { SerialManager::ACK_TYPE, seq, status };
  _tx->setValue(ack, sizeof(ack));
  _tx->notify();
}

void BLEManager::begin() {
  NimBLEDevice::init("ESP32C3-BLE Server");
  _led.setMode(BLINK_SOS);

  NimBLEServer* server = NimBLEDevice::createServer();
  NimBLEService* svc = server->createService(SERVICE_UUID);
  server->setCallbacks(new ServerCallbacks());

  _rx = svc->createCharacteristic(
    WRITE_UUID,
    (uint32_t)(NIMBLE_PROPERTY::WRITE |
               NIMBLE_PROPERTY::WRITE_NR));
  _rx->setCallbacks(new RXCallback());

  _tx = svc->createCharacteristic(
    NOTIFY_UUID,
    (uint32_t)(NIMBLE_PROPERTY::READ |
               NIMBLE_PROPERTY::NOTIFY));

  svc->start();

  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->setScanResponse(true);
  adv->start();

  DEBUG_PRINTLN("NimBLE server ready - advertising");
}

void BLEManager::loop() {
  // The server forwards BLE->UART via the RX callback; nothing to poll here.
}

bool BLEManager::isConnected() const {
  return _deviceConnected;
}

bool BLEManager::sendToPeer(const uint8_t* data, size_t len) {
  if (!_deviceConnected || !_tx) return false;
  _tx->setValue(data, len);
  _tx->notify();
  return true;
}

#else // BLE_FLAVOR_CLIENT

// ================================
//       CLIENT CALLBACKS
// ================================
namespace {
class AdvertisedDeviceCallbacks : public NimBLEAdvertisedDeviceCallbacks {
  void onResult(NimBLEAdvertisedDevice* advertisedDevice) override {
    if (g_instance) g_instance->onDeviceFound(advertisedDevice);
  }
};

class ClientCallbacks : public NimBLEClientCallbacks {
  void onConnect(NimBLEClient* /*pclient*/) override {
    if (g_instance) g_instance->onClientConnect();
  }

  void onDisconnect(NimBLEClient* /*pclient*/) override {
    if (g_instance) g_instance->onClientDisconnect();
  }
};
} // namespace

void BLEManager::onDeviceFound(NimBLEAdvertisedDevice* advertisedDevice) {
  if (advertisedDevice->haveServiceUUID() &&
      advertisedDevice->isAdvertisingService(NimBLEUUID(SERVICE_UUID))) {
    _targetDevice = advertisedDevice;
    NimBLEDevice::getScan()->stop();
  }
}

void BLEManager::onClientConnect() {
  _clientConnected = true;
  DEBUG_PRINTLN(">> CLIENT CONNECTED");
  _led.setMode(BLINK_RUNNING);
}

void BLEManager::onClientDisconnect() {
  _clientConnected = false;
  _remoteWriteChar = nullptr;
  _targetDevice = nullptr;
  DEBUG_PRINTLN(">> CLIENT DISCONNECTED");
  _led.setMode(BLINK_SOS);
}

bool BLEManager::_connectToServer() {
  if (!_targetDevice) return false;

  if (!_client) {
    _client = NimBLEDevice::createClient();
    _client->setClientCallbacks(new ClientCallbacks(), false);
  }

  if (!_client->connect(_targetDevice)) {
    DEBUG_PRINTLN("Failed to connect");
    return false;
  }

  NimBLERemoteService* service = _client->getService(SERVICE_UUID);
  if (!service) {
    DEBUG_PRINTLN("Service not found");
    _client->disconnect();
    return false;
  }

  _remoteWriteChar = service->getCharacteristic(WRITE_UUID);
  if (!_remoteWriteChar) {
    DEBUG_PRINTLN("Write characteristic not found");
    _client->disconnect();
    return false;
  }

  _clientConnected = true;
  DEBUG_PRINTLN("Connected and characteristic discovered");
  return true;
}

void BLEManager::_startScan() {
  NimBLEScan* scan = NimBLEDevice::getScan();
  scan->setAdvertisedDeviceCallbacks(new AdvertisedDeviceCallbacks(), true);
  scan->setActiveScan(true);
  scan->setInterval(45);
  scan->setWindow(15);
  scan->start(0, nullptr, false);   // non-blocking
}

void BLEManager::begin() {
  NimBLEDevice::init("ESP32C3-BLE Client");
  _led.setMode(BLINK_SOS);

  _startScan();
  if (!_targetDevice) {
    DEBUG_PRINTLN("No target found during initial scan");
  }

  if (!_connectToServer()) {
    DEBUG_PRINTLN("Initial connection failed");
  }
}

void BLEManager::loop() {
  static unsigned long _now = millis();

  if (!_clientConnected || !_client || !_client->isConnected() || !_remoteWriteChar) {
    unsigned long now = millis();

    if ((now - _now) < 1000)
      return;

    _now = now;
    DEBUG_PRINTLN("Reconnecting...");
    _startScan();
    _connectToServer();
    return;
  }

  _serial.pollFrames();
}

bool BLEManager::isConnected() const {
  return _clientConnected;
}

bool BLEManager::sendToPeer(const uint8_t* data, size_t len) {
  if (!_clientConnected || !_remoteWriteChar) return false;
  return _remoteWriteChar->writeValue(data, len, false);
}

#endif // BLE_FLAVOR

#endif // ARDUINO_ESP32C3_DEV
#endif // ESP32 && TRANSMISSION_BLE
