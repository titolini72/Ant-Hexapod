/**
 * @file SerialManager.cpp
 * @brief Implementation of the ESP32-C3 UART framing/CRC/ACK handler.
 */


#include "SerialManager.hpp"

#if defined(ESP32) && defined(TRANSMISSION_BLE)

#ifdef ARDUINO_ESP32C3_DEV

void SerialManager::begin() {
  WEACT.begin(UART_BAUDRATE, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
}

uint16_t SerialManager::crc16_ccitt(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x8000) {
        crc = (crc << 1) ^ 0x1021;
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

bool SerialManager::checkFrame(const uint8_t* data, size_t len) {
  if (len < (FRAME_CORE_SIZE + CRC_SIZE)) return false;

  // Data starts at the sequence byte and includes the CRC trailer.
  // [seq][ts0][ts1][ts2][ts3][type][len_l][len_h][payload...][crc_l][crc_h]
  const uint16_t received_crc = data[len - 2] | (data[len - 1] << 8);
  const uint16_t computed_crc = crc16_ccitt(data, len - 2);

  return received_crc == computed_crc;
}

void SerialManager::writeFrame(const uint8_t* data, size_t len) {
  WEACT.write(data, len);
  WEACT.flush();
}

void SerialManager::processAckFrame(const uint8_t* frame, size_t len) {
  // Frame:
  // [SOF1][SOF2][seq][ts0][ts1][ts2][ts3][type][len L][len H][payload...]
  if (len < FRAME_HEADER_SIZE) return;
  if (frame[0] != SOF1 || frame[1] != SOF2) return;

  const uint8_t seq = frame[2];
  const uint8_t type = frame[TYPE_OFFSET];
  const uint16_t payload_len = (uint16_t)frame[LENGTH_OFFSET] | ((uint16_t)frame[LENGTH_OFFSET + 1] << 8);

  if (type != ACK_TYPE) return;
  if (payload_len != 1) return;
  if (len != (FRAME_HEADER_SIZE + payload_len)) return;

  uint8_t status = frame[PAYLOAD_OFFSET];

  _lastAck.valid = true;
  _lastAck.seq = seq;
  _lastAck.status = status;
  _ackReady = true;
}

void SerialManager::pollUartAck() {
  static uint8_t buf[64];
  static size_t idx = 0;
  static uint16_t expectedLen = 0;

  while (WEACT.available()) {
    const uint8_t b = (uint8_t)WEACT.read();

    if (idx == 0) {
      if (b != SOF1) continue;
      buf[idx++] = b;
    } else if (idx == 1) {
      if (b != SOF2) {
        idx = 0;
        continue;
      }
      buf[idx++] = b;
    } else {
      buf[idx++] = b;

      // Once we have the full fixed header, compute the expected ACK length.
      if (idx == FRAME_HEADER_SIZE) {
        const uint16_t payload_len = (uint16_t)buf[LENGTH_OFFSET] | ((uint16_t)buf[LENGTH_OFFSET + 1] << 8);
        if (payload_len > MAX_PAYLOAD) {
          idx = 0;
          continue;
        }
        expectedLen = FRAME_HEADER_SIZE + payload_len;
        if (expectedLen > sizeof(buf)) {
          idx = 0;
          continue;
        }
      }

      if (expectedLen > 0 && idx == expectedLen) {
        processAckFrame(buf, expectedLen);
        idx = 0;
        expectedLen = 0;
      }

      if (idx >= sizeof(buf)) {
        idx = 0;
        expectedLen = 0;
      }
    }
  }
}

bool SerialManager::sendFrameAndWaitAck(const uint8_t* frame, size_t len, uint8_t seq, uint32_t timeout_ms) {
  _ackReady = false;
  _lastAck.valid = false;

  WEACT.write(frame, len);
  WEACT.flush();

  uint32_t start = millis();
  while (millis() - start < timeout_ms) {
    pollUartAck();

    if (_ackReady && _lastAck.valid && _lastAck.seq == seq) {
      _ackReady = false;
      return (_lastAck.status == 0x00);
    }

    delay(1);
  }

  return false; // timeout
}

void SerialManager::pollFrames() {
  static uint8_t buf[128];
  static size_t idx = 0;
  static uint16_t expectedLen = 0;

  while (WEACT.available()) {
    uint8_t b = WEACT.read();

    if (idx == 0) {
      if (b != SOF1) continue;
      buf[idx++] = b;
    } else if (idx == 1) {
      if (b != SOF2) {
        idx = 0;
        continue;
      }
      buf[idx++] = b;
    } else {
      buf[idx++] = b;

      if (idx == FRAME_HEADER_SIZE) {
        uint16_t payloadLen =
          (uint16_t)buf[LENGTH_OFFSET] |
          ((uint16_t)buf[LENGTH_OFFSET + 1] << 8);

        expectedLen =
          FRAME_HEADER_SIZE + payloadLen + CRC_SIZE;

        if (expectedLen > sizeof(buf)) {
          idx = 0;
          expectedLen = 0;
          continue;
        }
      }

      if (expectedLen && idx == expectedLen) {
        uint8_t type = buf[TYPE_OFFSET];
        uint16_t payloadLen = (uint16_t)buf[LENGTH_OFFSET] | ((uint16_t)buf[LENGTH_OFFSET + 1] << 8);
        const uint16_t rx_crc = (uint16_t)buf[expectedLen - 2] | ((uint16_t)buf[expectedLen - 1] << 8);
        const uint16_t calc_crc = crc16_ccitt(&buf[2], FRAME_CORE_SIZE + payloadLen);

        // Reject corrupted frames and explicitly signal CRC failure.
        if (rx_crc != calc_crc) {
          Serial.println("CRC error");
          idx = 0;
          expectedLen = 0;
          continue;
        }

        if (type == TRACE_TYPE) {
          if (_traceHandler) {
            _traceHandler(&buf[PAYLOAD_OFFSET], payloadLen);
          }
        } else {
          if (_frameHandler) {
            _frameHandler(buf, expectedLen);
          }
        }

        idx = 0;
        expectedLen = 0;
      }
    }
  }
}

#endif // ARDUINO_ESP32C3_DEV
#endif // ESP32 && TRANSMISSION_BLE
