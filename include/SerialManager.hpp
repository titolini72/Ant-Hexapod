/**
 * @file SerialManager.hpp
 * @brief UART (WeAct/STM32) framing, CRC and ACK handling for the ESP32-C3 bridge.
 *
 * Owns the byte-level UART protocol shared with the STM32 peer:
 *   [SOF1][SOF2][SEQ][TIMESTAMP_LE32][TYPE][LEN_LE16][PAYLOAD...][CRC_LE16]
 * STM32 ACK frames reuse the same header but omit the CRC trailer.
 */

#ifndef SERIAL_MANAGER_HPP
#define SERIAL_MANAGER_HPP

#ifdef ARDUINO_ESP32C3_DEV

#include <Arduino.h>
#include "../config.h"

class SerialManager {
public:
  // UART frame markers and layout, shared with the STM32 peer.
  static constexpr uint8_t SOF1 = 0xAA;
  static constexpr uint8_t SOF2 = 0x55;
  static constexpr uint8_t TIMESTAMP_SIZE = sizeof(uint32_t);
  static constexpr size_t SOF_SIZE = 2;
  static constexpr size_t FRAME_HEADER_SIZE = SOF_SIZE + 1 + TIMESTAMP_SIZE + 1 + 2;
  static constexpr size_t FRAME_CORE_SIZE = 1 + TIMESTAMP_SIZE + 1 + 2;
  static constexpr size_t CRC_SIZE = 2;
  static constexpr size_t TYPE_OFFSET = SOF_SIZE + 1 + TIMESTAMP_SIZE;
  static constexpr size_t LENGTH_OFFSET = TYPE_OFFSET + 1;
  static constexpr size_t PAYLOAD_OFFSET = LENGTH_OFFSET + 2;
  static constexpr uint8_t ACK_TYPE = 0xAC;
  static constexpr uint8_t TRACE_TYPE = 0xDE;
  static constexpr uint8_t DATA_TYPE = 0xDA;
  static constexpr uint16_t MAX_PAYLOAD = 64;

  static constexpr uint8_t ACK_STATUS_OK = 0x00;
  static constexpr uint8_t ACK_STATUS_CRC_ERROR = 0x01;
  static constexpr uint8_t ACK_STATUS_TIMEOUT = 0x03;

  static constexpr uint32_t STM32_ACK_TIMEOUT_MS = 200;

  /// Called for each complete non-trace frame (full frame incl. SOF and CRC).
  using FrameHandler = void (*)(const uint8_t* frame, size_t len);
  /// Called for the payload of each TRACE frame.
  using TraceHandler = void (*)(const uint8_t* payload, uint16_t len);

  /// Bring up the UART link to the STM32 peer.
  void begin();

  void setFrameHandler(FrameHandler handler) { _frameHandler = handler; }
  void setTraceHandler(TraceHandler handler) { _traceHandler = handler; }

  /// Write raw bytes to the UART and flush.
  void writeFrame(const uint8_t* data, size_t len);

  /// Send a frame over UART and block until the matching STM32 ACK or timeout.
  bool sendFrameAndWaitAck(const uint8_t* frame, size_t len, uint8_t seq, uint32_t timeout_ms);

  /// Drain UART; dispatch complete data frames and trace payloads to handlers.
  void pollFrames();

  /// Compute a CRC-16/CCITT checksum over @p data.
  static uint16_t crc16_ccitt(const uint8_t* data, size_t len);

  /// Validate the CRC trailer of a frame starting at the sequence byte.
  static bool checkFrame(const uint8_t* data, size_t len);

private:
  struct AckResult {
    bool valid = false;
    uint8_t seq = 0;
    uint8_t status = 0xFF;
  };

  void processAckFrame(const uint8_t* frame, size_t len);
  void pollUartAck();

  FrameHandler _frameHandler = nullptr;
  TraceHandler _traceHandler = nullptr;

  volatile AckResult _lastAck;
  volatile bool _ackReady = false;
};

#endif // ARDUINO_ESP32C3_DEV
#endif // SERIAL_MANAGER_HPP
