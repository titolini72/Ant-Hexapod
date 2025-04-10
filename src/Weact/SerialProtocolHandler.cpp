/**
 * @file SerialProtocolHandler.cpp
 * @brief Serial line and framed payload protocol handler implementation.
 *
 * Framed protocol layout (binary mode):
 * [SOF1][SOF2][SEQ][TS0][TS1][TS2][TS3][TYPE][LEN_L][LEN_H][PAYLOAD...][CRC_L][CRC_H]
 *
 * CRC is CRC16-CCITT over [SEQ..PAYLOAD] (timestamp + type + length + payload included).
 * In CMD_MIT_APPINVENTOR mode, a single-byte command is accepted until the
 * configured terminator is received.
 */

#include "SerialProtocolHandler.hpp"

#if defined(BLUETOOTH_BLE)
/**
 * @brief Constructs a serial protocol handler.
 *
 * Initializes protocol state for either command mode selected at compile time:
 * line-delimited single-byte command mode or framed binary mode.
 *
 * @param serial Stream backend used for RX/TX.
 * @param FramePayloadSize Expected payload length for frames.
 * @param terminator End-of-command marker for line mode.
 */
SerialProtocolHandler::SerialProtocolHandler(Stream& serial,
                                             uint16_t FramePayloadSize,
                                             char terminator)
    : _serial(serial),
      _terminator(terminator),
      _FramePayloadSize(FramePayloadSize) {
    clear();
}

/**
 * @brief Returns the configured expected servo-frame payload size.
 * @return Payload size in bytes.
 */
uint16_t SerialProtocolHandler::FramePayloadSize() const {
    return _FramePayloadSize;
}

/**
 * @brief Registers a callback to receive validated servo payload frames.
 * @param handler Function pointer invoked with payload bytes and length.
 */
void SerialProtocolHandler::setFrameHandler(FrameHandler handler) {
    _frameHandler = handler;
}

/**
 * @brief Polls the serial stream and updates RX parsing state.
 *
 * Behavior depends on compile-time command mode:
 * - Line mode: reads bytes until terminator and stores latest byte command.
 * - Binary mode: incrementally parses framed packets and dispatches complete
 *   frames via @ref processFrame.
 *
 * @retval true A command/frame is ready and can be consumed.
 * @retval false No complete command/frame is available yet.
 */
bool SerialProtocolHandler::update() {
#if (COMMANDER == CMD_MIT_APPINVENTOR)
    // Line mode: keep the last received byte and flag readiness at terminator.
    if (_lineReady) {
        return true;
    }

    while (_serial.available() > 0) {
        const char b = static_cast<char>(_serial.read());

        if (b == _terminator) {
            _lineReady = true;
            break;
        }

        _buffer[0] = static_cast<uint8_t>(b);
    }

    return _lineReady;
#else
    // Binary mode: incrementally parse frames from the serial stream.
    if (_lineReady) {
        return true;
    }

    size_t idx = _frameIndex;

    while (_serial.available() > 0) {
        const char b = static_cast<char>(_serial.read());

        if (idx == 0) {
            if (static_cast<uint8_t>(b) != SOF1) {
                continue;
            }
            _buffer[idx++] = static_cast<uint8_t>(b);
            continue;
        }

        if (idx == 1) {
            if (static_cast<uint8_t>(b) != SOF2) {
                idx = 0;
                continue;
            }
            _buffer[idx++] = static_cast<uint8_t>(b);
            continue;
        }

        _buffer[idx++] = static_cast<uint8_t>(b);

        if (idx == FRAME_HEADER_SIZE) {
            const uint16_t payload_len = (uint16_t)_buffer[LENGTH_OFFSET] |
                                         ((uint16_t)_buffer[LENGTH_OFFSET + 1] << 8);
            const size_t total_len = FRAME_HEADER_SIZE + payload_len + CRC_SIZE;

            if (payload_len > MAX_PAYLOAD || total_len > sizeof(_buffer)) {
                idx = 0;
#if (COMMANDER == CMD_PC)
                sendAck(0, ACK_STATUS_INVALID_LENGTH);
#else
                Serial.println("Invalid frame length"); // Debug print for invalid length
#endif
                continue;
            }
        }

        if (idx >= FRAME_HEADER_SIZE) {
            const uint16_t payload_len = (uint16_t)_buffer[LENGTH_OFFSET] |
                                         ((uint16_t)_buffer[LENGTH_OFFSET + 1] << 8);
            const size_t total_len = FRAME_HEADER_SIZE + payload_len + CRC_SIZE;

            if (idx == total_len) {
                // A full frame is available; validate and dispatch it.
                processFrame(_buffer, idx);
                _lineReady = true;
                _length = idx;
                idx = 0;
                break;
            }
        }

        if (idx >= sizeof(_buffer)) {
            idx = 0;
            _overflow = true;
        }
    }

    _frameIndex = idx;
    return _lineReady;
#endif
}

#if (COMMANDER == CMD_PC)
/**
 * @brief Sends an ACK frame to the serial peer.
 *
 * ACK payload format is:
 * [status][optional extra payload bytes].
 * ACK frame layout is:
 * [SOF1][SOF2][SEQ][TS0][TS1][TS2][TS3][TYPE][LEN_L][LEN_H][PAYLOAD...]
 *
 * @param seq Sequence number being acknowledged.
 * @param status ACK status code (OK, CRC error, invalid length, unknown cmd).
 * @param payload Optional extra payload bytes appended after status.
 * @param extra_len Number of optional payload bytes.
 */
void SerialProtocolHandler::sendAck(uint8_t seq, uint8_t status,
                                    const uint8_t* payload,
                                    uint16_t extra_len) {
    // ACK payload is: [status][optional extra bytes].
    const uint16_t payload_len = 1 + extra_len;

    uint8_t frame[FRAME_HEADER_SIZE + payload_len];
    size_t idx = 0;

    frame[idx++] = SOF1;
    frame[idx++] = SOF2;
    frame[idx++] = seq;
    writeTimestamp(&frame[idx], millis());
    idx += TIMESTAMP_SIZE;
    frame[idx++] = ACK_TYPE;
    frame[idx++] = (uint8_t)(payload_len & 0xFF);
    frame[idx++] = (uint8_t)(payload_len >> 8);
    frame[idx++] = status;

    if (payload && extra_len > 0) {
        memcpy(&frame[idx], payload, extra_len);
        idx += extra_len;
    }

    _serial.write(frame, idx);
    _serial.flush();
}
#endif

/**
 * @brief Indicates whether a complete input item is ready.
 * @return @c true when buffered data is ready to consume; otherwise @c false.
 */
bool SerialProtocolHandler::hasBuffer() const {
    return _lineReady;
}

/**
 * @brief Returns pointer to internal receive buffer.
 * @return Pointer to start of internal buffer storage.
 */
const uint8_t* SerialProtocolHandler::getBuffer() const {
    return _buffer;
}

/**
 * @brief Returns current valid byte count in the receive buffer.
 * @return Number of bytes available for current command/frame.
 */
size_t SerialProtocolHandler::length() const {
    return _length;
}

/**
 * @brief Reports whether RX overflow occurred during parsing.
 * @return @c true if parser overflowed at least once since last @ref clear.
 */
bool SerialProtocolHandler::overflowed() const {
    return _overflow;
}

/**
 * @brief Resets parser state and clears command/frame readiness flags.
 *
 * Leaves protocol configuration unchanged while clearing transient RX state.
 */
void SerialProtocolHandler::clear() {
    // Reset both text-mode and frame-mode receive state.
    _length = 0;
    _lineReady = false;
    _overflow = false;
    _frameIndex = 0;
    _buffer[0] = '\0';
}

/**
 * @brief Computes CRC-16/CCITT-FALSE over a byte span.
 *
 * Parameters:
 * - Initial value: 0xFFFF
 * - Polynomial: 0x1021
 * - Reflection: none
 *
 * @param data Pointer to first byte of input data.
 * @param len Number of bytes to process.
 * @return Calculated 16-bit CRC.
 */
uint16_t SerialProtocolHandler::crc16_ccitt(const uint8_t* data, size_t len) {
    // CRC-16/CCITT-FALSE parameters: init=0xFFFF, poly=0x1021, no reflection.
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

uint32_t SerialProtocolHandler::readTimestamp(const uint8_t* data) {
    return (uint32_t)data[0] |
           ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16) |
           ((uint32_t)data[3] << 24);
}

void SerialProtocolHandler::writeTimestamp(uint8_t* data, uint32_t timestamp) {
    data[0] = (uint8_t)(timestamp & 0xFF);
    data[1] = (uint8_t)((timestamp >> 8) & 0xFF);
    data[2] = (uint8_t)((timestamp >> 16) & 0xFF);
    data[3] = (uint8_t)((timestamp >> 24) & 0xFF);
}

/**
 * @brief Validates and dispatches a decoded frame payload by command type.
 *
 * Currently supports servo command frames only. Unknown types or invalid
 * payload lengths are rejected with explicit ACK error status.
 *
 * @param seq Sequence number to echo in response ACK.
 * @param timestamp Frame timestamp carried in the header.
 * @param type Frame command type.
 * @param payload Pointer to decoded payload bytes.
 * @param payload_len Payload size in bytes.
 */
void SerialProtocolHandler::handleFramePayload(uint8_t seq,
                                               uint32_t timestamp,
                                               uint8_t type,
                                               const uint8_t* payload,
                                               uint16_t payload_len) {
     (void)timestamp;

    switch (type) {
#if (COMMANDER == CMD_PC)
        case FRAME_TYPE_SERVO:
            break;
#endif
#if (COMMANDER == CMD_TRANSMITTER)
        case FRAME_TYPE_TRANSMITTER:
            break;
#endif
        default:
#if (COMMANDER == CMD_PC)
            sendAck(seq, ACK_STATUS_UNKNOWN_COMMAND);
#endif
            Serial.print("["); Serial.print(seq); Serial.print("] ");
            Serial.println("Unknown frame type"); // Debug print for unknown command
            break;
    }

#if (COMMANDER == CMD_PC)
    if (payload_len != _FramePayloadSize) {
        sendAck(seq, ACK_STATUS_INVALID_LENGTH);
        Serial.print("["); Serial.print(seq); Serial.print("] ");
        Serial.println("Invalid payload length for servo frame"); // Debug print for invalid length
        return;
    }
#endif

    if (_frameHandler != nullptr) {
        _frameHandler(payload, payload_len);
    }

#if (COMMANDER == CMD_PC)
    sendAck(seq, ACK_STATUS_OK);
#endif
}

/**
 * @brief Validates a complete framed message and routes its payload.
 *
 * Performs:
 * - Minimum size check
 * - Payload-length/total-length consistency check
 * - CRC verification over frame core fields
 * - Payload dispatch on success
 *
 * @param frame Pointer to complete received frame bytes.
 * @param len Total frame length in bytes.
 */
void SerialProtocolHandler::processFrame(const uint8_t* frame, size_t len) {
    // Minimum framed size is a full header plus CRC with zero payload.
    if (len < (FRAME_HEADER_SIZE + CRC_SIZE)) {
        return;
    }

    const uint8_t seq = frame[2];
    const uint32_t timestamp = readTimestamp(&frame[3]);
    const uint8_t type = frame[TYPE_OFFSET];
    const uint16_t payload_len = (uint16_t)frame[LENGTH_OFFSET] |
                                 ((uint16_t)frame[LENGTH_OFFSET + 1] << 8);

    if (len != (FRAME_HEADER_SIZE + payload_len + CRC_SIZE)) {
#if (COMMANDER == CMD_PC)
        sendAck(seq, ACK_STATUS_INVALID_LENGTH);
#else
        Serial.println("Invalid frame length");
#endif
        return;
    }

    const uint16_t rx_crc = (uint16_t)frame[len - 2] | ((uint16_t)frame[len - 1] << 8);
    const uint16_t calc_crc = crc16_ccitt(&frame[2], FRAME_CORE_SIZE + payload_len);

    // Reject corrupted frames and explicitly signal CRC failure.
    if (rx_crc != calc_crc) {
#if (COMMANDER == CMD_PC)
        sendAck(seq, ACK_STATUS_CRC_ERROR);
#else
        Serial.println("CRC error");
#endif
        return;
    }

    handleFramePayload(seq, timestamp, type, &frame[PAYLOAD_OFFSET], payload_len);
}
#endif // BLUETOOTH_BLE