/**
 * @file SerialProtocolHandler.cpp
 * @brief Unified serial protocol handler implementation.
 *
 * Framed protocol layout:
 * [SOF1][SOF2][SEQ][TS0][TS1][TS2][TS3][TYPE][LEN_L][LEN_H][PAYLOAD...][CRC_L][CRC_H]
 *
 * CRC is CRC16-CCITT over [SEQ..PAYLOAD].
 */

#include "SerialProtocolHandler.hpp"

SerialProtocolHandler::SerialProtocolHandler(Stream& serial,
                                             ReceiveMode receiveMode,
                                             uint16_t framePayloadSize,
                                             uint8_t frameType,
                                             bool ackEnabled,
                                             char terminator)
    : _serial(serial),
      _terminator(terminator),
      _receiveMode(receiveMode),
      _framePayloadSize(framePayloadSize),
      _frameType(frameType),
      _ackEnabled(ackEnabled) {
    clear();
}

uint16_t SerialProtocolHandler::FramePayloadSize() const {
    return _framePayloadSize;
}

void SerialProtocolHandler::setFrameHandler(FrameHandler handler) {
    _frameHandler = handler;
}

bool SerialProtocolHandler::sendFrame(uint8_t type, const uint8_t* payload, uint16_t length) {
    return sendFrame(_txSeq++, type, payload, length);
}

bool SerialProtocolHandler::sendFrame(uint8_t seq, uint8_t type, const uint8_t* payload, uint16_t length) {
    if (length > MAX_PAYLOAD) {
        return false;
    }

    const uint32_t timestamp = millis();
    const uint16_t total_len = FRAME_HEADER_SIZE + length + CRC_SIZE;

    if (total_len > MAX_BUFFER_SIZE) {
        return false;
    }

    uint8_t frame[MAX_BUFFER_SIZE];
    uint16_t idx = 0;

    frame[idx++] = SOF1;
    frame[idx++] = SOF2;
    frame[idx++] = seq;

    writeTimestamp(&frame[idx], timestamp);
    idx += TIMESTAMP_SIZE;

    frame[idx++] = type;
    frame[idx++] = (uint8_t)(length & 0xFF);
    frame[idx++] = (uint8_t)(length >> 8);

    for (uint16_t i = 0; i < length; i++) {
        frame[idx++] = payload[i];
    }

    const uint16_t crc = crc16_ccitt(frame + 2, idx - 2);
    frame[idx++] = (uint8_t)(crc & 0xFF);
    frame[idx++] = (uint8_t)(crc >> 8);

    _serial.write(frame, idx);
    _serial.flush();
    return true;
}

bool SerialProtocolHandler::update() {
    if (_lineReady) {
        return true;
    }

    if (!usesFramedProtocol()) {
        while (_serial.available() > 0) {
            const char b = static_cast<char>(_serial.read());

            if (b == _terminator) {
                _lineReady = true;
                break;
            }

            _buffer[0] = static_cast<uint8_t>(b);
            _length = 1;
        }
        return _lineReady;
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
                _overflow = true;
                sendAckIfEnabled(0, ACK_STATUS_INVALID_LENGTH);
                continue;
            }
        }

        if (idx >= FRAME_HEADER_SIZE) {
            const uint16_t payload_len = (uint16_t)_buffer[LENGTH_OFFSET] |
                                         ((uint16_t)_buffer[LENGTH_OFFSET + 1] << 8);
            const size_t total_len = FRAME_HEADER_SIZE + payload_len + CRC_SIZE;

            if (idx == total_len) {
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
}

void SerialProtocolHandler::sendAck(uint8_t seq, uint8_t status,
                                    const uint8_t* payload,
                                    uint16_t extra_len) {
    const uint16_t payload_len = 1 + extra_len;

    if (payload_len > MAX_PAYLOAD) {
        return;
    }

    uint8_t frame[FRAME_HEADER_SIZE + MAX_PAYLOAD];
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

    const uint16_t crc = crc16_ccitt(frame + 2, idx - 2);
    frame[idx++] = (uint8_t)(crc & 0xFF);
    frame[idx++] = (uint8_t)(crc >> 8);

    _serial.write(frame, idx);
    _serial.flush();
}

bool SerialProtocolHandler::hasBuffer() const {
    return _lineReady;
}

const uint8_t* SerialProtocolHandler::getBuffer() const {
    return _buffer;
}

size_t SerialProtocolHandler::length() const {
    return _length;
}

bool SerialProtocolHandler::overflowed() const {
    return _overflow;
}

void SerialProtocolHandler::clear() {
    _length = 0;
    _lineReady = false;
    _overflow = false;
    _frameIndex = 0;
    _buffer[0] = '\0';
}

uint16_t SerialProtocolHandler::crc16_ccitt(const uint8_t* data, size_t len) {
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

void SerialProtocolHandler::handleFramePayload(uint8_t seq,
                                               uint32_t timestamp,
                                               uint8_t type,
                                               const uint8_t* payload,
                                               uint16_t payload_len) {
    (void)timestamp;

    if (type != _frameType) {
        sendAckIfEnabled(seq, ACK_STATUS_UNKNOWN_COMMAND);
        return;
    }

    if (_framePayloadSize > 0 && payload_len != _framePayloadSize) {
        sendAckIfEnabled(seq, ACK_STATUS_INVALID_LENGTH);
        return;
    }

    if (_frameHandler != nullptr) {
        _frameHandler(payload, payload_len);
    }

    sendAckIfEnabled(seq, ACK_STATUS_OK);
}

void SerialProtocolHandler::processFrame(const uint8_t* frame, size_t len) {
    if (len < (FRAME_HEADER_SIZE + CRC_SIZE)) {
        return;
    }

    const uint8_t seq = frame[2];
    const uint32_t timestamp = readTimestamp(&frame[3]);
    const uint8_t type = frame[TYPE_OFFSET];
    const uint16_t payload_len = (uint16_t)frame[LENGTH_OFFSET] |
                                 ((uint16_t)frame[LENGTH_OFFSET + 1] << 8);

    if (len != (FRAME_HEADER_SIZE + payload_len + CRC_SIZE)) {
        sendAckIfEnabled(seq, ACK_STATUS_INVALID_LENGTH);
        return;
    }

    const uint16_t rx_crc = (uint16_t)frame[len - 2] | ((uint16_t)frame[len - 1] << 8);
    const uint16_t calc_crc = crc16_ccitt(&frame[2], FRAME_CORE_SIZE + payload_len);

    if (rx_crc != calc_crc) {
        sendAckIfEnabled(seq, ACK_STATUS_CRC_ERROR);
        return;
    }

    handleFramePayload(seq, timestamp, type, &frame[PAYLOAD_OFFSET], payload_len);
}

bool SerialProtocolHandler::usesFramedProtocol() const {
    return _receiveMode == ReceiveMode::Framed;
}

void SerialProtocolHandler::sendAckIfEnabled(uint8_t seq, uint8_t status,
                                             const uint8_t* payload,
                                             uint16_t extra_len) {
    if (!_ackEnabled) {
        return;
    }

    sendAck(seq, status, payload, extra_len);
}