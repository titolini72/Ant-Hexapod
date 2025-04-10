/**
 * @file SerialProtocolHandler.hpp
 * @brief Serial line and framed payload protocol handler.
 */

#ifndef SERIAL_PROTOCOL_HANDLER_HPP
#define SERIAL_PROTOCOL_HANDLER_HPP

#include <Arduino.h>
#include "../config.h"

#if defined(BLUETOOTH_BLE)

class SerialProtocolHandler {
public:
    static constexpr size_t MAX_BUFFER_SIZE = 128;
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
    static constexpr uint16_t MAX_PAYLOAD = 32;
    static constexpr uint8_t ACK_TYPE = 0xAC;
    static constexpr uint8_t FRAME_TYPE_SERVO = 0x01;
    static constexpr uint8_t FRAME_TYPE_TRANSMITTER = 0xDA;
    static constexpr uint8_t ACK_STATUS_OK = 0x00;
    static constexpr uint8_t ACK_STATUS_CRC_ERROR = 0x01;
    static constexpr uint8_t ACK_STATUS_INVALID_LENGTH = 0x02;
    static constexpr uint8_t ACK_STATUS_UNKNOWN_COMMAND = 0x03;

    using FrameHandler = void (*)(const uint8_t* payload, uint16_t len);

    explicit SerialProtocolHandler(Stream& serial,
                                   uint16_t FramePayloadSize,
                                   char terminator = '\n');

    uint16_t FramePayloadSize() const;

    void setFrameHandler(FrameHandler handler);

    bool update();

#if (COMMANDER == CMD_PC)
    void sendAck(uint8_t seq, uint8_t status,
                 const uint8_t* payload = nullptr,
                 uint16_t extra_len = 0);
#endif

    bool hasBuffer() const;

    const uint8_t* getBuffer() const;

    size_t length() const;

    bool overflowed() const;

    void clear();

private:
    static uint16_t crc16_ccitt(const uint8_t* data, size_t len);

    static uint32_t readTimestamp(const uint8_t* data);

    static void writeTimestamp(uint8_t* data, uint32_t timestamp);

    void handleFramePayload(uint8_t seq, uint32_t timestamp, uint8_t type,
                            const uint8_t* payload, uint16_t payload_len);

    void processFrame(const uint8_t* frame, size_t len);

    Stream& _serial;
    uint8_t _buffer[MAX_BUFFER_SIZE + 1] = {0};
    size_t _length = 0;
    char _terminator;
    bool _lineReady = false;
    bool _overflow = false;
    size_t _frameIndex = 0;
    uint16_t _FramePayloadSize;
    uint8_t _txSeq = 0;
    FrameHandler _frameHandler = nullptr;
};

#endif // BLUETOOTH_BLE

#endif // SERIAL_PROTOCOL_HANDLER_HPP