#ifndef NRF24_MANAGER_HPP
#define NRF24_MANAGER_HPP

#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>

#include "../config.h"

#if defined(NRF24_LINK)

class NRF24Manager {
public:
  using FrameHandler = void (*)(const uint8_t* payload, uint16_t len);

  NRF24Manager(SPIClass& spi, uint32_t cePin, uint32_t csnPin, uint32_t irqPin);

  bool begin();
  void setFrameHandler(FrameHandler handler);
  void poll();
  bool isReady() const;

private:
  SPIClass& _spi;
  RF24 _radio;
  uint32_t _cePin;
  uint32_t _csnPin;
  uint32_t _irqPin;
  bool _ready = false;
  FrameHandler _frameHandler = nullptr;
  uint8_t _rxBuffer[32] = {0};
};

#endif // NRF24_LINK

#endif // NRF24_MANAGER_HPP