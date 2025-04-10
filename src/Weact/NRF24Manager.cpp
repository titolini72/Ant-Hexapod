#include "NRF24Manager.hpp"

#if defined(NRF24_LINK)

namespace {
constexpr uint8_t kAddressWidth = 5;
constexpr uint8_t kMaxPayloadSize = 32;

rf24_pa_dbm_e toRf24PaLevel(uint8_t level) {
  switch (level) {
    case NRF24_PA_LEVEL_MIN:
      return RF24_PA_MIN;
    case NRF24_PA_LEVEL_LOW:
      return RF24_PA_LOW;
    case NRF24_PA_LEVEL_HIGH:
      return RF24_PA_HIGH;
    case NRF24_PA_LEVEL_MAX:
    default:
      return RF24_PA_MAX;
  }
}

rf24_datarate_e toRf24DataRate(uint8_t rate) {
  switch (rate) {
    case NRF24_DATA_RATE_1MBPS:
      return RF24_1MBPS;
    case NRF24_DATA_RATE_2MBPS:
      return RF24_2MBPS;
    case NRF24_DATA_RATE_250KBPS:
    default:
      return RF24_250KBPS;
  }
}

const uint8_t kRxAddress[kAddressWidth + 1] = NRF24_RX_ADDRESS;
const uint8_t kTxAddress[kAddressWidth + 1] = NRF24_TX_ADDRESS;
} // namespace

NRF24Manager::NRF24Manager(SPIClass& spi, uint32_t cePin, uint32_t csnPin, uint32_t irqPin)
  : _spi(spi),
    _radio(NRF24_SPI_SPEED_HZ),
    _cePin(cePin),
    _csnPin(csnPin),
    _irqPin(irqPin) {}

bool NRF24Manager::begin() {
  pinMode(_irqPin, INPUT_PULLUP);

  _spi.begin();
  if (!_radio.begin(&_spi, _cePin, _csnPin)) {
    DEBUG_PRINTLN("NRF24 init failed");
    return false;
  }

  _radio.setAddressWidth(kAddressWidth);
  _radio.setChannel(NRF24_CHANNEL);
  _radio.setRetries(NRF24_RETRY_DELAY, NRF24_RETRY_COUNT);
  _radio.setPALevel(toRf24PaLevel(NRF24_PA_LEVEL));
  _radio.setDataRate(toRf24DataRate(NRF24_DATA_RATE));
  _radio.setCRCLength(RF24_CRC_16);
  _radio.enableDynamicPayloads();
  _radio.maskIRQ(true, true, false);
  _radio.openWritingPipe(kTxAddress);
  _radio.openReadingPipe(1, kRxAddress);
  _radio.startListening();

  _ready = true;
  DEBUG_PRINTLN("NRF24 ready - listening");
  return true;
}

void NRF24Manager::setFrameHandler(FrameHandler handler) {
  _frameHandler = handler;
}

void NRF24Manager::poll() {
  if (!_ready) {
    return;
  }

  if ((digitalRead(_irqPin) != LOW) && !_radio.available()) {
    return;
  }

  while (_radio.available()) {
    const uint8_t len = _radio.getDynamicPayloadSize();
    if (len == 0 || len > kMaxPayloadSize) {
      _radio.flush_rx();
      DEBUG_PRINTLN("NRF24 invalid payload size");
      break;
    }

    memset(_rxBuffer, 0, sizeof(_rxBuffer));
    _radio.read(_rxBuffer, len);

    if (_frameHandler != nullptr) {
      _frameHandler(_rxBuffer, len);
    }
  }
}

bool NRF24Manager::isReady() const {
  return _ready;
}

#endif // NRF24_LINK