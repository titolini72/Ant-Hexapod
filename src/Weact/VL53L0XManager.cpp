/**
 * @file VL53L0XManager.cpp
 * @author Pierre-Yves Mordret <titolini72@gmail.com>
 * @version 1.0
 * @date 2026-04-15
 * @copyright Copyright (c) 2026 Pierre-Yves Mordret
 * @brief Implements an interrupt-driven manager for the VL53L0X ToF distance sensor.
 *
 * The class configures VL53L0X GPIO1 as a data-ready interrupt source, then performs
 * all I2C transactions in the main loop context to keep the ISR short and deterministic.
 */

#include "VL53L0XManager.hpp"

namespace {
/** @brief VL53L0X register: GPIO interrupt configuration. */
constexpr uint8_t REG_SYSTEM_INTERRUPT_CONFIG_GPIO = 0x0A;
/** @brief VL53L0X register: interrupt clear command. */
constexpr uint8_t REG_SYSTEM_INTERRUPT_CLEAR = 0x0B;
/** @brief VL53L0X register: current interrupt status flags. */
constexpr uint8_t REG_RESULT_INTERRUPT_STATUS = 0x13;
/** @brief VL53L0X register: GPIO polarity control (active high/low). */
constexpr uint8_t REG_GPIO_HV_MUX_ACTIVE_HIGH = 0x84;

/** @brief Configure interrupt when new range data is ready (active-low mode). */
constexpr uint8_t INTERRUPT_LEVEL_LOW_NEW_SAMPLE_READY = 0x04;
/** @brief Value used to acknowledge and clear a range interrupt. */
constexpr uint8_t CLEAR_RANGE_INTERRUPT = 0x01;
/** @brief Bit mask for GPIO polarity bit in REG_GPIO_HV_MUX_ACTIVE_HIGH. */
constexpr uint8_t GPIO_ACTIVE_HIGH_MASK = 0x10;
/** @brief Mask of range-ready interrupt bits in REG_RESULT_INTERRUPT_STATUS. */
constexpr uint8_t RANGE_STATUS_READY_MASK = 0x07;
} // namespace

/**
 * @brief Static pointer used by the ISR trampoline to access the active instance.
 */
VL53L0XManager *VL53L0XManager::s_instance = nullptr;

/**
 * @brief Construct a VL53L0X interrupt manager.
 * @param scl_pin I2C SCL pin used by the selected TwoWire instance.
 * @param sda_pin I2C SDA pin used by the selected TwoWire instance.
 * @param interruptPin GPIO connected to VL53L0X GPIO1 (data-ready interrupt).
 * @param xshutPin Optional GPIO connected to VL53L0X XSHUT pin; pass -1 if unused.
 * @param measurementPeriodMs Continuous ranging period in milliseconds.
 * @param obstacleThresholdMm Obstacle threshold in millimeters.
 */
VL53L0XManager::VL53L0XManager(uint8_t scl_pin,
                               uint8_t sda_pin,
                               uint8_t interruptPin,
                               int8_t xshutPin,
                               uint16_t measurementPeriodMs,
                               uint16_t obstacleThresholdMm)
    : _sclPin(scl_pin),
      _sdaPin(sda_pin),
      _interruptPin(interruptPin),
      _xshutPin(xshutPin),
      _measurementPeriodMs(measurementPeriodMs),
      _obstacleThresholdMm(obstacleThresholdMm) {}

/**
 * @brief Initialize the VL53L0X sensor and enable interrupt-driven ranging.
 *
 * Steps performed:
 * 1. Configure the I2C pins and bus.
 * 2. Optionally reset sensor through XSHUT.
 * 3. Initialize sensor and optional I2C address.
 * 4. Program interrupt mode (new sample ready, active low).
 * 5. Attach MCU GPIO interrupt and start continuous ranging.
 *
 * @param wire TwoWire bus to use.
 * @param i2cAddress Sensor I2C address (default is 0x29).
 * @return true if initialization succeeded, false otherwise.
 */
bool VL53L0XManager::setup(TwoWire &wire, uint8_t i2cAddress) {
    _wire = &wire;

    _wire->setSDA(_sdaPin);
    _wire->setSCL(_sclPin);

    _wire->begin();

    pinMode(_interruptPin, INPUT_PULLUP);

    if (_xshutPin >= 0) {
        pinMode(static_cast<uint8_t>(_xshutPin), OUTPUT);
        digitalWrite(static_cast<uint8_t>(_xshutPin), LOW);
        delay(5);
        digitalWrite(static_cast<uint8_t>(_xshutPin), HIGH);
        delay(5);
    }

    _sensor.setBus(_wire);
    _sensor.setTimeout(50);

    if (!_sensor.init()) {
        _initialized = false;
        DEBUG_PRINTLN("Failed to initialize VL53L0X sensor");
        return false;
    }

    if (i2cAddress != 0x29) {
        _sensor.setAddress(i2cAddress);
    }

    // A moderate budget keeps noise low while still reacting quickly.
    _sensor.setMeasurementTimingBudget(33000);

    if (!configureInterruptMode()) {
        _initialized = false;
        DEBUG_PRINTLN("Failed to configure VL53L0X interrupt mode");
        return false;
    }

    s_instance = this;
    _pendingInterrupts = 0;
    attachInterrupt(_interruptPin, irqRouter, FALLING);

    _sensor.startContinuous(_measurementPeriodMs);
    _initialized = true;
    return true;
}

/**
 * @brief Stop continuous ranging and detach MCU interrupt.
 */
void VL53L0XManager::end() {
    if (_initialized) {
        _sensor.stopContinuous();
    }
    detachInterrupt(_interruptPin);
    _initialized = false;
    _pendingInterrupts = 0;
}

/**
 * @brief Restart continuous ranging with the currently configured period.
 * @return true if restart was requested, false if sensor is not initialized.
 */
bool VL53L0XManager::restartContinuous() {
    if (!_initialized) {
        return false;
    }
    _sensor.stopContinuous();
    _sensor.startContinuous(_measurementPeriodMs);
    return true;
}

/**
 * @brief Program VL53L0X internal interrupt registers for GPIO1 data-ready signaling.
 * @return true when register writes are issued.
 */
bool VL53L0XManager::configureInterruptMode() {
    // Trigger GPIO1 when a new range sample is available.
    _sensor.writeReg(REG_SYSTEM_INTERRUPT_CONFIG_GPIO, INTERRUPT_LEVEL_LOW_NEW_SAMPLE_READY);

    // Use active-low interrupt level on GPIO1.
    uint8_t mux = _sensor.readReg(REG_GPIO_HV_MUX_ACTIVE_HIGH);
    _sensor.writeReg(REG_GPIO_HV_MUX_ACTIVE_HIGH, mux & ~GPIO_ACTIVE_HIGH_MASK);

    // Clear any stale interrupt from power-up.
    _sensor.writeReg(REG_SYSTEM_INTERRUPT_CLEAR, CLEAR_RANGE_INTERRUPT);
    return true;
}

/**
 * @brief Process pending sensor interrupts and update the latest measured distance.
 *
 * This method is intentionally non-blocking from ISR perspective:
 * - ISR only increments a pending counter.
 * - Main loop consumes pending events and performs I2C reads.
 *
 * On timeout, `_lastDistanceMm` is set to `0xFFFF` to mark an invalid sample.
 */
void VL53L0XManager::loop() {
    if (!_initialized) {
        return;
    }

    uint32_t pending = 0;
    noInterrupts();
    pending = _pendingInterrupts;
    _pendingInterrupts = 0;
    interrupts();

    if (pending == 0) {
        return;
    }

    // Read one latest sample per loop even if multiple interrupts happened.
    if ((_sensor.readReg(REG_RESULT_INTERRUPT_STATUS) & RANGE_STATUS_READY_MASK) == 0) {
        _sensor.writeReg(REG_SYSTEM_INTERRUPT_CLEAR, CLEAR_RANGE_INTERRUPT);
        return;
    }

    const uint16_t mm = _sensor.readRangeContinuousMillimeters();
    if (_sensor.timeoutOccurred()) {
        // Sentinel value used by the public API for invalid range.
        _lastDistanceMm = 0xFFFF;
    } else {
        _lastDistanceMm = mm;
    }
    DEBUG_PRINTLN("VL53L0X sample: " + String(_lastDistanceMm) + " mm");


    _newSampleReady = true;

    // Required to re-arm GPIO1 interrupt for the next sample.
    _sensor.writeReg(REG_SYSTEM_INTERRUPT_CLEAR, CLEAR_RANGE_INTERRUPT);
}

/**
 * @brief Check if the latest valid sample indicates an obstacle within threshold.
 * @return true when a valid sample exists and is <= obstacle threshold.
 */
bool VL53L0XManager::isObstacleAhead() const {
    return (_lastDistanceMm != 0xFFFF) && (_lastDistanceMm <= _obstacleThresholdMm);
}

/**
 * @brief Static ISR trampoline that routes to the current instance.
 */
void VL53L0XManager::irqRouter() {
    if (s_instance != nullptr) {
        s_instance->onInterrupt();
    }
}

/**
 * @brief ISR handler.
 *
 * Keeps ISR execution minimal by only incrementing a pending event counter.
 * Actual sensor reads occur later in loop().
 */
void VL53L0XManager::onInterrupt() {
    ++_pendingInterrupts;
}