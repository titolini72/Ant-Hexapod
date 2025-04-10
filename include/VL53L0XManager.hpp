/**
 * @file VL53L0XManager.hpp
 * @author Pierre-Yves Mordret <titolini72@gmail.com>
 * @brief Interrupt-driven manager for VL53L0X distance sensor.
 * @version 1.0
 * @date 2026-04-15
 * @copyright Copyright (c) 2026 Pierre-Yves Mordret
 */

#ifndef VL53L0XMANAGER_H
#define VL53L0XMANAGER_H

#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>
#include "../config.h"

class VL53L0XManager {
public:
    /**
     * @param interruptPin GPIO connected to VL53L0X GPIO1 pin.
     * @param xshutPin Optional GPIO connected to VL53L0X XSHUT pin (-1 if unused).
     * @param measurementPeriodMs Continuous measurement period in ms.
     * @param obstacleThresholdMm Obstacle threshold in mm.
     */
    VL53L0XManager(uint8_t scl_pin,
                   uint8_t sda_pin,
                   uint8_t interruptPin,
                   int8_t xshutPin = -1,
                   uint16_t measurementPeriodMs = 50,
                   uint16_t obstacleThresholdMm = 300);

    /**
     * @brief Initialize I2C and sensor, configure hardware interrupt mode.
     * @param wire I2C bus instance.
     * @param i2cAddress Sensor address (default VL53L0X address is 0x29).
     * @return true when initialization succeeds.
     */
    bool setup(TwoWire &wire = Wire, uint8_t i2cAddress = 0x29);

    /**
     * @brief Stops ranging and detaches interrupt.
     */
    void end();

    /**
     * @brief Handle pending interrupt events and update measured distance.
     *        Call from the main Arduino loop.
     */
    void loop();

    /**
     * @brief Last valid measured distance in mm. 0xFFFF means invalid/timeout.
     */
    uint16_t lastDistanceMm() const { return _lastDistanceMm; }

    /**
     * @brief Returns true if last valid sample is below threshold.
     */
    bool isObstacleAhead() const;

    /**
     * @brief Returns true once when new sample has been processed.
     */
    bool hasNewSample() const { return _newSampleReady; }

    /**
     * @brief Acknowledge consumed sample.
     */
    void clearNewSampleFlag() { _newSampleReady = false; }

    /**
     * @brief Returns whether sensor was initialized.
     */
    bool ready() const { return _initialized; }

    /**
     * @brief Update obstacle threshold in mm.
     */
    void setObstacleThreshold(uint16_t thresholdMm) { _obstacleThresholdMm = thresholdMm; }

    /**
     * @brief Update continuous period in ms.
     *        Call before begin(), or call restartContinuous() afterward.
     */
    void setMeasurementPeriod(uint16_t periodMs) { _measurementPeriodMs = periodMs; }

    /**
     * @brief Restart continuous ranging with current period.
     * @return true when restart succeeded.
     */
    bool restartContinuous();

private:
    static void irqRouter();
    void onInterrupt();
    bool configureInterruptMode();

    static VL53L0XManager *s_instance;

    VL53L0X _sensor;
    TwoWire *_wire = nullptr;

    uint8_t _sclPin;
    uint8_t _sdaPin;
    uint8_t _interruptPin;
    int8_t _xshutPin;
    uint16_t _measurementPeriodMs;
    uint16_t _obstacleThresholdMm;

    volatile uint32_t _pendingInterrupts = 0;

    uint16_t _lastDistanceMm = 0xFFFF;
    bool _newSampleReady = false;
    bool _initialized = false;
};

#endif // VL53L0XMANAGER_H