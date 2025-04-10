/**
 * @file SensorManager.h
 * @author Pierre-Yves Mordret <titolini72@gmail.com>
 * @brief Defines the SensorManager class for managing an ultrasonic distance sensor (e.g., HC-SR04).
 * @version 1.0
 * @date 2026-01-05
 * @copyright Copyright (c) 2026 Pierre-Yves Mordret
 */

#ifndef SENSORMANAGER_H
#define SENSORMANAGER_H

#include <Arduino.h>
#include "../config.h"

/**
 * @brief SensorManager class for managing an ultrasonic distance sensor (e.g., HC-SR04).
 * @note The SensorManager class controls an ultrasonic distance sensor, providing setup, loop, reset, checks, and
 *       distance measurement.
 * @note It stores the trigger and echo pins, delay between measurements, last measured distance, and timing
 *       information.
 */
class SensorManager {
public:
    /**
     * @brief Construct a new SensorManager object and initialize the ultrasonic sensor pins.
     * @param trigPin Pin number for the trigger signal.
     * @param echoPin Pin number for the echo signal.
     * @param delay Minimum delay (ms) between distance measurements.
     */
    SensorManager(unsigned char trigPin,
                  unsigned char echoPin,
                  uint16_t measurementPeriodMs = 50,
                  uint16_t obstacleThresholdMm = 300);

    /**
     * @brief Compute the distance measured by the ultrasonic sensor.
     *        Sends a trigger pulse and measures the echo time to calculate distance.
     * @return Distance in centimeters.
     */
    int computeDistance();

    void setup() { instance = this; }

    /**
     * @brief Main loop for periodically measuring distance.
     *        Should be called regularly in the main program loop.
     *        Updates the last measured distance if the delay has elapsed.
     */
    void loop();

    /**
     * @brief Get the last measured distance.
     * @return Last measured distance in centimeters.
     */
    int last_distance() const { return _last_distance; }

    /**
     * @brief Check if there is an obstacle ahead based on the last measured distance.
     * @return true if an obstacle is detected within a predefined threshold, false otherwise.
     */
    bool isObstacleAhead();

private:

    static void echoISR();
    static SensorManager * instance;

    unsigned char _trigPin;
    unsigned char _echoPin;
    int _measurementPeriodMs;
    int _obstacleThresholdMm;

    unsigned long _now;
    int _last_distance;

    volatile uint32_t _pulseStart = 0;
    volatile uint32_t _pulseWidth = 0;

    volatile bool _waitingForPulse = false;
    volatile bool _measurementDone = false;

    uint32_t _requestTime = 0;
};

#endif // SENSORMANAGER_H
