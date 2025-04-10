/**
 * @file SensorManager.cpp
 * @author Pierre-Yves Mordret <titolini72@gmail.com>
 * @brief Implements the SensorManager class for managing an ultrasonic distance sensor (e.g., HC-SR04).
 * @version 1.0
 * @date 2026-01-05
 * @copyright Copyright (c) 2026 Pierre-Yves Mordret
 */

#include <Arduino.h>
#include "SensorManager.hpp"

#define SONAR_CALIBRATION (0.034 / 2)

SensorManager *SensorManager::instance = NULL;

void SensorManager::echoISR() {
  if (!instance->_waitingForPulse) return;

  if (digitalRead(instance->_echoPin)) {
    // Rising edge
    instance->_pulseStart = micros();
  } else {
    // Falling edge
    instance->_pulseWidth = micros() - instance->_pulseStart;
    instance->_measurementDone = true;
    instance->_waitingForPulse = false;
  }
}

/**
 * @brief Construct a new SensorManager object and initialize the ultrasonic sensor pins.
 * @param trigPin Pin number for the trigger signal.
 * @param echoPin Pin number for the echo signal.
 * @param delay Minimum delay (ms) between distance measurements.
 */
SensorManager::SensorManager(unsigned char trigPin,
                             unsigned char echoPin,
                             uint16_t measurementPeriodMs,
                             uint16_t obstacleThresholdMm) {
  this->_trigPin = trigPin;
  this->_echoPin = echoPin;
  this->_now = millis();
  this->_measurementPeriodMs = measurementPeriodMs;
  this->_obstacleThresholdMm = obstacleThresholdMm;
  this->_last_distance = 0xFFFF;
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input


  attachInterrupt(digitalPinToInterrupt(echoPin),
                  this->echoISR,
                  CHANGE);

}

/**
 * @brief Check if there is an obstacle ahead based on the last measured distance.
 * @return true if an obstacle is detected within a predefined threshold, false otherwise.
 */
bool  SensorManager::isObstacleAhead() {
  return _last_distance < this->_obstacleThresholdMm;
}

/**
 * @brief Compute the distance measured by the ultrasonic sensor.
 * @return 0xFFFF if an error occurs (e.g., timeout), otherwise the distance in centimeters.
 */
int SensorManager::computeDistance() {
  //float distance;
  //unsigned long duration;

  // Clears the trigPin
  digitalWrite(this->_trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(this->_trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(this->_trigPin, LOW);

  // Start waiting for response
  this->_waitingForPulse = true;
  this->_measurementDone = false;

  return 0;
}

/**
 * @brief Main loop for periodically measuring distance.
 *        Should be called regularly in the main program loop.
 *        Updates the last measured distance if the delay has elapsed.
 */
void SensorManager::loop() {
  unsigned long now = millis();

  if ((now - this->_now) > (unsigned long)this->_measurementPeriodMs) {
    computeDistance();
    this->_now = now;
    this->_requestTime = now * 1000;
  }

  // Handle result
  if (this->_measurementDone) {
    float distance;
    noInterrupts();
    uint32_t width = this->_pulseWidth;
    interrupts();

    distance = width * SONAR_CALIBRATION; // distance in cm
    this->_last_distance = distance;
    this->_measurementDone = false;
#if defined(SENSOR_MANAGER_DEBUG)
    DEBUG_PRINT("Distance: "); DEBUG_PRINT(distance); DEBUG_PRINTLN(" cm");
#endif
  }

  // Handle timeout (200 ms max)
  if (this->_waitingForPulse && (micros() - this->_requestTime > (unsigned long)this->_measurementPeriodMs * 1000)) {
#if defined(SENSOR_MANAGER_DEBUG)
    DEBUG_PRINT("Sensor timeout: No echo received within "); DEBUG_PRINT((unsigned long)this->_measurementPeriodMs * 1000); DEBUG_PRINTLN(" us");
#endif
    this->_last_distance = 0xFFFF;
    this->_waitingForPulse = false;
  }
}