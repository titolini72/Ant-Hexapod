/**
 * @file PowerManager.cpp
 * @author Pierre-Yves Mordret <titolini72@gmail.com>
 * @brief Implements the PowerManager class for monitoring and managing power (battery voltage) and alerting when below threshold.
 * @version 1.0
 * @date 2026-01-02
 * @copyright Copyright (c) 2026 Pierre-Yves Mordret
 */

/*
 This file implements the PowerManager class, which monitors battery voltage using an analog pin and
 manages alerts when the voltage drops below a specified threshold.

  Key Features
    Constructor: Initializes the analog pin, voltage threshold, divider ratio, and delay between
                 checks.
    Setters: Methods to update pin, threshold, ratio, and delay.
    loop(): Main monitoring function. Reads voltage at intervals, updates alert status if voltage is
            below threshold.
  Usage
    Call loop() regularly in your main program to keep voltage monitoring and alerting up to date.

  Important Members
    _pin: Analog pin for voltage reading.
    _threshold: Voltage threshold for alert.
    _ratio: Divider ratio for converting ADC value to real voltage.
    _delay: Minimum delay between checks.
    _alert: Boolean flag for alert status.
    _last_voltage: Stores last measured voltage.
    Example
*/

#include <Arduino.h>
#include "PowerManager.hpp"

/**
 * @brief Construct a new PowerManager object and initialize the analog pin for voltage monitoring.
 * @param pin Analog pin number to read battery voltage from.
 * @param delay Minimum delay (ms) between voltage checks.
 * @param threshold Voltage threshold for alert (in volts).
 * @param ratio Voltage divider ratio to scale ADC reading to real voltage.
 */
PowerManager::PowerManager(unsigned char pin, int delay, float threshold, float ratio) {
  this->_pin = pin;
  this->_now = millis();
  this->_threshold = threshold;
  this->_alert = false;
  this->_ratio = ratio;
  this->_delay = delay;
  pinMode(pin, INPUT);
}

/**
 * @brief Set the analog pin for voltage monitoring.
 * @param pin Analog pin number.
 */
void PowerManager::set_pin(unsigned char pin) {
  this->_pin = pin;
}

/**
 * @brief Set the voltage threshold for alerting.
 * @param threshold Voltage threshold (in volts).
 */
void PowerManager::set_threshold(float threshold) {
  this->_threshold = threshold;
}

/**
 * @brief Set the voltage divider ratio.
 * @param ratio Ratio to convert ADC reading to real voltage.
 */
void PowerManager::set_ratio(float ratio) {
  this->_ratio = ratio;
}

/**
 * @brief Set the minimum delay between voltage checks.
 * @param delay Delay in milliseconds.
 */
void PowerManager::set_delay(int delay) {
  this->_delay = delay;
}

/**
 * @brief Main loop for monitoring battery voltage and updating alert status.
 *        Should be called regularly in the main program loop.
 *        Sets the alert flag if voltage is below the threshold.
 */
void PowerManager::loop() {
  unsigned long now = millis();
  int sensorValue = 0;
  float voltage = 0.;

  if ((now - this->_now) < (unsigned long)this->_delay)
    return;

  // Monitor the battery voltage
  sensorValue = analogRead(this->_pin);
  voltage = sensorValue * (ADC_MAX_VOLTAGE / ADC_RANGE) * this->_ratio; // Convert the reading values to voltage
  this->_last_voltage = voltage;
  // Set alert if voltage is below threshold
  this->_alert = (voltage < this->_threshold) ? true : false;
#if defined(POWER_MANAGER_DEBUG)
  DEBUG_PRINT("Voltage: "); DEBUG_PRINT(voltage); DEBUG_PRINTLN(" V");
  DEBUG_PRINT("Alert: "); DEBUG_PRINTLN(this->_alert ? "ON" : "OFF");
#endif
  this->_now = now;
}