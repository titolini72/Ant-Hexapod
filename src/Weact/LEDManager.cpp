/**
 * @file LEDManager.cpp
 * @author Pierre-Yves Mordret <titolini72@gmail.com>
 * @brief Implements the LEDManager class for controlling status LEDs.
 * @version 1.0
 * @date 2026-01-02
 * @copyright Copyright (c) 2026 Pierre-Yves Mordret
 */

#include "LEDManager.hpp"

/**
 * @brief Construct a new LEDManager object and initialize the pin.
 * @param pin LED pin number.
 */
LEDManager::LEDManager(unsigned char pin) {
  this->_pin = pin;
  this->_now = millis();
  pinMode(_pin, OUTPUT);
}

/**
 * @brief Toggle the LED on and off for specified durations.
 * @param On Duration to keep LED on (ms).
 * @param Off Duration to keep LED off (ms).
 */
void LEDManager::toggle(int On, int Off) {
  digitalWrite(_pin, HIGH);
  delay(On);
  digitalWrite(_pin, LOW);
  delay(Off);
}

/**
 * @brief Set the LED mode pattern for blinking.
 * @param modePattern List of pattern steps.
 */
void LEDManager::setMode(
  const std::initializer_list<unsigned int> &modePattern) {
  // Temporary buffer to hold the new pattern for comparison
  unsigned int newPattern[MODE_PATTERN_MAX + 1] = {0};

  size_t index = 0;
  for (auto pattern : modePattern) {
    if (index >= MODE_PATTERN_MAX)
      break;
    newPattern[index++] = pattern;
  }
  newPattern[index] = 0;  // null-terminate

  // Check if newPattern is different from current modePattern
  bool isDifferent = false;
  for (size_t i = 0; i <= MODE_PATTERN_MAX; ++i) {
    if (this->_modePattern[i] != newPattern[i]) {
      isDifferent = true;
      break;
    }
    // If both patterns end early with 0, break early
    if (this->_modePattern[i] == 0 && newPattern[i] == 0)
      break;
  }

  if (!isDifferent) {
    // Patterns are identical, no update needed
    return;
  }

  // Update the internal pattern
  for (size_t i = 0; i <= MODE_PATTERN_MAX; ++i) {
    this->_modePattern[i] = newPattern[i];
    if (newPattern[i] == 0)
      break;
  }

  // Reset mode step and counter since pattern changed
  this->_modeStep = 0;
  this->_modeCounter = 0;
}

/**
 * @brief Main loop for LED blinking and pattern control.
 */
void LEDManager::loop() {
  unsigned long now = millis();

  if ((now - this->_now) < DELAY)
    return;

  this->_modeCounter++;
  if (this->_modeCounter >= this->_modePattern[this->_modeStep]) {
    this->_modeCounter = 0;
    this->_modeStep+=MODE_PATTERN_INFO;
    if ((this->_modePattern[this->_modeStep] == 0) || (this->_modeStep >= MODE_PATTERN_MAX)) {
      this->_modeStep = 0;
    }

    digitalWrite(this->_pin, this->_modeStep % 2 ? HIGH : LOW);
  }
  this->_now = now;
}
