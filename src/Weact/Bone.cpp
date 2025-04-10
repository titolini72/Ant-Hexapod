/**
 * @file Bone.cpp
 * @author Pierre-Yves Mordret <titolini72@gmail.com>
 * @brief Implements Hardware, Mouvement, and Bone classes for servo and movement control.
 * @version 1.0
 * @date 2026-01-02
 * @copyright Copyright (c) 2026 Pierre-Yves Mordret
 */

#include <Arduino.h>
#include "Bone.hpp"

static const unsigned int EXCEPTION_MESSAGE_SIZE =  80; // [bytes]
char exceptionMessage[EXCEPTION_MESSAGE_SIZE];

/**
 * @brief Attach the servo to the specified pin and pulse range.
 */
void Hardware::setup() {
  s.attach(_pin, _min, _max);
}

/**
 * @brief Move the servo to the specified angle.
 * @param angle Target angle.
 */
void Hardware::move(int angle) {
  _angle = angle;
  s.write(angle);
}

/**
 * @brief Get movement parameter by tag from JSON config.
 * @param tag Parameter name.
 * @return Value of parameter.
 * @throws std::runtime_error if tag not found.
 */
int Mouvement::get(const char* tag) {
  if (_mov.containsKey(tag))
  {
    return _mov[tag];
  }
  else
  {
    snprintf_P(exceptionMessage, EXCEPTION_MESSAGE_SIZE, PSTR("required entry '%s' not found in config file"), tag);
    throw std::runtime_error(exceptionMessage);
    return 0;
  }
}