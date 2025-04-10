/**
 * @file Tail.cpp
 * @author Pierre-Yves Mordret <titolini72@gmail.com>
 * @brief Implements the Tail class for controlling the robot's tail actuator.
 * @version 1.0
 * @date 2026-01-02
 * @copyright Copyright (c) 2026 Pierre-Yves Mordret
 */

#include <Arduino.h>
#include "Tail.hpp"

/**
 * @brief Initialize the tail actuator and set to idle.
 */
void Tail::setup()
{
  _tail.setup();

  _tail.move(_tail.get("idle"));
  delay(20);

  _prev = millis();
  _state = STATE_TAIL::UNKNOWN;
  reset();
}

/**
 * @brief Check if the tail actuator is attached.
 * @return true if attached, false otherwise.
 */
bool Tail::check() {
  DEBUG_PRINT("Tail: ") ; DEBUG_PRINTLN(_name);
  DEBUG_PRINT("\t- Tail: "); DEBUG_PRINTLN((_tail.attached()? "attached" : "disattached"));

  return ( _tail.attached() );
}

/**
 * @brief Reset the tail movement state.
 */
void Tail::reset() {
  _step = 0;
}

/**
 * @brief Invalidates the tail pose state.
 *
 * Forces the next move()/idle() to re-detect its target and re-capture the
 * current servo angle. Used when the servo is driven externally (e.g. raw
 * servo frames in CMD_PC mode) that bypass the tail state machine.
 */
void Tail::invalidate() {
  _step = 0;
  _per = 255;
  _state = STATE_TAIL::UNKNOWN;
}

/**
 * @brief Shake the tail left and right in a sequence.
 */
void  Tail::wag() {
  static unsigned int step;
  static unsigned int ori;

  if (millis() - _prev > _speed)
    _prev = millis();
  else
    return;

  if (step == 0)
    ori = _tail.angle();
  
  if (step <= 25) {
    _tail.move( map(step, 0, 25, ori,  _tail.get("right")));
    step++;
  }
  if ( step >= 25 && step <= 50) {
    _tail.move( map(step - 25, 0, 25, _tail.get("right"), _tail.get("left")));
    step++;
  }
  if (step >= 50)
    step = 0;
}

/**
 * @brief Set the tail to idle position.
 */
void Tail::idle() {
  move(0);
}

/**
 * @brief Move the tail actuator to a position based on percentage.
 * @param percentage Target position (-100 to 100).
 */
void Tail::move(int percentage) {
  if ((percentage > 100) || (percentage < -100))
    return;

  if (millis() - _prev > _speed)
    _prev = millis();
  else
    return;

  if (_per != percentage)
    _step = 0;

  if (_step == 0) {
    _ori = _tail.angle();
    _angle = map (percentage, -100, 100, _tail.get("right"), _tail.get("left"));
    _per = percentage;
  }

  if (_step <= 25) {
    _tail.move( map(_step, 0, 25,  _ori,  _angle));
    _step++;
  }
}