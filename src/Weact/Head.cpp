/**
 * @file Head.cpp
 * @author Pierre-Yves Mordret <titolini72@gmail.com>
 * @brief Implements the Head class for controlling the robot's head actuators (roll, pitch, grip).
 * @version 1.0
 * @date 2026-01-02
 * @copyright Copyright (c) 2026 Pierre-Yves Mordret
 */

#include <Arduino.h>
#include "Head.hpp"

/**
 * @brief Initializes all head actuators and sets them to their idle positions.
 * @note Enum defines possible states for leg operation and transitions.
 * @note Used to manage leg behavior during robot actions.
 */
void Head::setup() {
  _roll.setup();
  _pitch.setup();
  _grip.setup();

  _roll.move(_roll.get("idle"));
  _pitch.move(_pitch.get("idle"));
  _grip.move(_grip.get("idle"));

  _prev_roll = _prev_grip = _prev_pitch = millis();

  delay(20);
  reset();
}

/**
 * @brief Checks if all actuators (roll, pitch, grip) are attached.
 * @return true if all actuators are attached, false otherwise.
 */
bool Head::check() {
  DEBUG_PRINT("Head: ");
  DEBUG_PRINTLN(_name);
  DEBUG_PRINT("\t- Roll: ");
  DEBUG_PRINTLN((_roll.attached() ? "attached" : "disattached"));
  DEBUG_PRINT("\t- Pitch: ");
  DEBUG_PRINTLN((_pitch.attached() ? "attached" : "disattached"));
  DEBUG_PRINT("\t- Grip : ");
  DEBUG_PRINTLN((_grip.attached() ? "attached" : "disattached"));

  return (_roll.attached() && _pitch.attached() && _grip.attached());
}

/**
 * @brief Resets all step counters and internal state variables for movement sequences.
 */
void Head::reset() {
  _step1 = _step2 = _step3 = _step4 = 0;
  _step5 = _step6 = _step7 = _step8 = 0;
  _step9 = 0;

  _grip_ori = _pitch_ori = _roll_ori = 0;
  _pitch_per = _roll_per = _grip_per = 255;
}

/**
 * @brief Invalidates the head pose state.
 *
 * Marks the head as no longer being in a known pose so a subsequent
 * idle()/move() call re-captures the current servo angles and animates from
 * there. Used when the servos are driven externally (e.g. raw servo frames in
 * CMD_PC mode) that bypass the head state machine.
 */
void Head::invalidate() {
  reset();
  _state = STATE_HEAD::UNKNOWN;
}

/**
 * @brief Runs a demonstration sequence for the head, moving roll, pitch, and grip through a series
          of actions.
 * @return true if the demo is in progress, false if idle.
 */
bool Head::move_demo() {
  unsigned long now = millis();

  if (now - _prev > _speed)
    _prev = now;
  else
    return (_state == STATE_HEAD::IDLE);

  if ( _state == STATE_HEAD::IDLE)
    reset();

  if (_step1 == 0) {
    _roll_ori = _roll.angle();
    _pitch_ori = _pitch.angle();
    _grip_ori = _grip.angle();
  }

  if (_step1 <= 40) {
    // Roll Right
    _state = STATE_HEAD::MOVING;
    _pitch.move(map(_step1, 0, 40, _pitch_ori, _pitch.get("idle")));
    _roll.move(map(_step2, 0, 40, _roll_ori, _roll.get("max_rt")));
    _grip.move(map(_step3, 0, 20, _grip_ori, _grip.get("idle")));
    _step1++;
  }

  if (_step1 >= 40 && _step2 <= 40) {
    // Pitch up and Roll to idle
    _roll.move(map(_step2, 0, 40, _roll.get("max_rt"), _roll.get("idle")));
    _pitch.move(
        map(_step2, 0, 40, _pitch.get("idle"), _pitch.get("max_up") / 2));
    _step2++;
  }

  if (_step2 >= 40 && _step3 <= 20) {
    // Open Grip
    _grip.move(map(_step3, 0, 20, _grip.get("idle"), _grip.get("open")));
    _step3++;
  }

  if (_step3 >= 20 && _step4 <= 10) {
    // Close Grip
    _grip.move(map(_step4, 0, 10, _grip.get("open"), _grip.get("close")));
    _step4++;
  }
  if (_step4 >= 10 && _step5 <= 40) {
    _grip.move(map(_step5, 0, 40, _grip.get("close"), _grip.get("open")));
    _step5++;
  }
  if (_step5 >= 40 && _step6 <= 10) {
    _grip.move(map(_step6, 0, 10, _grip.get("open"), _grip.get("close")));
    _step6++;
  }
  if (_step6 >= 10 && _step7 <= 20) {
    _grip.move(map(_step7, 0, 10, _grip.get("close"), _grip.get("idle")));
    _step7++;
  }
  if (_step7 >= 20 && _step8 <= 40) {
    // Roll to left and pitch back to idle
    _roll.move(map(_step8, 0, 40, _roll.get("idle"), _roll.get("max_lf")));
    _pitch.move(
        map(_step8, 0, 40, _pitch.get("max_up") / 2, _pitch.get("idle")));
    _step8++;
  }
  if (_step8 >= 40 && _step9 <= 40) {
    // Roll to idle
    _roll.move(map(_step8, 0, 40, _roll.get("max_lf"), _roll.get("idle")));
    _step9++;
  }

  if (_step9 >= 40) {
    _state = STATE_HEAD::IDLE;
    reset();
  }

  return (_state == STATE_HEAD::MOVING);
}

/**
 * @brief Returns the head to its idle state by setting pitch, roll, and grip to zero.
 * @return true if idle, otherwise false.
 */
bool Head::idle() {
  bool res = false;

  res = pitch_head(0);
  res &= roll_head(0);
  res &= grip(0);

  if (res)
    _state = STATE_HEAD::IDLE;

  return (_state == STATE_HEAD::IDLE);
}

/**
 * @brief Moves the pitch actuator smoothly to a target position based on a percentage (-100 to 100).
 * @param percentage Target position as a percentage (-100 to 100).
 * @return true when the target is reached, false otherwise.
 */
bool Head::pitch_head(int percentage) {
  unsigned long now = millis();

  // Range check
  if (percentage < -100 || percentage > 100)
    return false;

  if (now - _prev_pitch > _speed)
    _prev_pitch = now;
  else
    return (false);

  // Detect new target
  if (percentage != _pitch_per) {
    _pitch_per   = percentage;
    _step1       = 0;
    _pitch_ori   = _pitch.angle();
    _pitch_angle = map(
        percentage,
        -100, 100,
        _pitch.get("max_dw"),
        _pitch.get("max_up")
    );
  }

  // Already at target
  if (_step1 >= 25)
    return true;

  // Smooth incremental motion
  float t = (float)_step1 / 25.0f;
  float angle = _pitch_ori + t * (_pitch_angle - _pitch_ori);
  _pitch.move(angle);

  _step1++;

  return false;
}

/**
 * @brief Moves the roll actuator smoothly to a target position based on a percentage (-100 to 100).
 * @param percentage Target position as a percentage (-100 to 100).
 * @return true when the target is reached, false otherwise.
 */
bool  Head::roll_head(int percentage) {
  unsigned long now = millis();

  // Range check
  if (percentage < -100 || percentage > 100)
    return false;

  if (now - _prev_roll > _speed)
    _prev_roll = now;
  else
    return (false);

  // Detect new target
  if (percentage != _roll_per) {
    _roll_per   = percentage;
    _step2       = 0;
    _roll_ori   = _roll.angle();
    _roll_angle =  map(percentage, -100, 100, _roll.get("max_lf"), _roll.get("max_rt"));
  }

  // Already at target
  if (_step2 >= 25)
    return true;

  // Smooth incremental motion
  float t = (float)_step2 / 25.0f;
  float angle = _roll_ori + t * (_roll_angle - _roll_ori);
  _roll.move(angle);

  _step2++;

  return false;
}

/**
 * @brief Moves the grip actuator smoothly to a target position based on a percentage (-100 to 100).
 * @param percentage Target position as a percentage (-100 to 100).
 * @return true when the target is reached, false otherwise.
 */
bool Head::grip(int percentage)
{
  unsigned long now = millis();
  // Range check
  if (percentage < -100 || percentage > 100)
      return false;

  if (now - _prev_grip > _speed) {
    _prev_grip = now;
  } else {
    return (false);
  }

  // Detect new target
  if (percentage != _grip_per) {
    _grip_per   = percentage;
    _step3      = 0;
    _grip_ori   = _grip.angle();
    _grip_angle = map(
        percentage,
        -100, 100,
        _grip.get("close"),
        _grip.get("open")
    );
  }

  // Target already reached
  if (_step3 >= 20)
    return true;

  // Smooth incremental motion
  float t = (float)_step3 / 20.0f;
  float angle = _grip_ori + t * (_grip_angle - _grip_ori);
  _grip.move(angle);

  _step3++;

  return false;
}

/**
 * @brief Executes a bite sequence with the grip actuator, moving through open and close actions in 
          a timed sequence.
 */
void Head::bite() {
  unsigned long now = millis();

  if (_state != STATE_HEAD::BITTING) {
    reset();
    _state = STATE_HEAD::BITTING;
  }

  if (now - _prev_grip > _speed)
    _prev_grip = now;
  else
    return;

  if (_step1 <= 20)
    _grip.move(map(_step1++, 0, 20, _grip.get("idle"),  _grip.get("open") ));
  else if (_step2 <= 10)
    _grip.move(map(_step2++, 0, 10, _grip.get("open"),  _grip.get("close")));
  else if (_step3 <= 40)
    _grip.move(map(_step3++, 0, 40, _grip.get("close"), _grip.get("open") ));
  else if (_step4 <= 10)
    _grip.move(map(_step4++, 0, 10, _grip.get("open"),  _grip.get("close")));
  else if (_step5 <= 40)
    _grip.move(map(_step5++, 0, 40, _grip.get("close"), _grip.get("open") ));
  else if (_step6 <= 20)
    _grip.move(map(_step6++, 0, 20, _grip.get("open"),  _grip.get("idle") ));
  else {
    // Sequence finished
    _state = STATE_HEAD::IDLE;
  }
}