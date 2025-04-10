/**
 * @file Leg.cpp
 * @author Pierre-Yves Mordret <titolini72@gmail.com>
 * @brief Implements the Leg class for controlling a robot leg with femur, tibia, and feet actuators.
 * @version 1.0
 * @date 2026-01-02
 * @copyright Copyright (c) 2026 Pierre-Yves Mordret
 */

#include <Arduino.h>
#include "Leg.hpp"

/**
 * @brief Initializes all leg actuators and sets them to their folded positions.
 */
void Leg::setup()
{
  _femur.setup();
  _tibia.setup();
  _feet.setup();

  _femur.move(_femur.get("fold"));
  _tibia.move(_tibia.get("fold"));
  _feet.move(_feet.get("fold"));
  delay(20);

  _prev = millis();

  reset();
  _state = STATE_LEG::UNKNOWN;
  _pos = POSITION::RESTING;
}

/**
 * @brief Resets all step counters and internal state variables for movement sequences.
 */
void Leg::reset() {
  _step1 = _step2 = _step3 = _step4 = 0;
  //_phase = 0;
  //_init_cycle = false;
  _fem_ori =_tib_ori = _feet_ori = 0;
  _pos = POSITION::RESTING;
}

/**
 * @brief Invalidates the leg pose state.
 *
 * Marks the leg as no longer being in a known pose so that a subsequent
 * idle()/move() call re-captures the current servo angles and animates from
 * there. Used when the servos are driven externally (e.g. raw servo frames in
 * CMD_PC mode) that bypass the leg state machine.
 */
void Leg::invalidate() {
  reset();
  _state = STATE_LEG::UNKNOWN;
}

/**
 * @brief Checks if all actuators (femur, tibia, feet) are attached.
 * @return true if all actuators are attached, false otherwise.
 */
bool Leg::check() {
  DEBUG_PRINT("Leg: ") ; DEBUG_PRINTLN(_name);
  DEBUG_PRINT("\t- Femur: "); DEBUG_PRINTLN((_femur.attached()? "attached" : "disattached"));
  DEBUG_PRINT("\t- Tibia: "); DEBUG_PRINTLN((_tibia.attached()? "attached" : "disattached"));
  DEBUG_PRINT("\t- Feet : "); DEBUG_PRINTLN((_feet.attached() ? "attached" : "disattached"));

  return ( _femur.attached() && _tibia.attached() && _feet.attached());
}

/**
 * @brief Moves the leg forward through a swing and stance phase.
 * @param turn Interpolation factor for femur movement.
 * @return true if the leg is in the FORWARD state, false otherwise.
 */
bool Leg::up_and_move(float angle, uint8_t phase)
{
  unsigned long now = millis();

  if (now - _prev > _speed)
    _prev = now;
  else
    return false;

  // --- Phase mapping (with offset) ---
  uint8_t effective = (phase + _phaseOffset) % CYCLE_LEN;

  // --- Init capture, independent of phase value ---
  //if (!_init_cycle) {
  if ( _state != STATE_LEG::MOVE) {
    _feet_ori = _feet.angle();
    _fem_ori  = _femur.angle();
    _tib_ori  = _tibia.angle();
    //_init_cycle = true;
    _state = STATE_LEG::MOVE;
  }

  // --- Direction vector ---
  float rad = angle * DEG_TO_RAD;
  float vx = cos(rad);   // forward/back
  float vy = sin(rad);   // left/right

  // mix depending on leg position
  float stride = vx + vy * _sideSign;

  // clamp
  if (stride > 1) stride = 1;
  if (stride < -1) stride = -1;

  bool pushForward = true;
  const float STRIDE_DEADZONE = 0.1f;
  if (stride >  STRIDE_DEADZONE)      pushForward = true;
  else if (stride < -STRIDE_DEADZONE) pushForward = false;

  // Convert to femur target
  float femForward  = _femur.get("forward");
  float femBackward = _femur.get("backward");

  float femRange = femForward - femBackward;

  // center mouvement
  float center = (femForward + femBackward) * 0.5f;

  // amplitude depends on stride
  float amplitude = femRange * 0.5f * abs(stride);

  // direction
  float femSwingStart = center - amplitude;
  float femSwingEnd   = center + amplitude;

  // -------------------------
  // SWING PHASE
  // -------------------------
  if (effective < SWING_LEN)
  {
    float t = effective / float(SWING_LEN);
    float s = 0.5f - 0.5f * cos(t * PI);

    float femPos;
    if (pushForward) {
      // forward: swing back -> front
      femPos = femSwingStart + (femSwingEnd - femSwingStart) * s;
    } else {
      // backward: swing front -> back
      femPos = femSwingEnd + (femSwingStart - femSwingEnd) * s;
    }

    float lift = sin(t * PI);

    float feetIdle = _feet.get("idle");
    float feetUp   = _feet.get("up");

    float tibIdle  = _tibia.get("idle");
    float tibUp    = _tibia.get("up");

    _femur.move(femPos);
    _feet.move(feetIdle + (feetUp - feetIdle) * lift);
    _tibia.move(tibIdle + (tibUp - tibIdle) * lift);

    _pos = POSITION::SWINGING;
  }
  // -------------------------
  // STANCE PHASE
  // -------------------------
  else
  {
    float t = (effective - SWING_LEN) / float(CYCLE_LEN - SWING_LEN);
    float s = 0.5f - 0.5f * cos(t * PI);

    float femPos;
    if (pushForward) {
      // forward: stance front -> back (push body forward)
      femPos = femSwingEnd + (femSwingStart - femSwingEnd) * s;
    } else {
      // backward: stance back -> front (push body backward)
      femPos = femSwingStart + (femSwingEnd - femSwingStart) * s;
    }
    _femur.move(femPos);
    _feet.move(_feet.get("idle"));
    _tibia.move(_tibia.get("idle"));

    _pos = POSITION::STANCING;
  }

  return true;
}

/**
 * @brief Unfolds the leg to its extended position.
 * @return true if the leg is in the UNFOLD state, false otherwise.
 */
bool Leg::unfold() {
  unsigned long now = millis();

  if (now - _prev > _speed)
    _prev = now;
  else
    return (_state == STATE_LEG::UNFOLD);

  if (_state == STATE_LEG::UNFOLD)
    return true;

  if (_step1 == 0) {
    _feet_ori = _feet.angle();
    _fem_ori = _femur.angle();
    _tib_ori = _tibia.angle();
  }

  if (_step1 <= 30) {
    _feet.move( map(_step1, 0, 30, _feet_ori, _feet.get("unfold")));
    _tibia.move(map(_step1, 0, 30, _tib_ori,  _tibia.get("unfold")));
    _femur.move(map(_step2, 0, 30, _fem_ori,  _femur.get("unfold")));
    _pos = POSITION::UNFOLDING;
    _step1++;
  }

  // Reset the counters for repeating the process
  if (_step1 >= 30) {
    _state = STATE_LEG::UNFOLD;
    reset();
  }

  return (_state == STATE_LEG::UNFOLD);
}

/**
 * @brief Folds the leg to its retracted position.
 * @return true if the leg is in the FOLD state, false otherwise.
 */
bool Leg::fold() {
  unsigned long now = millis();

  if (now - _prev > _speed)
    _prev = now;
  else
    return (_state == STATE_LEG::FOLD);

  if (_state == STATE_LEG::FOLD)
    return true;

  if (_step1 == 0) {
    _feet_ori = _feet.angle();
    _fem_ori = _femur.angle();
    _tib_ori = _tibia.angle();
  }

  if (_step1 <= 30) {
    _feet.move( map(_step1, 0, 30, _feet_ori,  _feet.get("fold")));
    _tibia.move(map(_step1, 0, 30, _tib_ori, _tibia.get("fold")));
    _femur.move(map(_step1, 0, 30, _fem_ori, _femur.get("fold")));
    _pos = POSITION::FOLDING;
    _step1++;
  }

  if (_step1 >= 30) {
    _state = STATE_LEG::FOLD;
    reset();
  }

  return (_state == STATE_LEG::FOLD);
}

/**
 * @brief Sets the leg to its idle position.
 * @return true if the leg is in the IDLE state, false otherwise.
 */
bool Leg::idle() {
  unsigned long now = millis();

  if (now - _prev > _speed)
    _prev = now;
  else
    return (_state == STATE_LEG::IDLE);

  if ((_state == STATE_LEG::IDLE) && (_pos == POSITION::RESTING))
    return true;

  if ((_step1 == 0) || (_pos != POSITION::IDLING)) {
    _feet_ori = _feet.angle();
    _fem_ori = _femur.angle();
    _tib_ori = _tibia.angle();
  }

  if (_step1 <= 30) {
    _feet.move( map(_step1, 0, 30, _feet_ori,  _feet.get("idle")));
    _tibia.move(map(_step1, 0, 30, _tib_ori, _tibia.get("idle")));
    _femur.move(map(_step1, 0, 30, _fem_ori, _femur.get("idle")));
    _pos = POSITION::IDLING;
    _step1++;
  }

  if (_step1 >= 30) {
    _state = STATE_LEG::IDLE;
    _pos = POSITION::RESTING;
    reset();
  }

  return (_state == STATE_LEG::IDLE);
}

/**
 * @brief Prepares the leg for an attack by moving to a ready position.
 * @return true if the leg is in the PREPARE state, false otherwise.
 */
bool Leg::prepare_attack() {
  unsigned long now = millis();

  if (now - _prev > _speed)
    _prev = now;
  else
    return (_state == STATE_LEG::PREPARE);

  if (_state == STATE_LEG::PREPARE)
    return true;

  if (_step1 == 0) {
    _feet_ori = _feet.angle();
    _fem_ori = _femur.angle();
    _tib_ori = _tibia.angle();
  }

  if (_step1 <= 15) {
    _feet.move( map(_step1, 0, 15, _feet_ori,  _feet.get("jib")));
    _tibia.move(map(_step1, 0, 15, _tib_ori, _tibia.get("jib")));
    _pos = POSITION::PREPARING;
  }

  if (_step1 <= 30) {
    _femur.move(map(_step1, 0, 30, _fem_ori, _femur.get("moment")));
    _step1++;
  }

  if (_step1 >= 30) {
    _state = STATE_LEG::PREPARE;
    _pos = POSITION::RESTING;
    reset();
  }

  return (_state == STATE_LEG::PREPARE);
}

/**
 * @brief Executes an attack movement with the leg.
 * @return true if the leg is in the ATTACK state, false otherwise.
 */
bool Leg::attack() {
  unsigned long now = millis();

  if (now - _prev > _speed)
    _prev = now;
  else
    return (_state == STATE_LEG::ATTACK);

  if (_state == STATE_LEG::ATTACK)
    return true;

  if (_step1 == 0) {
    _feet_ori = _feet.angle();
    _fem_ori = _femur.angle();
    _tib_ori = _tibia.angle();
  }

  if (_step1 <= 10) {
    _feet.move( map(_step1, 0, 10, _feet_ori, _feet.get("attack")));
    _tibia.move(map(_step1, 0, 10, _tib_ori,  _tibia.get("attack")));
    _pos = POSITION::ATTACKING;
  }
  if (_step1 <= 15) {
    _femur.move(map(_step1, 0, 15, _fem_ori, _femur.get("attack")));
    _step1++;
  }

  if (_step1 >= 15) {
    _state = STATE_LEG::ATTACK;
    _pos = POSITION::RESTING;
    reset();
  }

  return (_state == STATE_LEG::ATTACK);
}

