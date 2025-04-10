/**
 * @file Head.h
 * @author Pierre-Yves Mordret <titolini72@gmail.com>
 * @brief Defines the Head class for controlling the robot's head actuators (roll, pitch, grip).
 * @version 1.0
 * @date 2026-01-02
 * @copyright Copyright (c) 2026 Pierre-Yves Mordret
 */

#ifndef HEAD_H
#define HEAD_H

#include <Arduino.h>
#include "Bone.hpp"
#include "../config.h"

/**
 * @brief State of the head for movement and control.
 */
enum class STATE_HEAD {
  UNKNOWN,   ///< Unknown state
  IDLE,      ///< Idle state
  MOVING,    ///< Moving
  BITTING,   ///< Biting
};

/**
 * @brief Head class for controlling the robot's head actuators (roll, pitch, grip).
 * @note The Head class controls a robot head’s roll, pitch, and grip actuators, providing setup,
 * loop, reset, checks, demo/idle motions, direct actuator commands, and a bite sequence.
 * It stores references to three Bone actuators, motion state/steps, angles/percentages, speed, timing,
 * and a name.
 */
class Head {
public:
  /**
   * @brief Construct a new Head object.
   * @param roll Reference to roll Bone.
   * @param pitch Reference to pitch Bone.
   * @param grip Reference to grip Bone.
   * @param name Name of the head.
   */
  Head(Bone &roll, Bone &pitch, Bone &grip, const char *name)
      : _roll(roll), _pitch(pitch), _grip(grip), _step1(0), _step2(0),
        _step3(0), _step4(0), _step5(0), _step6(0), _step7(0), _step8(0),
        _step9(0) {
    _name = String(name);
  }

  /** @brief Initialize actuators. */
  void setup();
  /** @brief Main loop for head control. */
  void loop();
  /** @brief Reset movement state. */
  void reset();
  /** @brief Invalidate pose state so the next idle()/move() re-animates. */
  void invalidate();
  /** @brief Check hardware attachment. */
  bool check();
  /** @brief Run demonstration sequence. */
  bool move_demo();
  /** @brief Set head to idle. */
  bool idle();
  /** @brief Move grip actuator. */
  bool grip(int percentage);
  /** @brief Move roll actuator. */
  bool roll_head(int percentage);
  /** @brief Move pitch actuator. */
  bool pitch_head(int percentage);
  /** @brief Execute bite sequence. */
  void bite();

  void set_speed(int speed)  { _speed = speed; }

private:
  Bone &_roll;
  Bone &_pitch;
  Bone &_grip;

  String _name;
  unsigned int _step1, _step2, _step3, _step4;
  unsigned int _step5, _step6, _step7, _step8, _step9;

  float _roll_ori = 0, _pitch_ori = 0, _grip_ori = 0;
  float _pitch_angle = 0, _roll_angle = 0, _grip_angle = 0;

  int _pitch_per = 0 , _roll_per = 0, _grip_per = 0;

  enum STATE_HEAD _state = STATE_HEAD::UNKNOWN;

  unsigned int _speed = 0;
  unsigned long _prev_roll = 0, _prev_pitch = 0, _prev_grip = 0, _prev = 0;
};
#endif