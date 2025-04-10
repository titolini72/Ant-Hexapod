/**
 * @file Tail.h
 * @author Pierre-Yves Mordret <titolini72@gmail.com>
 * @brief Defines the Tail class for controlling the robot's tail actuator.
 * @version 1.0
 * @date 2026-01-02
 * @copyright Copyright (c) 2026 Pierre-Yves Mordret
 */

#ifndef TAIL_H
#define TAIL_H

#include <Arduino.h>
#include "../config.h"
#include "Bone.hpp"

/**
 * @brief State of the tail for movement and control.
 */
enum class STATE_TAIL {
    UNKNOWN,   ///< Unknown state
    IDLE,      ///< Idle state
    MOVING,    ///< Moving
};

/**
 * @brief Tail class for controlling the robot's tail actuator.
 * @note The Tail class controls a robot tail’s actuator, providing setup, loop, reset, checks, idle motion, shaking,
 *       and direct movement commands.
 * @note It stores a reference to a Bone actuator, movement state/step, angles/percentages, speed, timing, and a name.
 */
class Tail {
public:
    /**
     * @brief Construct a new Tail object.
     * @param tail Reference to tail Bone.
     * @param name Name of the tail.
     */
    Tail(Bone &tail, const char *name) : _tail(tail)
    { _name = String(name); }

    /** @brief Initialize actuator. */
    void setup();
    /** @brief Main loop for tail control. */
    void loop();
    /** @brief Reset movement state. */
    void reset();
    /** @brief Invalidate pose state so the next idle()/move() re-animates. */
    void invalidate();
    /** @brief Check hardware attachment. */
    bool check();
    /** @brief Set tail to idle. */
    void idle();
    /** @brief Shake tail. */
    void wag();
    /** @brief Move tail actuator. */
    void move(int percentage);

    void set_speed( int speed)  { _speed = speed; }

private:
    Bone &_tail;
    unsigned int _step = 0;
    int _ori = 0;
    int _angle = 0;
    int _per = 0;
    String _name;
    enum STATE_TAIL _state = STATE_TAIL::UNKNOWN;

    unsigned int _speed = 0;
    unsigned long _prev = 0;
};
#endif