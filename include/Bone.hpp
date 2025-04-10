
/**
 * @file Bone.h
 * @author Pierre-Yves Mordret <titolini72@gmail.com>
 * @brief Defines Hardware, Mouvement, and Bone classes for servo and movement control.
 * @version 1.0
 * @date 2026-01-02
 * @copyright Copyright (c) 2026 Pierre-Yves Mordret
 */

#ifndef BONE_H
#define BONE_H

#include <Arduino.h>
#include <ArduinoJson.h>
#include <Servo.h>

/**
 * @brief Hardware class to control a servo.
 * @note Handles movement, folding, unfolding, and attack actions for a robotic leg.
 * @note Tracks leg state and position using enums for precise control.
 * @note Manages speed and steps to ensure accurate leg operation.
 */
class Hardware {
  public:
    /**
     * @brief Construct a new Hardware object.
     * @param pin Servo pin.
     * @param min Minimum pulse width.
     * @param max Maximum pulse width.
     */
    Hardware(int pin, int min, int max) : _pin(pin), _min(min), _max(max) {}
    /**
     * @brief Construct a new Hardware object with default pulse range.
     * @param pin Servo pin.
     */
    Hardware(int pin) : _pin(pin), _min(544), _max(2400) {}

    /** @brief Attach servo. */
    void setup();
    /** @brief Move to angle. */
    void move (int angle);
    /** @brief Get current angle. */
    int angle() { return _angle; };
    /** @brief Check if attached. */
    bool attached() { return (s.attached()); };

  private:
    Servo s;
    int _pin;
    int _min;
    int _max;
    int _angle = 0;
};

/**
 * @brief Mouvement class to define movement constraints.
 */
class Mouvement {
  public:
    /**
     * @brief Construct a new Mouvement object from JSON string.
     * @param mov JSON string of movement parameters.
     */
    Mouvement(const char *mov) {
      deserializeJson(_mov, mov);
    }

    /**
     * @brief Get movement parameter by tag.
     * @param tag Parameter name.
     * @return Value of parameter.
     */
    int get(const char* tag);

  private:
    StaticJsonDocument<255> _mov;
};

/**
 * @brief Bone class to represent a single bone in the leg.
 */
class Bone {
  public:
    Bone() : _hw(nullptr), _mov(nullptr) {}

    /**
     * @brief Construct a new Bone object.
     * @param hw Reference to Hardware.
     * @param mov Reference to Mouvement.
     */
    Bone(Hardware &hw, Mouvement &mov) : _hw(&hw), _mov(&mov) {}

    /** @brief Setup hardware. */
    void setup() {_hw->setup(); };
    /** @brief Move bone to angle. */
    void move(int angle) { _hw->move(angle); };
    /** @brief Check if bone is attached. */
    bool attached() { return _hw->attached(); };
    /** @brief Get movement parameter. */
    int get(const char *tag) { return _mov->get(tag); };
    /** @brief Get current angle. */
    int angle() { return _hw->angle(); };

  private:
    Hardware *_hw;
    Mouvement *_mov;
};

#endif