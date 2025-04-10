/**
 * @file Leg.h
 * @author Pierre-Yves Mordret <titolini72@gmail.com>
 * @brief Defines the Leg class for controlling a robot leg with femur, tibia, and feet actuators.
 * @version 1.0
 * @date 2026-01-02
 * @copyright Copyright (c) 2026 Pierre-Yves Mordret
 */

#ifndef LEG_H
#define LEG_H

#include <Arduino.h>
#include "../config.h"
#include "Bone.hpp"

/**
 * @brief State of the leg for movement and control.
 */
enum class STATE_LEG {
    UNKNOWN,   ///< Unknown state
    IDLE,      ///< Idle state
    FOLD,      ///< Folded state
    UNFOLD,    ///< Unfolded state
    MOVE,      /// Moving
    PREPARE,   ///< Preparing for attack
    ATTACK    ///< Attacking
};

/**
 * @brief Position of the leg during movement phases.
 */
enum class POSITION {
    RESTING,    ///< Resting position
    SWINGING,   ///< Swinging phase
    STANCING,   ///< Stancing phase
    UNFOLDING,  ///< Unfolding phase
    FOLDING,    ///< Folding phase
    IDLING,     ///< Idling phase
    PREPARING,  ///< Preparing phase
    ATTACKING  ///< Attacking phase
};

/**
 * @brief Leg class for controlling a robot leg with femur, tibia, and feet actuators.
 */
class Leg {
public:
    /**
     * @brief Construct a new Leg object.
     * @param femu Reference to femur Bone.
     * @param tibia Reference to tibia Bone.
     * @param feet Reference to feet Bone.
     * @param name Name of the leg.
     */
    Leg(Bone &femu, Bone &tibia, Bone &feet, const char *name)
    : _femur(femu), _tibia(tibia), _feet(feet), _state(STATE_LEG::IDLE), _pos(POSITION::RESTING),
    _step1(0), _step2(0), _step3(0), _step4(0) {
      _name = String(name);
    }

    /** @brief Initialize actuators. */
    void setup();
    /** @brief Main loop for leg control. */
    void loop();
    /** @brief Reset movement state. */
    void reset();
    /** @brief Invalidate pose state so the next idle()/move() re-animates. */
    void invalidate();
    /** @brief Check hardware attachment. */
    bool check();
    /** @brief Get current state. */
    enum STATE_LEG get_state() { return _state; };
    /** @brief Move leg forward through swing and stance phases. */
    bool up_and_move(float angle, uint8_t phase);
    /** @brief Fold leg. */
    bool fold();
    /** @brief Unfold leg. */
    bool unfold();
    /** @brief Set leg to idle. */
    bool idle();
    /** @brief Prepare for attack. */
    bool prepare_attack();
    /** @brief Execute attack. */
    bool attack();

    /** @brief Get current position. */
    enum POSITION get_position() { return _pos; }

    void set_speed(int speed)  { _speed = speed; }

    //void set_turn_sign(int sign) { _rotSign = sign; }
    void set_side_sign(int sign) { _sideSign = sign; }

    void init_phase(int o) { _phaseOffset = o % CYCLE_LEN; }

    static const uint8_t CYCLE_LEN = 80;   // 0..79



private:
    Bone &_femur;
    Bone &_tibia;
    Bone &_feet;

    enum STATE_LEG _state;
    enum POSITION _pos;

    String _name;
    unsigned int _step1, _step2,  _step3, _step4;
    int _fem_ori = 0, _tib_ori = 0, _feet_ori = 0;

    //int _rotSign;
    int _sideSign = 0;

    static const uint8_t SWING_LEN = 40;

    //uint8_t  _phase        = 0;           // dynamic phase counter (0..CYCLE_LEN-1)
    uint8_t  _phaseOffset  = 0;           // per-leg static offset
    //bool     _init_cycle   = false;        // flag to indicate the start of a new cycle

    unsigned int _speed = 0;
    unsigned long _prev = 0;
};
#endif