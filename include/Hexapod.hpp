/**
 * @file Hexapod.hpp
 * @author Pierre-Yves Mordret <titolini72@gmail.com>
 * @brief Defines the Hexapod class for controlling the robot's hexapod actuators.
 * @version 1.0
 * @date 2026-03-08
 * @copyright Copyright (c) 2026 Pierre-Yves Mordret
 */

#ifndef HEXAPOD_H
#define HEXAPOD_H
#include <Arduino.h>
#include "../config.h"

#include "Bone.hpp"
#include "Leg.hpp"
#include "Head.hpp"
#include "Tail.hpp"

class Hexapod {
public:
  static constexpr uint8_t kLegCount = 6;
  static constexpr size_t  kBoneCount = 3 * kLegCount + 4; // 3 bones/leg + 3 head + 1 tail

  // Constructor: builds the whole robot
  Hexapod();

  // Accessors
  Leg*   getLeg(uint8_t index);      // 0..5
  Head&  getHead();
  Tail&  getTail();

  Bone*  getBoneArray();
  size_t getBoneCount() const;

  Leg**  getLegArray();
  size_t getLegCount() const;

  bool check();
  void set_speed(int speed) {
    for (Leg* l : Legs) {
      l->set_speed(speed);
    }
    GripHead.set_speed(speed);
    BigTail.set_speed(speed);
  }

  // High-level API examples (optional)
  void idlePose();
  void foldPose();
  void wagTail();
  void lookForward();

  void attack_sequence(unsigned long now);

  void move();

  // Applies a full frame of servo angles (one per bone, in Bones array order),
  // clamping each value to the bone's configured limits before driving it.
  void servoApplyAngles(const uint8_t* angles);

  static constexpr int kServoMinAngle = 0;
  static constexpr int kServoMaxAngle = 180;

  float direction = 0.0f; // 0..360

private:
  // LEFT FRONT LEG (LF)
  Hardware  LF_Femur_Hw;
  Hardware  LF_Tibia_Hw;
  Hardware  LF_Feet_Hw;
  Mouvement LF_Femur_Mov;
  Mouvement LF_Tibia_Mov;
  Mouvement LF_Feet_Mov;
  Bone      LF_Femur;
  Bone      LF_Tibia;
  Bone      LF_Feet;
  Leg       LF_Leg;

  // LEFT MIDDLE LEG (LM)
  Hardware  LM_Femur_Hw;
  Hardware  LM_Tibia_Hw;
  Hardware  LM_Feet_Hw;
  Mouvement LM_Femur_Mov;
  Mouvement LM_Tibia_Mov;
  Mouvement LM_Feet_Mov;
  Bone      LM_Femur;
  Bone      LM_Tibia;
  Bone      LM_Feet;
  Leg       LM_Leg;

  // LEFT BACK LEG (LB)
  Hardware  LB_Femur_Hw;
  Hardware  LB_Tibia_Hw;
  Hardware  LB_Feet_Hw;
  Mouvement LB_Femur_Mov;
  Mouvement LB_Tibia_Mov;
  Mouvement LB_Feet_Mov;
  Bone      LB_Femur;
  Bone      LB_Tibia;
  Bone      LB_Feet;
  Leg       LB_Leg;

  // RIGHT FRONT LEG (RF)
  Hardware  RF_Femur_Hw;
  Hardware  RF_Tibia_Hw;
  Hardware  RF_Feet_Hw;
  Mouvement RF_Femur_Mov;
  Mouvement RF_Tibia_Mov;
  Mouvement RF_Feet_Mov;
  Bone      RF_Femur;
  Bone      RF_Tibia;
  Bone      RF_Feet;
  Leg       RF_Leg;

  // RIGHT MIDDLE LEG (RM)
  Hardware  RM_Femur_Hw;
  Hardware  RM_Tibia_Hw;
  Hardware  RM_Feet_Hw;
  Mouvement RM_Femur_Mov;
  Mouvement RM_Tibia_Mov;
  Mouvement RM_Feet_Mov;
  Bone      RM_Femur;
  Bone      RM_Tibia;
  Bone      RM_Feet;
  Leg       RM_Leg;

  // RIGHT BACK LEG (RB)
  Hardware  RB_Femur_Hw;
  Hardware  RB_Tibia_Hw;
  Hardware  RB_Feet_Hw;
  Mouvement RB_Femur_Mov;
  Mouvement RB_Tibia_Mov;
  Mouvement RB_Feet_Mov;
  Bone      RB_Femur;
  Bone      RB_Tibia;
  Bone      RB_Feet;
  Leg       RB_Leg;

  // HEAD
  Hardware  Head_Roll_Hw;
  Hardware  Head_Pitch_Hw;
  Hardware  Head_Grip_Hw;
  Mouvement Head_Roll_Mov;
  Mouvement Head_Pitch_Mov;
  Mouvement Head_Grip_Mov;
  Bone      Head_Roll;
  Bone      Head_Pitch;
  Bone      Head_Grip;
  Head      GripHead;

  // TAIL
  Hardware  Tail_Hw;
  Mouvement Tail_Mov;
  Bone      Tail_Bone;
  Tail      BigTail;

  enum LegIndex
  {
    LF = 0,
    LM = 1,
    LB = 2,
    RF = 3,
    RM = 4,
    RB = 5,
  };

  // Arrays
  Leg* Legs[kLegCount];
  Bone Bones[kBoneCount];

  void initLegArrays();
  void initBonesArray();

  uint8_t _gaitPhase = 0;

};
#endif // HEXAPOD_H