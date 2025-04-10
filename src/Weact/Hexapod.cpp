/**
 * @file Hexapod.cpp
 * @brief Implementation of the Hexapod body assembly and high-level behaviors.
 *
 * This unit wires all hardware servo channels, movement profiles, and composite
 * body parts (legs, head, and tail) into a ready-to-use @ref Hexapod instance.
 * It also provides runtime helpers for initialization checks, tripod-gait
 * locomotion, and simple behavior presets (idle, fold, attack, tail wag).
 */
#include "Hexapod.hpp"

/**
 * @brief Constructs and assembles the full hexapod body model.
 *
 * Initializes every joint's hardware abstraction, movement profile, and
 * composed part instance for:
 * - 6 legs (LF, LM, LB, RF, RM, RB)
 * - 1 head assembly
 * - 1 tail assembly
 *
 * After member initialization, this constructor configures internal arrays used
 * for indexed access and synchronized control.
 */
Hexapod::Hexapod()
  // ----------------- LEFT FRONT LEG (LF) -----------------
  : LF_Femur_Hw(S16, 600, 2400)
  , LF_Tibia_Hw(S17, 600, 2400)
  , LF_Feet_Hw (S18, 600, 2400)

  , LF_Femur_Mov(
      "{ \"idle\": 110, \"fold\": 110, \"max\": 180, \"min\": 0,"
      "  \"forward\": 130, \"backward\": 80, \"moment\": 140, \"attack\": 80 }")
  , LF_Tibia_Mov(
      "{ \"idle\": 108, \"fold\": 180, \"max\": 180, \"min\": 0,"
      "  \"up\": 150, \"jib\": 127, \"attack\": 67 }")
  , LF_Feet_Mov(
      "{ \"idle\": 145, \"fold\": 180, \"max\": 180, \"min\": 0,"
      "  \"up\": 110, \"jib\": 130, \"attack\": 125 }")

  , LF_Femur(LF_Femur_Hw, LF_Femur_Mov)
  , LF_Tibia(LF_Tibia_Hw, LF_Tibia_Mov)
  , LF_Feet (LF_Feet_Hw,  LF_Feet_Mov)

  , LF_Leg(LF_Femur, LF_Tibia, LF_Feet, "Left Front")

  // ----------------- LEFT MIDDLE LEG (LM) -----------------
  , LM_Femur_Hw(S19, 600, 2400)
  , LM_Tibia_Hw(S20, 600, 2400)
  , LM_Feet_Hw (S21, 600, 2400)

  , LM_Femur_Mov(
      "{ \"idle\": 85,  \"fold\": 90,  \"max\": 180, \"min\": 0,"
      "  \"forward\": 95, \"backward\": 60, \"moment\": 120, \"attack\": 60 }")
  , LM_Tibia_Mov(
      "{ \"idle\": 80, \"fold\": 180, \"max\": 180, \"min\": 0,"
      "  \"up\": 150, \"jib\": 108, \"attack\": 88 }")
  , LM_Feet_Mov(
      "{ \"idle\": 135, \"fold\": 180, \"max\": 180, \"min\": 0,"
      "  \"up\": 110, \"jib\": 145, \"attack\": 130 }")

  , LM_Femur(LM_Femur_Hw, LM_Femur_Mov)
  , LM_Tibia(LM_Tibia_Hw, LM_Tibia_Mov)
  , LM_Feet (LM_Feet_Hw,  LM_Feet_Mov)

  , LM_Leg(LM_Femur, LM_Tibia, LM_Feet, "Left Middle")

  // ----------------- LEFT BACK LEG (LB) -----------------
  , LB_Femur_Hw(S22, 600, 2400)
  , LB_Tibia_Hw(S23, 600, 2400)
  , LB_Feet_Hw (S24, 600, 2400)

  , LB_Femur_Mov(
      "{ \"idle\": 90,  \"fold\": 90,  \"max\": 180, \"min\": 0,"
      "  \"forward\": 110, \"backward\": 70, \"moment\": 120, \"attack\": 60 }")
  , LB_Tibia_Mov(
      "{ \"idle\": 105, \"fold\": 180, \"max\": 180, \"min\": 0,"
      "  \"up\": 150, \"jib\": 92, \"attack\": 97 }")
  , LB_Feet_Mov(
      "{ \"idle\": 145, \"fold\": 180, \"max\": 180, \"min\": 0,"
      "  \"up\": 110, \"jib\": 160, \"attack\": 132 }")

  , LB_Femur(LB_Femur_Hw, LB_Femur_Mov)
  , LB_Tibia(LB_Tibia_Hw, LB_Tibia_Mov)
  , LB_Feet (LB_Feet_Hw,  LB_Feet_Mov)

  , LB_Leg(LB_Femur, LB_Tibia, LB_Feet, "Left Back")

  // ----------------- RIGHT FRONT LEG (RF) -----------------
  , RF_Femur_Hw(S4, 600, 2400)
  , RF_Tibia_Hw(S5, 600, 2400)
  , RF_Feet_Hw (S6, 600, 2400)

  , RF_Femur_Mov(
      "{ \"idle\": 98, \"fold\": 98, \"max\": 180, \"min\": 0,"
      "  \"forward\": 70, \"backward\": 118, \"moment\": 60, \"attack\": 120 }")
  , RF_Tibia_Mov(
      "{ \"idle\": 76, \"fold\": 0,  \"max\": 180, \"min\": 0,"
      "  \"up\": 20, \"jib\": 65, \"attack\": 121 }")
  , RF_Feet_Mov(
      "{ \"idle\": 30, \"fold\": 0,  \"max\": 180, \"min\": 0,"
      "  \"up\": 70, \"jib\": 45, \"attack\": 50 }")

  , RF_Femur(RF_Femur_Hw, RF_Femur_Mov)
  , RF_Tibia(RF_Tibia_Hw, RF_Tibia_Mov)
  , RF_Feet (RF_Feet_Hw,  RF_Feet_Mov)

  , RF_Leg(RF_Femur, RF_Tibia, RF_Feet, "Right Front")

  // ----------------- RIGHT MIDDLE LEG (RM) -----------------
  , RM_Femur_Hw(S7, 600, 2400)
  , RM_Tibia_Hw(S8, 600, 2400)
  , RM_Feet_Hw (S9, 600, 2400)

  , RM_Femur_Mov(
      "{ \"idle\": 105, \"fold\": 101, \"max\": 170, \"min\": 0,"
      "  \"forward\": 81, \"backward\": 121, \"moment\": 71, \"attack\": 131 }")
  , RM_Tibia_Mov(
      "{ \"idle\": 90, \"fold\": 0,   \"max\": 180, \"min\": 0,"
      "  \"up\": 20, \"jib\": 73, \"attack\": 93 }")
  , RM_Feet_Mov(
      "{ \"idle\": 40, \"fold\": 0,   \"max\": 180, \"min\": 0,"
      "  \"up\": 70, \"jib\": 40, \"attack\": 55 }")

  , RM_Femur(RM_Femur_Hw, RM_Femur_Mov)
  , RM_Tibia(RM_Tibia_Hw, RM_Tibia_Mov)
  , RM_Feet (RM_Feet_Hw,  RM_Feet_Mov)

  , RM_Leg(RM_Femur, RM_Tibia, RM_Feet, "Right Middle")

  // ----------------- RIGHT BACK LEG (RB) -----------------
  , RB_Femur_Hw(S10, 600, 2400)
  , RB_Tibia_Hw(S11, 600, 2400)
  , RB_Feet_Hw (S12, 600, 2400)

  , RB_Femur_Mov(
      "{ \"idle\": 97, \"fold\": 97, \"max\": 180, \"min\": 0,"
      "  \"forward\": 77, \"backward\": 117, \"moment\": 67, \"attack\": 127 }")
  , RB_Tibia_Mov(
      "{ \"idle\": 62, \"fold\": 0,  \"max\": 180, \"min\": 0,"
      "  \"up\": 20, \"jib\": 77, \"attack\": 72 }")
  , RB_Feet_Mov(
      "{ \"idle\": 25, \"fold\": 0,  \"max\": 180, \"min\": 0,"
      "  \"up\": 70, \"jib\": 5, \"attack\": 40 }")

  , RB_Femur(RB_Femur_Hw, RB_Femur_Mov)
  , RB_Tibia(RB_Tibia_Hw, RB_Tibia_Mov)
  , RB_Feet (RB_Feet_Hw,  RB_Feet_Mov)

  , RB_Leg(RB_Femur, RB_Tibia, RB_Feet, "Right Back")

  // ----------------- HEAD -----------------
  , Head_Roll_Hw (S15, 600, 2400)
  , Head_Pitch_Hw(S14, 600, 2400)
  , Head_Grip_Hw (S13, 600, 2400)

  , Head_Roll_Mov(
      "{ \"idle\": 69, \"max_lf\": 18, \"max_rt\": 120, \"min\": 0, \"max\": 120 }")
  , Head_Pitch_Mov(
      "{ \"idle\": 90, \"max_dw\": 125, \"max_up\": 50, \"min\": 50, \"max\": 125 }")
  , Head_Grip_Mov(
      "{ \"idle\": 110, \"open\": 85, \"close\": 130, \"min\": 85, \"max\": 130 }")

  , Head_Roll (Head_Roll_Hw,  Head_Roll_Mov)
  , Head_Pitch(Head_Pitch_Hw, Head_Pitch_Mov)
  , Head_Grip (Head_Grip_Hw,  Head_Grip_Mov)

  , GripHead(Head_Roll, Head_Pitch, Head_Grip, "Big Head")

  // ----------------- TAIL -----------------
  , Tail_Hw(S3, 600, 2400)
  , Tail_Mov(
      "{ \"idle\": 95, \"left\": 100, \"right\": 90, \"min\": 0, \"max\": 180 }")

  , Tail_Bone(Tail_Hw, Tail_Mov)
  , BigTail(Tail_Bone, "Big Tail")
{
  initLegArrays();
  initBonesArray();
}

/**
 * @brief Initializes indexed leg access and gait-related leg metadata.
 *
 * This function:
 * - Populates the @c Legs pointer array with each concrete leg instance.
 * - Assigns lateral side signs used by direction-aware leg logic.
 * - Seeds initial gait phase offsets for two tripod groups.
 * - Resets each leg so first movement cycle can capture baseline pose.
 */
void Hexapod::initLegArrays() {
  Legs[LF] = &LF_Leg;
  Legs[LM] = &LM_Leg;
  Legs[LB] = &LB_Leg;
  Legs[RF] = &RF_Leg;
  Legs[RM] = &RM_Leg;
  Legs[RB] = &RB_Leg;

  // lateral signs
  Legs[LF]->set_side_sign(+1);
  Legs[LM]->set_side_sign(+1);
  Legs[LB]->set_side_sign(+1);

  Legs[RF]->set_side_sign(-1);
  Legs[RM]->set_side_sign(-1);
  Legs[RB]->set_side_sign(-1);

  // Tripod A: LF, RM, LB – phase 0
  Legs[LF]->init_phase(0);
  Legs[RM]->init_phase(0);
  Legs[LB]->init_phase(0);

  // Tripod B: RF, LM, RB – phase 40 (half cycle out of phase)
  Legs[RF]->init_phase(40);
  Legs[LM]->init_phase(40);
  Legs[RB]->init_phase(40);

  // Put legs in idle/non-initialized state; next move() captures pose
  for ( Leg *l : Legs) {
    l->reset();
  }
}

/**
 * @brief Builds a contiguous bone array used by bulk operations.
 *
 * Copies all body bones (legs, head, tail) into the internal @c Bones array.
 * This enables generic iteration without having to traverse each subsystem
 * separately.
 */
void Hexapod::initBonesArray() {
  Bone tmp[] = {
    LF_Femur, LF_Tibia, LF_Feet,
    LM_Femur, LM_Tibia, LM_Feet,
    LB_Femur, LB_Tibia, LB_Feet,
    RF_Femur, RF_Tibia, RF_Feet,
    RM_Femur, RM_Tibia, RM_Feet,
    RB_Femur, RB_Tibia, RB_Feet,
    Head_Roll, Head_Pitch, Head_Grip,
    Tail_Bone
  };

  const size_t n = sizeof(tmp) / sizeof(tmp[0]);
  for (size_t i = 0; i < n; ++i) {
    Bones[i] = tmp[i];
  }
}

/**
 * @brief Retrieves a leg pointer by index.
 * @param index Zero-based leg index in [0, 5].
 * @return Pointer to the selected @ref Leg, or @c nullptr if index is invalid.
 */
Leg* Hexapod::getLeg(uint8_t index) {
  if (index >= 6) return nullptr;
  return Legs[index];
}

/**
 * @brief Returns the hexapod head assembly.
 * @return Reference to the internal @ref Head object.
 */
Head& Hexapod::getHead() {
  return GripHead;
}

/**
 * @brief Returns the hexapod tail assembly.
 * @return Reference to the internal @ref Tail object.
 */
Tail& Hexapod::getTail() {
  return BigTail;
}

/**
 * @brief Provides raw access to the contiguous bone array.
 * @return Pointer to first element of internal @c Bones storage.
 */
Bone* Hexapod::getBoneArray() {
  return Bones;
}

/**
 * @brief Returns the number of bones tracked in the internal array.
 * @return Total bone count.
 */
size_t Hexapod::getBoneCount() const {
  return sizeof(Bones) / sizeof(Bones[0]);
}

/**
 * @brief Provides raw access to the indexed leg pointer array.
 * @return Pointer to first element of internal @c Legs storage.
 */
Leg** Hexapod::getLegArray() {
  return Legs;
}

/**
 * @brief Returns the number of legs managed by this robot model.
 * @return Always 6 for a hexapod.
 */
size_t Hexapod::getLegCount() const {
  return 6;
}

/**
 * @brief Verifies actuator attachment and readiness for all subsystems.
 *
 * Calls setup and health checks for every leg, then head, then tail.
 * On the first failure, a diagnostic message is emitted and @c false is
 * returned.
 *
 * @retval true All subsystems report ready.
 * @retval false At least one subsystem failed setup or check.
 */
bool Hexapod::check() {
  for ( Leg *l : Legs) {
    l->setup();
    if ( !l->check() ) {
      DEBUG_PRINTLN("Problem with leg attachment. Halting.");
      return false;
    }
  }

  GripHead.setup();
  if (!GripHead.check()) {
    DEBUG_PRINTLN("Problem with head attachment. Halting.");
    return false;
  }

  BigTail.setup();
  if (!BigTail.check()) {
    DEBUG_PRINTLN("Problem with tail attachment. Halting.");
    return false;
  }
  return true;
}

/**
 * @brief Advances one locomotion step using a tripod gait.
 *
 * Each call attempts to move both tripod groups using the current direction and
 * gait phase. If any leg reports movement progress, the shared gait phase is
 * incremented and wrapped at @c CYCLE_LEN.
 */
void Hexapod::move()
{
  float dir = direction;
  bool move = false;

  // tripod A
  move |= Legs[LF]->up_and_move(dir, _gaitPhase);
  move |= Legs[RM]->up_and_move(dir, _gaitPhase);
  move |= Legs[LB]->up_and_move(dir, _gaitPhase);

  // tripod B
  move |= Legs[RF]->up_and_move(dir, _gaitPhase);
  move |= Legs[LM]->up_and_move(dir, _gaitPhase);
  move |= Legs[RB]->up_and_move(dir, _gaitPhase);

  if (move) {
    _gaitPhase++;

    if (_gaitPhase >= Legs[LM]->CYCLE_LEN)
        _gaitPhase = 0;
  }
}

/**
 * @brief Sends the robot to its nominal idle pose.
 *
 * Applies idle posture to all legs, head, and tail.
 */
void Hexapod::idlePose() {
  for ( Leg *l : Legs) {
    l->idle();
  }
  GripHead.idle();
  BigTail.idle();
}

/**
 * @brief Folds all legs into the configured compact pose.
 */
void Hexapod::foldPose() {
  for ( Leg *l : Legs) {
    l->fold();
  }
}

/**
 * @brief Runs a timed three-stage attack behavior.
 *
 * Behavior stages:
 * 1. Prepare: legs prepare, head pitches forward, grip closes.
 * 2. Attack: after 2 seconds, legs attack, head pitches back, grip opens.
 * 3. Recover: after 3 more seconds, all parts return to idle.
 *
 * @param now Current timestamp in milliseconds (typically from @c millis()).
 */
void Hexapod::attack_sequence(unsigned long now) {
  static unsigned long prev = millis();
  // Prepare attack
  if (LF_Leg.get_state() != STATE_LEG::PREPARE &&
      LF_Leg.get_state() != STATE_LEG::ATTACK) {
    for ( Leg *l : Legs)
      l->prepare_attack();
    GripHead.pitch_head(100);
    GripHead.grip(100);
    BigTail.wag();
    if (LF_Leg.get_state() == STATE_LEG::PREPARE)
      prev = millis();
  }
  else if (LF_Leg.get_state() == STATE_LEG::PREPARE) {
    if ( ( now - prev ) > 2000 ) {
      for ( Leg *l : Legs)
        l->attack();
      GripHead.pitch_head(0);
      GripHead.grip(-100);
      BigTail.wag();
      if (LF_Leg.get_state() == STATE_LEG::ATTACK)
        prev = millis();
    }
  }
  else if (LF_Leg.get_state() == STATE_LEG::ATTACK) {
    if ( ( now - prev ) > 3000 ) {
      for ( Leg *l : Legs)
        l->idle();
      GripHead.idle();
      BigTail.idle();
    }
  }
}

/**
 * @brief Commands the tail wag behavior once.
 */
void Hexapod::wagTail() {
   BigTail.wag();
}

/**
 * @brief Centers the head to its forward/idle orientation.
 */
void Hexapod::lookForward() {
  GripHead.idle();
}

/**
 * @brief Applies a full frame of raw servo angles to every bone.
 *
 * The @p angles buffer holds one value per bone, ordered exactly as the
 * internal @c Bones array (see @ref initBonesArray). Each angle is clamped to
 * the bone's configured movement limits when available (the @c "min" / @c "max"
 * tags of its movement profile) and otherwise to the safe servo range
 * [@ref kServoMinAngle, @ref kServoMaxAngle] before being written to hardware.
 *
 * @param angles Pointer to @ref kBoneCount angle values. Ignored if @c nullptr.
 */
void Hexapod::servoApplyAngles(const uint8_t* angles) {
  if (angles == nullptr) {
    return;
  }

  for (size_t i = 0; i < kBoneCount; ++i) {
    Bone& bone = Bones[i];

    // Default to the universal servo range, then tighten with per-bone limits
    // when the movement profile defines them (leg bones do; head/tail do not).
    int lo = kServoMinAngle;
    int hi = kServoMaxAngle;

    try {
      lo = bone.get("min");
    } catch (...) {
      lo = kServoMinAngle;
    }
    try {
      hi = bone.get("max");
    } catch (...) {
      hi = kServoMaxAngle;
    }

    if (lo > hi) {
      const int tmp = lo;
      lo = hi;
      hi = tmp;
    }

    const int angle = constrain(static_cast<int>(angles[i]), lo, hi);
    bone.move(angle);
  }

  // The bones were driven directly, bypassing the leg state machines. Mark the
  // legs as invalidated so a later idle()/move() animates from the new pose
  // instead of short-circuiting (e.g. on the servo-frame timeout).
  for (Leg* l : Legs) {
    l->invalidate();
  }
  GripHead.invalidate();
  BigTail.invalidate();
}

