#pragma once

#include <stdint.h>

namespace SymmetricModuleKinematics {

constexpr uint8_t AXIS_COUNT = 6;
constexpr uint8_t NO_COUPLED_AXIS = 255;

// Logical joint order is
// [catheter linear, rotation, bend, sheath linear, rotation, bend].
// Both modules use the same differential rotary handle: the physical bend
// motor must follow axial rotation plus the requested logical bend.
inline void logicalToMotor(const float logical[AXIS_COUNT],
                           float motor[AXIS_COUNT]) {
  for (uint8_t axis = 0; axis < AXIS_COUNT; ++axis) {
    motor[axis] = logical[axis];
  }
  motor[2] = logical[2] + logical[1];
  motor[5] = logical[5] + logical[4];
}

inline void motorToLogical(const float motor[AXIS_COUNT],
                           float logical[AXIS_COUNT]) {
  for (uint8_t axis = 0; axis < AXIS_COUNT; ++axis) {
    logical[axis] = motor[axis];
  }
  logical[2] = motor[2] - motor[1];
  logical[5] = motor[5] - motor[4];
}

inline uint8_t coupledAxis(uint8_t axis) {
  if (axis == 1) return 2;
  if (axis == 2) return 1;
  if (axis == 4) return 5;
  if (axis == 5) return 4;
  return NO_COUPLED_AXIS;
}

}  // namespace SymmetricModuleKinematics
