#include "../teensy_tekceleo/symmetric_module_kinematics.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>

namespace {

void assertNear(float actual, float expected) {
  assert(fabsf(actual - expected) < 1.0e-6f);
}

void test_logical_to_motor_couples_rotation_into_bend() {
  const float logical[6] = {10.0f, 20.0f, -5.0f,
                            -7.0f, -30.0f, 8.0f};
  float motor[6] = {};
  SymmetricModuleKinematics::logicalToMotor(logical, motor);

  assertNear(motor[0], 10.0f);
  assertNear(motor[1], 20.0f);
  assertNear(motor[2], 15.0f);
  assertNear(motor[3], -7.0f);
  assertNear(motor[4], -30.0f);
  assertNear(motor[5], -22.0f);
}

void test_round_trip_restores_both_logical_modules() {
  const float logical[6] = {2.0f, -120.0f, 35.0f,
                            4.0f, 75.0f, -40.0f};
  float motor[6] = {};
  float recovered[6] = {};
  SymmetricModuleKinematics::logicalToMotor(logical, motor);
  SymmetricModuleKinematics::motorToLogical(motor, recovered);
  for (uint8_t axis = 0; axis < 6; ++axis) {
    assertNear(recovered[axis], logical[axis]);
  }
}

void test_fault_coupling_matches_both_differential_handles() {
  using SymmetricModuleKinematics::coupledAxis;
  assert(coupledAxis(0) == 255);
  assert(coupledAxis(1) == 2);
  assert(coupledAxis(2) == 1);
  assert(coupledAxis(3) == 255);
  assert(coupledAxis(4) == 5);
  assert(coupledAxis(5) == 4);
}

}  // namespace

int main() {
  test_logical_to_motor_couples_rotation_into_bend();
  test_round_trip_restores_both_logical_modules();
  test_fault_coupling_matches_both_differential_handles();
  return 0;
}
