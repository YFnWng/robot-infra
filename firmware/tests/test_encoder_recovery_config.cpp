#include "../teensy_tekceleo/encoder_recovery_config.h"
#include <cassert>
#include <limits>

int main() {
  using namespace EncoderRecoveryConfig;
  assert(kHistoricalCounts[0] == 47535);
  assert(kHistoricalCounts[2] == -6);
  assert(std::fabs(kInsertionOffsetCounts*(67.319841/160000.0)-20.0) < .0003);
  float speed[6] = {2, 0, 0, 0, 0, 0};
  assert(validPositionSpeeds(speed));
  speed[0] = 2.01f;
  assert(!validPositionSpeeds(speed));
  speed[0] = -2;
  assert(!validPositionSpeeds(speed));
  speed[0] = 0;
  assert(!validPositionSpeeds(speed));
  speed[0] = std::numeric_limits<float>::quiet_NaN();
  assert(!validPositionSpeeds(speed));
  speed[0] = 2;
  for (int axis = 1; axis < 6; ++axis) {
    speed[axis] = 1;
    assert(!validPositionSpeeds(speed));
    speed[axis] = std::numeric_limits<float>::quiet_NaN();
    assert(!validPositionSpeeds(speed));
    speed[axis] = 0;
  }
  assert(!validPositionSpeeds(nullptr));
}
