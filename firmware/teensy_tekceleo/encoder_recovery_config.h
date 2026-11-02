#pragma once

#include <stdint.h>
#include <cmath>

// TEMPORARY, INCIDENT-SPECIFIC RECOVERY BUILD.
//
// 2026-10-14: operator's chassis mark identifies a +20 mm insertion offset
// after an off-home reflash. Latest reported raw frame was [1,0,-6,0,0,0].
// This is mark-derived restoration, NOT an exact historical encoder frame.
// Other axes retain that reported frame; their physical home must be checked
// before returning to production firmware. These are compile-time data:
// there is deliberately no serial/ROS interface for supplying or changing an
// encoder frame.  This build may command only the established all-zero
// physical home target.  After reaching home, disable this seed and flash the
// production image while the mechanism remains at home.
#define TKCTL_ENCODER_RECOVERY_SEED_ENABLED 0
#define TKCTL_ENCODER_RECOVERY_IDENTITY "tkctl:recovery-chassis-mark-20261014|"

namespace EncoderRecoveryConfig {

constexpr uint8_t kAxisCount = 6;
// Axis 0: 67.319841/20 mm per revolution, 8000 encoder counts/revolution.
constexpr int32_t kInsertionOffsetCounts = 47534;
constexpr float kMaximumInsertionSpeedMmS = 2.0f;
constexpr int32_t kHistoricalCounts[kAxisCount] = {
    1 + kInsertionOffsetCounts, 0, -6, 0, 0, 0,
};
inline bool validPositionSpeeds(const float* speeds) {
  if (!speeds || !std::isfinite(speeds[0]) || speeds[0] <= 0.0f ||
      speeds[0] > kMaximumInsertionSpeedMmS) return false;
  for (uint8_t axis = 1; axis < kAxisCount; ++axis)
    if (speeds[axis] != 0.0f) return false;
  return true;
}
static_assert(
    kHistoricalCounts[0] != 0 || kHistoricalCounts[1] != 0
        || kHistoricalCounts[2] != 0 || kHistoricalCounts[3] != 0
        || kHistoricalCounts[4] != 0 || kHistoricalCounts[5] != 0,
    "incident recovery must not become an encoder-zero operation");

}  // namespace EncoderRecoveryConfig
