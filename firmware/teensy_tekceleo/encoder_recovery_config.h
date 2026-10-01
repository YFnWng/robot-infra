#pragma once

#include <stdint.h>

// TEMPORARY, INCIDENT-SPECIFIC RECOVERY BUILD.
//
// These are the last raw encoder counts recorded before the stationary robot
// was reflashed away from home on 2026-09-19.  They are compile-time data:
// there is deliberately no serial/ROS interface for supplying or changing an
// encoder frame.  This build may command only the established all-zero
// physical home target.  After reaching home, disable this seed and flash the
// production image while the mechanism remains at home.
#define TKCTL_ENCODER_RECOVERY_SEED_ENABLED 0

namespace EncoderRecoveryConfig {

constexpr uint8_t kAxisCount = 6;
constexpr int32_t kHistoricalCounts[kAxisCount] = {
    82914, 6, 9, 0, 0, 0,
};
static_assert(
    kHistoricalCounts[0] != 0 || kHistoricalCounts[1] != 0
        || kHistoricalCounts[2] != 0 || kHistoricalCounts[3] != 0
        || kHistoricalCounts[4] != 0 || kHistoricalCounts[5] != 0,
    "incident recovery must not become an encoder-zero operation");

}  // namespace EncoderRecoveryConfig
