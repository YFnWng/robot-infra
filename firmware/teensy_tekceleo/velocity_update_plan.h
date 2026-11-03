#pragma once
#include <stdint.h>

// Arduino-independent classification of one velocity update into per-axis
// driver actions. The legacy policy restarts every active axis whenever any
// active axis must start from rest or reverse. The on-the-fly policy keeps
// running axes running: same-direction axes only refresh their speed, and
// reversing axes change direction in place. Only axes that are not running
// receive the full disabled-to-enabled start sequence.
namespace VelocityUpdatePlan {

struct AxisState {
  bool active;            // nonzero commanded RPM and not blocked or latched
  bool enabled;           // driver output enabled
  bool mode_valid;        // closed-loop control mode configured
  bool direction_valid;   // cached driver direction is known
  uint8_t target_direction;
  uint8_t current_direction;
};

struct Plan {
  uint8_t speed_mask;    // running, same direction: speed refresh only
  uint8_t reverse_mask;  // running, direction changes: direction then speed
  uint8_t start_mask;    // full stop, settle, configure and coordinated enable
};

inline Plan plan(const AxisState* axes, uint8_t axis_count, bool on_the_fly) {
  Plan result = {0, 0, 0};
  uint8_t active_mask = 0;
  bool restart = false;
  for (uint8_t axis = 0; axis < axis_count; ++axis) {
    const AxisState& a = axes[axis];
    if (!a.active) continue;
    const uint8_t bit = static_cast<uint8_t>(1U << axis);
    active_mask |= bit;
    const bool running = a.enabled && a.mode_valid && a.direction_valid;
    if (!running) {
      result.start_mask |= bit;
      restart = true;
    } else if (a.target_direction != a.current_direction) {
      result.reverse_mask |= bit;
      restart = true;
    } else {
      result.speed_mask |= bit;
    }
  }
  if (!on_the_fly && restart) {
    result.speed_mask = 0;
    result.reverse_mask = 0;
    result.start_mask = active_mask;
  }
  return result;
}

}  // namespace VelocityUpdatePlan
