#pragma once

#include <stdint.h>

// Arduino-independent phase ordering for a multi-axis start. Every active axis
// must configure successfully before the single enable phase is entered.
namespace CoordinatedStart {

template <typename ConfigureAxis, typename EnableAll, typename RollbackAll>
bool run(uint8_t active_mask, uint8_t axis_count,
         ConfigureAxis configure_axis, EnableAll enable_all,
         RollbackAll rollback_all) {
  for (uint8_t axis = 0; axis < axis_count; ++axis) {
    if ((active_mask & (1U << axis)) == 0) {
      continue;
    }
    if (!configure_axis(axis)) {
      rollback_all(active_mask);
      return false;
    }
  }
  if (!enable_all(active_mask)) {
    rollback_all(active_mask);
    return false;
  }
  return true;
}

}  // namespace CoordinatedStart
