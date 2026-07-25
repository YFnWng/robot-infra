#pragma once

#include <stddef.h>
#include <stdint.h>

// Arduino-independent, atomic encoder-frame validation. A frame is accepted
// only when every axis remains inside the physically plausible count delta.
namespace EncoderIntegrityGuard {

struct Result {
  bool valid = true;
  uint8_t invalid_mask = 0;
  uint8_t first_invalid_axis = 255;
  int32_t first_invalid_delta = 0;
  uint32_t allowed_delta = 0;
};

inline uint32_t allowedDelta(
    uint32_t elapsed_ms,
    uint32_t maximum_counts_per_second,
    uint8_t speed_margin_multiplier,
    uint16_t count_margin) {
  if (elapsed_ms == 0) {
    elapsed_ms = 1;
  }
  const uint64_t dynamic =
      static_cast<uint64_t>(elapsed_ms) * maximum_counts_per_second *
      speed_margin_multiplier;
  const uint64_t total = (dynamic + 999U) / 1000U + count_margin;
  return total > UINT32_MAX ? UINT32_MAX : static_cast<uint32_t>(total);
}

template <size_t AxisCount>
Result validate(
    const int32_t (&candidate)[AxisCount],
    const int32_t (&last_valid)[AxisCount],
    uint32_t elapsed_ms,
    uint32_t maximum_counts_per_second,
    uint8_t speed_margin_multiplier,
    uint16_t count_margin) {
  Result result;
  result.allowed_delta = allowedDelta(
      elapsed_ms, maximum_counts_per_second, speed_margin_multiplier,
      count_margin);

  for (size_t axis = 0; axis < AxisCount; ++axis) {
    const int64_t signed_delta =
        static_cast<int64_t>(candidate[axis]) - last_valid[axis];
    const uint64_t magnitude =
        signed_delta < 0 ? static_cast<uint64_t>(-signed_delta)
                         : static_cast<uint64_t>(signed_delta);
    if (magnitude <= result.allowed_delta) {
      continue;
    }
    result.valid = false;
    if (axis < 8) {
      result.invalid_mask |= static_cast<uint8_t>(1U << axis);
    }
    if (result.first_invalid_axis == 255) {
      result.first_invalid_axis = static_cast<uint8_t>(axis);
      result.first_invalid_delta =
          signed_delta > INT32_MAX
              ? INT32_MAX
              : (signed_delta < INT32_MIN
                     ? INT32_MIN
                     : static_cast<int32_t>(signed_delta));
    }
  }
  return result;
}

}  // namespace EncoderIntegrityGuard
