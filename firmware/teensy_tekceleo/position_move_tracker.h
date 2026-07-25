#pragma once

#include <cmath>
#include <cstdint>
#include <cstring>

// Encoder-feedback completion tracker for one-shot driver position moves.
//
// The motor driver accepts a relative angular target but does not provide a
// completion notification. This tracker therefore owns the firmware-side
// completion semantics: an axis is complete only after its encoder remains
// within tolerance for settle_ms. A move also has a hard timeout.
class PositionMoveTracker {
 public:
  static constexpr uint8_t kAxes = 6;

  enum Status : uint8_t {
    NO_EVENT = 0,
    COMPLETE = 1,
    TIMED_OUT = 2,
  };

  struct Update {
    Status status = NO_EVENT;
    uint8_t stop_mask = 0;
    uint8_t retry_mask = 0;
    uint8_t command_mask = 0;
  };

  void begin(uint32_t now_ms, uint8_t active_mask, const float* target,
             const float* tolerance, const float* position) {
    active_mask_ = active_mask;
    command_mask_ = active_mask;
    start_ms_ = now_ms;
    memcpy(target_, target, sizeof(target_));
    memcpy(tolerance_, tolerance, sizeof(tolerance_));
    memset(within_, 0, sizeof(within_));
    memset(within_since_ms_, 0, sizeof(within_since_ms_));
    memset(retry_count_, 0, sizeof(retry_count_));
    for (uint8_t axis = 0; axis < kAxes; ++axis) {
      const float value = std::isfinite(position[axis]) ? position[axis] : 0.0f;
      segment_start_error_[axis] = std::fabs(target_[axis] - value);
      motion_anchor_[axis] = value;
      last_motion_ms_[axis] = now_ms;
    }
    running_ = true;
  }

  void cancel() {
    active_mask_ = 0;
    command_mask_ = 0;
    running_ = false;
    memset(within_, 0, sizeof(within_));
  }

  Update update(uint32_t now_ms, const float* position, uint32_t settle_ms,
                uint32_t timeout_ms, uint32_t retry_idle_ms,
                uint8_t max_retries, float minimum_progress_fraction) {
    Update result;
    result.command_mask = command_mask_;
    if (!running_) return result;

    if (now_ms - start_ms_ >= timeout_ms) {
      result.status = TIMED_OUT;
      result.stop_mask = active_mask_;
      active_mask_ = 0;
      running_ = false;
      return result;
    }

    for (uint8_t axis = 0; axis < kAxes; ++axis) {
      const uint8_t bit = static_cast<uint8_t>(1U << axis);
      if ((active_mask_ & bit) == 0) continue;
      if (!std::isfinite(position[axis])) continue;
      const float error = std::fabs(target_[axis] - position[axis]);
      const bool in_tolerance =
          error <= tolerance_[axis];
      if (!in_tolerance) {
        within_[axis] = false;
        if (std::fabs(position[axis] - motion_anchor_[axis]) >=
            tolerance_[axis]) {
          motion_anchor_[axis] = position[axis];
          last_motion_ms_[axis] = now_ms;
        }
        if (now_ms - last_motion_ms_[axis] < retry_idle_ms) continue;

        const float required_progress = std::fmax(
            tolerance_[axis],
            segment_start_error_[axis] * minimum_progress_fraction);
        const float progress = segment_start_error_[axis] - error;
        if (retry_count_[axis] < max_retries &&
            progress >= required_progress) {
          ++retry_count_[axis];
          segment_start_error_[axis] = error;
          motion_anchor_[axis] = position[axis];
          last_motion_ms_[axis] = now_ms;
          result.retry_mask |= bit;
          continue;
        }

        // An endpoint correction that cannot make meaningful encoder progress
        // is treated as an incomplete position move, not as a velocity stall.
        result.status = TIMED_OUT;
        result.stop_mask = active_mask_;
        active_mask_ = 0;
        running_ = false;
        return result;
      }
      if (!within_[axis]) {
        within_[axis] = true;
        within_since_ms_[axis] = now_ms;
        continue;
      }
      if (now_ms - within_since_ms_[axis] >= settle_ms) {
        active_mask_ &= static_cast<uint8_t>(~bit);
        result.stop_mask |= bit;
      }
    }

    if (active_mask_ == 0) {
      result.status = COMPLETE;
      running_ = false;
    }
    return result;
  }

  bool running() const { return running_; }
  uint8_t activeMask() const { return active_mask_; }
  uint8_t commandMask() const { return command_mask_; }

 private:
  bool running_ = false;
  uint8_t active_mask_ = 0;
  uint8_t command_mask_ = 0;
  uint32_t start_ms_ = 0;
  float target_[kAxes] = {};
  float tolerance_[kAxes] = {};
  bool within_[kAxes] = {};
  uint32_t within_since_ms_[kAxes] = {};
  float segment_start_error_[kAxes] = {};
  float motion_anchor_[kAxes] = {};
  uint32_t last_motion_ms_[kAxes] = {};
  uint8_t retry_count_[kAxes] = {};
};
