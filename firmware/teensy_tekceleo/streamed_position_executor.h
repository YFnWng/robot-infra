#pragma once

#include "streamed_position_protocol.h"

#include <cstdint>
#include <cmath>

namespace StreamedPosition {

// Hardware-independent endpoint authority. Native mode emits one relative-angle
// start after stop/standstill; the sketch applies guarded/ACK-qualified commands
// and feeds applied shaft RPM back. No driver calls here.
class Executor {
 public:
  struct Config {
    int32_t lower_counts[kAxes] = {};
    int32_t upper_counts[kAxes] = {};
    uint32_t maximum_step_counts[kAxes] = {};
    uint32_t tolerance_counts[kAxes] = {};
    uint16_t minimum_rpm[kAxes] = {};
    uint16_t maximum_rpm[kAxes] = {};
    uint32_t progress_counts = 1;
    uint32_t stationary_counts = 1;
    uint32_t direction_settle_us = 5000;
    uint32_t no_progress_us = 500000;
    bool native_position = false;
  };

  enum class Admission : uint8_t {
    ACCEPTED, HEARTBEAT, STALE_ID, REJECTED_REPLAY, ID_CONFLICT, INVALID_COMMAND,
    INVALID_FEEDBACK, LIMIT_REJECTED, INHIBITED, PROGRESS_BLOCKED,
  };
  enum class Status : uint8_t {
    IDLE, RUNNING, COMPLETE, EXPIRED, CANCELLED, INVALID_COMMAND,
    INVALID_FEEDBACK, INHIBITED, NO_PROGRESS,
  };
  struct Update {
    Status status = Status::IDLE;
    uint32_t command_id = 0;
    int16_t requested_rpm[kAxes] = {};
    int64_t residual_counts[kAxes] = {};
    uint8_t stop_mask = 0;
    uint8_t reversal_wait_mask = 0;
    uint8_t overshoot_mask = 0;
    uint8_t native_start_mask = 0;
    uint32_t native_deci_degrees[kAxes] = {};
  };

  explicit Executor(const Config& config) : config_(config) {
    config_valid_ = config.progress_counts > 0 && config.stationary_counts > 0 &&
        config.direction_settle_us > 0 && config.direction_settle_us < 0x80000000U &&
        config.no_progress_us > 0 && config.no_progress_us < 0x80000000U;
    for (uint8_t axis = 0; axis < kAxes; ++axis) {
      config_valid_ = config_valid_ &&
          config.lower_counts[axis] <= config.upper_counts[axis] &&
          config.maximum_step_counts[axis] > 0 &&
          config.tolerance_counts[axis] > 0 &&
          config.minimum_rpm[axis] > 0 &&
          config.minimum_rpm[axis] <= config.maximum_rpm[axis] &&
          config.maximum_rpm[axis] <= kMaxRpm;
    }
  }

  Admission submit(const Command& candidate, uint32_t now_us,
                   const int32_t* counts, bool feedback_valid,
                   uint8_t allowed_mask) {
    if (!config_valid_ || !valid(candidate)) {
      stop(Status::INVALID_COMMAND);
      return Admission::INVALID_COMMAND;
    }
    if (has_seen_command_ && candidate.id == seen_command_.id) {
      if (!same(candidate, seen_command_)) {
        stop(Status::INVALID_COMMAND);
        return Admission::ID_CONFLICT;
      }
      if (!has_command_ || candidate.id != command_.id)
        return Admission::REJECTED_REPLAY;
      if (!feedback_valid || counts == nullptr) {
        stop(Status::INVALID_FEEDBACK);
        return Admission::INVALID_FEEDBACK;
      }
      if ((candidate.active_mask & allowed_mask) != candidate.active_mask) {
        stop(Status::INHIBITED);
        return Admission::INHIBITED;
      }
      // A heartbeat may never modify the endpoint, restart completed motion,
      // renew the lease, or erase the progress budget.
      return Admission::HEARTBEAT;
    }
    if (has_seen_command_ && !newerId(candidate.id, seen_command_.id))
      return Admission::STALE_ID;
    // Consume structurally valid IDs even on rejection. Repeating a rejected
    // request after feedback/safety recovery must not initiate old motion.
    seen_command_ = candidate;
    has_seen_command_ = true;
    if (!feedback_valid || counts == nullptr) {
      stop(Status::INVALID_FEEDBACK);
      return Admission::INVALID_FEEDBACK;
    }
    if ((candidate.active_mask & allowed_mask) != candidate.active_mask) {
      stop(Status::INHIBITED);
      return Admission::INHIBITED;
    }
    if (progress_blocked_ && candidate.active_mask != 0)
      return Admission::PROGRESS_BLOCKED;
    // Validate every active endpoint before changing any command state.
    for (uint8_t axis = 0; axis < kAxes; ++axis) {
      if ((candidate.active_mask & (1U << axis)) == 0) continue;
      const int64_t error = static_cast<int64_t>(candidate.target_counts[axis]) - counts[axis];
      if (counts[axis] < config_.lower_counts[axis] ||
          counts[axis] > config_.upper_counts[axis] ||
          candidate.target_counts[axis] < config_.lower_counts[axis] ||
          candidate.target_counts[axis] > config_.upper_counts[axis] ||
          magnitude(error) > config_.maximum_step_counts[axis] ||
          candidate.speed_rpm[axis] < config_.minimum_rpm[axis] ||
          candidate.speed_rpm[axis] > config_.maximum_rpm[axis]) {
        stop(Status::INVALID_COMMAND);
        return Admission::LIMIT_REJECTED;
      }
    }
    command_ = candidate;
    native_started_mask_ = 0;
    diagnostic_live_ = false;
    for (uint8_t a = 0; a < kAxes; ++a) native_settling_[a] = false;
    has_command_ = true;
    accepted_us_ = now_us;
    active_mask_ = candidate.active_mask;
    status_ = active_mask_ == 0 ? Status::CANCELLED : Status::RUNNING;
    for (uint8_t axis = 0; axis < kAxes; ++axis) {
      const int64_t error = static_cast<int64_t>(candidate.target_counts[axis]) - counts[axis];
      direction_[axis] = sign(error);
      // Do not rebase no-progress evidence on replacements. Only measured
      // encoder travel or an explicitly verified session reset can clear it.
      if (!progress_initialized_[axis]) {
        progress_anchor_[axis] = counts[axis];
        progress_initialized_[axis] = true;
      }
    }
    return Admission::ACCEPTED;
  }

  Update update(uint32_t now_us, const int32_t* counts,
                const int16_t* applied_rpm, bool feedback_valid,
                uint8_t allowed_mask) {
    Update result;
    result.command_id = has_command_ ? command_.id : 0;
    const uint32_t elapsed = clock_initialized_ ? now_us - previous_us_ : 0;
    previous_us_ = now_us;
    clock_initialized_ = true;
    bool applied_valid = applied_rpm != nullptr;
    if (applied_valid) {
      for (uint8_t axis = 0; axis < kAxes; ++axis)
        applied_valid = applied_valid && applied_rpm[axis] >= -kMaxRpm &&
                        applied_rpm[axis] <= kMaxRpm;
    }
    if (!feedback_valid || counts == nullptr || !applied_valid) {
      stop(Status::INVALID_FEEDBACK);
      for (uint8_t axis = 0; axis < kAxes; ++axis) {
        // A feedback gap is not evidence of stationary motor motion.
        zero_tracking_[axis] = false;
        previous_active_[axis] = false;
        previous_applied_[axis] = 0;
      }
      result.stop_mask = 0x3f;
      result.status = status_;
      return result;
    }
    for (uint8_t axis = 0; axis < kAxes; ++axis) {
      const uint8_t bit = static_cast<uint8_t>(1U << axis);
      result.residual_counts[axis] = has_command_
          ? static_cast<int64_t>(command_.target_counts[axis]) - counts[axis] : 0;
      if (applied_rpm[axis] != 0) result.stop_mask |= bit;
      if (!progress_initialized_[axis]) {
        progress_anchor_[axis] = counts[axis];
        progress_initialized_[axis] = true;
      }
      const int64_t travel = static_cast<int64_t>(counts[axis]) - progress_anchor_[axis];
      if (magnitude(travel) >= config_.progress_counts) {
        progress_anchor_[axis] = counts[axis];
        no_progress_elapsed_[axis] = 0;
      } else if (previous_applied_[axis] != 0 && previous_active_[axis]) {
        const uint32_t room = config_.no_progress_us - no_progress_elapsed_[axis];
        no_progress_elapsed_[axis] += elapsed < room ? elapsed : room;
      }
      if (applied_rpm[axis] != 0) {
        last_direction_[axis] = sign(applied_rpm[axis]);
        zero_tracking_[axis] = false;
      } else if (!zero_tracking_[axis] ||
                 magnitude(static_cast<int64_t>(counts[axis])-stationary_anchor_[axis])
                     >= config_.stationary_counts) {
        zero_tracking_[axis] = true;
        zero_since_us_[axis] = now_us;
        stationary_anchor_[axis] = counts[axis];
      }
    }
    if (status_ == Status::RUNNING) {
      if ((active_mask_ & allowed_mask) != active_mask_) stop(Status::INHIBITED);
      else if (now_us - accepted_us_ >= command_.duration_us) stop(Status::EXPIRED);
    }
    if (status_ == Status::RUNNING) {
      for (uint8_t axis = 0; axis < kAxes; ++axis) {
        const uint8_t bit = static_cast<uint8_t>(1U << axis);
        if ((active_mask_ & bit) == 0) continue;
        if (counts[axis] < config_.lower_counts[axis] ||
            counts[axis] > config_.upper_counts[axis]) {
          stop(Status::INHIBITED);
          break;
        }
        const int64_t remaining = result.residual_counts[axis] * direction_[axis];
        bool finish = remaining <= config_.tolerance_counts[axis];
        if (config_.native_position && (native_started_mask_ & bit) != 0) {
          // The driver owns normal endpoint execution. Near-target samples
          // alone never stop its native position move. Confirm actual encoder
          // standstill, or stop immediately on a large crossed-target error.
          const bool near = magnitude(result.residual_counts[axis]) <= config_.tolerance_counts[axis];
          if (!near || !native_settling_[axis] ||
              magnitude(static_cast<int64_t>(counts[axis])-native_settle_anchor_[axis]) >= config_.stationary_counts) {
            native_settling_[axis] = near;
            native_settle_since_[axis] = now_us;
            native_settle_anchor_[axis] = counts[axis];
          }
          finish = (near && now_us-native_settle_since_[axis] >= config_.direction_settle_us)
              || remaining < -static_cast<int64_t>(config_.tolerance_counts[axis]);
        }
        if (diagnostic_live_) finish = false; // diagnostic retains lease, bounds and no-progress guard
        if (finish) {
          // Crossing the endpoint is terminal for this axis. Never synthesize
          // a reverse correction, even if subsequent feedback drifts away.
          if (remaining < -static_cast<int64_t>(config_.tolerance_counts[axis]))
            result.overshoot_mask |= bit;
          active_mask_ &= static_cast<uint8_t>(~bit);
        } else if (no_progress_elapsed_[axis] >= config_.no_progress_us) {
          progress_blocked_ = true;
          stop(Status::NO_PROGRESS);
          break;
        }
      }
      if (status_ == Status::RUNNING && active_mask_ == 0) status_ = Status::COMPLETE;
    }
    bool native_stop_ready = true;
    if (config_.native_position && native_started_mask_ == 0) {
      for (uint8_t axis = 0; axis < kAxes; ++axis) {
        if (applied_rpm[axis] != 0 || (last_direction_[axis] != 0 &&
            (!zero_tracking_[axis] || now_us-zero_since_us_[axis] < config_.direction_settle_us)))
          native_stop_ready = false;
      }
    }
    for (uint8_t axis = 0; axis < kAxes; ++axis) {
      const uint8_t bit = static_cast<uint8_t>(1U << axis);
      if (status_ == Status::RUNNING && (active_mask_ & bit) != 0) {
        const bool reversing = last_direction_[axis] != 0 &&
                               direction_[axis] != last_direction_[axis];
        const bool stopped_and_settled = applied_rpm[axis] == 0 &&
            zero_tracking_[axis] && now_us-zero_since_us_[axis] >= config_.direction_settle_us;
        if (config_.native_position && (native_started_mask_ & bit) == 0) {
          // All native replacements stop, including same-direction updates.
          // A new ID cannot reset this measured standstill history.
          if (!native_stop_ready) {
            result.reversal_wait_mask |= bit;
          } else {
            // Read-only 0.045 degree/count calibration: exact integer rounding
            // avoids binary floating-point ties for decimal angular targets.
            const uint32_t deci = static_cast<uint32_t>((magnitude(result.residual_counts[axis])*9+10)/20);
            if (deci == 0 || deci > 99999) {
              stop(Status::INVALID_COMMAND);
              result.stop_mask |= bit;
            } else {
              result.native_start_mask |= bit;
              result.native_deci_degrees[axis] = deci;
              result.requested_rpm[axis] = static_cast<int16_t>(direction_[axis]*command_.speed_rpm[axis]);
            }
          }
        } else if (!config_.native_position && reversing && !stopped_and_settled) result.reversal_wait_mask |= bit;
        else {
          result.requested_rpm[axis] = static_cast<int16_t>(
              direction_[axis]*command_.speed_rpm[axis]);
          result.stop_mask &= static_cast<uint8_t>(~bit);
        }
      }
      previous_applied_[axis] = applied_rpm[axis];
      previous_active_[axis] = status_ == Status::RUNNING && (active_mask_ & bit) != 0;
    }
    result.status = status_;
    return result;
  }

  bool diagnosticLiveUpdate(uint32_t command_id) {
    if (!config_.native_position || diagnostic_live_ || status_ != Status::RUNNING ||
        command_.id != command_id || active_mask_ != 1 || native_started_mask_ != 1)
      return false;
    diagnostic_live_ = true;
    return true;
  }
  void nativeStarted(uint8_t mask) {
    if (config_.native_position && status_ == Status::RUNNING)
      native_started_mask_ |= mask & active_mask_;
  }

  // Explicit cancellation retains ID replay protection and progress history.
  void cancel() { stop(Status::CANCELLED); }
  bool expired(uint32_t now_us) const {
    return status_ == Status::RUNNING && now_us-accepted_us_ >= command_.duration_us;
  }

  // M2 must call this only while disarmed, externally permitted, and after
  // verified encoder standstill. Never clear manager/firmware fault latches.
  bool resetAfterVerifiedStop(bool disarmed_and_stationary,
                             const int16_t* applied_rpm) {
    if (!disarmed_and_stationary || applied_rpm == nullptr) return false;
    for (uint8_t axis = 0; axis < kAxes; ++axis)
      if (applied_rpm[axis] != 0) return false;
    cancel();
    progress_blocked_ = false;
    for (uint8_t axis = 0; axis < kAxes; ++axis) {
      no_progress_elapsed_[axis] = 0;
      progress_initialized_[axis] = false;
      previous_active_[axis] = false;
      previous_applied_[axis] = 0;
    }
    return true;
  }

 private:
  static int8_t sign(int64_t value) { return value > 0 ? 1 : value < 0 ? -1 : 0; }
  static uint64_t magnitude(int64_t value) {
    return static_cast<uint64_t>(value < 0 ? -value : value);
  }
  void stop(Status status) { active_mask_ = 0; native_started_mask_ = 0; diagnostic_live_ = false; status_ = status; }

  Config config_;
  bool config_valid_ = false;
  Command command_;
  bool has_command_ = false;
  Command seen_command_;
  bool has_seen_command_ = false;
  Status status_ = Status::IDLE;
  uint8_t active_mask_ = 0;
  uint32_t accepted_us_ = 0;
  bool clock_initialized_ = false;
  uint32_t previous_us_ = 0;
  int8_t direction_[kAxes] = {};
  int8_t last_direction_[kAxes] = {};
  bool zero_tracking_[kAxes] = {};
  uint32_t zero_since_us_[kAxes] = {};
  int32_t stationary_anchor_[kAxes] = {};
  bool progress_initialized_[kAxes] = {};
  int32_t progress_anchor_[kAxes] = {};
  uint32_t no_progress_elapsed_[kAxes] = {};
  int16_t previous_applied_[kAxes] = {};
  bool previous_active_[kAxes] = {};
  bool progress_blocked_ = false;
  uint8_t native_started_mask_ = 0;
  bool diagnostic_live_ = false;
  bool native_settling_[kAxes] = {};
  int32_t native_settle_anchor_[kAxes] = {};
  uint32_t native_settle_since_[kAxes] = {};
};
}  // namespace StreamedPosition
