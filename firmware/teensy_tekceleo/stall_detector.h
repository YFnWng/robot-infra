#pragma once

#include <math.h>
#include <stdint.h>

// Encoder-progress based motion monitor. It is deliberately independent of
// Arduino APIs so the state machine can be exercised by a desktop C++ test.
class StallDetector {
 public:
  enum State : uint8_t {
    IDLE = 0,
    STARTING = 1,
    MONITORING = 2,
    SUSPECT = 3,
    FAULT_LATCHED = 4,
  };

  enum Fault : uint8_t {
    FAULT_NONE = 0,
    STALL = 1,
    OVERSPEED = 2,
    WRONG_DIRECTION = 3,
    FEEDBACK_FAULT = 4,
    SPEED_UNOBSERVABLE = 5,
    DRIVER_COMMUNICATION = 6,
  };

  enum Transition : uint8_t {
    NO_EVENT = 0,
    SUSPECTED = 1,
    CONFIRMED = 2,
    RECOVERED = 3,
    RESET = 4,
    UNOBSERVABLE = 5,
    STATUS = 6,
  };

  struct Config {
    uint16_t startup_grace_ms = 500;
    uint16_t normal_window_ms = 200;
    uint16_t slow_window_ms = 500;
    uint16_t normal_confirm_ms = 300;
    uint16_t slow_confirm_ms = 500;
    uint16_t slow_rpm_boundary = 10;
    uint16_t minimum_window_counts = 4;
    float stall_fraction = 0.25f;
    float overspeed_fraction = 2.5f;
    float feedback_jump_fraction = 50.0f;
    uint16_t feedback_jump_margin_counts = 100;
  };

  struct Event {
    Transition transition = NO_EVENT;
    Fault fault = FAULT_NONE;
    float commanded_velocity = 0.0f;
    float measured_velocity = 0.0f;
    float window_displacement = 0.0f;
    uint16_t target_rpm = 0;
    uint16_t window_ms = 0;
    uint8_t detail = 0;
  };

  StallDetector() : config_() {}
  explicit StallDetector(const Config& config) : config_(config) {}

  void begin(uint32_t now_ms, int32_t encoder_count) {
    clearHistory();
    addSample(now_ms, encoder_count);
    state_ = IDLE;
    fault_ = FAULT_NONE;
    target_rpm_ = 0;
    target_direction_ = 0;
    commanded_velocity_ = 0.0f;
    state_since_ms_ = now_ms;
    unobservable_reported_ = false;
  }

  void setCommand(uint32_t now_ms, uint16_t rpm, uint8_t direction,
                  float commanded_velocity) {
    const uint16_t previous_rpm = target_rpm_;
    const uint8_t previous_direction = target_direction_;
    target_rpm_ = rpm;
    target_direction_ = direction;
    commanded_velocity_ = commanded_velocity;

    if (state_ == FAULT_LATCHED) {
      return;
    }
    if (rpm == 0 || commanded_velocity == 0.0f) {
      state_ = IDLE;
      fault_ = FAULT_NONE;
      state_since_ms_ = now_ms;
      unobservable_reported_ = false;
      return;
    }

    const bool started = previous_rpm == 0;
    const bool reversed =
        previous_rpm != 0 && previous_direction != direction;
    const uint16_t rpm_delta =
        rpm > previous_rpm ? rpm - previous_rpm : previous_rpm - rpm;
    const bool large_step =
        previous_rpm != 0 && rpm_delta >= 5 &&
        static_cast<uint32_t>(rpm_delta) * 2U >= previous_rpm;
    if (state_ == IDLE || started || reversed || large_step) {
      state_ = STARTING;
      fault_ = FAULT_NONE;
      state_since_ms_ = now_ms;
      unobservable_reported_ = false;
    }
  }

  Event update(uint32_t now_ms, int32_t encoder_count,
               float joint_units_per_count) {
    addSample(now_ms, encoder_count);
    Event event;
    event.commanded_velocity = commanded_velocity_;
    event.target_rpm = target_rpm_;

    if (state_ == IDLE || state_ == FAULT_LATCHED ||
        target_rpm_ == 0 || commanded_velocity_ == 0.0f) {
      return event;
    }

    if (state_ == STARTING) {
      if (elapsed(now_ms, state_since_ms_) < config_.startup_grace_ms) {
        return event;
      }
      state_ = MONITORING;
      state_since_ms_ = now_ms;
    }

    const bool slow = target_rpm_ < config_.slow_rpm_boundary;
    const uint16_t requested_window =
        slow ? config_.slow_window_ms : config_.normal_window_ms;
    const Sample* old = sampleAtLeast(now_ms, requested_window);
    if (old == nullptr) {
      return event;
    }

    const uint32_t age_ms = elapsed(now_ms, old->time_ms);
    if (age_ms == 0 || age_ms > 65535U) {
      return event;
    }
    const int32_t delta_counts = encoder_count - old->encoder_count;
    const float window_s = static_cast<float>(age_ms) * 0.001f;
    const float displacement =
        static_cast<float>(delta_counts) * joint_units_per_count;
    const float measured_velocity = displacement / window_s;
    const float expected_counts =
        fabsf(commanded_velocity_) * window_s /
        fmaxf(fabsf(joint_units_per_count), 1.0e-12f);

    event.measured_velocity = measured_velocity;
    event.window_displacement = displacement;
    event.window_ms = static_cast<uint16_t>(age_ms);

    if (expected_counts < config_.minimum_window_counts) {
      if (!unobservable_reported_) {
        unobservable_reported_ = true;
        event.transition = UNOBSERVABLE;
        event.fault = SPEED_UNOBSERVABLE;
      }
      return event;
    }
    unobservable_reported_ = false;

    const float abs_command = fabsf(commanded_velocity_);
    const float abs_measured = fabsf(measured_velocity);
    const uint32_t abs_delta_counts =
        delta_counts < 0 ? static_cast<uint32_t>(-static_cast<int64_t>(delta_counts))
                         : static_cast<uint32_t>(delta_counts);
    const float count_velocity_margin =
        static_cast<float>(config_.minimum_window_counts) *
        fabsf(joint_units_per_count) / window_s;

    Fault observed = FAULT_NONE;
    if (abs_delta_counts >= config_.minimum_window_counts &&
        measured_velocity * commanded_velocity_ < 0.0f) {
      observed = WRONG_DIRECTION;
    } else if (abs_delta_counts >
               expected_counts * config_.feedback_jump_fraction +
                   config_.feedback_jump_margin_counts) {
      observed = FEEDBACK_FAULT;
    } else if (abs_measured >
                   abs_command * config_.overspeed_fraction &&
               abs_measured - abs_command > count_velocity_margin) {
      observed = OVERSPEED;
    } else if (abs_measured < abs_command * config_.stall_fraction) {
      observed = STALL;
    }

    if (observed == FAULT_NONE) {
      if (state_ == SUSPECT) {
        event.transition = RECOVERED;
        event.fault = fault_;
      }
      state_ = MONITORING;
      fault_ = FAULT_NONE;
      state_since_ms_ = now_ms;
      return event;
    }

    if (state_ != SUSPECT || fault_ != observed) {
      state_ = SUSPECT;
      fault_ = observed;
      state_since_ms_ = now_ms;
      event.transition = SUSPECTED;
      event.fault = observed;
      return event;
    }

    const uint16_t confirm_ms =
        slow ? config_.slow_confirm_ms : config_.normal_confirm_ms;
    if (elapsed(now_ms, state_since_ms_) >= confirm_ms) {
      state_ = FAULT_LATCHED;
      event.transition = CONFIRMED;
      event.fault = observed;
    }
    return event;
  }

  Event clearFault(uint32_t now_ms, int32_t encoder_count) {
    Event event;
    const bool was_latched = state_ == FAULT_LATCHED;
    begin(now_ms, encoder_count);
    if (was_latched) {
      event.transition = RESET;
    }
    return event;
  }

  void forceLatch(Fault fault, uint32_t now_ms) {
    state_ = FAULT_LATCHED;
    fault_ = fault;
    state_since_ms_ = now_ms;
  }

  State state() const { return state_; }
  Fault fault() const { return fault_; }
  bool latched() const { return state_ == FAULT_LATCHED; }

 private:
  static constexpr uint16_t HISTORY_SIZE = 112;

  struct Sample {
    uint32_t time_ms;
    int32_t encoder_count;
  };

  static uint32_t elapsed(uint32_t now, uint32_t before) {
    return now - before;
  }

  void clearHistory() {
    history_head_ = 0;
    history_count_ = 0;
  }

  void addSample(uint32_t now_ms, int32_t encoder_count) {
    history_[history_head_].time_ms = now_ms;
    history_[history_head_].encoder_count = encoder_count;
    history_head_ = (history_head_ + 1U) % HISTORY_SIZE;
    if (history_count_ < HISTORY_SIZE) {
      ++history_count_;
    }
  }

  const Sample* sampleAtLeast(uint32_t now_ms, uint16_t requested_age_ms) const {
    const Sample* best = nullptr;
    uint32_t best_age = UINT32_MAX;
    for (uint16_t i = 0; i < history_count_; ++i) {
      const Sample& sample = history_[i];
      const uint32_t age = elapsed(now_ms, sample.time_ms);
      if (age >= requested_age_ms && age < best_age) {
        best = &sample;
        best_age = age;
      }
    }
    return best;
  }

  Config config_;
  Sample history_[HISTORY_SIZE] = {};
  uint16_t history_head_ = 0;
  uint16_t history_count_ = 0;
  State state_ = IDLE;
  Fault fault_ = FAULT_NONE;
  uint16_t target_rpm_ = 0;
  uint8_t target_direction_ = 0;
  float commanded_velocity_ = 0.0f;
  uint32_t state_since_ms_ = 0;
  bool unobservable_reported_ = false;
};
