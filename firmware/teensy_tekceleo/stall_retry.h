#pragma once

#include <stdint.h>

// Bounded retry budget for transient motor-driver stalls. It deliberately
// contains no Arduino dependencies so rollover and escalation behavior can be
// tested on a desktop compiler.
class StallRetryBudget {
 public:
  struct Config {
    uint8_t maximum_retries = 2;
    uint32_t retry_window_ms = 5000;
    uint16_t cooldown_ms = 150;
  };

  StallRetryBudget() : config_() {}
  explicit StallRetryBudget(const Config& config) : config_(config) {}

  bool request(uint32_t now_ms) {
    if (retry_count_ == 0 || elapsed(now_ms, window_start_ms_) >
                                 config_.retry_window_ms) {
      window_start_ms_ = now_ms;
      retry_count_ = 0;
    }
    if (retry_count_ >= config_.maximum_retries) {
      return false;
    }
    ++retry_count_;
    retry_not_before_ms_ = now_ms + config_.cooldown_ms;
    return true;
  }

  bool coolingDown(uint32_t now_ms) const {
    return static_cast<int32_t>(now_ms - retry_not_before_ms_) < 0;
  }

  void reset() {
    retry_count_ = 0;
    window_start_ms_ = 0;
    retry_not_before_ms_ = 0;
  }

  uint8_t retryCount() const { return retry_count_; }
  uint32_t retryNotBeforeMs() const { return retry_not_before_ms_; }

 private:
  static uint32_t elapsed(uint32_t now, uint32_t before) {
    return now - before;
  }

  Config config_;
  uint8_t retry_count_ = 0;
  uint32_t window_start_ms_ = 0;
  uint32_t retry_not_before_ms_ = 0;
};
