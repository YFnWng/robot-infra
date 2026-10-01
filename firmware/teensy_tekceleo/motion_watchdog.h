#pragma once

#include <stdint.h>

// Arduino-independent motion-authority watchdog. Only a semantically valid
// VEL/POS command refreshes this object; generic host traffic must not do so.
class MotionWatchdog {
 public:
  explicit MotionWatchdog(uint32_t timeout_ms) : timeout_ms_(timeout_ms) {}

  void refresh(uint32_t now_ms) {
    last_refresh_ms_ = now_ms;
    armed_ = true;
  }

  void disarm() { armed_ = false; }

  bool armed() const { return armed_; }

  bool expired(uint32_t now_ms) const {
    return armed_ && static_cast<uint32_t>(now_ms - last_refresh_ms_) >
                         timeout_ms_;
  }

  uint32_t timeoutMs() const { return timeout_ms_; }

 private:
  uint32_t timeout_ms_;
  uint32_t last_refresh_ms_ = 0;
  bool armed_ = false;
};
