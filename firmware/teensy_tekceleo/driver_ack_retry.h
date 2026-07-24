#pragma once

#include <stdint.h>

// Small Arduino-independent retry policy so driver communication behavior can
// be exercised by a desktop C++ regression test.
namespace DriverAckRetry {

template <typename Attempt, typename BetweenAttempts>
bool run(uint8_t max_attempts, Attempt attempt,
         BetweenAttempts between_attempts, uint8_t* attempts_used = nullptr) {
  uint8_t used = 0;
  for (uint8_t index = 0; index < max_attempts; ++index) {
    ++used;
    if (attempt()) {
      if (attempts_used != nullptr) {
        *attempts_used = used;
      }
      return true;
    }
    if (index + 1U < max_attempts) {
      between_attempts(used);
    }
  }
  if (attempts_used != nullptr) {
    *attempts_used = used;
  }
  return false;
}

}  // namespace DriverAckRetry
