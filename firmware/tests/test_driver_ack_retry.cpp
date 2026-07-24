#include "../teensy_tekceleo/driver_ack_retry.h"

#include <assert.h>
#include <stdint.h>

namespace {

void test_first_attempt_success() {
  uint8_t calls = 0;
  uint8_t waits = 0;
  uint8_t used = 0;
  const bool ok = DriverAckRetry::run(
      3,
      [&]() { ++calls; return true; },
      [&](uint8_t) { ++waits; },
      &used);
  assert(ok);
  assert(calls == 1);
  assert(waits == 0);
  assert(used == 1);
}

void test_delayed_ack_succeeds_on_retry() {
  uint8_t calls = 0;
  uint8_t waits = 0;
  uint8_t used = 0;
  const bool ok = DriverAckRetry::run(
      3,
      [&]() { ++calls; return calls == 2; },
      [&](uint8_t completed_attempts) {
        ++waits;
        assert(completed_attempts == 1);
      },
      &used);
  assert(ok);
  assert(calls == 2);
  assert(waits == 1);
  assert(used == 2);
}

void test_retry_exhaustion_is_bounded() {
  uint8_t calls = 0;
  uint8_t waits = 0;
  uint8_t used = 0;
  const bool ok = DriverAckRetry::run(
      3,
      [&]() { ++calls; return false; },
      [&](uint8_t) { ++waits; },
      &used);
  assert(!ok);
  assert(calls == 3);
  assert(waits == 2);
  assert(used == 3);
}

void test_zero_attempts_fail_without_calling_driver() {
  uint8_t calls = 0;
  uint8_t used = 99;
  const bool ok = DriverAckRetry::run(
      0,
      [&]() { ++calls; return true; },
      [&](uint8_t) {},
      &used);
  assert(!ok);
  assert(calls == 0);
  assert(used == 0);
}

}  // namespace

int main() {
  test_first_attempt_success();
  test_delayed_ack_succeeds_on_retry();
  test_retry_exhaustion_is_bounded();
  test_zero_attempts_fail_without_calling_driver();
  return 0;
}
