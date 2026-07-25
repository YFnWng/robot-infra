#include "../teensy_tekceleo/encoder_integrity_guard.h"

#include <assert.h>
#include <stdint.h>

namespace {

constexpr uint32_t kMaximumCountsPerSecond = 33333;
constexpr uint8_t kSpeedMargin = 2;
constexpr uint16_t kCountMargin = 100;

void test_healthy_frame_is_accepted() {
  const int32_t last[6] = {1000, -1000, 0, 20, 30, 40};
  const int32_t next[6] = {1300, -1300, 350, 20, 29, 41};
  const auto result = EncoderIntegrityGuard::validate(
      next, last, 10, kMaximumCountsPerSecond, kSpeedMargin, kCountMargin);
  assert(result.valid);
  assert(result.invalid_mask == 0);
  assert(result.allowed_delta == 767);
}

void test_one_bad_axis_rejects_entire_frame() {
  const int32_t last[6] = {10, 20, 30, 40, 50, 60};
  const int32_t next[6] = {11, 21, 1000000, 41, 51, 61};
  const auto result = EncoderIntegrityGuard::validate(
      next, last, 10, kMaximumCountsPerSecond, kSpeedMargin, kCountMargin);
  assert(!result.valid);
  assert(result.invalid_mask == (1U << 2));
  assert(result.first_invalid_axis == 2);
  assert(result.first_invalid_delta == 999970);
  // The guard has no mutable accepted state: callers retain all six values.
  assert(last[0] == 10 && last[2] == 30 && last[5] == 60);
}

void test_multiple_bad_axes_are_reported() {
  const int32_t last[6] = {0, 0, 0, 0, 0, 0};
  const int32_t next[6] = {500000, 0, -600000, 0, 700000, 0};
  const auto result = EncoderIntegrityGuard::validate(
      next, last, 10, kMaximumCountsPerSecond, kSpeedMargin, kCountMargin);
  assert(!result.valid);
  assert(result.invalid_mask == ((1U << 0) | (1U << 2) | (1U << 4)));
  assert(result.first_invalid_axis == 0);
}

void test_elapsed_time_scales_limit_after_blocking_io() {
  const int32_t last[6] = {0, 0, 0, 0, 0, 0};
  const int32_t next[6] = {10000, -10000, 0, 0, 0, 0};
  const auto result = EncoderIntegrityGuard::validate(
      next, last, 200, kMaximumCountsPerSecond, kSpeedMargin, kCountMargin);
  assert(result.valid);
  assert(result.allowed_delta == 13434);
}

void test_signed_extremes_do_not_overflow() {
  const int32_t last[6] = {INT32_MIN, 0, 0, 0, 0, 0};
  const int32_t next[6] = {INT32_MAX, 0, 0, 0, 0, 0};
  const auto result = EncoderIntegrityGuard::validate(
      next, last, 10, kMaximumCountsPerSecond, kSpeedMargin, kCountMargin);
  assert(!result.valid);
  assert(result.first_invalid_delta == INT32_MAX);
}

}  // namespace

int main() {
  test_healthy_frame_is_accepted();
  test_one_bad_axis_rejects_entire_frame();
  test_multiple_bad_axes_are_reported();
  test_elapsed_time_scales_limit_after_blocking_io();
  test_signed_extremes_do_not_overflow();
  return 0;
}
