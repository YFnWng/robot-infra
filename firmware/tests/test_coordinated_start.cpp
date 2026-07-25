#include "../teensy_tekceleo/coordinated_start.h"

#include <assert.h>
#include <stdint.h>
#include <vector>

namespace {

void test_all_configuration_precedes_enable() {
  std::vector<int> order;
  const bool ok = CoordinatedStart::run(
      static_cast<uint8_t>((1U << 0) | (1U << 2)), 6,
      [&](uint8_t axis) {
        order.push_back(10 + axis);
        return true;
      },
      [&](uint8_t mask) {
        order.push_back(100 + mask);
        return true;
      },
      [&](uint8_t) { order.push_back(999); });
  assert(ok);
  assert(order.size() == 3);
  assert(order[0] == 10);
  assert(order[1] == 12);
  assert(order[2] == 105);
}

void test_configuration_failure_never_enables_and_rolls_back() {
  uint8_t enable_calls = 0;
  uint8_t rollback_mask = 0;
  const uint8_t mask = static_cast<uint8_t>((1U << 0) | (1U << 2));
  const bool ok = CoordinatedStart::run(
      mask, 6,
      [&](uint8_t axis) { return axis != 2; },
      [&](uint8_t) {
        ++enable_calls;
        return true;
      },
      [&](uint8_t value) { rollback_mask = value; });
  assert(!ok);
  assert(enable_calls == 0);
  assert(rollback_mask == mask);
}

void test_enable_failure_rolls_back_entire_mask() {
  uint8_t configured = 0;
  uint8_t rollback_mask = 0;
  const uint8_t mask = static_cast<uint8_t>((1U << 1) | (1U << 4));
  const bool ok = CoordinatedStart::run(
      mask, 6,
      [&](uint8_t) {
        ++configured;
        return true;
      },
      [&](uint8_t) { return false; },
      [&](uint8_t value) { rollback_mask = value; });
  assert(!ok);
  assert(configured == 2);
  assert(rollback_mask == mask);
}

}  // namespace

int main() {
  test_all_configuration_precedes_enable();
  test_configuration_failure_never_enables_and_rolls_back();
  test_enable_failure_rolls_back_entire_mask();
  return 0;
}
