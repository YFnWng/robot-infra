#include "../teensy_tekceleo/stall_detector.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>

namespace {

void advance(StallDetector& detector, uint32_t& now, int32_t& counts,
             float counts_per_second, float units_per_count,
             StallDetector::Transition* last_transition = nullptr) {
  now += 10;
  counts += static_cast<int32_t>(lroundf(counts_per_second * 0.01f));
  const StallDetector::Event event =
      detector.update(now, counts, units_per_count);
  if (last_transition != nullptr &&
      event.transition != StallDetector::NO_EVENT) {
    *last_transition = event.transition;
  }
}

void test_healthy_motion() {
  StallDetector detector;
  uint32_t now = 0;
  int32_t counts = 0;
  detector.begin(now, counts);
  detector.setCommand(now, 20, '1', 1.0f);
  for (int i = 0; i < 200; ++i) {
    advance(detector, now, counts, 100.0f, 0.01f);
  }
  assert(!detector.latched());
}

void test_startup_grace_then_stall() {
  StallDetector detector;
  uint32_t now = 0;
  int32_t counts = 0;
  detector.begin(now, counts);
  detector.setCommand(now, 20, '1', 1.0f);
  StallDetector::Transition last = StallDetector::NO_EVENT;
  for (int i = 0; i < 49; ++i) {
    advance(detector, now, counts, 0.0f, 0.01f, &last);
  }
  assert(last == StallDetector::NO_EVENT);
  for (int i = 0; i < 90; ++i) {
    advance(detector, now, counts, 0.0f, 0.01f, &last);
  }
  assert(last == StallDetector::CONFIRMED);
  assert(detector.latched());
}

void test_wrong_direction() {
  StallDetector detector;
  uint32_t now = 0;
  int32_t counts = 0;
  detector.begin(now, counts);
  detector.setCommand(now, 20, '1', 1.0f);
  StallDetector::Transition last = StallDetector::NO_EVENT;
  for (int i = 0; i < 120; ++i) {
    advance(detector, now, counts, -100.0f, 0.01f, &last);
  }
  assert(last == StallDetector::CONFIRMED);
  assert(detector.fault() == StallDetector::WRONG_DIRECTION);
}

void test_suspect_recovers() {
  StallDetector detector;
  uint32_t now = 0;
  int32_t counts = 0;
  detector.begin(now, counts);
  detector.setCommand(now, 20, '1', 1.0f);
  StallDetector::Transition last = StallDetector::NO_EVENT;
  for (int i = 0; i < 65; ++i) {
    advance(detector, now, counts, 0.0f, 0.01f, &last);
  }
  assert(last == StallDetector::SUSPECTED);
  for (int i = 0; i < 25; ++i) {
    advance(detector, now, counts, 100.0f, 0.01f, &last);
  }
  assert(last == StallDetector::RECOVERED);
  assert(!detector.latched());
}

void test_latch_requires_clear() {
  StallDetector detector;
  uint32_t now = 0;
  int32_t counts = 0;
  detector.begin(now, counts);
  detector.forceLatch(StallDetector::STALL, now);
  detector.setCommand(now, 20, '1', 1.0f);
  assert(detector.latched());
  StallDetector::Event reset = detector.clearFault(now, counts);
  assert(reset.transition == StallDetector::RESET);
  assert(!detector.latched());
}

}  // namespace

int main() {
  test_healthy_motion();
  test_startup_grace_then_stall();
  test_wrong_direction();
  test_suspect_recovers();
  test_latch_requires_clear();
  return 0;
}
