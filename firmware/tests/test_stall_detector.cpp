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

uint16_t rpmForVelocity(float velocity) {
  const long rpm = lroundf(fabsf(velocity) * 2.2f);
  return static_cast<uint16_t>(rpm < 1 ? 1 : rpm);
}

void followCommand(StallDetector& detector, uint32_t& now, int32_t& counts,
                   double& exact_counts, float velocity,
                   float units_per_count,
                   StallDetector::Transition* last_transition = nullptr) {
  detector.setCommand(now, rpmForVelocity(velocity),
                      velocity >= 0.0f ? '1' : '0', velocity);
  now += 10;
  exact_counts += static_cast<double>(velocity) * 0.01 /
                  static_cast<double>(units_per_count);
  counts = static_cast<int32_t>(llround(exact_counts));
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

void test_twenty_one_percent_progress_is_not_stall() {
  StallDetector detector;
  uint32_t now = 0;
  int32_t counts = 0;
  double exact_counts = 0.0;
  constexpr float units_per_count = 0.001f;
  detector.begin(now, counts);
  detector.setCommand(now, 20, '1', 1.0f);
  for (int i = 0; i < 150; ++i) {
    now += 10;
    exact_counts += 0.21 * 0.01 / units_per_count;
    counts = static_cast<int32_t>(llround(exact_counts));
    detector.update(now, counts, units_per_count);
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

void test_gradual_deceleration_uses_matching_command_window() {
  StallDetector detector;
  uint32_t now = 0;
  int32_t counts = 0;
  double exact_counts = 0.0;
  const float units_per_count = 0.01f;
  detector.begin(now, counts);

  for (int i = 0; i < 80; ++i) {
    followCommand(detector, now, counts, exact_counts, -25.0f,
                  units_per_count);
  }
  for (int i = 0; i < 120; ++i) {
    const float fraction = static_cast<float>(i + 1) / 120.0f;
    const float velocity = -25.0f + fraction * (25.0f - 0.425f);
    followCommand(detector, now, counts, exact_counts, velocity,
                  units_per_count);
  }
  for (int i = 0; i < 100; ++i) {
    followCommand(detector, now, counts, exact_counts, -0.425f,
                  units_per_count);
  }
  assert(!detector.latched());
}

void test_fault_window_deceleration_does_not_report_overspeed() {
  StallDetector detector;
  uint32_t now = 0;
  int32_t counts = 0;
  double exact_counts = 0.0;
  const float units_per_count = 0.01f;
  detector.begin(now, counts);

  for (int i = 0; i < 100; ++i) {
    followCommand(detector, now, counts, exact_counts, -3.23f,
                  units_per_count);
  }

  bool saw_overspeed = false;
  for (int i = 0; i < 51; ++i) {
    const float fraction = static_cast<float>(i + 1) / 51.0f;
    const float command = -3.23f + fraction * (3.23f - 0.425f);
    detector.setCommand(now, rpmForVelocity(command), '0', command);
    now += 10;
    // Reproduce the reported 509 ms encoder-window average. It is above the
    // average decelerating command, but below 2.5x the recent peak command.
    exact_counts += -7.3865 * 0.01 / units_per_count;
    counts = static_cast<int32_t>(llround(exact_counts));
    const StallDetector::Event event =
        detector.update(now, counts, units_per_count);
    if (event.fault == StallDetector::OVERSPEED) {
      saw_overspeed = true;
    }
  }
  assert(!saw_overspeed);
  assert(!detector.latched());
}

void test_gradual_acceleration_uses_matching_command_window() {
  StallDetector detector;
  uint32_t now = 0;
  int32_t counts = 0;
  double exact_counts = 0.0;
  const float units_per_count = 0.01f;
  detector.begin(now, counts);

  for (int i = 0; i < 80; ++i) {
    followCommand(detector, now, counts, exact_counts, 0.425f,
                  units_per_count);
  }
  for (int i = 0; i < 120; ++i) {
    const float fraction = static_cast<float>(i + 1) / 120.0f;
    const float velocity = 0.425f + fraction * (25.0f - 0.425f);
    followCommand(detector, now, counts, exact_counts, velocity,
                  units_per_count);
  }
  for (int i = 0; i < 80; ++i) {
    followCommand(detector, now, counts, exact_counts, 25.0f,
                  units_per_count);
  }
  assert(!detector.latched());
}

void test_healthy_reversal_does_not_mix_pre_reversal_command() {
  StallDetector detector;
  uint32_t now = 0;
  int32_t counts = 0;
  double exact_counts = 0.0;
  const float units_per_count = 0.01f;
  detector.begin(now, counts);

  for (int i = 0; i < 80; ++i) {
    followCommand(detector, now, counts, exact_counts, 5.0f,
                  units_per_count);
  }
  for (int i = 0; i < 120; ++i) {
    followCommand(detector, now, counts, exact_counts, -5.0f,
                  units_per_count);
  }
  assert(!detector.latched());
}

void test_real_overspeed_still_latches() {
  StallDetector detector;
  uint32_t now = 0;
  int32_t counts = 0;
  detector.begin(now, counts);
  detector.setCommand(now, 20, '1', 1.0f);
  StallDetector::Transition last = StallDetector::NO_EVENT;
  for (int i = 0; i < 120; ++i) {
    advance(detector, now, counts, 400.0f, 0.01f, &last);
  }
  assert(last == StallDetector::CONFIRMED);
  assert(detector.fault() == StallDetector::OVERSPEED);
}

void test_slow_overspeed_requires_an_additional_window() {
  StallDetector detector;
  uint32_t now = 0;
  int32_t counts = 0;
  detector.begin(now, counts);
  detector.setCommand(now, 6, '1', 2.0f);
  StallDetector::Transition last = StallDetector::NO_EVENT;

  // 500 ms startup + 900 ms persistent violation is shorter than the new
  // 500 ms base confirmation + 500 ms additional-window requirement.
  for (int i = 0; i < 140; ++i) {
    advance(detector, now, counts, 800.0f, 0.01f, &last);
  }
  assert(!detector.latched());

  for (int i = 0; i < 20; ++i) {
    advance(detector, now, counts, 800.0f, 0.01f, &last);
  }
  assert(last == StallDetector::CONFIRMED);
  assert(detector.fault() == StallDetector::OVERSPEED);
}

void test_slow_stall_confirmation_timing_is_unchanged() {
  StallDetector detector;
  uint32_t now = 0;
  int32_t counts = 0;
  detector.begin(now, counts);
  detector.setCommand(now, 6, '1', 2.0f);
  StallDetector::Transition last = StallDetector::NO_EVENT;
  for (int i = 0; i < 120; ++i) {
    advance(detector, now, counts, 0.0f, 0.01f, &last);
  }
  assert(last == StallDetector::CONFIRMED);
  assert(detector.fault() == StallDetector::STALL);
}

void test_idle_monitor_tracks_position_profile_without_velocity_faults() {
  StallDetector detector;
  uint32_t now = 0;
  int32_t counts = 0;
  detector.begin(now, counts);

  // Position mode deliberately leaves the velocity monitor idle because the
  // driver, rather than Teensy, owns the instantaneous velocity profile.
  detector.setCommand(now, 0, '1', 0.0f);
  for (int i = 0; i < 200; ++i) {
    const float counts_per_second =
        i < 60 ? 300.0f : (i < 120 ? 0.0f : -150.0f);
    StallDetector::Transition transition = StallDetector::NO_EVENT;
    advance(
        detector, now, counts, counts_per_second, 0.01f, &transition);
    assert(transition == StallDetector::NO_EVENT);
    assert(!detector.latched());
  }
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
  test_twenty_one_percent_progress_is_not_stall();
  test_startup_grace_then_stall();
  test_wrong_direction();
  test_suspect_recovers();
  test_gradual_deceleration_uses_matching_command_window();
  test_fault_window_deceleration_does_not_report_overspeed();
  test_gradual_acceleration_uses_matching_command_window();
  test_healthy_reversal_does_not_mix_pre_reversal_command();
  test_real_overspeed_still_latches();
  test_slow_overspeed_requires_an_additional_window();
  test_slow_stall_confirmation_timing_is_unchanged();
  test_idle_monitor_tracks_position_profile_without_velocity_faults();
  test_latch_requires_clear();
  return 0;
}
