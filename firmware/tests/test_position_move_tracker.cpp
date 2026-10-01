#include "../teensy_tekceleo/position_move_tracker.h"

#include <cassert>

int main() {
  PositionMoveTracker tracker;
  const float target[6] = {1.0f, 2.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  const float tolerance[6] = {
      0.1f, 0.2f, 0.1f, 0.1f, 0.1f, 0.1f};
  float position[6] = {};
  constexpr uint32_t retry_idle_ms = 500;
  constexpr uint8_t max_retries = 4;
  constexpr float minimum_progress = 0.25f;

  tracker.begin(100, 0x03, target, tolerance, position);
  assert(tracker.running());
  assert(tracker.commandMask() == 0x03);

  auto update = tracker.update(
      110, position, 50, 1000, retry_idle_ms, max_retries,
      minimum_progress);
  assert(update.status == PositionMoveTracker::NO_EVENT);
  assert(update.stop_mask == 0);

  position[0] = 0.95f;
  update = tracker.update(
      120, position, 50, 1000, retry_idle_ms, max_retries,
      minimum_progress);
  assert(update.stop_mask == 0);
  update = tracker.update(
      169, position, 50, 1000, retry_idle_ms, max_retries,
      minimum_progress);
  assert(update.stop_mask == 0);
  update = tracker.update(
      170, position, 50, 1000, retry_idle_ms, max_retries,
      minimum_progress);
  assert(update.stop_mask == 0x01);
  assert(tracker.activeMask() == 0x02);

  // Leaving tolerance resets the stable-arrival clock.
  position[1] = 1.9f;
  tracker.update(
      180, position, 50, 1000, retry_idle_ms, max_retries,
      minimum_progress);
  position[1] = 1.0f;
  tracker.update(
      200, position, 50, 1000, retry_idle_ms, max_retries,
      minimum_progress);
  position[1] = 2.0f;
  tracker.update(
      210, position, 50, 1000, retry_idle_ms, max_retries,
      minimum_progress);
  update = tracker.update(
      259, position, 50, 1000, retry_idle_ms, max_retries,
      minimum_progress);
  assert(update.stop_mask == 0);
  update = tracker.update(
      260, position, 50, 1000, retry_idle_ms, max_retries,
      minimum_progress);
  assert(update.status == PositionMoveTracker::COMPLETE);
  assert(update.stop_mask == 0x02);
  assert(!tracker.running());

  position[0] = 0.0f;
  position[1] = 0.0f;
  tracker.begin(1000, 0x05, target, tolerance, position);
  update = tracker.update(
      1999, position, 50, 1000, 2000, max_retries,
      minimum_progress);
  assert(update.status == PositionMoveTracker::NO_EVENT);
  update = tracker.update(
      2000, position, 50, 1000, 2000, max_retries,
      minimum_progress);
  assert(update.status == PositionMoveTracker::TIMED_OUT);
  assert(update.stop_mask == 0x05);
  assert(!tracker.running());

  tracker.begin(3000, 0x01, target, tolerance, position);
  tracker.cancel();
  assert(!tracker.running());
  assert(tracker.activeMask() == 0);

  // A substantial partial move may request a bounded residual correction.
  const float tight_tolerance[6] = {
      0.01f, 0.01f, 0.01f, 0.01f, 0.01f, 0.01f};
  position[0] = 0.0f;
  tracker.begin(4000, 0x01, target, tight_tolerance, position);
  position[0] = 0.75f;
  tracker.update(
      4200, position, 50, 5000, retry_idle_ms, max_retries,
      minimum_progress);
  update = tracker.update(
      4700, position, 50, 5000, retry_idle_ms, max_retries,
      minimum_progress);
  assert(update.status == PositionMoveTracker::NO_EVENT);
  assert(update.retry_mask == 0x01);
  assert(tracker.running());

  // If that correction produces no further progress, stop instead of retrying
  // indefinitely into a possible obstruction.
  update = tracker.update(
      5200, position, 50, 5000, retry_idle_ms, max_retries,
      minimum_progress);
  assert(update.status == PositionMoveTracker::TIMED_OUT);
  assert(update.stop_mask == 0x01);
  assert(!tracker.running());

  // Repeated corrections are allowed only while every segment substantially
  // reduces the remaining encoder error.
  position[0] = 0.0f;
  tracker.begin(6000, 0x01, target, tight_tolerance, position);
  const float endpoints[] = {0.75f, 0.9375f, 0.984375f};
  uint32_t now = 6000;
  for (float endpoint : endpoints) {
    now += 200;
    position[0] = endpoint;
    tracker.update(
        now, position, 50, 10000, retry_idle_ms, max_retries,
        minimum_progress);
    now += retry_idle_ms;
    update = tracker.update(
        now, position, 50, 10000, retry_idle_ms, max_retries,
        minimum_progress);
    assert(update.retry_mask == 0x01);
  }
  position[0] = 0.995f;
  now += 200;
  update = tracker.update(
      now, position, 50, 10000, retry_idle_ms, max_retries,
      minimum_progress);
  assert(update.status == PositionMoveTracker::NO_EVENT);
  now += 50;
  update = tracker.update(
      now, position, 50, 10000, retry_idle_ms, max_retries,
      minimum_progress);
  assert(update.status == PositionMoveTracker::COMPLETE);
  assert(update.stop_mask == 0x01);

  // A known external all-axis stop (for example, a bending-limit state
  // transition) must not be mistaken for an endpoint that made insufficient
  // progress. Axis 0 still needs motion, while axis 1 is already within its
  // tolerance and should only complete its settling interval.
  const float interrupted_target[6] = {
      20.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  const float interrupted_tolerance[6] = {
      0.1f, 0.05f, 0.01f, 0.1f, 0.1f, 0.1f};
  float interrupted_position[6] = {
      39.55f, 0.0f, 1.45f, 0.0f, 0.0f, 0.0f};
  tracker.begin(
      12000, 0x05, interrupted_target, interrupted_tolerance,
      interrupted_position);
  interrupted_position[0] = 34.88f;
  interrupted_position[2] = -0.0013f;
  const uint8_t resume_mask = tracker.resumeMaskAfterExternalStop(
      13500, interrupted_position);
  assert(resume_mask == 0x01);
  assert(tracker.running());
  assert(tracker.activeMask() == 0x05);

  // The old progress window would have failed after 500 ms because 4.67 mm
  // was just below 25% of the original 19.55 mm move. The rebased window does
  // not time out while the explicitly resumed axis is moving.
  interrupted_position[0] = 33.0f;
  update = tracker.update(
      14000, interrupted_position, 50, 45000, retry_idle_ms, max_retries,
      minimum_progress);
  assert(update.status == PositionMoveTracker::NO_EVENT);
  assert(update.stop_mask == 0x04);
  assert(tracker.activeMask() == 0x01);

  interrupted_position[0] = 20.0f;
  update = tracker.update(
      14500, interrupted_position, 50, 45000, retry_idle_ms, max_retries,
      minimum_progress);
  assert(update.status == PositionMoveTracker::NO_EVENT);
  update = tracker.update(
      14550, interrupted_position, 50, 45000, retry_idle_ms, max_retries,
      minimum_progress);
  assert(update.status == PositionMoveTracker::COMPLETE);
  assert(update.stop_mask == 0x01);
  assert(!tracker.running());

  // Regression for the 2026-09-29 axis-0 home failure. A 3.6643 mm residual
  // produced only 0.2953 mm of encoder motion on the first driver segment as
  // reversal play was taken up. That is credible motion (> 0.1 mm tolerance)
  // even though it is less than 25% of the requested residual. It must request
  // a bounded residual correction instead of terminating immediately.
  const float backlash_target[6] = {
      20.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  const float backlash_tolerance[6] = {
      0.1f, 0.5f, 0.1f, 0.1f, 0.5f, 0.5f};
  float backlash_position[6] = {
      16.3357f, 0.0f, -0.0007f, 0.0f, 0.0f, 0.0f};
  tracker.begin(
      15000, 0x01, backlash_target, backlash_tolerance,
      backlash_position);
  backlash_position[0] = 16.6310f;
  update = tracker.update(
      15500, backlash_position, 50, 45000, retry_idle_ms, max_retries,
      minimum_progress);
  assert(update.status == PositionMoveTracker::NO_EVENT);
  assert(update.retry_mask == 0);
  update = tracker.update(
      16000, backlash_position, 50, 45000, retry_idle_ms, max_retries,
      minimum_progress);
  assert(update.status == PositionMoveTracker::NO_EVENT);
  assert(update.retry_mask == 0x01);
  assert(tracker.running());

  // The retry remains bounded: a correction that produces no additional
  // encoder motion terminates rather than repeatedly driving an obstruction.
  update = tracker.update(
      16500, backlash_position, 50, 45000, retry_idle_ms, max_retries,
      minimum_progress);
  assert(update.status == PositionMoveTracker::TIMED_OUT);
  assert(update.stop_mask == 0x01);
  assert(!tracker.running());
}
