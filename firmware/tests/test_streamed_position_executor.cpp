#include "../teensy_tekceleo/streamed_position_executor.h"

#include <cassert>
#include <climits>
#include <iostream>
#include <iterator>
#include <vector>

using StreamedPosition::Command;
using StreamedPosition::Executor;
using Admission = Executor::Admission;
using Status = Executor::Status;

static Executor::Config config() {
  Executor::Config result;
  for (uint8_t axis = 0; axis < 6; ++axis) {
    result.lower_counts[axis] = -10000;
    result.upper_counts[axis] = 10000;
    result.maximum_step_counts[axis] = 1000;
    result.tolerance_counts[axis] = 2;
    result.minimum_rpm[axis] = 20;
    result.maximum_rpm[axis] = 250;
  }
  return result;
}

static Command command(uint32_t id = 1, int32_t target = 100) {
  Command result;
  result.id = id;
  result.duration_us = 66667;
  result.active_mask = 1;
  result.target_counts[0] = target;
  result.speed_rpm[0] = 20;
  return result;
}

static void test_protocol() {
  uint8_t payload[StreamedPosition::kPayloadSize];
  Command a = command(0xffffffffU, INT_MIN);
  a.target_counts[5] = INT_MAX;
  assert(StreamedPosition::encode(a, payload, sizeof(payload)));
  Command b;
  assert(StreamedPosition::decode(payload, sizeof(payload), b));
  assert(StreamedPosition::same(a, b));
  const Command saved = b;
  payload[0] = 2;
  assert(!StreamedPosition::decode(payload, sizeof(payload), b));
  assert(StreamedPosition::same(saved, b));
  payload[0] = 1;
  assert(!StreamedPosition::decode(payload, sizeof(payload)-1, b));
  assert(!StreamedPosition::decode(nullptr, sizeof(payload), b));
  a.speed_rpm[1] = 1;
  assert(!StreamedPosition::encode(a, payload, sizeof(payload)));
  assert(StreamedPosition::newerId(1, 0xffffffffU));
  assert(!StreamedPosition::newerId(0, 0xffffffffU));
  assert(!StreamedPosition::newerId(0x80000001U, 1));
}

static void test_replacement_and_heartbeat() {
  Executor executor(config());
  int32_t counts[6] = {};
  int16_t applied[6] = {};
  const Command first = command();
  assert(executor.submit(first, 1000, counts, true, 0x3f) == Admission::ACCEPTED);
  auto output = executor.update(1000, counts, applied, true, 0x3f);
  assert(output.requested_rpm[0] == 20);
  applied[0] = 20;
  counts[0] = 30;
  Command replacement = command(2, 150);
  assert(executor.submit(replacement, 30000, counts, true, 0x3f) == Admission::ACCEPTED);
  output = executor.update(30000, counts, applied, true, 0x3f);
  assert(output.requested_rpm[0] == 20 && output.stop_mask == 0);
  assert(executor.submit(first, 31000, counts, true, 0x3f) == Admission::STALE_ID);
  assert(executor.submit(replacement, 95000, counts, true, 0x3f) == Admission::HEARTBEAT);
  output = executor.update(96666, counts, applied, true, 0x3f);
  assert(output.status == Status::RUNNING);
  output = executor.update(96667, counts, applied, true, 0x3f);
  assert(output.status == Status::EXPIRED && output.stop_mask == 1);
  assert(output.requested_rpm[0] == 0);
  assert(executor.submit(replacement, 100000, counts, true, 0x3f) == Admission::HEARTBEAT);
  assert(executor.update(100000, counts, applied, true, 0x3f).status == Status::EXPIRED);
}

static void test_completion_overshoot_and_axis_hold() {
  Executor executor(config());
  int32_t counts[6] = {};
  int16_t applied[6] = {20, 0, 30, 0, 0, 0};
  Command c = command();
  c.active_mask = 5;
  c.target_counts[2] = -100;
  c.speed_rpm[2] = 30;
  assert(executor.submit(c, 0, counts, true, 0x3f) == Admission::ACCEPTED);
  counts[0] = 98;
  auto output = executor.update(1000, counts, applied, true, 0x3f);
  assert(output.requested_rpm[0] == 0 && (output.stop_mask & 1));
  // Axis 2 was moving positive, so requesting negative must first stop it.
  assert(output.requested_rpm[2] == 0 && (output.reversal_wait_mask & 4));
  applied[0] = 0;
  applied[2] = 0;
  executor.update(2000, counts, applied, true, 0x3f);
  counts[0] = 90;  // Completed shaft must not chase subsequent drift.
  output = executor.update(8000, counts, applied, true, 0x3f);
  assert(output.requested_rpm[0] == 0 && output.requested_rpm[2] == -30);
  counts[2] = -105;
  applied[2] = -30;
  output = executor.update(10000, counts, applied, true, 0x3f);
  assert(output.status == Status::COMPLETE && output.overshoot_mask == 4);
  assert(output.requested_rpm[2] == 0 && (output.stop_mask & 4));
  assert(executor.submit(c, 20000, counts, true, 0x3f) == Admission::HEARTBEAT);
  counts[2] = -80;
  assert(executor.update(21000, counts, applied, true, 0x3f).requested_rpm[2] == 0);
}

static void test_reversal_waits_for_encoder_standstill() {
  Executor executor(config());
  int32_t counts[6] = {};
  int16_t applied[6] = {20, 0, 0, 0, 0, 0};
  executor.submit(command(), 0, counts, true, 0x3f);
  executor.update(0, counts, applied, true, 0x3f);
  counts[0] = 10;
  executor.submit(command(2, -100), 10000, counts, true, 0x3f);
  auto output = executor.update(10000, counts, applied, true, 0x3f);
  assert(output.stop_mask == 1 && output.reversal_wait_mask == 1);
  applied[0] = 0;
  executor.update(11000, counts, applied, true, 0x3f);
  counts[0] = 11;  // Coasting restarts the settling clock.
  executor.update(14000, counts, applied, true, 0x3f);
  output = executor.update(18999, counts, applied, true, 0x3f);
  assert(output.requested_rpm[0] == 0);
  output = executor.update(19000, counts, applied, true, 0x3f);
  assert(output.requested_rpm[0] == -20 && output.reversal_wait_mask == 0);
}

static void test_progress_budget_survives_replacement_and_hold() {
  auto cfg = config();
  cfg.no_progress_us = 100000;
  Executor executor(cfg);
  int32_t counts[6] = {};
  int16_t applied[6] = {20, 0, 0, 0, 0, 0};
  executor.submit(command(), 0, counts, true, 0x3f);
  executor.update(0, counts, applied, true, 0x3f);
  executor.submit(command(2, 101), 60000, counts, true, 0x3f);
  executor.update(60000, counts, applied, true, 0x3f);
  executor.submit(command(3, 102), 110000, counts, true, 0x3f);
  auto output = executor.update(110000, counts, applied, true, 0x3f);
  assert(output.status == Status::NO_PROGRESS && output.stop_mask == 1);
  assert(executor.submit(command(4), 111000, counts, true, 0x3f) == Admission::PROGRESS_BLOCKED);
  Command hold = command(5);
  hold.active_mask = 0;
  hold.speed_rpm[0] = 0;
  assert(executor.submit(hold, 112000, counts, true, 0x3f) == Admission::ACCEPTED);
  assert(executor.submit(command(6), 113000, counts, true, 0x3f) == Admission::PROGRESS_BLOCKED);
  assert(!executor.resetAfterVerifiedStop(false, applied));
  assert(!executor.resetAfterVerifiedStop(true, applied));
  applied[0] = 0;
  assert(executor.resetAfterVerifiedStop(true, applied));
  assert(executor.submit(command(3), 114000, counts, true, 0x3f) == Admission::STALE_ID);
  assert(executor.submit(command(6), 114000, counts, true, 0x3f) == Admission::REJECTED_REPLAY);
  assert(executor.submit(command(7), 114000, counts, true, 0x3f) == Admission::ACCEPTED);
}

static void test_measured_progress_and_pause() {
  auto cfg = config();
  cfg.no_progress_us = 50000;
  Executor executor(cfg);
  int32_t counts[6] = {};
  int16_t applied[6] = {20, 0, 0, 0, 0, 0};
  executor.submit(command(), 0, counts, true, 0x3f);
  executor.update(0, counts, applied, true, 0x3f);
  counts[0] = 1;
  assert(executor.update(40000, counts, applied, true, 0x3f).status == Status::RUNNING);
  executor.cancel();
  applied[0] = 0;
  executor.update(41000, counts, applied, true, 0x3f);
  executor.update(200000, counts, applied, true, 0x3f);
  executor.submit(command(2), 200000, counts, true, 0x3f);
  assert(executor.update(200000, counts, applied, true, 0x3f).status == Status::RUNNING);
}

static void test_rejections_and_cancellation() {
  int32_t counts[6] = {};
  int16_t applied[6] = {20, 0, 0, 0, 0, 0};
  Executor unconfigured(Executor::Config{});
  assert(unconfigured.submit(command(), 0, counts, true, 0x3f) == Admission::INVALID_COMMAND);
  assert(unconfigured.update(0, counts, applied, true, 0x3f).requested_rpm[0] == 0);
  Executor step_guard(config());
  assert(step_guard.submit(command(1, 1500), 0, counts, true, 0x3f) == Admission::LIMIT_REJECTED);
  Command partial = command(2);
  partial.active_mask = 5;
  partial.speed_rpm[2] = 20;
  partial.target_counts[2] = -20000;
  assert(step_guard.submit(partial, 1, counts, true, 0x3f) == Admission::LIMIT_REJECTED);
  assert(step_guard.update(1, counts, applied, true, 0x3f).requested_rpm[0] == 0);
  Executor executor(config());
  executor.submit(command(), 0, counts, true, 0x3f);
  Command changed = command();
  changed.target_counts[0] = 101;
  assert(executor.submit(changed, 1, counts, true, 0x3f) == Admission::ID_CONFLICT);
  assert(executor.update(1, counts, applied, true, 0x3f).requested_rpm[0] == 0);
  Command invalid = command(2);
  invalid.duration_us = 250001;
  assert(executor.submit(invalid, 2, counts, true, 0x3f) == Admission::INVALID_COMMAND);
  invalid = command(2, 20000);
  assert(executor.submit(invalid, 3, counts, true, 0x3f) == Admission::LIMIT_REJECTED);
  invalid = command(3);
  invalid.speed_rpm[0] = 1;
  assert(executor.submit(invalid, 4, counts, true, 0x3f) == Admission::LIMIT_REJECTED);
  assert(executor.submit(command(4), 5, counts, false, 0x3f) == Admission::INVALID_FEEDBACK);
  assert(executor.submit(command(4), 6, counts, true, 0x3f) == Admission::REJECTED_REPLAY);
  assert(executor.submit(command(5), 6, counts, true, 0) == Admission::INHIBITED);
  executor.submit(command(6), 7, counts, true, 0x3f);
  assert(executor.update(8, counts, applied, true, 0).status == Status::INHIBITED);
  executor.submit(command(7), 9, counts, true, 0x3f);
  assert(executor.update(10, counts, nullptr, false, 0x3f).stop_mask == 0x3f);
  executor.submit(command(8), 11, counts, true, 0x3f);
  executor.cancel();
  auto output = executor.update(12, counts, applied, true, 0x3f);
  assert(output.status == Status::CANCELLED && output.stop_mask == 1);
  assert(executor.submit(command(8), 13, counts, false, 0x3f) == Admission::INVALID_FEEDBACK);
}

static void test_wraparound_and_extreme_counts() {
  auto cfg = config();
  cfg.lower_counts[0] = INT_MIN;
  cfg.upper_counts[0] = INT_MAX;
  cfg.maximum_step_counts[0] = UINT_MAX;
  Executor executor(cfg);
  int32_t counts[6] = {INT_MIN, 0, 0, 0, 0, 0};
  int16_t applied[6] = {};
  Command c = command(0xffffffffU, INT_MAX);
  c.duration_us = 20;
  assert(executor.submit(c, 0xfffffff0U, counts, true, 0x3f) == Admission::ACCEPTED);
  assert(executor.update(0xffffffffU, counts, applied, true, 0x3f).requested_rpm[0] == 20);
  assert(executor.update(4, counts, applied, true, 0x3f).status == Status::EXPIRED);
  c.id = 1;
  assert(executor.submit(c, 5, counts, true, 0x3f) == Admission::ACCEPTED);
  c.id = 0x80000001U;
  assert(executor.submit(c, 6, counts, true, 0x3f) == Admission::STALE_ID);
}

static void test_feedback_gap_cannot_supply_reversal_settling() {
  Executor executor(config());
  int32_t counts[6] = {};
  int16_t applied[6] = {20, 0, 0, 0, 0, 0};
  executor.submit(command(), 0, counts, true, 0x3f);
  executor.update(0, counts, applied, true, 0x3f);
  executor.cancel();
  applied[0] = 0;
  executor.update(1000, counts, applied, true, 0x3f);
  executor.update(10000, counts, applied, false, 0x3f);
  executor.submit(command(2, -100), 20000, counts, true, 0x3f);
  assert(executor.update(20000, counts, applied, true, 0x3f).requested_rpm[0] == 0);
  assert(executor.update(25000, counts, applied, true, 0x3f).requested_rpm[0] == -20);
  applied[0] = 251;
  assert(executor.update(26000, counts, applied, true, 0x3f).status == Status::INVALID_FEEDBACK);
}

int main(int argc, char**) {
  // Cross-language codec check: stdin payload -> decoded/encoded stdout payload.
  if (argc > 1) {
    const std::vector<uint8_t> input{std::istreambuf_iterator<char>(std::cin),
                                     std::istreambuf_iterator<char>()};
    Command decoded;
    if (!StreamedPosition::decode(input.data(), input.size(), decoded)) return 2;
    uint8_t output[StreamedPosition::kPayloadSize];
    if (!StreamedPosition::encode(decoded, output, sizeof(output))) return 3;
    std::cout.write(reinterpret_cast<const char*>(output), sizeof(output));
    return 0;
  }
  test_protocol();
  test_replacement_and_heartbeat();
  test_completion_overshoot_and_axis_hold();
  test_reversal_waits_for_encoder_standstill();
  test_progress_budget_survives_replacement_and_hold();
  test_measured_progress_and_pause();
  test_rejections_and_cancellation();
  test_wraparound_and_extreme_counts();
  test_feedback_gap_cannot_supply_reversal_settling();
  std::cout << "9 streamed-position scenario groups passed\n";
}
