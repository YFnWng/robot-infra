#include "streamed_position_session.h"

extern "C" {
// C ABI for isolated simulation. Firmware and simulation share the same owner.
struct PositionConfig {
  int32_t lower[6], upper[6];
  uint32_t step[6], tolerance[6];
  uint16_t minimum_rpm[6], maximum_rpm[6];
  uint8_t native_position;
};
struct PositionUpdate {
  uint32_t command_id;
  int16_t rpm[6];
  int64_t residual[6];
  uint8_t status, stop_mask, reversal_mask, overshoot_mask;
  uint8_t native_start_mask;
  uint32_t native_deci_degrees[6];
};
uint32_t position_abi_version() { return 2; }
void* position_create(const PositionConfig* input) {
  if (!input) return nullptr;
  StreamedPosition::Executor::Config config;
  config.native_position = input->native_position != 0;
  if (config.native_position) config.direction_settle_us = 100000;
  for (int a = 0; a < 6; ++a) {
    config.lower_counts[a] = input->lower[a]; config.upper_counts[a] = input->upper[a];
    config.maximum_step_counts[a] = input->step[a]; config.tolerance_counts[a] = input->tolerance[a];
    config.minimum_rpm[a] = input->minimum_rpm[a]; config.maximum_rpm[a] = input->maximum_rpm[a];
  }
  return new StreamedPosition::Session(config);
}
void position_destroy(void* handle) { delete static_cast<StreamedPosition::Session*>(handle); }
uint32_t position_open(void* handle, bool stopped, uint32_t nonce) {
  return static_cast<StreamedPosition::Session*>(handle)->open(stopped, nonce);
}
int position_submit(void* handle, const uint8_t* payload, size_t size, uint32_t time,
                    const int32_t* counts, bool fresh, uint8_t allowed) {
  StreamedPosition::Envelope envelope;
  if (!StreamedPosition::decodeEnvelope(payload, size, envelope)) return -2;
  StreamedPosition::Executor::Admission admission;
  if (!static_cast<StreamedPosition::Session*>(handle)->submit(
      envelope, time, counts, fresh, allowed, admission)) return -1;
  return static_cast<int>(admission);
}
void position_cancel(void* handle) { static_cast<StreamedPosition::Session*>(handle)->cancel(); }
void position_native_started(void* handle, uint8_t mask) {
  static_cast<StreamedPosition::Session*>(handle)->nativeStarted(mask);
}
void position_update(void* handle, uint32_t time, const int32_t* counts,
                    const int16_t* applied, bool fresh, uint8_t allowed, PositionUpdate* out) {
  const auto result = static_cast<StreamedPosition::Session*>(handle)->update(time, counts, applied, fresh, allowed);
  out->command_id = result.command_id;
  for (int a = 0; a < 6; ++a) { out->rpm[a] = result.requested_rpm[a]; out->residual[a] = result.residual_counts[a]; }
  out->status = static_cast<uint8_t>(result.status); out->stop_mask = result.stop_mask;
  out->reversal_mask = result.reversal_wait_mask; out->overshoot_mask = result.overshoot_mask;
  out->native_start_mask = result.native_start_mask;
  for (int a = 0; a < 6; ++a) out->native_deci_degrees[a] = result.native_deci_degrees[a];
}
}
