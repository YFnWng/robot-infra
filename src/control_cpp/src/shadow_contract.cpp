#include "control_cpp/shadow_contract.hpp"

#include <algorithm>
#include <cmath>

namespace control_cpp
{

std::string decision_rejection_reason(
  const ShadowDecisionRecord & decision,
  const ShadowRequestRecord * request,
  const ShadowValidationContext & context)
{
  if (decision.schema_version != kShadowSchemaVersion) {
    return "schema_version_mismatch";
  }
  if (request == nullptr) {
    return "request_unknown";
  }
  if (decision.shell_epoch != context.current_epoch) {
    return "shell_epoch_mismatch";
  }
  if (decision.request_sequence != request->request_sequence) {
    return "request_sequence_mismatch";
  }
  if (decision.request_sequence <= context.last_accepted_sequence) {
    return "request_sequence_stale";
  }
  if (request->shell_epoch != context.current_epoch) {
    return "request_epoch_stale";
  }
  if (
    decision.target_revision != request->target_revision ||
    decision.target_revision != context.current_target_revision)
  {
    return "target_revision_mismatch";
  }
  if (
    decision.device_sequence != request->device_sequence ||
    decision.marker_sequence != request->marker_sequence ||
    decision.manager_sequence != request->manager_sequence)
  {
    return "input_watermark_mismatch";
  }
  if (!decision.valid) {
    return "worker_decision_invalid";
  }
  const bool finite = std::all_of(
    decision.logical_velocity.begin(), decision.logical_velocity.end(),
    [](double value) {return std::isfinite(value);});
  if (!finite) {
    return "velocity_nonfinite";
  }
  const auto start = decision.computation_start_steady_ns;
  const auto end = decision.computation_end_steady_ns;
  const auto now = context.now_steady_ns;
  if (start <= 0 || end < start) {
    return "computation_time_invalid";
  }
  if (end > now + context.future_tolerance_ns) {
    return "result_from_future";
  }
  if (now - end > context.maximum_result_age_ns) {
    return "result_stale";
  }
  return "accepted";
}

}  // namespace control_cpp
