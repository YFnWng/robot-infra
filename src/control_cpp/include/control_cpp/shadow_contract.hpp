#ifndef CONTROL_CPP__SHADOW_CONTRACT_HPP_
#define CONTROL_CPP__SHADOW_CONTRACT_HPP_

#include <array>
#include <cstdint>
#include <string>

namespace control_cpp
{

constexpr std::uint32_t kShadowSchemaVersion = 1U;

struct ShadowRequestRecord
{
  std::uint64_t shell_epoch{};
  std::uint64_t request_sequence{};
  std::uint64_t target_revision{};
  std::uint64_t device_sequence{};
  std::uint64_t marker_sequence{};
  std::uint64_t manager_sequence{};
  std::int64_t request_steady_ns{};
};

struct ShadowDecisionRecord
{
  std::uint32_t schema_version{};
  std::uint64_t shell_epoch{};
  std::uint64_t request_sequence{};
  std::uint64_t target_revision{};
  std::uint64_t device_sequence{};
  std::uint64_t marker_sequence{};
  std::uint64_t manager_sequence{};
  std::int64_t computation_start_steady_ns{};
  std::int64_t computation_end_steady_ns{};
  bool valid{};
  std::array<double, 6> logical_velocity{};
};

struct ShadowValidationContext
{
  std::uint64_t current_epoch{};
  std::uint64_t current_target_revision{};
  std::uint64_t last_accepted_sequence{};
  std::int64_t now_steady_ns{};
  std::int64_t maximum_result_age_ns{};
  std::int64_t future_tolerance_ns{1000000};
};

std::string decision_rejection_reason(
  const ShadowDecisionRecord & decision,
  const ShadowRequestRecord * request,
  const ShadowValidationContext & context);

}  // namespace control_cpp

#endif  // CONTROL_CPP__SHADOW_CONTRACT_HPP_
