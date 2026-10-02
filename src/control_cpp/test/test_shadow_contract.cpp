#include "control_cpp/shadow_contract.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace
{

std::vector<std::string> split(const std::string & line)
{
  std::vector<std::string> fields;
  std::stringstream stream(line);
  std::string field;
  while (std::getline(stream, field, ',')) {
    fields.push_back(field);
  }
  return fields;
}

std::uint64_t u64(const std::vector<std::string> & fields, std::size_t index)
{
  return static_cast<std::uint64_t>(std::stoull(fields.at(index)));
}

std::int64_t i64(const std::vector<std::string> & fields, std::size_t index)
{
  return static_cast<std::int64_t>(std::stoll(fields.at(index)));
}

TEST(ShadowContract, SharedDecisionCases)
{
  std::ifstream fixture(SHADOW_FIXTURE_PATH);
  ASSERT_TRUE(fixture.good());
  std::string line;
  ASSERT_TRUE(static_cast<bool>(std::getline(fixture, line)));
  std::size_t cases = 0;
  while (std::getline(fixture, line)) {
    if (line.empty()) {
      continue;
    }
    const auto fields = split(line);
    ASSERT_EQ(fields.size(), 24U) << line;
    control_cpp::ShadowRequestRecord request{
      u64(fields, 1), u64(fields, 2), u64(fields, 3),
      u64(fields, 4), u64(fields, 5), u64(fields, 6), 0};
    control_cpp::ShadowDecisionRecord decision;
    decision.schema_version = static_cast<std::uint32_t>(u64(fields, 7));
    decision.shell_epoch = u64(fields, 8);
    decision.request_sequence = u64(fields, 9);
    decision.target_revision = u64(fields, 10);
    decision.device_sequence = u64(fields, 11);
    decision.marker_sequence = u64(fields, 12);
    decision.manager_sequence = u64(fields, 13);
    decision.valid = u64(fields, 14) != 0U;
    decision.computation_start_steady_ns = i64(fields, 15);
    decision.computation_end_steady_ns = i64(fields, 16);
    decision.logical_velocity = {
      std::stod(fields.at(19)), 0.0, 0.0, 0.0, 0.0, 0.0};
    const control_cpp::ShadowValidationContext context{
      u64(fields, 20), u64(fields, 21), u64(fields, 22),
      i64(fields, 17), i64(fields, 18), 1000000};
    EXPECT_EQ(
      control_cpp::decision_rejection_reason(decision, &request, context),
      fields.at(23)) << fields.at(0);
    ++cases;
  }
  EXPECT_GE(cases, 10U);
}

TEST(ShadowContract, UnknownRequestFailsClosed)
{
  control_cpp::ShadowDecisionRecord decision;
  decision.schema_version = control_cpp::kShadowSchemaVersion;
  const control_cpp::ShadowValidationContext context{1, 0, 0, 1, 1, 1};
  EXPECT_EQ(
    control_cpp::decision_rejection_reason(decision, nullptr, context),
    "request_unknown");
}

}  // namespace
