#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

// Immutable command payload inside the opt-in session envelope. Retain all
// manager/firmware gates. Never put this payload inside a legacy POS packet.
namespace StreamedPosition {
constexpr uint8_t kVersion = 1;
constexpr uint8_t kAxes = 6;
constexpr uint16_t kMaxRpm = 250;
constexpr uint32_t kMaxDurationUs = 250000;
constexpr size_t kPayloadSize = 46;

struct Command {
  uint32_t id = 0;
  uint32_t duration_us = 0;
  uint8_t active_mask = 0;
  int32_t target_counts[kAxes] = {};
  uint16_t speed_rpm[kAxes] = {};
};

inline bool valid(const Command& command) {
  if (command.id == 0 || command.duration_us == 0 ||
      command.duration_us > kMaxDurationUs || command.active_mask > 0x3f)
    return false;
  for (uint8_t axis = 0; axis < kAxes; ++axis) {
    const bool active = (command.active_mask & (1U << axis)) != 0;
    if (command.speed_rpm[axis] > kMaxRpm ||
        active != (command.speed_rpm[axis] != 0)) return false;
  }
  return true;
}

inline bool newerId(uint32_t candidate, uint32_t previous) {
  const uint32_t distance = candidate - previous;
  return candidate != 0 && previous != 0 && distance != 0 &&
         distance < 0x80000000U;
}

inline bool same(const Command& a, const Command& b) {
  if (a.id != b.id || a.duration_us != b.duration_us ||
      a.active_mask != b.active_mask) return false;
  for (uint8_t axis = 0; axis < kAxes; ++axis) {
    if (a.target_counts[axis] != b.target_counts[axis] ||
        a.speed_rpm[axis] != b.speed_rpm[axis]) return false;
  }
  return true;
}

inline uint32_t read32(const uint8_t* p) {
  return static_cast<uint32_t>(p[0]) |
         (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) |
         (static_cast<uint32_t>(p[3]) << 24);
}

inline void write32(uint8_t* p, uint32_t value) {
  for (uint8_t i = 0; i < 4; ++i) p[i] = static_cast<uint8_t>(value >> (8*i));
}

inline bool decode(const uint8_t* payload, size_t size, Command& output) {
  if (payload == nullptr || size != kPayloadSize || payload[0] != kVersion)
    return false;
  Command candidate;
  candidate.id = read32(payload + 1);
  candidate.duration_us = read32(payload + 5);
  candidate.active_mask = payload[9];
  for (uint8_t axis = 0; axis < kAxes; ++axis) {
    const uint32_t bits = read32(payload + 10 + axis*4);
    memcpy(&candidate.target_counts[axis], &bits, sizeof(bits));
    const uint8_t* speed = payload + 34 + axis*2;
    candidate.speed_rpm[axis] = static_cast<uint16_t>(speed[0]) |
                              static_cast<uint16_t>(speed[1] << 8);
  }
  if (!valid(candidate)) return false;
  output = candidate;  // Atomic: a rejected packet cannot partially update state.
  return true;
}

inline bool encode(const Command& command, uint8_t* payload, size_t size) {
  if (!valid(command) || payload == nullptr || size != kPayloadSize) return false;
  payload[0] = kVersion;
  write32(payload + 1, command.id);
  write32(payload + 5, command.duration_us);
  payload[9] = command.active_mask;
  for (uint8_t axis = 0; axis < kAxes; ++axis) {
    uint32_t bits;
    memcpy(&bits, &command.target_counts[axis], sizeof(bits));
    write32(payload + 10 + axis*4, bits);
    payload[34 + axis*2] = static_cast<uint8_t>(command.speed_rpm[axis]);
    payload[35 + axis*2] = static_cast<uint8_t>(command.speed_rpm[axis] >> 8);
  }
  return true;
}
}  // namespace StreamedPosition
