#pragma once

#include <stddef.h>
#include <stdint.h>

// Arduino-independent Tekceleo acknowledgement classification. The transport
// layer captures bytes from the leading '!' through (but excluding) LF and
// passes the bounded result here for validation and diagnostics.
namespace DriverAck {

constexpr uint8_t MAX_RESPONSE_BYTES = 8;

enum Failure : uint8_t {
  NONE = 0,
  TIMEOUT = 1,
  PARTIAL_FRAME = 2,
  RESPONSE_OVERFLOW = 3,
  EXPLICIT_REJECTION = 4,
  MALFORMED_RESPONSE = 5,
};

struct Result {
  bool success = false;
  Failure failure = TIMEOUT;
  uint8_t attempts = 0;
  uint8_t response_length = 0;
  uint8_t response[MAX_RESPONSE_BYTES] = {0};
};

inline uint8_t asciiUpper(uint8_t value) {
  return value >= 'a' && value <= 'z' ? value - ('a' - 'A') : value;
}

inline bool startsWithIgnoreCase(
    const uint8_t* value, size_t length, const char* prefix) {
  size_t index = 0;
  while (prefix[index] != '\0') {
    if (index >= length || asciiUpper(value[index]) !=
                               asciiUpper(static_cast<uint8_t>(prefix[index]))) {
      return false;
    }
    ++index;
  }
  return true;
}

inline Failure classify(const uint8_t* response, size_t length) {
  if (length == 0 || response[0] != '!') {
    return MALFORMED_RESPONSE;
  }

  size_t begin = 1;
  while (begin < length &&
         (response[begin] == ' ' || response[begin] == '\t')) {
    ++begin;
  }
  size_t end = length;
  while (end > begin &&
         (response[end - 1] == '\r' || response[end - 1] == ' ' ||
          response[end - 1] == '\t')) {
    --end;
  }
  if (begin == end) {
    return NONE;  // The documented bare acknowledgement: !<LF>.
  }

  const uint8_t* payload = response + begin;
  const size_t payload_length = end - begin;
  if (payload[0] == '?' || payload[0] == '-' ||
      startsWithIgnoreCase(payload, payload_length, "ERR") ||
      startsWithIgnoreCase(payload, payload_length, "NACK") ||
      startsWithIgnoreCase(payload, payload_length, "INVALID")) {
    return EXPLICIT_REJECTION;
  }

  // Some controller revisions echo the command or return an informational
  // payload after '!'. Retain that payload for diagnostics and accept it unless
  // it explicitly reports rejection above.
  return NONE;
}

}  // namespace DriverAck
