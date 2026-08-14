#include "../teensy_tekceleo/driver_ack.h"

#include <assert.h>
#include <stdint.h>

namespace {

void test_bare_ack_is_valid() {
  const uint8_t response[] = {'!'};
  assert(DriverAck::classify(response, sizeof(response)) == DriverAck::NONE);
}

void test_echo_and_ok_payloads_are_valid() {
  const uint8_t echo[] = {'!', 'S', '1'};
  const uint8_t ok[] = {'!', 'O', 'K', '\r'};
  assert(DriverAck::classify(echo, sizeof(echo)) == DriverAck::NONE);
  assert(DriverAck::classify(ok, sizeof(ok)) == DriverAck::NONE);
}

void test_explicit_errors_are_rejected() {
  const uint8_t error[] = {'!', 'E', 'R', 'R', 'O', 'R'};
  const uint8_t nack[] = {'!', 'n', 'a', 'c', 'k'};
  const uint8_t question[] = {'!', '?'};
  assert(DriverAck::classify(error, sizeof(error)) ==
         DriverAck::EXPLICIT_REJECTION);
  assert(DriverAck::classify(nack, sizeof(nack)) ==
         DriverAck::EXPLICIT_REJECTION);
  assert(DriverAck::classify(question, sizeof(question)) ==
         DriverAck::EXPLICIT_REJECTION);
}

void test_malformed_response_is_rejected() {
  const uint8_t response[] = {'O', 'K'};
  assert(DriverAck::classify(response, sizeof(response)) ==
         DriverAck::MALFORMED_RESPONSE);
}

}  // namespace

int main() {
  test_bare_ack_is_valid();
  test_echo_and_ok_payloads_are_valid();
  test_explicit_errors_are_rejected();
  test_malformed_response_is_rejected();
  return 0;
}
