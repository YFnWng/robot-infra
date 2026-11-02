#pragma once
#include "streamed_position_executor.h"

namespace StreamedPosition {
constexpr size_t kEnvelopeSize = kPayloadSize + 8;
struct Envelope {
  uint32_t session_id = 0;
  uint32_t remaining_us = 0;
  Command command;
};
inline bool decodeEnvelope(const uint8_t* p, size_t size, Envelope& output) {
  if (p == nullptr || size != kEnvelopeSize) return false;
  Envelope candidate;
  candidate.session_id = read32(p);
  candidate.remaining_us = read32(p+4);
  if (!decode(p+8, kPayloadSize, candidate.command) || candidate.session_id == 0 ||
      candidate.remaining_us == 0 || candidate.remaining_us > candidate.command.duration_us)
    return false;
  output = candidate;
  return true;
}

class Session {
 public:
  explicit Session(const Executor::Config& config, uint32_t seed = 0)
      : config_(config), executor_(config), generation_(seed) {}
  uint32_t open(bool verified_stop, uint32_t nonce = 0) {
    if (!verified_stop) return 0;
    if (nonce != 0 && nonce == id_) return 0;
    if (nonce != 0) generation_ = nonce;
    else ++generation_;
    if (generation_ == 0) ++generation_;
    executor_ = Executor(config_);
    id_ = generation_;
    return id_;
  }
  uint32_t id() const { return id_; }
  bool submit(const Envelope& envelope, uint32_t now_us, const int32_t* counts,
              bool feedback_valid, uint8_t allowed_mask,
              Executor::Admission& admission) {
    if (id_ == 0 || envelope.session_id != id_ || envelope.remaining_us == 0 ||
        envelope.remaining_us > envelope.command.duration_us) return false;
    // Transport age shortens authority; duplicate heartbeats cannot move the
    // M1 acceptance deadline forward because submit ignores their start time.
    const uint32_t start = now_us-(envelope.command.duration_us-envelope.remaining_us);
    admission = executor_.submit(envelope.command, start, counts, feedback_valid, allowed_mask);
    return true;
  }
  void cancel() { executor_.cancel(); }
  void nativeStarted(uint8_t mask) { executor_.nativeStarted(mask); }
  bool diagnosticLiveUpdate(uint32_t command) { return executor_.diagnosticLiveUpdate(command); }
  bool expired(uint32_t now_us) const { return executor_.expired(now_us); }
  Executor::Update update(uint32_t now_us, const int32_t* counts,
                          const int16_t* applied, bool feedback_valid,
                          uint8_t allowed_mask) {
    return executor_.update(now_us, counts, applied, feedback_valid, allowed_mask);
  }
 private:
  Executor::Config config_;
  Executor executor_;
  uint32_t generation_;
  uint32_t id_ = 0;
};
}  // namespace StreamedPosition
