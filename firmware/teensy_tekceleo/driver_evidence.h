#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>
namespace DriverEvidence {
inline char lowerAscii(char c) { return c >= 'A' && c <= 'Z' ? c+('a'-'A') : c; }
inline bool prefixMatches(const char* text, const char* prefix) {
  for (size_t i=0; prefix[i]; ++i)
    if (!text[i] || lowerAscii(text[i]) != lowerAscii(prefix[i])) return false;
  return true;
}
inline bool validReadback(const char* text, const char* prefix) {
  if (!text || !prefix) return false;
  size_t begin=strlen(prefix);
  if (!prefixMatches(text,prefix)) {
    if (!prefixMatches(prefix,"speed:") || !prefixMatches(text,"vitesse:")) return false;
    begin=8;
  }
  const char* p=text+begin;
  while (*p==' ' || *p=='\t') ++p;
  if (*p=='+' || *p=='-') ++p;
  if (*p<'0' || *p>'9') return false;
  while (*p>='0' && *p<='9') ++p;
  if (*p=='.') {
    ++p;
    if (*p<'0' || *p>'9') return false;
    while (*p>='0' && *p<='9') ++p;
  }
  if (*p=='\r') ++p;
  return *p==0;
}
constexpr uint8_t kBytes = 24;
struct Record {
  uint32_t sequence = 0, time_us = 0, dropped = 0;
  uint8_t axis = 0, phase = 0, length = 0;
  uint8_t bytes[kBytes] = {};
};
// Fixed memory, no allocation/USB I/O in UART capture. Phase 0=TX, 1=reply,
// 2=discarded input, 3=partial reply. Full/oversize records are explicitly lost.
class Buffer {
 public:
  void capture(uint32_t time, uint8_t axis, uint8_t phase, const void* data, size_t size) {
    if (!size) return;
    ++sequence_;
    if (size > kBytes || count_ == 64) { ++dropped_; return; }
    Record& r = records_[(head_+count_)%64];
    r.sequence=sequence_; r.time_us=time; r.dropped=dropped_;
    r.axis=axis; r.phase=phase; r.length=static_cast<uint8_t>(size);
    memcpy(r.bytes, data, size); ++count_;
  }
  bool pop(Record& r) {
    if (!count_) return false;
    r=records_[head_]; r.dropped=dropped_; head_=(head_+1)%64; --count_; return true;
  }
 private:
  Record records_[64]; uint8_t head_=0, count_=0;
  uint32_t sequence_=0, dropped_=0;
};
}
