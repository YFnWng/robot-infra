#pragma once
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

namespace NativePositionCommand {
// Tekceleo Protocol Guide p10: relative degrees, including one decimal place.
inline size_t format(char* output, size_t capacity, uint32_t deci_degrees) {
  if (!output || deci_degrees == 0 || deci_degrees > 99999) return 0;
  const int length = snprintf(output, capacity, "$A%lu.%u\n",
      static_cast<unsigned long>(deci_degrees/10),
      static_cast<unsigned>(deci_degrees%10));
  return length > 0 && static_cast<size_t>(length) < capacity
      ? static_cast<size_t>(length) : 0;
}
}  // namespace NativePositionCommand
