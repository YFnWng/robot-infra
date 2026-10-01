#include <cassert>
#include <cstdint>

#include "../teensy_tekceleo/motion_watchdog.h"

int main() {
  MotionWatchdog watchdog(250);
  assert(!watchdog.armed());
  assert(!watchdog.expired(1000));

  watchdog.refresh(1000);
  assert(watchdog.armed());
  assert(!watchdog.expired(1250));
  assert(watchdog.expired(1251));

  watchdog.refresh(1300);
  assert(!watchdog.expired(1500));
  watchdog.disarm();
  assert(!watchdog.expired(10000));

  // Unsigned subtraction preserves the timeout across millis() wraparound.
  watchdog.refresh(UINT32_MAX - 100);
  assert(!watchdog.expired(49));
  assert(watchdog.expired(200));
  return 0;
}
