#include "../teensy_tekceleo/driver_evidence.h"
#include <cassert>
#include <cstring>
int main() {
  assert(DriverEvidence::validReadback("angle:338.085","Angle:"));
  assert(DriverEvidence::validReadback("ANGLE: -338.085\r","Angle:"));
  assert(DriverEvidence::validReadback("Speed:0.000","speed:"));
  assert(DriverEvidence::validReadback("VITESSE:+36.0","speed:"));
  const char* invalid[]={"!", "!OK", "angle:NaN", "angle:", "angle:1.2junk", "angle:1.", "angle:.2"};
  for (const char* bad : invalid)
    assert(!DriverEvidence::validReadback(bad,"Angle:"));
  assert(!DriverEvidence::validReadback("angle:3","speed:"));
  DriverEvidence::Buffer b; DriverEvidence::Record r;
  b.capture(123,0,0,"$A4.5\n",6);
  assert(b.pop(r) && r.sequence==1 && r.time_us==123 && r.length==6);
  assert(std::memcmp(r.bytes,"$A4.5\n",6)==0 && !b.pop(r));
  for (int i=0;i<65;++i) b.capture(i,0,0,"$O0\n",4);
  int n=0; while(b.pop(r)) { assert(r.dropped==1); ++n; }
  assert(n==64);
  char oversized[25]={}; b.capture(1,0,1,oversized,25);
  b.capture(2,0,1,"!",1); assert(b.pop(r) && r.dropped==2);
}
