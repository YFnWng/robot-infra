#include "../teensy_tekceleo/streamed_position_executor.h"
#include "../teensy_tekceleo/native_position_command.h"
#include "../teensy_tekceleo/driver_ack_retry.h"
#include <cassert>
#include <cstring>

using E = StreamedPosition::Executor;
using C = StreamedPosition::Command;
static E::Config config() {
  E::Config c;
  c.native_position = true;
  c.direction_settle_us = 100000;
  for (int a=0;a<6;++a) {
    c.lower_counts[a]=-10000; c.upper_counts[a]=10000;
    c.maximum_step_counts[a]=1000; c.tolerance_counts[a]=67;
    c.minimum_rpm[a]=36; c.maximum_rpm[a]=250;
  }
  return c;
}
static C command(uint32_t id=1, int32_t target=400) {
  C c; c.id=id; c.active_mask=1; c.duration_us=250000;
  c.target_counts[0]=target; c.speed_rpm[0]=36; return c;
}
int main() {
  E e(config()); int32_t counts[6]={}; int16_t applied[6]={};
  C c=command();
  assert(e.submit(c,0,counts,true,63)==E::Admission::ACCEPTED);
  auto u=e.update(0,counts,applied,true,63);
  assert(u.native_start_mask==1 && u.native_deci_degrees[0]==180);
  e.nativeStarted(1); applied[0]=36; counts[0]=350;
  u=e.update(10000,counts,applied,true,63);
  assert(u.status==E::Status::RUNNING && !u.stop_mask && !u.native_start_mask);
  counts[0]=400; e.update(20000,counts,applied,true,63);
  assert(e.update(119999,counts,applied,true,63).status==E::Status::RUNNING);
  u=e.update(120000,counts,applied,true,63);
  assert(u.status==E::Status::COMPLETE && u.stop_mask==1);

  E replacement(config()); counts[0]=0; applied[0]=0;
  replacement.submit(command(),0,counts,true,63);
  replacement.update(0,counts,applied,true,63); replacement.nativeStarted(1);
  applied[0]=36; counts[0]=100; replacement.update(10000,counts,applied,true,63);
  replacement.submit(command(2,600),20000,counts,true,63);
  u=replacement.update(20000,counts,applied,true,63);
  assert(u.stop_mask==1 && !u.native_start_mask);
  applied[0]=0; counts[0]=130; replacement.update(30000,counts,applied,true,63);
  replacement.submit(command(3,700),50000,counts,true,63);
  assert(!replacement.update(129999,counts,applied,true,63).native_start_mask);
  u=replacement.update(130000,counts,applied,true,63);
  assert(u.native_start_mask==1 && u.native_deci_degrees[0]==257);
  replacement.nativeStarted(1); applied[0]=36;
  assert(replacement.submit(command(3,700),200000,counts,true,63)==E::Admission::HEARTBEAT);
  u=replacement.update(300000,counts,applied,true,63);
  assert(u.status==E::Status::EXPIRED && u.stop_mask==1 && !u.native_start_mask);

  char output[16];
  assert(NativePositionCommand::format(output,sizeof(output),257)==7);
  assert(std::strcmp(output,"$A25.7\n")==0);
  assert(!NativePositionCommand::format(output,2,257));
  assert(!NativePositionCommand::format(output,sizeof(output),100000));
  bool cancelled=false; int calls=0, waits=0;
  assert(!DriverAckRetry::run(3,[&](){++calls;cancelled=true;return false;},
      [&](uint8_t){++waits;},nullptr,&cancelled));
  assert(calls==1 && waits==0);

  E diagnostic(config()); counts[0]=0; applied[0]=0;
  diagnostic.submit(command(),0,counts,true,63);
  assert(!diagnostic.diagnosticLiveUpdate(1)); // cannot update before native enable
  diagnostic.update(0,counts,applied,true,63); diagnostic.nativeStarted(1);
  applied[0]=36;
  assert(!diagnostic.diagnosticLiveUpdate(2));
  assert(diagnostic.diagnosticLiveUpdate(1));
  assert(!diagnostic.diagnosticLiveUpdate(1)); // never retry A
  counts[0]=600;
  assert(diagnostic.update(100000,counts,applied,true,63).status==E::Status::RUNNING);
  assert(diagnostic.update(250000,counts,applied,true,63).status==E::Status::EXPIRED);
  assert(!diagnostic.diagnosticLiveUpdate(1));
}
