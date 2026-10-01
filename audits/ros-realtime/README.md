# Catheter ROS real-time audit

Status: static architecture, passive live audits, and bounded non-actuating
shadow-planning audits are complete. The first 180-second post-remediation
shadow audit verifies the F-004/F-007 planner-decoupling result and the F-005
PyTorch cap; F-011 and the estimator-owner overrun tail need follow-up as noted
below. Actuated command/stop timing remains unmeasured; command output stayed
disabled throughout the audits. F-001/F-003 source remediation is implemented;
reflash and powered stop-latency fault injection are still required.

## Executive result

The previously observed feedback/reference fault is now fail-closed and passed
post-reflash passive verification. The next hardware gate is the newly
implemented stop/transport contract: motion commands are source-expiring and
latest-only, and firmware has a 250 ms valid-motion-command watchdog. Do not
begin routine closed-loop output until the new firmware is flashed and the
mechanically safe stop-latency fault injection passes.

The recurring marker and planner failures also have credible structural causes:
the marker-future tolerance equals one encoder timer period, marker rewind/replay
serializes against encoder advance, and several unconstrained ROS/PyTorch/OpenCV
thread pools share one 20-logical-CPU host. The disarmed live run used about
2.5 aggregate CPU cores and did not establish CPU saturation, so planner-load
The instrumented shadow run identified marker correction as the dominant
runtime critical section: it held the shared lock for up to 96.5 ms, delaying
encoder recurrence up to 87.7 ms and planner snapshots up to 52.1 ms. It also
reproduced `observation_from_future` and an `estimator_degraded` fault. Raw
transport pairing stayed below 1.6 ms, ruling out device pairing as the source.

After remediation, planner snapshot wait was 0.00024/0.00061/0.00089/0.134 ms
P50/P95/P99/max, all 1,801 estimator samples were `TRACKING`, and all marker
updates were accepted. Planner callback-window maxima improved from
46.4/67.7/74.4/86.6 ms to 19.0/32.8/38.9/72.9 ms. The remaining long tail is
inside the single estimator owner: marker correction itself remains about
35.5/46.4/52.4/93.1 ms and causes estimator timer lateness up to 87.8 ms.

Post-fix EKF and UKF both ran the active shadow planner without estimator faults
or marker rejections. Under matched load, UKF reduced marker-owner P50/P95
window maxima from 25.8/38.3 ms to 16.5/29.6 ms and controller CPU from 86.9%
to 72.3% of one core. Rare tails remain unbounded: the UKF combined estimator
callback still reached 112.4 ms, and both filters had isolated planner deadline
misses. Both also expose uncalibrated covariance in a usually rank-nine,
ten-dimensional estimator state.

Source remediation now restores the requested 20 Hz correction phase, exposes
true pre-update innovation statistics and covariance eigenvalues, prevents
process noise from accumulating in the instantaneous unobservable gauge,
identifies per-rig camera starvation, and decomposes marker and MPPI work into
phase timings. Unit tests and package builds pass; the subsequent non-actuating
shadow capture results are summarized below.

The post-remediation UKF capture verified 19.98 Hz accepted corrections,
continuous dual-rig tracking, zero marker rejection, and covariance trace
bounded to 7.28–8.31. The new phase metrics localized the remaining isolated
planner miss to a 90.98 ms learned rollout and showed marker tails split between
UKF correction and causal replay. Innovation NIS per degree of freedom remains
far below one, so statistical noise calibration is still open.

The subsequent 2/1-thread live run had no deadline miss: rollout maximum fell
to 40.83 ms, plan maximum to 43.05 ms, and marker-total maximum to 34.32 ms.
Controller threads fell from 64 to 46. The complete callback still reached
60.33 ms and the estimator owner still exceeded its 20 ms period, so these are
strong soft-real-time improvements rather than hard bounds.

## Priority order

1. F-014 gate remediation has passed post-reflash passive and shadow-planning
   verification. Keep command output disabled while resolving the startup
   power-domain mechanism and the critical stop/transport findings.
2. Reflash and fault-inject the implemented bounded stop contract (F-001).
3. Process-block/release test the implemented latest-only, expiration-aware
   velocity transport (F-003).
4. Extend the successful causal/snapshot shadow result to at least 10 minutes
   and include representative motion before enabling adaptation (F-004/F-007).
5. The 2/1 pool removed the observed 90.98 ms rollout tail. Retain the phase
   metrics and fault-inject deadline behavior before claiming a bound
   (F-005/F-011).
6. Design snapshot/validate/commit marker correction if the estimator owner's
   20 ms release contract must be met; its current heavy correction is still
   synchronous even though planning is decoupled (F-007).
7. F-016 is closed. Calibrate filter noise using dynamic recorded motion and
   the new true innovation statistics (F-018).
8. Fault-inject one camera worker and verify the new per-rig liveness and
   `PAIRING_STALE` diagnostics (F-017).
9. Size and isolate the remaining ROS/OpenCV pools; the PyTorch cap reduced
   controller threads from 127 to 46 but did not constrain vision (F-005).
10. Convert firmware UART work to a bounded state machine (F-006).

F-002 is an accepted project exception: the firmware zero opcode remains, but
normal host paths must continue rejecting it and it must never be invoked.

The earlier instrumentation identified the runtime-lock owner and is retained
for before/after remediation measurements. New status keys identify the
single estimator owner, snapshot exchange, snapshot age, and PyTorch limits.

## C++ and PREEMPT_RT decision

A wholesale Python-to-C++ rewrite is not the first move. C++ is most valuable
for the small paths that require bounded callback and command latency: final
command arbitration/expiry, serial transmit, timestamped estimator input
ordering, and possibly the fixed-rate command heartbeat. MPPI and learned-model
inference can remain Python/PyTorch in a lower-priority worker if command output
uses only fresh, deadline-qualified results. Rewriting without first fixing
queues, blocking firmware I/O, lock ownership, and thread oversubscription would
preserve the main failure mechanisms.

PREEMPT_RT can reduce kernel scheduling and interrupt latency, but it cannot fix
stale DDS queues, 10-second firmware logic, blocking driver transactions, the
runtime mutex, or native thread-pool oversubscription. Apply it only after the
application paths are bounded and instrumented; then compare identical
full-load p99/p99.9 and maximum latencies on the generic and RT kernels.

## Artifact map

- [findings.md](findings.md): prioritized defects, consequences, fixes, tests
- [evidence.md](evidence.md): source and runtime evidence index
- [workspace.md](workspace.md): scope and repository/process inventory
- [architecture.yaml](architecture.yaml): machine-readable architecture
- [control-sequence.md](control-sequence.md): causal sequence and timestamps
- [runtime-snapshot.md](runtime-snapshot.md): host and ROS graph state
- [timing-summary.csv](timing-summary.csv): configured budgets and unknowns
- `ros-topology.svg`: ROS/interface graph
- `execution-topology.svg`: executors, thread pools, and shared locks
- `control-paths.svg`: feedback-to-actuation and stop path
- `gating-model.svg`: controller readiness/fault progression
