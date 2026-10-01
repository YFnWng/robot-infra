# ROS real-time and safety audit findings

Severity reflects potential catheter-motion impact, not just software quality.
Remediation status is stated explicitly where implementation has begun; no
hardware validation is implied by a source-level fix.

## F-001 — Critical — Firmware may preserve the last velocity for 10 seconds

**Confidence:** observed.

The final firmware watchdog is `PC_SILENCE_MS = 10000` and only then stops all
axes. It is reset when any frame has valid start/length/end framing, before the
command is semantically validated. If the host control path dies while motion
is active, the last firmware velocity may persist for nearly 10 seconds; other
framed traffic can also keep the watchdog alive.

**Recommendation:** Design and test a layered stop budget, then reduce the
firmware watchdog to a value supported by measured worst-case host/USB jitter
(initial engineering target: 100–250 ms, not a blind configuration change).
Reset it only for an allow-listed set of valid liveness/control messages. Keep
the manager's explicit-zero watchdog as the first layer.

**Verification:** With motors mechanically safe and unloaded, inject manager,
serial-process, USB, and whole-host failures separately; measure command-to-zero
latency at firmware and driver outputs. Confirm malformed/unsupported frames do
not refresh motion authority.

**Remediation implemented; powered verification pending:** The firmware motion
watchdog is now 250 ms and is refreshed only after a semantically valid `VEL`
or `POS` frame. Generic framed traffic no longer extends motion authority. On
expiry, firmware writes `O0` directly to all six independent driver UARTs,
clears every pending motion state, and checks the watchdog inside driver-ACK
waits as well as the main loop. The timeout state machine has an
Arduino-independent wraparound regression. This establishes the source-level
contract, but the physical command-to-disabled latency must still be measured
after reflashing before routine control output.

## F-002 — Critical — Firmware still implements the forbidden encoder-zero opcode

**Confidence:** observed.

The project invariant is “never send `SET_ZERO`,” and host layers reject it, but
firmware `case ZERO` writes encoder hardware counts and resets local state. Any
direct serial client or future host regression can invalidate the model/training
coordinate alignment.

**Disposition:** Accepted project exception. Per operator direction, leave the
firmware implementation unchanged. Keep host rejection as defense in depth and
retain the non-negotiable rule that normal startup, recovery, testing, and
audit paths never invoke it.

**Verification:** Host-layer tests must continue proving that normal ROS manager
and serial-bridge paths reject zeroing. Do not test the retained firmware opcode
against the calibrated hardware.

## F-003 — High — Reliable command queues can replay stale motion as fresh

**Confidence:** inferred-high-confidence (mechanism observed; live delay unknown).

Both `/teleop/control` and `/manager/control` use reliable keep-last depth 10.
The manager ignores the incoming header time, marks source freshness at callback
receipt, and re-stamps the forwarded message. The serial bridge also performs no
command-age check. At 100 Hz, either queue can retain about 100 ms of commands,
and executor blockage can add more. Old queued velocities can therefore be
forwarded and can postpone the manager source watchdog.

**Recommendation:** Make motion commands latest-only (depth 1, with an explicit
QoS choice) and carry a monotonic sequence/deadline or source stamp through the
manager. Reject expired or non-monotonic commands at both manager and serial
bridge. A zero/stop must preempt queued nonzero commands.

**Verification:** Deliberately block each consumer, release it, and prove that no
old nonzero command reaches serial. Record source stamp, manager receipt,
manager publish, serial receipt, and firmware apply times.

**Remediation implemented; live verification pending:** Both motion topics now
use explicit reliable keep-last depth 1. The manager preserves the producer's
ROS timestamp and rejects missing, expired, future, or non-increasing nonzero
commands without refreshing its source watchdog. The serial bridge repeats the
same check under a serialized transmit lock. Delayed zero velocity remains an
unconditional preemption. Firmware drains all complete queued host frames before
one motor-control cycle, so intermediate USB-queued targets are not actuated.
Unit fault-injection tests cover expiry, ordering, timestamp preservation, and
stale-zero preemption; a process-block/release run remains required.

## F-004 — High — Estimator timing/ordering causes future-observation faults

**Confidence:** observed.

Encoder recurrence runs at 50 Hz, while marker observations use camera time. A
marker is rejected when it is more than 20 ms newer than current estimator
state—exactly one nominal encoder timer period, leaving no margin for executor,
serial, or camera jitter. Marker fusion timestamps the spatial average with the
newest rig time. This structure matches the previously observed
`observation_from_future` and marker-degradation faults.

The 180-second instrumented shadow run reproduced the failure chain: two
`observation_from_future` marker updates, two degraded estimator samples, one
controller `DEGRADED` status, and one `FAULTED` status with reason
`estimator_degraded`. Encoder runtime-lock wait reached 87.700 ms and encoder
timer-start lateness reached 74.750 ms, both far beyond the 20 ms future-time
tolerance. Raw device delivery remained prompt. Nominal inputs can therefore
become causally misordered inside the controller.

**Recommendation:** Advance estimator state from the newest causal encoder data
before correction, or schedule correction from an ordered timestamp buffer.
Define one explicit clock/latency contract for camera and encoder observations.
Treat tolerance enlargement only as a temporary diagnostic, not the primary
fix.

**Verification:** Trace estimator-state time minus marker time at correction
entry, with executor-ready delay and encoder arrival time. Run at least 10
minutes and across CPU/GPU cold paths; require zero future/too-old rejections in
nominal operation.

**Remediation implemented; initial live verification passed:** Device and marker
subscriptions now only replace pending inputs. One 50 Hz callback is the sole
runtime owner; it advances the newest encoder first and defers a future marker
until estimator time reaches the image timestamp. It drains encoder once more
after correction. This removes the competing callback ordering mechanism
without widening the runtime's timestamp tolerance. The 180-second
post-remediation run had zero future/old marker rejections, all 1,801 estimator
samples `TRACKING`, and no degraded/faulted controller state. The specified
10-minute and representative-motion verification remains open.

## F-005 — High — Unbounded native-thread competition threatens tail latency

**Confidence:** inferred.

The controller has eight ROS executor threads while PyTorch is configured for
12 intra-op and 12 inter-op threads. Marker tracking has a ROS thread, two
capture threads, four Python workers, and OpenCV configured for 20 threads.
Manager and serial add eight executor threads plus serial RX. These pools share
20 logical CPUs without affinity, budgets, or thread-count configuration.
Average MPPI time can remain low while cold-path or contention tails miss gates
and deadlines.

The passive disarmed run observed 82 marker threads and 39 controller threads,
all unrestricted across CPUs 0–19. Aggregate application use was only about 2.5
cores, however, so CPU saturation was not demonstrated and planning load was
absent.

The first bounded shadow-planning run captured 133 `ACTIVE` diagnostic samples.
Reported solve time was 22.082 ms P50, 29.059 ms P95, 33.465 ms P99, and
34.508 ms maximum, with zero deadline misses against the 80 ms configured
deadline. The monitor failed to resolve the controller PID, so controller CPU,
thread, and context-switch changes were not captured. This run weakens the
hypothesis that ordinary warm MPPI computation alone is missing its deadline,
but it does not cover cold starts, long-duration tails, or simultaneous fault
logging.

The longer instrumented run resolved the controller process and observed its
thread count range from 39 to 127. It averaged 109.4% of one
CPU across a window active for about two thirds of its duration, used every
logical CPU, and accumulated 4,340 involuntary plus 262,450 voluntary context
switches. This proves substantial pool expansion and migration, but not host
CPU saturation or that native-thread competition caused the measured lock
waits.

**Recommendation:** First bound PyTorch/OpenCV threads and isolate processes or
callback roles by measured CPU demand. Use separate processes for vision and
control, explicit affinity, and priority only after validating that every
high-priority path is bounded and nonblocking.

**Verification:** Trace under worst-case simultaneous vision, marker correction,
planning, logging, and serial traffic. Compare p50/p95/p99/p99.9 callback-ready
and execution time before and after thread caps.

**Partial remediation implemented and observed live:** The controller
now configures PyTorch to four intra-op and one inter-op thread before loading
the model. Diagnostics confirmed 4/1, and controller maximum thread count fell
from 127 to 66; CPU use and context switching also decreased. ROS executor
sizing and marker/OpenCV pools remain unchanged, all processes still migrated
across all 20 CPUs, and marker tracking still used about 82 threads.

**Second-stage remediation implemented; live verification pending:** An
isolated 500-plan sweep showed negligible benefit from 2–4 threads, but a
same-process concurrent planner/UKF benchmark exposed shared-pool contention.
Rollout P50/P95/P99/max was 13.60/27.60/31.84/41.26 ms with four intra-op
threads, versus 11.89/22.66/25.01/26.31 ms with two and
12.02/22.46/24.83/27.67 ms with one. The default is now two intra-op and one
inter-op thread. This is the smallest supported cap; process isolation and
vision-pool sizing remain open if the live rollout tail persists.

**Live 2/1 verification:** No deadline miss occurred. Rollout P99/max was
36.014/40.829 ms, versus the prior 90.978 ms maximum, and marker-total maximum
fell from 60.434 to 34.322 ms. Controller thread count fell from 64 to 46.
Aggregate controller CPU unexpectedly rose from 80.2% to 130.0% of one core,
so the sequential sessions do not establish a CPU-efficiency improvement.
Both controller and vision still migrated across all 20 CPUs and vision kept
82 threads; F-005 remains open for whole-host isolation even though the
specific PyTorch cap improved latency tails.

## F-006 — High — Firmware motor-driver transactions block feedback and watchdog work

**Confidence:** observed.

The firmware loop performs encoder reads and `sendMotorCmds()` inline. Driver
commands flush UARTs, delay, and can wait 50 ms per acknowledgement with up to
three attempts and UART recovery. Direction changes and coordinated starts span
multiple axes. During these operations, host receive processing, feedback, and
the firmware watchdog do not progress.

**Recommendation:** Refactor driver transactions into a bounded nonblocking
state machine, prioritize host RX/watchdog evaluation, and publish cycle-overrun
telemetry. Avoid per-cycle configuration when a cached setting is unchanged.

**Verification:** Instrument maximum loop gap, encoder-report gap, PC-RX gap,
and watchdog-check gap for steady velocity, speed changes, reversals, missing
driver ACKs, and one-axis faults.

**Partial safety remediation:** Driver transactions remain synchronous, so F-006
is not closed. The critical stop path no longer waits for those transactions:
the motion watchdog is polled inside each ACK wait and broadcasts unacknowledged
`O0` on all driver UARTs before clearing motion state. Full conversion to a
nonblocking transaction state machine and cycle-gap telemetry remain follow-up
work after the bounded-stop hardware test.

## F-007 — High — Marker correction stalls encoder state and planner snapshots

**Confidence:** observed for contention; inferred-high-confidence for exact
peak-to-fault temporal attribution.

Marker Gauss–Newton correction plus delayed rewind/replay holds the same runtime
mutex used by encoder recurrence and planner-state cloning. A slow correction
can leave estimator time behind, make subsequent marker timestamps appear from
the future, delay planning, and produce gate failures. Separate ROS callback
groups do not remove this mutex coupling.

The shadow run strengthened the evidence for an internal controller delay while
leaving its exact owner unresolved. Raw observer POS/ENC arrival skew stayed at
0.213/0.490/0.702/1.156 ms P50/P95/P99/max, but controller-reported feedback
skew rose from the passive run's 10.618/21.074/21.501/65.990 ms to
11.017/21.934/54.361/87.493 ms. Reported encoder age similarly rose from
19.714/25.112/27.659/70.564 ms to 21.864/29.444/60.306/95.001 ms. The raw
transport did not exhibit a corresponding tail, so device pairing is not the
source of this discrepancy. Aggregate diagnostics cannot distinguish executor
wait from `_runtime_lock` contention or timer phase.

The instrumented run resolves that ambiguity. Marker correction held
`_runtime_lock` for 36.926/45.883/51.489/96.469 ms P50/P95/P99/max of the
per-diagnostic-window maxima. Encoder recurrence then waited
26.648/41.926/48.537/87.700 ms, while planner snapshots waited
22.160/39.107/44.279/52.142 ms. Planner snapshot hold itself was small
(1.362 ms P95 and 3.390 ms max), so cloning is not the dominant critical
section. Encoder callback duration reached 90.856 ms and timer-start lateness
reached 74.750 ms. These measurements establish shared-lock contention; the
aggregate monitor does not preserve the event-level correlation needed to
pair each absolute peak with the exact future-observation fault.

**Recommendation:** Move correction to a snapshot/commit design or a dedicated
ordered estimator worker so callbacks only enqueue latest timestamped inputs.
Give planning an immutable recent state without waiting on full rewind/replay.

**Verification:** Trace runtime-lock owner, hold duration, waiter duration, and
the resulting estimator timestamp lag. Establish a bounded correction budget.

**Remediation direction:** Replace competing estimator timers with one causal,
timestamp-ordered owner that drains encoder state through the marker timestamp
before correction and publishes an immutable recent planner snapshot. Do not
let MPPI wait on marker rewind/replay. If correction remains too long, move it
to snapshot/validate/commit semantics with a generation check.

**Remediation implemented; planner decoupling verified live:** The single estimator
owner publishes deep-cloned, replace-only states through a dedicated snapshot
exchange lock. Planning reads this clone and no longer acquires or shares a
mutable-runtime lock. A long correction may still make estimator feedback
stale, but it cannot block an already published planner snapshot. P95 planner
snapshot wait fell from 39.107 ms to 0.00061 ms and P95 plan-callback duration
fell from 67.664 ms to 32.792 ms.

The remaining issue is now isolated rather than removed: marker correction P95
was 46.371 ms and the combined estimator-owner callback reached 107.173 ms,
against a 20 ms timer period. This produced timer lateness up to 87.849 ms and
increased typical committed-marker age, although no freshness gate failed.
Snapshot/validate/commit correction is the next design step if a bounded 50 Hz
estimator recurrence is required.

**Matched filter comparison:** The post-fix EKF and replacement UKF sessions
each contained 1,201 `ACTIVE` samples with command output disabled, continuous
estimator `TRACKING`, and no marker rejection. UKF reduced marker-owner
P50/P95 window maxima from 25.797/38.262 ms to 16.540/29.619 ms and combined
estimator callback maxima from 31.788/47.268 ms to 21.732/37.901 ms. Controller
process CPU fell from 86.9% to 72.3% of one core. UKF therefore has the better
typical compute profile under matched shadow-planning load.

Neither filter bounds the 20 ms estimator-owner release: UKF estimator
P99/max remained 53.792/112.414 ms, nearly the same absolute maximum as EKF.
Plan-callback window P50 improved from 22.704 to 16.423 ms, but UKF P95 was
slightly worse (36.193 versus 33.904 ms); both sessions had isolated deadline
misses and maxima above 80 ms without a repeated-miss fault. The aggregate
status samples are insufficient to correlate the rare estimator and planner
peaks.

Marker updates now expose rewind, correction, replay, and total phase times.
This closes the instrumentation gap in source; a post-change shadow capture is
still required before deciding whether snapshot/validate/commit correction is
worth its additional concurrency and generation-validation complexity.

**Post-remediation measurement:** The UKF marker path measured
1.075/2.750/5.540/7.349 ms for rewind, 8.428/15.089/19.028/31.574 ms for filter
correction, 7.161/12.944/16.612/26.016 ms for replay, and
16.401/28.380/32.902/60.434 ms total at P50/P95/P99/max. Typical work is now
localized and acceptable for a 20 Hz correction release, but the 60.4 ms total
still overruns the 20 ms estimator-owner timer. Both correction and replay
contribute to the tail; moving only the filter math would not bound recurrence.

With the 2/1 PyTorch cap, marker-total P50/P95/P99/max improved to
12.633/26.866/31.391/34.322 ms and estimator-callback maximum fell to
61.069 ms. The 20 ms owner period is still exceeded at P95, so recurrence is
not bounded, but the prior 60–83 ms marker/owner tails were reduced.

## F-008 — Medium — Mode claim uses elapsed time rather than acknowledgement

**Confidence:** observed.

The controller publishes `JOINT_VEL`, sets `mode_claimed = True`, waits a fixed
100 ms, and then may command. The manager does not provide a correlated mode
acceptance response. If manager execution is delayed or motion is inhibited,
the controller and manager can disagree about authority.

**Recommendation:** Add a request identifier and explicit accepted/rejected mode
state from the manager. Enter command-enabled state only after a fresh matching
acceptance, and revoke it on manager epoch/restart changes.

**Verification:** Delay and restart the manager around mode acquisition; prove
no command is enabled before acknowledgement and that rejection remains safely
recoverable rather than ambiguous.

## F-009 — Medium — Marker processing can block its own watchdog

**Confidence:** observed.

Marker tracking uses the single-threaded default ROS executor. Its processing
timer waits for image work and processes the two rigs sequentially. Its watchdog
timer shares that executor, so slow or hung processing also delays the component
that should report the stall. The controller's independent diagnostic timeout
eventually mitigates this, but only downstream.

**Recommendation:** Keep the ROS callbacks bounded: enqueue work to a dedicated
worker and publish completed generations, with the watchdog on an independent
callback group/executor or process. Add per-rig processing deadlines.

**Verification:** Stall one processing worker and confirm diagnostics remain on
schedule and downstream command output reaches zero within the safety budget.

## F-010 — Medium — Single-rig fallback is reported as normal tracking

**Confidence:** observed.

`minimum_valid_rigs` defaults to 1. When rig timestamp skew exceeds its limit,
the newest rig alone can be used; the marker estimate may still be published as
`TRACKING`. This silently changes observability and accuracy while the controller
continues accepting feedback.

**Current runtime:** Mitigated for the audited launch, which explicitly used
`minimum_valid_rigs=2`. The unsafe default remains available to other launches.

**Recommendation:** For closed-loop hardware runs, require both qualified rigs
or publish a distinct degraded mode that the controller explicitly gates. Keep
single-rig operation only as an intentional, separately validated mode.

**Verification:** Disconnect or delay either rig and prove the controller
transitions to the intended safe state before commands continue.

## F-011 — Medium — Planner deadline is larger than its scheduling period

**Confidence:** observed.

The planner period is 66.7 ms (15 Hz), but its allowed deadline is 80 ms and is
checked only after the complete solve. A 70–80 ms solve is declared valid while
already exceeding its timer period, encouraging back-to-back planner work. No
in-flight computation is cancelled at the deadline.

Live instrumentation showed a second accounting gap: the complete plan
callback's per-window maximum was 46.406/67.664/74.363/86.648 ms
P50/P95/P99/max. It exceeded both its 66.7 ms period and, once, the nominal
80 ms deadline while reporting zero deadline misses. The planner's own maximum
recorded solve was 66.832 ms because runtime-snapshot wait and wrapper overhead
occur outside its deadline clock.

**Recommendation:** Define separate budgets for release period, complete
callback execution, and command freshness. Start deadline accounting before
snapshot acquisition. The execution budget must be comfortably below its
period and measured at a high percentile. Consider a worker with one latest
request and explicit completion deadline.

**Verification:** Inject controlled planning delays around 60–90 ms and verify
no callback storm, no stale command, and the intended repeated-miss behavior.

**Remediation implemented; runtime verification incomplete:** The deadline is 60 ms,
below the 66.7 ms period. Its clock starts at planner callback entry and is
checked again under the lifecycle lock immediately before command commit. An
overdue action is discarded and the existing zero/repeated-miss policy applies.
The post-remediation run reported zero deadline misses and improved plan
callback P95/P99 to 32.792/38.942 ms, but one complete callback reached
72.949 ms. The aggregate capture cannot determine whether this occurred before
planning, after command commit, or while disarmed. MPPI now reports separate
sample/projection, learned rollout, cost/weighting, and final projection times.
A post-change shadow capture plus injected-delay test remains necessary to
verify the whole callback contract.

**Post-remediation measurement:** One isolated miss remained. The failed plan
reported 92.390 ms total, including 90.978 ms in learned-model rollout; the
complete callback maximum was 93.040 ms. Sampling/projection, cost/weighting,
and final projection were each below 3.5 ms. The rare deadline failure is
therefore a rollout scheduling/compute tail, not MPPI sampling or limit
projection. Repeated-miss fail-closed behavior was not triggered in this run.

The concurrent offline benchmark reproduced thread-count sensitivity and
supports reducing the shared PyTorch intra-op pool from four to two. Under its
synthetic planner/UKF load, the rollout maximum fell from 41.26 to 26.31 ms.
Live verification under dual-camera load is still required; this does not yet
prove the 90.98 ms OS/native scheduling tail is eliminated.

**Live 2/1 verification passed for the observed window:** No deadline miss was
recorded. Plan P50/P95/P99/max was 25.326/33.123/39.205/43.050 ms; learned
rollout was 23.626/31.222/36.014/40.829 ms. The complete callback maximum was
60.334 ms, only 0.334 ms over the command-commit budget, while timer lateness
remained below 1.306 ms. This closes the reproduced 90.98 ms rollout-tail
remediation but not injected-delay verification or a hard worst-case bound.

## F-012 — Medium — Manager state spans concurrent callback groups without a lock

**Confidence:** hypothesis.

Normal manager callbacks use the default mutually exclusive group, but
qualification and device-client completion use reentrant groups in a four-thread
executor. They read and mutate multi-field safety/mode/transport state without a
common state lock. The Python GIL does not make multi-step state transitions
atomic.

**Recommendation:** Serialize state transitions through one owner callback or a
small explicit lock and immutable snapshots. Keep blocking qualification work
off the control-state executor.

**Verification:** Run qualification while injecting feedback/transport changes
and record state-transition invariants under a thread sanitizer-equivalent test
harness or deterministic callback interleavings.

## F-013 — Low — Dry-run mode reports internally inconsistent claim state

**Confidence:** observed.

When command output is disabled, mode publication is suppressed but the
controller still sets internal `mode_claimed`; diagnostics mask this field while
`control_session_started` uses it directly. This complicates log interpretation
and auditability, though it does not directly command hardware.

**Recommendation:** Represent requested, acknowledged, and output-enabled state
as separate fields in both logic and diagnostics.

## F-014 — Critical — Out-of-envelope feedback passes manager and model readiness

**Confidence:** observed.

During the passive live run, `/device/state` repeatedly reported ENC values near
`[1.197e9, -1.032e9, -24, ...]` and POS values including approximately
`503783 mm`, `-3.484e6 deg`, and `-3.134e6 deg`. Five of six POS axes were far
outside the `imricor_test` configured position envelope. Values were stable and
finite, so driver-power qualification and manager readiness accepted them.
The controller likewise reported estimator `TRACKING` and `model_valid=True`,
while its motor/downstream diagnostic state was orders of magnitude outside a
plausible training range.

**Consequence:** Arming would give MPPI a nonsensical root joint position and
motor state. The manager's limit clamp treats out-of-range values as a direction
constraint rather than invalid feedback, so it does not provide a sound
recovery path from values this far outside the envelope. Visual correction can
produce low marker residuals while masking the invalid actuator state.

**Recommendation:** Do not arm. Add an explicit plausible-feedback gate before
manager readiness and controller model initialization, including configured
position bounds, raw-count/reference validity, and model training-envelope
checks. Diagnose the underlying reference/firmware/decoding state without
changing or invoking encoder zero.

**Remediation implemented and passively verified after reflash:** The manager now
requires every POS axis to lie inside the selected catheter's hard limits for
qualification and readiness. An implausible sample revokes qualification and
stops/releases an active source. The controller independently validates POS and
the first three raw ENC channels before initializing or advancing v171. The
declared raw-count envelope is a conservative expansion of the 17,022 valid
v171 training samples: training extrema were approximately
`[-35806, -79999, -100404] .. [94903, 79955, 0]`; runtime limits are
`[-50000, -100000, -120000] .. [110000, 100000, 10000]`.

The live POS values were independently reproduced from the matching firmware
constants and live ENC values. This rules out ROS float/int decoding as the
cause of the observed values, but does not yet establish why the hardware
counters/reference entered that state.

On the post-reflash run, manager status was `MANAGER_READY`; raw ENC was
`[0,12,-2,4,4,30]`, POS was approximately
`[0.00030,0.0405,0.00030,0.00168,0.0135,0.0878]`, and both controller feedback
validity fields were true. The controller remained disarmed with command output
disabled, estimator `TRACKING`, and zero consecutive marker rejections.

**Verification:** Unit tests prove both gates reject the recorded anomaly, and
the subsequent passive live run proved that valid post-reflash feedback passes
both gates. Controlled fault injection remains prohibited on calibrated
hardware. Resolve the startup power-domain mechanism separately without
invoking encoder zero.

## F-015 — High — EKF correction crossed an inference/autograd tensor boundary

**Confidence:** observed and reproduced; remediated in source.

Encoder prediction and delayed rewind/replay intentionally execute under
PyTorch inference mode. After the first EKF correction, replay produced the
pose and strain used by the next correction as inference tensors. The EKF's
autograd measurement-Jacobian calculation attempted to save the pose for
backward and raised `Inference tensors cannot be saved for backward`. The ROS
wrapper latched the generic `marker_update:RuntimeError` fault after exactly
one accepted observation; camera tracking and callback scheduling remained
healthy.

**Remediation:** The EKF now clones pose and strain outside inference mode
before constructing its local autograd graph. The ROS wrapper also logs the
exception message while retaining the stable fail-closed fault reason.

**Verification:** A regression exercises two delayed EKF corrections across
the replay boundary. The captured geometry/timestamp reproduction completed
50 of 50 corrections, reached `TRACKING`, and retained finite bounded
covariance. The full learned-model suite (313 tests), controller suite (53
tests), lint, and ROS package build pass. The subsequent full-stack EKF shadow
run remained `TRACKING`, accepted every reported correction, accumulated no
rejection streak, and ran the planner actively without an estimator fault.

## F-016 — Medium — The 20 Hz marker limiter executes near 16.7 Hz

**Confidence:** observed and explained by source scheduling semantics.

The marker limiter advances its deadline to `now + 50 ms` after each commit,
while the sole estimator owner runs every 20 ms. The next eligible timer tick
therefore normally occurs 60 ms later, quantizing correction to about 16.7 Hz.
The EKF run increased its accepted-observation counter by 2,991 over 180 s,
or 16.62 Hz, despite `marker_update_rate_hz=20`. The matched active UKF run
independently measured 16.57 Hz, confirming that this is scheduler
quantization rather than EKF compute behavior.

**Consequence:** The estimator receives fewer corrections and slightly older
visual state than configured. This was not a rejection or fault source in the
audited runs, but it makes the parameter misleading and wastes available UKF
headroom.

**Recommendation:** Use an absolute phase accumulator that increments by the
configured period, or choose an owner rate commensurate with the correction
rate. Specify how missed releases are skipped so the estimator never builds a
backlog.

**Remediation implemented; live verification pending:** The limiter now
advances an absolute phase and skips missed releases. A deterministic 50 Hz
owner test produces the configured 20 Hz average without bursts. The next
shadow audit must confirm the accepted-observation slope and observation age
under live camera load.

**Live verification passed:** Accepted observations advanced by 3,596 over
180.007 seconds, or approximately 19.98 Hz, with no rejection. F-016 is closed.

## F-017 — Medium — One camera stream can starve synchronized output without a rig-specific error

**Confidence:** inferred-high-confidence for a one-rig capture/pairing outage;
root cause inside the camera path is unknown.

The UKF session lost marker output for 14.900 s. During the gap, the watchdog
published 20 `FEEDBACK_STALE` diagnostics with age reaching 14.780 s and the
controller correctly reported `marker_feedback_degraded`. Synchronizer drops
increased by roughly 454, matching one continuing 30 Hz stream for about 15 s,
while detector rejection count increased by only one. This points upstream of
marker estimation: one rig stopped supplying pairable frames while the other
continued. It does not implicate UKF, and the continuing watchdog callbacks
argue against a 15 s block in `_process_latest` or its thread pool.

**Recommendation:** Publish per-rig sequence age, last-frame timestamp, capture
worker liveness/error, and synchronizer queue/drop deltas. Give `no synchronized
pair` an explicit diagnostic instead of returning silently. Use these fields
to distinguish camera acquisition loss, USB/SDK stalls, and timestamp-pairing
failure on the next occurrence.

**Verification:** A longer passive dual-camera run should show no output gaps
and bounded per-rig frame age. If another gap occurs, correlate which rig's
sequence stopped with USB-controller and SDK diagnostics.

The replacement UKF run had continuous `TRACKING`, 29.072 Hz marker output,
and a 101.331 ms maximum gap. This shows that the 14.9 s outage was not an
intrinsic consequence of selecting UKF, but one clean 180 s run does not close
the intermittent per-rig liveness finding.

**Remediation implemented; fault/live verification pending:** Tracking,
pair-starvation, and stale diagnostics now include per-rig sequence, frame age,
image timestamp, synchronizer drops, and worker error. A pure regression shows
that a stalled oblique rig is distinguishable from a current primary rig. A
dual-camera shadow run and an intentional worker/camera interruption are still
required to validate the runtime diagnostic transition.

The post-remediation capture remained continuously `TRACKING` at 29.933 Hz.
Both per-rig frame-age P95/max values were 4.191/7.353 ms and every worker-error
sample was `none`. This validates healthy-path instrumentation; the intentional
single-rig interruption test remains open.

The 2/1 verification again remained continuously `TRACKING` with no worker
errors. One marker interarrival reached 134.320 ms, below the controller's
150 ms feedback limit and well below the marker node's 500 ms stale/pairing
diagnostic threshold. Healthy-path evidence is now repeated, but deliberate
single-rig fault injection is still required to close F-017.

## F-018 — Medium — Filter covariance/process noise is not yet statistically calibrated

**Confidence:** observed for covariance scale and local rank; inferred-high-
confidence for accumulation in a weakly observable direction.

EKF covariance trace was 75.6–188.3 and UKF 67.8–243.4 during the captures,
versus 2.5 at initialization. EKF's local observation Jacobian had rank nine
throughout its ten-dimensional error state; UKF was also usually rank nine.
The current isotropic normalized process noise injects variance at 1.0 per
state dimension per second, including the weak or unobservable direction. This
can enlarge UKF sigma points and degrade numerical/physical locality over long
runs even though both 180 s sessions remained finite and accepted corrections.

The reported `marker_nis` cannot establish covariance consistency: it is a
post-fit mean squared normalized residual, not the conventional pre-update
innovation quadratic form using the innovation covariance.

**Recommendation:** Identify or fix the estimator gauge, use structured
process noise that does not excite the unobservable direction, and record
covariance eigenvalue time series. Add a true pre-update normalized innovation
statistic before tuning process or observation noise from live data.

The matched active UKF capture narrowed the observed covariance-trace range to
90.8–102.7 during its 180 s audit window, but collection began after 1,507
accepted observations. This demonstrates bounded behavior over that window,
not calibration from initialization or statistical consistency.

**Structural remediation implemented; calibration pending:** EKF and UKF now
compute conventional pre-update innovation NIS and NIS per degree of freedom,
publish covariance eigenvalue bounds, project process noise into the last
accepted observable subspace, and pin the instantaneous unobservable gauge at
the covariance floor. Regressions cover finite NIS and bounded gauge variance
for both filters. Recorded innovation distributions from a fresh-start shadow
run are still needed before tuning or claiming statistical consistency.

**Post-remediation measurement:** Covariance trace remained 7.275–8.307,
substantially below the previous matched-run 90.8–102.7 range, so the gauge
accumulation mechanism is remediated over this window. Conventional innovation
NIS per degree of freedom was only 0.0147–0.0269 (median 0.0193), far below the
unit scale expected from a calibrated innovation model. F-018 therefore remains
open as a noise-calibration issue; this stationary run supports reducing or
restructuring uncertainty, but does not by itself identify whether measurement
noise, process noise, or both should change.

The 2/1 verification reproduced the same result: covariance trace was
7.095–8.432 and NIS/DoF median/P95/max was 0.0204/0.0239/0.0293. Thread-pool
changes therefore did not alter the outstanding statistical calibration issue.

## F-019 — Medium — An in-flight planner overrun can race the command-stale watchdog

**Confidence:** observed for the failure sequence; inferred-high-confidence
for same-process estimator/planner contention as the initiating scheduler load.

The powered multi-target session completed three control intervals without a
planner fault, then faulted during the final return with
`planned_command_stale`. The last published plan preceded the fault by 247.278
ms, exceeding the 150 ms command timeout. In the fault's diagnostic window the
complete planner callback reached 105.890 ms, planner release lateness reached
48.552 ms, the estimator callback reached 93.175 ms, and estimator lateness
reached 78.809 ms. The previously committed plan itself was healthy at 20.168
ms. This is therefore distinct from repeated MPPI solve deadline failure: a
late/in-flight callback failed to commit a replacement before the independent
heartbeat checked command age. The watchdog correctly failed closed.

**Recommendation:** Do not address this by merely increasing command age.
Separate plan computation from the command supervisor. At every control
release, the supervisor should atomically accept a completed, generation-matched
plan or publish an explicit zero and count a deadline miss. Late results must
be discarded. Repeated misses may retain the existing fault latch. Moving the
learned rollout into a dedicated worker process would also isolate it from the
UKF's Python/native-thread load and make cancellation-by-generation practical.

**Verification:** Under representative full camera/UKF load, inject a planner
delay longer than 60 ms. Prove that zero is emitted by the deadline, late
nonzero output is discarded, the manager remains fresh, and the configured
consecutive-miss count deterministically controls fault latching.

## F-020 — Medium — Forecast response diagnostics compare unequal time intervals

**Confidence:** observed.

The response forecast is nominally 40 ms, but across the four powered control
segments the stored start camera observation preceded planner state by a median
74.8–85.1 ms, and the endpoint camera observation followed forecast due time by
a median 13.1–22.8 ms. Thus `response_measured_tip_delta_mm` commonly spans
roughly 128–142 ms while `response_predicted_tip_delta_mm` represents 40 ms.
The resulting direction-cosine and endpoint-error values are not calibrated
model-response evidence. They are diagnostic-only and do not affect control,
but using them to tune the model would be misleading.

**Recommendation:** Evaluate forecasts retrospectively from a timestamped
camera ring. Interpolate observations bracketing both planner-root and forecast
due timestamps, and compare equal-duration displacements. Also retain the
published prediction-kind label because the current weighted feasible-candidate
mean is an approximation to the re-projected weighted command.

**Verification:** Unit-test timestamp bracketing/interpolation and record start
and endpoint interpolation gaps. Reject a response metric when either timestamp
lacks bounded bracketing observations.

## F-021 — Medium — Exact feedback limits created a boundary-recovery deadlock

**Confidence:** observed.

After an exact-zero position transaction, catheter-linear feedback was
`-0.0000973574 mm` (`-2` encoder counts) against a configured lower bound of
`0.0 mm`. The manager treated any sub-bound value as implausible, revoked
driver-power qualification, and then refused both requalification and the
corrective motion. Strict command clamping was appropriate, but applying the
same zero-tolerance contract to quantized feedback made a safe recovery through
the manager impossible.

**Remediation implemented; live verification pending:** Each catheter profile
may now specify a non-negative, per-axis `feedback_limit_tolerance`. The active
`imricor_test` values cover only a few encoder counts. Feedback plausibility and
qualification use this measurement-only tolerance; position targets remain
strictly clamped to the original hard limits, and material excursions still
fail closed. Qualification failures now identify the first violating axis,
value, hard range, and tolerance. Focused regressions verify acceptance of a
sub-resolution overshoot, rejection of a larger overshoot, and unchanged exact
command clamping.

**Verification:** Restart the rebuilt manager, confirm that the near-zero POS
sample produces `DRIVER_POWER_NOT_QUALIFIED` rather than
`POSITION_FEEDBACK_OUT_OF_RANGE`, and run the normal explicit driver-power
qualification. Do not arm or command motion until `MANAGER_READY` is observed.

Live verification exposed a distinct, material power-on corruption of
`5142.06738 mm`; this correctly remains outside the measurement tolerance and
is tracked separately in F-022.

## F-022 — High — Qualification validated corrupt POS before encoder restoration

**Confidence:** observed.

With the firmware encoder-integrity fault latched after driver power-on, the
manager sent STOP and then called `_wait_for_stable_feedback()`. The first
plausibility test rejected axis 0 at `5142.06738 mm`, so qualification returned
before querying fault status or issuing the firmware recovery transaction. The
firmware already retains the last atomic encoder frame and restores it on a
stopped `RESET_FAULT`, but the host gate order made that recovery unreachable.

**Remediation implemented; live verification pending:** Qualification now sends
STOP, requires the firmware to report every motor disabled, rejects every fault
class except encoder integrity, and only then requests restoration. It requires
the exact `OK_ENCODER_RESTORED` acknowledgement before validating stable,
in-range POS/ENC. Ambiguous latch state, any non-encoder fault, a missing restore
acknowledgement, or an invalid retained frame remains fail-closed. Focused
manager tests cover the allowed and rejected recovery cases.

**Verification:** Restart the rebuilt manager and run one qualification after
driver power-on. Require a successful service response and `MANAGER_READY`.
Before arming, confirm that POS/ENC returned to the retained near-zero frame. If
post-restore feedback is still outside limits, do not retry motion; inspect the
reported axis and firmware fault status.

**Live verification failed safely:** Firmware acknowledged restoration, but the
restored POS remained `5142.06738 mm` on axis 0. The manager consequently kept
qualification false. This proves the qualification ordering fix works as a
gate, but the firmware's retained frame was already invalid; see F-023.

## F-023 — High — Per-cycle encoder integrity does not bound disabled-state drift

**Confidence:** observed for the invalid restored frame and static guard
mechanism; inferred-high-confidence for incremental driver-power pulse
accumulation as the initiating mechanism.

After motor-driver power-on, qualification observed axis-0 POS at
`5142.06738 mm`. The encoder-integrity reset completed, yet post-restore POS was
unchanged and remained outside `[0, 40]`. Firmware updates its retained atomic
frame whenever every axis changes by less than a rate-derived allowance during
one read interval. It has no absolute physical-position envelope and no tighter
stationary/disabled-state drift budget. Consequently, a sequence of individually
accepted increments can move the retained reference arbitrarily far before a
later frame finally trips the latch.

A passive post-failure snapshot localized the corrupt raw channels to axes 2,
4, and 5: `[-2, -2025, -34550380, 0, 110074596, 1125170380]`, versus the
confirmed stationary pre-power frame `[-2, -2020, -5, 0, 0, 0]`. Axis-0 POS is
large because firmware reports the coupled catheter linear position from raw
axes 0 and 2; raw axis 0 itself remained at `-2` counts.

**Consequence:** Once the accepted baseline has drifted, the existing reset can
only restore that corrupted baseline. The manager correctly prevents motion,
but automatic recovery is no longer possible without an independently retained
pre-power reference.

**Potential remediation:** While all motors are disabled, compare each encoder
frame against a fixed stationary baseline rather than advancing the baseline on
every accepted sample, with a small cumulative count allowance. Preserve the
last qualified frame separately from the rolling motion frame. Qualification
must verify restored raw counts and physical POS against both the pre-power
reference and configured envelopes. Do not introduce a generic host command
that can redefine encoder zero.

**Remediation implemented; live verification pending:** Firmware now maintains
a separate restoration frame. It advances during enabled motion and a 250 ms
post-motion settling window, then freezes while all motors are disabled. During
that disabled interval, raw frames are checked cumulatively against the frozen
reference with a 16-count allowance; accepted telemetry cannot walk the
restoration reference. Encoder-integrity reset writes the frozen frame to all
hardware counters and recomputes the derived state. Host-independent tests
cover cumulative drift and signed overflow, and both recovery-enabled and
production-mode Teensy 4.0 builds compile cleanly with all warnings enabled.

The authorized incident recovery is compile-time-only and fixed to the
confirmed historical frame `[-2,-2020,-5,0,0,0]`. Its firmware identity is
`tkctl:recovery-exact-frame-v1`; it rejects nonzero velocity, non-home position,
and raw debug commands. It exposes no runtime count-setting interface and must
be removed after the mechanism reaches physical home.

**Verification:** With the robot stationary and Teensy continuously powered,
capture the qualified encoder frame, power-cycle only the motor drivers, and
prove that the first cumulative disabled-state excursion latches before the
qualified frame changes. Reset must reproduce the captured frame exactly, with
motors disabled throughout.

## F-024 — High — Coupled bend response can strand logical insertion outside its limit

**Confidence:** observed for the limit crossing and encoder mismatch;
inferred-high-confidence for differential physical-axis response as the cause.

Session `20260912_183239_mppi_demo` crossed the logical catheter-linear upper
limit from `39.997749` to `40.020813 mm` while transmitted logical insertion
was zero. Catheter bending drives physical axes 0 and 2 oppositely, while POS
reconstructs logical insertion as their sum. Over the final active interval,
their encoder-derived motions were approximately `+0.408` and `-0.175 mm`,
leaving `+0.233 mm` uncancelled. Controller intent, manager output, and serial
TX agreed. The manager stopped and inhibited correctly, but ordinary
qualification then could not authorize the inward move needed for recovery.

**Partial remediation implemented; prevention pending:** The manager exposes
`/manager/recover_catheter_linear_limit`. The Trigger service accepts only one
axis-0 violation within a 0.25 mm envelope, requires mode NONE, no active
source, fresh POS/ENC, ready transport, disabled motors, no non-encoder fault,
and responsive driver UARTs. It sends only the configured minimum reliable
axis-0 velocity inward, stops at a 0.10 mm interior margin, aborts on external
STOP, stale/malformed/unexpected/farther-outward feedback, applies a firmware
STOP barrier, and requires stable in-range feedback. It never changes encoder
zero and deliberately leaves driver-power qualification false.

This service resolves the bounded recovery deadlock; it does not prevent the
crossing. Add a conservative coupled-axis boundary guard before further MPPI
trials near insertion limits.

**Prevention implemented; live verification pending (2026-09-12):** Session
`20260912_192354_mppi_demo` showed a second crossing at 40.000225 mm after
MPPI entered the final target episode at 33.459 mm and requested approximately
6.96 mm further insertion. The controller contract now keeps a 1.0 mm
autonomous axis-0 reserve and re-projects every 100 Hz cached heartbeat against
the newest POS sample. The hard manager limit remains `[0, 40]` and still
fails closed. See `session-20260912-192354-adaptive-mppi-audit.md`.

**Verification:** Rebuild and restart the manager. From the recorded-size
`40.022 mm` excursion, call the recovery service once and require a successful
response near `39.9 mm`, followed by `DRIVER_POWER_NOT_QUALIFIED`. Verify that
external STOP cancels recovery and that oversized, multi-axis, or non-axis-0
violations are rejected without a nonzero command. Then perform normal driver
qualification and require `MANAGER_READY` before homing.

## F-025 — Medium — Simulator zero commands erased mechanical backlash state

**Confidence:** observed.

Session `20260913_200350_mppi_sim` showed repeated 1.3--1.7 s intervals in
which the controller requested rotation in the previously engaged direction
but simulated rotation remained zero. Every zero velocity heartbeat called
`ModelInLoopPlant.stop()`, which reset the complete actuator perturbation. With
`actuator_initial_backlash_unengaged=true`, the next same-direction command
therefore started a fresh full take-up budget while the controller correctly
retained its engaged estimate and did not issue another feedforward boost.

**Remediation implemented; trajectory replay pending (2026-09-13):** Routine
zero commands and watchdog stops now halt velocity, delay, and filter state
without erasing simulated transmission position. A full perturbation reset is
reserved for explicit simulated-mechanism initialization/reset. The backlash
operator also tracks partial travel across an interrupted reversal, so returning
to the last contacted side unwinds only the traversed portion rather than
charging another complete directional width. Regression tests cover both
same-direction resume after a halt and partial-reversal unwinding.

**Verification:** Replay the same 36-waypoint simulation with identical
directional widths. Same-direction commands after waypoint zero holds must
produce motion immediately; only initial unknown engagement and actual shaft
direction reversals may produce a take-up delay.

## F-026 — High — Shared simulation GPU and mixed timestamp semantics caused false pair-skew faults

**Confidence:** observed for timing, launch routing, and timestamp semantics;
inferred-high-confidence that shared CUDA contention produced the measured
tail.

In `20260914_191121_mppi_sim`, the 1,024-sample CUDA controller normally
planned in 37.8--49.6 ms but produced one 77.174 ms plan against its 60 ms
deadline. The same diagnostic window reported 221.924 ms estimator-timer
lateness and 43.733 ms encoder-owner duration. The simulated device published
POS and ENC from one snapshot with the same timestamp at 100 Hz, but the
controller computed pair skew between immediate POS callback arrival and the
time of the last estimator-processed ENC arrival. It therefore labeled
estimator scheduling delay as `feedback_pair_skew`. The simulation launch also
routed the independent 30 Hz truth/perception runtime to the controller's
`device:=cuda`, making it compete with MPPI and UKF for one GPU.

**Remediation implemented; replay pending (2026-09-14):** Simulation now has a
separate `truth_model_device` that defaults to CPU while `device` continues to
select the controller runtime. POS/ENC pair skew now compares the latest valid
device-message header stamps. Position age remains callback-arrival freshness,
and encoder age remains estimator-processed freshness, so estimator starvation
still fails closed as `encoder_stale` rather than being hidden or mislabeled.
Diagnostics identify the pair-skew basis as `device_header_stamp`.

**Verification:** Rebuild and replay the 1,024 x 4 CUDA circle simulation with
`truth_model_device:=cpu`. Require no deadline, stale-feedback, or pair-skew
fault and compare plan/estimator P50/P95/P99/max with the failed session. Then
retain this separation for robustness simulations so plant truth remains
independent of controller compute resources.

## F-027 — Medium — Shared-GPU UKF bursts can exhaust the three-miss planner budget

**Confidence:** observed for timing and fault sequence; inferred-high-confidence
for CUDA contention as the coupling mechanism.

The continuous-path run `20260914_194754_mppi_sim` completed 76.9 mm without a
fault: MPPI plan P50/P95/P99/max was 41.702/52.910/55.964/62.434 ms, with one
60 ms miss. The immediate retry `20260914_195112_mppi_sim` failed after three
consecutive misses. Its 97 reported plans had P50/P95/P99/max
42.847/55.413/109.202/109.202 ms and five plans over 60 ms. At the terminal
burst, UKF marker correction reached 108.031 ms, total rewind/correction/replay
147.998 ms, estimator callback duration 235.762 ms, and MPPI rollout 99.779
ms. Reference generation was healthy: the governor never paused and maximum
closest-path error before the fault was 2.330 mm.

The controller estimator and MPPI operate on the same CUDA runtime from
separate callback groups. Their extreme durations rose together while ROS
topic arrival, snapshot-lock wait, and normal plan time remained small. This
supports shared accelerator contention, although the bag does not contain a
CUDA-kernel trace proving exact overlap.

**Remediation implemented (simulation only, 2026-09-14):** The simulation
launch now permits five consecutive deadline misses; each miss still replaces
the executable command with zero. Hardware retains the stricter three-miss
default. Terminal diagnostics now retain the plan that actually crosses the
miss threshold instead of reporting the prior plan.

**Verification:** Repeat the 1,024 x 4 continuous path at 1, 2, and 3 mm/s.
Require completion without five consecutive misses and report plan and UKF
phase distributions. A future CUDA trace or split estimator/planner device
experiment is required before calling accelerator contention proven.

**2 mm/s replay result (2026-09-14):** Session
`20260914_195646_mppi_sim` completed the full 76.918 mm path and the action
reported terminal `SUCCEEDED`; no controller fault occurred. Across 576
reported plans, elapsed time P50/P95/P99/max was
42.466/50.325/56.657/78.606 ms, with three plans over the 60 ms deadline and a
maximum of two consecutive misses. The largest burst coincided with a 70.454
ms UKF correction, 89.610 ms total marker update, and a diagnostic-window
estimator-callback maximum of 160.360 ms. Thus the five-miss simulation policy
successfully tolerated the observed burst, but the shared-compute tail remains
observed rather than resolved. Closest-path error P50/P95/P99/max was
2.252/4.582/4.592/4.592 mm (RMS 2.826 mm), and 42.3% of samples were within
1.8 mm. The 1 and 3 mm/s results follow below.

**Speed-sweep completion (2026-09-14):** Sessions
`20260914_200821_mppi_sim` (1 mm/s) and
`20260914_201116_mppi_sim` (3 mm/s) also completed the full path with terminal
`SUCCEEDED` and no controller fault. The 1 mm/s run had four isolated deadline
misses (maximum consecutive count one); plan P50/P95/P99/max was
42.192/50.462/54.825/68.749 ms. The 3 mm/s run had three misses (maximum
consecutive count two); plan P50/P95/P99/max was
41.119/50.735/59.446/139.559 ms. This verifies that the five-miss simulation
policy tolerated all three measured workloads, but it does not satisfy the
original zero-miss performance gate. Hardware must retain its stricter
three-miss policy.

## F-028 — Medium — Continuous-circle speed sweep is confounded by the insertion boundary

**Confidence:** observed.

All three 76.918 mm continuous-path simulations completed, but each drove
joint 0 to approximately `-8.7` to `-8.9 mm`, matching the autonomous lower
bound of `-9 mm` produced by the configured `[-10, 50] mm` hard range and 1 mm
control margin. Boundary contact began at 66.5--68.3% path progress. Depending
on speed, 34.2--54.1% of recorded path samples were within 0.5 mm of that
lower bound. Their mean closest-path error was 3.18--3.74 mm, versus
0.81--1.40 mm away from the boundary. Maximum error occurred near 80--81% path
progress with joint 0 still near its lower bound.

The raw time-weighted closest-path RMS values were 2.985, 2.826, and 2.437 mm
at 1, 2, and 3 mm/s, respectively. The apparent advantage at 3 mm/s is partly
because the faster run contributes fewer samples while constrained. After
uniform arc-length resampling, RMS was 1.789, 1.901, and 1.994 mm; no speed
dominates every statistic. Approach-only RMS stayed below 0.64 mm in every
run, while circle-only RMS was 2.64--3.17 mm.

**Consequence:** The sweep proves continuous-path completion and exercises the
timing policy, but cannot identify a hardware tracking speed or controller
tuning from aggregate error. The Cartesian circle is not fully feasible under
the deployed joint constraint, so speed, governor dwell time, and limit
projection are entangled.

**Recommended verification:** Construct a model-validated interior path whose
entire inverse-reachable joint envelope retains explicit margin on all axes.
Repeat 1/2/3 mm/s from identical initialized states and report both
time-weighted and uniform-arc-length errors, plus minimum joint margin and
projection count. Do not promote the current full circle to hardware.

## F-029 — High — CUDA hardware run faults on processed-encoder starvation despite healthy transport

**Confidence:** observed for telemetry continuity and gate timing;
inferred-high-confidence for shared CUDA/runtime contention as the initiating
load burst.

In hardware session `20260914_202605_mppi_demo`, the controller faulted
`encoder_stale` shortly after path activation. Raw predicate-69 ENC and
predicate-80 POS frames continued arriving with approximately 11 ms median
spacing, less than 28 ms maximum spacing, and less than 4 ms maximum
arrival-minus-header age. The manager remained `MANAGER_READY`; this was not a
serial, firmware, or DDS telemetry outage.

Immediately before the fault, accepted UKF marker updates took 85.451 and
101.556 ms total. The controller diagnostic window then reported a 160.437 ms
estimator callback and 143.744 ms estimator-timer lateness. The hardware
feedback freshness limit is 150 ms. A concurrent plan first missed its 60 ms
deadline at 67.622 ms. Although the next published diagnostic already showed
only 21.871 ms encoder age, a concurrent estimator commit had refreshed the
value after the plan callback's fail-closed gate evaluation. The fault thus
correctly reflects the age observed at the gate, while the later diagnostic is
not a snapshot of the triggering inputs.

**Consequence:** The combined CUDA UKF plus 1,024-sample MPPI configuration is
not qualified for hardware output. Increasing the feedback timeout would
permit planning from older recurrent state and would mask the scheduling
failure rather than resolve it.

**Recommended remediation:** Keep hardware output disabled for the 1,024
sample shared-runtime configuration. For an immediate conservative hardware
path test, return to the previously exercised CPU/32-sample configuration.
For GPU promotion, separate estimator and rollout compute resources or make
marker correction snapshot/validate/commit so encoder propagation has a
bounded release path. Preserve the 150 ms freshness gate and the three-miss
hardware deadline policy during verification.

## F-030 — High — Hardware continuous-path approach diverges before the reference governor pauses

**Confidence:** observed for the motion, command, and governor behavior;
inferred-high-confidence for proximal-model/backlash-state mismatch as the
cause.

In hardware session `20260914_203226_mppi_demo`, the continuous-path governor
correctly transitioned from `RUNNING` to `SLOWED` and then `PAUSED` after 3.31
seconds. At pause, reference progress was only 2.931 mm (3.9% of the path),
reference error was 5.095 mm, and closest-path error was 4.631 mm. The pause
therefore prevented the reference from continuing to escape; it did not stop
the armed controller from trying to recover to the frozen reference.

The plant was not stationary before the pause. Logical feedback moved from
approximately `[19.9985, 0.0236, 0.0012]` to
`[25.8384, -33.6994, 4.8511]` by the pause, while the measured tip left the
desired approach. Across the run the tip spanned approximately
`[4.00, 8.35, 3.40]` mm in base x/y/z. Afterward the controller became
intermittent: 80.2% of active status samples reported an all-zero desired
command, while the rotation axis accumulated approximately 74 desired-command
reversals. Backlash compensation changed 13.1% of commands, with configured
directional widths `[8.369, 7.020, 34.858]` rad positive and
`[6.856, 7.090, 3.587]` rad negative. The run ended disarmed without a
controller or manager fault.

No `/catheter_mppi/response_trace` rows were recorded. The controller currently
suppresses forecast registration whenever backlash compensation changes the
command, precisely excluding the intervals most important for validating this
failure. The persistent backlash estimator also begins every axis in
`UNKNOWN`; the first detected direction starts a full prior-width take-up even
though stationary encoder feedback cannot identify which flank is already
engaged at process start.

**Consequence:** The hardware path run does not qualify continuous tracking.
The current fixed proximal Jacobian and startup backlash state can command a
large mixed joint displacement whose measured tip response disagrees with the
planned direction. Once the governor pauses, the short-horizon sampler often
selects zero and cannot recover. Relaxing governor thresholds would allow a
larger reference error without correcting the underlying prediction failure.

**Recommended remediation:** Keep continuous hardware paths disabled pending a
small, interior, single-direction validation. Record forecasts for both the
logical post-backlash command and the actual compensated motor command. Replace
the unconditional full-gap assumption at startup with an explicit engagement
initialization/probing policy or a bounded response-terminated take-up state.
Only enable online Jacobian updates on causal, direction-pure intervals after
transmission engagement is confirmed; do not adapt from the reversal-rich
portion of this run.

**Cross-session causal refinement (2026-09-15):** The same pause was reproduced
at 3.74% in the readable portion of `20260915_111843_mppi_demo`; a third run
paused at 8.87%. At the 3.91% pause, the model's 160 ms terminal displacement
had cosine `+0.97` with the target error, while the measured displacement had
cosine `-0.78`, despite `+5.72 deg` of measured rotation-shaft motion. Across
rotation-only, sign-consistent windows in the first five seconds after the
three pauses, median measured/model displacement norm was only 0.017--0.028
and 41--50% of measured increments pointed away from the target. The path
tangent varied by less than 0.06 degrees before pause, but delayed distal
motion changed the target-error direction by 37--99 degrees.

The deployed bags all used CPU, 32 samples, four 40 ms steps, and 20 deg/s
rotation noise—not the 1,024-sample CUDA profile. Their 160 ms rollout is much
shorter than the approximately 0.75 s represented by the configured 7 rad
rotation take-up at the 40 deg/s compensated command. Because compensation is
applied after a rollout with zero backlash, MPPI predicts immediate
post-engagement response while hardware is still taking up or releasing stored
torsion. The current first-step sign change has only a small generic slew cost;
the explicit reversal cost covers future steps but not previous-to-first.
Detailed evidence and remediation boundaries are in
`session-20260915-continuous-path-rotation-pause-audit.md`.

## F-031 — High — In-horizon take-up prediction creates a zero-command deadlock

**Confidence:** observed.

The corrected-backlash simulation `20260915_121215_mppi_sim` reached only
4.438% progress. Its governor remained nominally `SLOWED`, but speed scale
decayed to `1.31e-10` at a 5 mm reference error. After the first five seconds,
every MPPI command was zero while axis 0 remained `TAKEUP` with 6.242 rad of
estimated gap. Rotation was never commanded, so the immediate regression is
not rotation-latch behavior.

The four-step, 40 ms planner cannot see post-gap response when take-up lasts
longer than 160 ms. Such candidates incur effort without improving predicted
tip cost, so MPPI selects zero; zero then prevents gap consumption. The
candidate-local geometric model is internally consistent but incompatible
with the intended short-horizon post-engagement control abstraction.

**Consequence:** Transmission-aware continuous tracking cannot progress and
must not be promoted to hardware in its present form.

**Required remediation:** Optimize post-engagement response directly, attach
a bounded direction-specific take-up cost to selection, and execute an accepted
reversal/take-up as a response-terminated macro action. Split simulated
upstream encoder motion from downstream transmitted model motion before using
simulation to qualify the estimator. Independently make the governor enter
literal `PAUSED` below a small scale/error epsilon so its reported state cannot
asymptotically remain `SLOWED`. Full evidence is in
`session-20260915-transmission-aware-sim-stall-audit.md`.

**Remediation implemented (2026-09-15):** The learned rollout now receives
post-engagement motor rates; a separate direction-specific take-up delay cost
informs candidate selection. All controlled shafts use a response-terminated,
bounded direction hold during `TAKEUP`, with current-position projection still
applied. Simulated upstream ENC and downstream transmitted model state are now
separate, and the path governor reports a true `PAUSED` state at its numerical
stop boundary. This source-level remediation is unit-tested but remains
unqualified under a new full-stack simulation session.

## F-032 — High — UKF-predicted response prematurely terminates take-up

**Confidence:** observed.

In `20260915_123815_mppi_sim`, the initial negative-rotation command correctly
entered `TAKEUP`, but the marker-response test declared rotation `ENGAGED` while
4.67 rad of the calibrated shaft gap remained. The response test used the UKF
posterior after raw-encoder propagation, so a response partly inherited from
the model prediction could satisfy the direction/cosine checks before the
independent geometric travel criterion. Separately, the feedforward stage
continued its maximum take-up magnitude whenever the epistemic phase remained
`TAKEUP`, even after `remaining_rad` reached zero. Small sampled bend commands
therefore became prolonged maximum-rate, insertion-coupled motion while three
marker confirmations were awaited.

**Consequence:** The compensation macro can release a shaft early and can
apply an unmodelled high-speed burst after geometric engagement. The observed
truth tip moved away from the path until the 15 mm hard-error gate aborted the
simulation.

**Remediation implemented (2026-09-15):** Engagement now requires both
calibrated gap exhaustion and the existing repeated direction-consistent
marker response. Full-speed feedforward is applied only while positive
geometric gap remains; after exhaustion the desired post-engagement magnitude
resumes, while the existing direction latch remains active until confirmation.
Focused regressions cover premature UKF-like response, rollout magnitude, and
the pre-confirmation interval. The subsequent full-stack simulation confirmed
these mechanics were active but exposed the separate model-input defect F-033.

## F-033 — High — Learned state integrates non-transmitted shaft travel

**Confidence:** observed.

The backlash simulator correctly separated upstream encoder motion from the
downstream state that drives its truth model, but the controller continued to
feed upstream counts directly into v171. UKF marker correction repaired the
currently visible centerline without undoing all motor-conditioned material
roll, downstream transmission, and tendon-history evolution. For example, at
0.613 s in `20260915_131257_mppi_sim`, the controller state contained motor
angles `[-1.198,-3.168,-1.901]` rad while shaft 0 and rotation had not yet
transmitted in the plant. The planner then optimized from a state that was
marker-close but dynamically inconsistent with simulation truth.

**Consequence:** Post-engagement predictions can use the wrong material-frame
orientation and distal history. MPPI can therefore repeatedly select a command
whose predicted displacement points toward the reference while the independent
plant moves away, eventually reaching the hard-path-error gate.

**Remediation implemented (2026-09-15):** The persistent backlash estimator
now advances on every processed raw encoder sample and exposes a virtual motor
angle containing only travel beyond the estimated geometric gap. V171 is
propagated with that estimated transmitted angle. Raw counts remain unchanged
for validity/safety checks, source timestamps, gap estimation, and trace
recording. Visual response confirms engagement but no longer defines the
model's geometric transmission input. Offline replay against the simulator's
hidden transmitted state agreed within 0.0007 rad maximum on every axis; full
closed-loop simulation qualification remains pending.

## F-034 — High — Single-axis response attribution leaves mixed take-up latched

**Confidence:** observed.

In post-F-033 simulation `20260915_134828_mppi_sim`, the UKF interface pose
matched independent truth throughout the path interval: translation error was
0.038/0.077/0.144 mm and orientation error was
0.222/0.475/0.831 degrees at P50/P95/max. At matching source timestamps, the
controller's virtual motor angles agreed with transmitted truth within about
0.0003 rad normally. Thus neither interface-pose estimation nor the prior
raw-versus-transmitted model-input defect explains this failure.

The path progressed only 1.527 mm before pausing; closest-path error rose from
zero to 7.516 mm. By 1.37 s, the terminal reference error was approximately
`[-0.538,-2.498,+0.722]` mm, but the next predicted and measured increments
were `[+0.483,-0.041,-0.255]` and `[+0.474,-0.037,-0.277]` mm: both moved away
from the reference. Subsequent model-versus-plant response agreement was
strong (direction cosine 0.959--0.999), proving that MPPI's local response
model described the executed motion while the transmission macro overrode the
recovering direction.

The terminal transmission phases were `[TAKEUP, TAKEUP, ENGAGED]`. During the
action, nonzero insertion commands were negative in all 50 recorded plans and
nonzero rotation commands were negative in all 61; neither axis could reverse
as the reference moved behind the diverging tip. However, response
confirmation is not inherently rotation-specific: insertion and bending
widths are uncertain too. The actual defect is that `observe_response()`
selects only the largest motor increment and requires a direction-purity
threshold. Mixed MPPI excitation therefore cannot attribute simultaneous
interface response to insertion and rotation, leaving both epistemically in
`TAKEUP` even after physical transmission occurs. In addition,
`advance_motor()` treats the nominal directional width as an exact blocked
distance for model propagation; observation only adapts that width after the
single-axis confirmation succeeds.

**Consequence:** A stale take-up hypothesis can force continued motion in the
wrong Cartesian direction after the true gap is crossed. The path governor
pauses correctly, but MPPI cannot command the reversal needed to recover.
Nominal widths are being used both as plant-like model parameters and as
uncertain estimator priors, obscuring the required separation between hidden
simulation truth and controller belief.

**Recommended remediation:** Keep independent fixed directional gaps only in
the simulated plant. In the controller, represent each nominal width as an
uncertain prior and infer engagement from observed interface response on every
axis. Replace dominant-axis/purity gating with a joint observation model (the
three-axis case permits evaluating all engagement subsets) so simultaneous
commands can be identified through `J delta_u` and posterior covariance.
Update width estimates from response-onset shaft travel, retain bounded
response-terminated direction holds for all axes, and use the larger rotation
prior only as a quantitative difference. Record optimizer-selected,
compensated, estimated-transmitted, and truth-transmitted commands separately,
then re-run the same deterministic simulation before hardware use.

**Remediation implemented; deterministic simulation pending:** The response
observer now uses source-time-matched raw encoder increments and jointly fits
all active shafts against the accepted UKF interface-pose increment. The
three-axis bounded active-set solve constrains inferred transmission to the
raw direction and travel, penalizes noise-sized extra columns, and reports
per-axis leave-one-out evidence. Engagement no longer requires nominal-width
exhaustion, but still requires the transmitted-motion floor, direction,
response cosine, independent evidence, and repeated confirmations. Thus the
directional widths serve as bounded priors while observed response is the
engagement authority. Mixed distinguishable axes can confirm together;
collinear ambiguity remains fail-closed. New status and estimator-trace fields
record inferred increments, response evidence, and joint residual. Focused
regressions and both affected ROS packages build cleanly; the exact failing
simulation remains the required dynamic verification.

## F-035 — High — Independent take-up realizes partial coupled MPPI actions

**Confidence:** observed.

The first post-F-034 simulation, `20260915_143429_mppi_sim`, shows that joint
response attribution worked: all three axes reached `ENGAGED` by 1.29 s, and
the controller virtual motor matched hidden transmitted truth within about
0.0006 rad at P95 on every axis. Nevertheless, the path reached only 1.615 mm
(2.10%) before pausing, with 7.765 mm maximum closest-path error.

The failure occurs between post-engagement planning and upstream execution.
MPPI evaluates the useful coupled post-engagement vector immediately and adds
a take-up delay cost, while the feedforward compensator independently drives
each shaft through its gap. At 0.89 s bending had confirmed `ENGAGED` while
insertion and rotation remained in `TAKEUP`; the tip had already moved off the
path. At 1.29 s all axes were engaged, but closest-path error was already
2.29 mm. Subsequent asynchronous reversals repeated the pattern and error
reached 5.93 mm by 2.39 s. Thus the physical plant received partial components
of a vector that the learned rollout evaluated only as a coordinated action.

The candidate delay cost also summed per-axis gap time although shafts take up
concurrently. That biases MPPI away from useful multi-axis actions and toward
serial direction changes. Response forecasts were labeled transmission-aware
even during take-up, although their learned tip prediction intentionally used
post-engagement rates rather than the interim physical command.

**Consequence:** Even with accurate pose and transmission state, the execution
macro can drive the catheter in a Cartesian direction that MPPI never scored.
The resulting error changes subsequent local optima and induces additional
reversals and take-up events.

**Remediation implemented; deterministic simulation pending:** Execution now
uses a coordinated take-up barrier in physical motor coordinates. While any
shaft is pending, only `TAKEUP` shafts move and already engaged shafts wait at
zero; the coupled MPPI vector is released after all pending shafts confirm.
Candidate take-up latency is the maximum concurrent axis delay rather than its
sum. Short-term model-response forecasts are suppressed during this macro,
because the post-engagement prediction does not represent interim take-up
motion. Regression tests cover motor-coordinate isolation, full-vector release,
and parallel delay accounting. Re-run the identical simulation before any
hardware use.

## F-036 — High — Frame-local response evidence and reactive coordination delay engagement

**Confidence:** observed.

The post-F-035 simulation `20260915_144532_mppi_sim` improved maximum
closest-path error from 7.765 mm to 6.131 mm and progress from 1.615 mm to
2.785 mm, confirming that motor-coordinate coordination was active. It still
paused for 378 of 476 path updates. The remaining failure begins at the
take-up/engagement boundary, not in UKF pose recovery or ROS scheduling.

`observe_response()` compared each accepted camera pose only to the immediately
preceding pose, despite requiring 0.10 rad of inferred transmitted motion.
Normal per-frame shaft travel was often below that floor. Rotation truth began
moving the tip near 0.9 s, but rotation remained `TAKEUP` until about 1.5 s;
the tip moved approximately `[+2.44,-1.91,-0.43]` mm before confirmation.
During that interval the planner reduced the rotation request, but the
response-terminated latch continued the same direction.

The coordinated barrier also derived its pending set only from the current
estimator phase. A newly introduced axis or reversal is still `UNKNOWN` or
`ENGAGED` until an encoder update processes the first motion. This allowed one
full coupled-command interval to leak before the new shaft was labeled
`TAKEUP`. At 1.60 s, for example, status already showed insertion and bending
in `TAKEUP`, but the retained command still included productive rotation from
the preceding commit. Similar one-cycle phase/command mismatch recurred near
4.8 and 5.8 s.

Once the initial transient had displaced the plant, MPPI did not recover: from
1.6 s onward its reported weighted-command terminal error was consistently
worse than the current reference error, reaching 4.70 versus 4.29 mm at 3.0 s.
The three valid response forecasts agreed closely with the plant but pointed
away from the contemporaneous target. This remains a candidate-selection
robustness issue to address only if eliminating the causal transition impulse
does not remove the bad basin.

**Remediation implemented; deterministic simulation pending:** Marker response
is now accumulated over a common multi-frame SE(3) window until every moving
`TAKEUP` shaft has enough travel for the configured response floor. The window
then resets after evaluation, preserving repeated-confirmation semantics. A
shaft reversal resets the window immediately so it cannot mix
opposite-direction evidence.
Coordination now predicts pending shafts directly from the proposed physical
motor signs, directional widths, and prior motion direction, so first motion
and reversals are isolated before the encoder state transition rather than one
cycle afterward. Focused and package-level regressions pass 262/262 and both
ROS packages rebuild cleanly. Repeat the deterministic simulation before
changing MPPI candidate selection.

The repeat `20260915_145904_mppi_sim` validates this remediation: rotation
entered `TAKEUP` at 0.384 s, every axis was in `TAKEUP` at 0.484 s, bending
engaged at 0.784 s, insertion at 0.984 s, and all three axes were engaged by
1.184 s. The remaining divergence is tracked separately in F-037.

## F-037 — High — Diffuse feasible-sample averaging can synthesize a harmful command

**Confidence:** observed symptom; source-supported mechanism.

The post-F-036 simulation `20260915_145904_mppi_sim` still progressed only
1.965 of 76.918 mm and reached 6.050 mm maximum closest-path error. The path
governor reported `PAUSED` for 405 of 479 samples. This was not a lifecycle,
deadline, estimator-pose, or engagement failure: the controller remained
`ACTIVE`, had all axes engaged by 1.184 s, and issued ordinary valid plans.

Starting near 1.48 s, the reported prediction for the weighted MPPI command
was already worse than holding the current state: current/terminal reference
errors were 1.402/1.856 mm, then 2.057/2.650 mm at 1.68 s,
3.300/3.735 mm at 1.88 s, and 4.163/4.600 mm at 2.08 s. The controller then
fell into zero and reversal/take-up cycles while the governor remained paused.

The source weighted individually projected candidate controls and their
predicted tips. The weighted tip sequence is not a learned-model rollout of
the weighted command. Because v171 is nonlinear, the averaged control and
averaged prediction need not correspond to any scored trajectory. The
normalized-temperature softmax was also diffuse: effective sample count in
the failing follow-up began near 513 of 1,024 candidates.

Backlash makes the physical shaft-command-to-response map hybrid and
non-convex, but it is deliberately outside the learned rollout here: MPPI
passes desired post-engagement motor rates directly to v171. Backlash still
alters candidate weights through take-up-delay/reversal costs and constrains
or replaces execution through the direction latch and coordinated macro. It
was therefore incorrect to attribute the learned candidate-response
non-convexity itself to backlash.

**First remediation insufficient:** An opt-in
`mppi_best_candidate_guard` compares the approximate weighted-trajectory tip
cost with the deterministic zero candidate. If the weighted result is worse
than zero while the minimum-total-cost sampled candidate has lower tracking
cost than zero, execution switches to that already-scored feasible candidate.
It adds no learned-model rollout and preserves the normal MPPI update whenever
the weighted result is useful. Diagnostics now expose the selection flag and
weighted, zero, and selected-best tracking costs. The causal-v2 profile enables
the guard; generic defaults remain disabled. Cross-package regressions pass
351/351 (one unavailable `ros2_igtl_bridge` test excluded), and
`catheter_control` builds cleanly.

The follow-up `20260915_150941_mppi_sim` reproduced 1.997 mm progress and
6.030 mm maximum closest-path error, with 146/222 path updates `PAUSED`.
No guard selection occurred in 71 active status samples. At a representative
plan, weighted/zero/best tracking costs were 72.633/52.246/52.246; because the
guard requires the best candidate to be strictly better than zero, it did not
select zero even though the weighted action was worse. Another recorded plan
had 188.227/181.132/165.841 and still reported the guard disabled, indicating
the launch parameter was not active for this run. The required next correction
is to treat deterministic zero as an eligible fallback whenever weighted
tracking is worse, independently verify the runtime parameter, and then decide
whether diffuse softmax temperature and backlash-derived candidate penalties
belong in the post-take-up planner.

**Second remediation implemented; simulation validation pending:** MPPI now
has no measured backlash/take-up input in sampling, rollout, or cost. With the
guard enabled, execution uses the minimum-total-cost evaluated candidate and
candidate zero is an unconditional eligible fallback; the weighted sequence
only updates the next sampling nominal. Physical take-up is owned by a
stateful downstream transaction arbiter. It latches participating motor shafts
and directions, holds ready participants at zero while commanding only pending
shafts, then emits zero and requires a fresh plan after response confirmation.
The path governor freezes its phase during that transaction. Focused tests pass
194/194 and the ROS packages build; do not close F-037 until a new full
simulation demonstrates bounded path error and transaction completion.

The validation run `20260915_162120_mppi_sim` improved progress to 15.477 mm
and bounded maximum closest-path error to 3.046 mm, but it did not close this
finding. The scored-candidate guard was false in every active sample, while
the downstream transaction was active for 41.50 of 60.50 armed seconds. Its
terminal failure is tracked as F-038.

## F-038 — High — Logical position projection can make a physical take-up transaction unfinishable

**Confidence:** directly observed; source-supported mechanism.

In `20260915_162120_mppi_sim`, transaction generation 44 requested positive
physical shaft-0 and shaft-2 take-up. Bend reached 0.0847 mm near its 0 mm
lower bound while shaft 2 still reported 24.94 rad remaining. The autonomous
position-horizon projection reduced the coupled negative logical bend command
below its 2 mm/s reliable-speed floor and replaced it with zero. Shaft 2 could
therefore no longer consume its pending gap.

The same logical-axis projection did not atomically remove the coupled
insertion component. That component alternated with the independently pending
shaft-0 command, reversing raw shaft 0 and repeatedly moving its estimator
between `ENGAGED` and `TAKEUP`. The path remained in `TRANSMISSION_HOLD` until
the action aborted. The arbiter's physical pending mask is therefore not
preserved across logical coupling plus per-axis position projection.

Required remediation is to arbitrate on the final feasible quantized physical
shaft command, explicitly terminate a transaction whose pending shaft is
blocked by a position/rate constraint, and add a bounded transaction watchdog.
Repeat deterministic simulation with the scored-candidate guard confirmed
active before hardware testing.

## F-039 — High — Full-rate response confirmation creates a reversal limit cycle near the target

**Confidence:** directly observed; source-supported mechanism.

Session `20260915_183741_mppi_sim` made only 1.480 mm of 76.918 mm path
progress. It spent 36.592 of 43.401 armed seconds in `TAKEUP_ACTIVE`, opened
50 transactions, and reversed rotation transaction direction 39 times even
though path-tangent direction changed by at most 0.00964 degrees. No
`SATURATED_REPLAN` occurred and the planner direction block remained zero.

A direct Cartesian error-vector comparison shows that the next MPPI direction
was usually justified: among 35 consecutive rotation reversals, 31 had a
negative dot product between the target-minus-tip vector before take-up and
the vector seen by the next plan. Median cosine was -0.544. The fixed-rate
take-up macro therefore crosses the nearby target while waiting for repeated
visual response confirmation; MPPI commands the correction back, initiating
another full-rate take-up macro.

Required remediation is a two-stage transaction: fast bounded gap traversal,
then hold or low-rate probing immediately after the first credible response
while repeated observations confirm engagement. Planner-memory preservation
remains a secondary continuity improvement but must not suppress a reversal
when measured Cartesian error actually crossed the target. See
`session-20260915-183741-post-takeup-replan-chatter-audit.md`.

**Remediation status (2026-09-15): implemented, awaiting recorded simulation
validation.** The first credible response now enters `PROVISIONAL`, releases
the full-rate transaction, commands the existing zero transition, and forces
a fresh plan. Two inconsistent moving observations fall back to bounded
take-up; repeated consistent observations commit `ENGAGED` and permit width
learning. The path governor also catches a bounded along-path overshoot up to
the measured tip inside its soft-error tube, while guarding coincident closed
path endpoints. Unit and package suites pass; F-039 remains open until the
recorded matched- and mismatched-width simulations pass.

## F-040 — High — Successful take-up completion erases MPPI direction memory and excites insertion reversals

**Confidence:** observed mechanism; inferred-high-confidence causal effect.

Session `20260915_191324_mppi_sim` recorded 56 insertion reversals in 108
selected plans, while rotation reversed only six times and bend remained
zero. Closest-path error stayed below 0.456 mm and the target-error vector
usually remained aligned across insertion reversals, excluding target crossing
and bend/insertion coupling as explanations. `TAKEUP_ACTIVE` occupied 80.4%
of controller status samples, limiting progress to 5.739 mm.

Before remediation, every ordinary `REPLAN_REQUIRED` called `planner.reset()` and
zeros `last_effective_command`. The fresh solve is therefore zero-centered and
its slew cost has no memory of the previous post-engagement direction. In the
weakly constrained insertion direction, minimum-cost sampled candidates
alternate sign and repeatedly reopen the slow insertion take-up transaction.
The declared `mppi_first_step_reversal_weight` was not used in the cost
function.

Required remediation is to preserve sampling/previous-command memory across
ordinary response completion while still generating a fresh plan; retain hard
resets for saturation, target changes, disarm, and faults. Apply the existing
first-step reversal penalty in physical shaft coordinates as a soft cost. See
`session-20260915-191324-insertion-reversal-audit.md`.

**Remediation status (2026-09-15): implemented, awaiting recorded simulation
validation.** Ordinary response-confirmed transaction release now preserves
the shifted MPPI nominal sequence and `last_effective_command`; the executable
take-up command is still zeroed and a fresh solve is required. Saturation keeps
the former hard reset because it changes the feasible set. The first-step
physical-shaft reversal penalty is now included in candidate total cost with a
deployed default of `0.10`; it remains soft, so a sufficiently better
reversing rollout can win. Focused regression tests cover both handoff modes
and soft-cost selection.

**Recorded validation (2026-09-15): partially effective.** Session
`20260915_192609_mppi_sim` increased maximum progress from 5.739 to 10.565 mm,
reduced physical insertion reversals from 56 to 33, and reduced transmission
hold occupancy from 80.4% to 75.0%. It did not pass the reversal or hold gates
and ultimately faulted on an unconfirmed bend transaction. F-040's memory and
cost changes are retained, but temporal reversal arbitration is still needed.

## F-041 — High — One sampled sign can open a stateful take-up transaction

**Confidence:** observed behavior; inferred-high-confidence mechanism.

Session `20260915_192609_mppi_sim` made 10.565 mm of progress with sub-millimetre
closest-path error, then faulted with
`backlash_takeup_unconfirmed:axis_2`. Feedback, UKF correction, and planning
timing were healthy. Bend appeared in only 12 of 123 plans but reversed nine
times; five sign changes occurred in consecutive plans. Physical insertion
also reversed 33 times, 29 of them directly from one nonzero sign to the
opposite sign.

The controller presently promotes each independently selected minimum-cost
candidate directly into the take-up arbiter. The soft reversal cost has no
cross-plan persistence requirement, and the 2 mm/s bend minimum maps a weak
sample preference to a material command. One noisy or weakly constrained
sample sign can therefore open a long-lived transmission transaction.

Required remediation is a physical-shaft reversal-intent gate: require a
consistent sign across at least two fresh plans plus a minimum improvement over
deterministic hold before beginning take-up. Command zero and freeze the path
reference while intent is pending, retain last-nonzero direction across zero
plans, scope unconfirmed-takeup faults to the active transaction/direction,
and include the new post-take-up replan reason strings in the path hold set.
See `session-20260915-192609-reversal-intent-audit.md`.

## F-042 — High — Maximum-width guard bypasses provisional rejection policy

**Confidence:** observed; source-confirmed mechanism.

**Remediation:** implemented 2026-09-15; simulation validation pending.

Hardware-equivalent sparse simulation `20260915_201546_mppi_sim` reached its
first target, then faulted during target 2 with
`backlash_takeup_unconfirmed:axis_2`. Scheduling, feedback, UKF health, model
validity, and CUDA planning were healthy. One status cycle before the fault,
axis 2 entered `PROVISIONAL` with response evidence 1.0 and inferred
transmission of -0.555 rad. The transaction arbiter correctly stopped axis-2
take-up and removed it from the pending mask.

At the next accepted response window, no new axis-2 contribution was
confirmed. `BacklashStateEstimator.observe_response` incremented the
provisional rejection count, but then applied the maximum-width failure check
to the still-provisional shaft. Because accumulated travel had already passed
the calibrated limit, the shaft changed directly to `FAILED` after one miss,
bypassing the configured two-observation fallback to `TAKEUP`.

This violates the intended response-terminated handoff: the first credible
response releases full-rate take-up, while repeated evidence commits
`ENGAGED` and two inconsistent moving observations reject the provisional
state. Restrict the maximum-width failure check to unconfirmed `TAKEUP` after
the provisional fallback has actually occurred. Add a regression covering one
credible response followed by one miss beyond the width bound, then repeat the
same eight-point hardware-equivalent simulation before hardware use. See
`session-20260915-201546-provisional-axis2-fault-audit.md`.

The estimator now evaluates the calibrated maximum-width guard only when the
post-rejection state is `TAKEUP`. A still-valid `PROVISIONAL` state therefore
receives its configured rejection window; on the observation that completes
fallback to `TAKEUP`, the guard applies immediately and remains fail-closed.
The new boundary regression and all focused controller tests pass, and the ROS
package rebuilds cleanly. Hardware readiness still requires the specified
eight-point simulation rerun.

## F-043 — High — Fresh MPPI plans can create a reversal-transaction limit cycle

**Confidence:** observed; source-confirmed mechanism.

Sparse-target simulation `20260915_203808_mppi_sim` proves the reversal issue
is not unique to a moving path reference. Point 2 opened rotation transaction
generations 3--11; after the initial direction, every generation alternated
sign. Eight reversals consumed about 6.8 seconds, mostly in 0.8--0.9 second
take-up intervals, while the small rotation-sensitive error component crossed
zero and the dominant x/z error remained roughly `[-4,+5]` mm. Point 2 timed
out about 6.43 mm from target. Point 3 then repeated the pattern on bend and
eventually faulted axis 2 at the bounded take-up limit.

`TakeupTransactionArbiter.begin` opens a transaction from every fresh
non-ready plan direction. It has no persistence, cost-margin, productive-motion,
or cooldown gate. MPPI's current reversal term is soft and does not represent
the measured transaction duration; sampled reversing plans therefore appear
beneficial and repeatedly preempt useful motion on other axes.

Implement per-axis response-clocked direction leases. Same-direction and zero
must remain admissible; opposite direction must first beat an actually scored
constrained alternative by absolute and fractional margins, persist across
fresh plans, and satisfy productive-response/cooldown gates. During pending
intent, execute the constrained plan so other axes can progress. Preserve all
existing fail-closed take-up, saturation, freshness, and manager gates. See
`session-20260915-203808-reversal-transaction-limit-cycle.md`.

**Remediation implemented 2026-09-15; simulation validation pending.** The
planner now scores unrestricted and lease-constrained alternatives in the same
candidate batch, and executes only candidates permitted by the scheduler's
current lease or explicit reversal approval. The scheduler requires three
consistent fresh plans, both absolute and fractional cost improvement, three
accepted observations after engagement, and a one-second cooldown before it
can approve an opposite-direction transaction. Approval is consumed when that
single transaction starts; target identity changes, disarm, and faults clear
pending intent. Focused regressions, the complete source package test suite,
style checks for touched Python files, package build, and launch-argument
inspection pass. F-043 remains open until the eight-point sparse simulation
shows that transaction-sign alternation is removed without a safety regression.

**Acceptance result:** `20260915_223716_mppi_sim` failed functionally. It
avoided a terminal fault but reached only one of eight points; see F-044. F-043
therefore remains open.

## F-044 — High — Direction-lease constraint commands the old direction and can lock permanently

**Confidence:** observed; source-confirmed mechanism.

Sparse simulation `20260915_223716_mppi_sim` completed without a controller
fault but reached only point 1. Points 2--8 timed out with 8.84--32.91 mm
minimum errors and 22.20--45.94 mm final errors. For points 3--8, 94--98% of
active plans were direction-lease constrained. The selected commands continued
the old lease direction while the unrestricted plans consistently requested
the opposite direction, driving the tip away from the targets.

The implementation defines the constrained set as “not opposite the lease,”
so it includes continued old-direction motion rather than holding the affected
physical shaft at zero. Its reversal gate additionally requires a 15% decrease
in complete horizon cost. Stable useful reversals in this run improved the
large total cost by hundreds of absolute units but only 2--7%, leaving pending
counts above 100 without approval. The one global comparison also cannot
attribute benefit when multiple axes reverse; point 2 still admitted ten
transactions and repeatedly alternated axis 0.

Replace the sign-only constrained set with explicitly scored physical-axis
hold branches, derive per-axis reversal benefit by ablation, and gate on
predicted geometric descent above the observation-noise floor rather than a
fraction of irreducible total tracking cost. Pending execution must use a
scored command with unapproved reversing shafts exactly neutralized and must
not increase predicted terminal error. See
`session-20260915-223716-direction-lease-regression.md`.

**Initial remediation implemented 2026-09-16; timing regression corrected,
simulation validation pending.** The
planner now converts the unrestricted optimum to physical motor coordinates,
constructs a combined hold plus one single-axis ablation per proposed
reversal, and projects those branches through the hardware contract. The first
implementation evaluated those branches with a second learned-model call. In
`20260916_095434_mppi_sim`, that fixed accelerator/runtime overhead produced
four deadline-zero cycles and a terminal `planner:repeated_deadline_miss` on
point 2; completed plans were 42.092/68.474/71.144 ms P50/P95/max against the
60 ms deadline.

The corrected implementation partitions the existing fixed-size sample bank
between ordinary candidates, the combined hold, and per-axis ablations. All
branches are now evaluated by one projection and one learned-model invocation;
the total population remains at or below the configured sample count. Pending
execution uses the scored combined
hold only when it improves both horizon tracking cost and terminal error over
deterministic zero; otherwise it commands zero. Scheduler admission now uses
per-axis tracking-cost improvement and at least 0.25 mm predicted terminal
benefit, plus the existing persistence, observation, and cooldown gates. The
legacy total-cost fraction is retained only for launch compatibility and
defaults to zero. F-044 remains open until the sparse GPU simulation validates
tracking and deadline behavior.

**Second acceptance result and correction, 2026-09-16.** In
`20260916_100502_mppi_sim`, points 1 and 2 completed and all 190 reported plans
remained below the deadline (37.909/46.301/53.904 ms P50/P95/max). Point 3
then alternated physical axis-2 take-up direction across generations 11--24
and faulted closed on `backlash_takeup_unconfirmed:axis_2`. The planner
reported `hold_branch_applied=True` during these unapproved reversals, but the
selected hold still commanded the reversing shaft.

Source inspection found a coordinate-sign mismatch in the new candidate bank:
it compared firmware motor-axis signs with leases expressed in physical shaft
radians/s. Axis 2 has a negative units-per-RPM constant, so its reversal was
misclassified as continuation and was not neutralized. Reversal classification
now converts motor-axis units to physical rates first. The hold guard also
verifies the paired candidate's projected physical first rate is exactly zero
on every unapproved axis, otherwise it falls back to deterministic zero.
Per-axis evidence is likewise accepted only from a projected zero-rate
ablation. F-044 remains open pending another sparse simulation.

## F-045 — High — Sparse encoder homing does not reset transmitted tendon state

**Confidence:** observed; source-confirmed mechanism.

In `20260916_101403_mppi_sim`, target 1 left simulator transmitted tendon
state at -40,432 counts even though the subsequent position transaction
returned raw tendon feedback to -617 counts / approximately 0.092 logical
bend. Targets 2--8 then commanded exactly zero tendon motion and retained the
same transmitted tendon value. Their home tips remained strongly bent and
differed by roughly `[+9.4,+10.1,-11.8]` mm from target 1's initial tip before
other-axis drift.

The sparse experiment declares home solely from logical POS tolerance, while
the simulator integrates encoder-side and transmitted motor state separately.
The later targets were therefore not independent common-home trials. Add a
simulation-only reset of transmitted motor angle and perturbation history
between sparse points. Do not emulate this as an unobservable hardware reset;
hardware homing must use measured shape/response evidence.

See `session-20260916-101403-sparse-tendon-remanence-audit.md`.

## F-046 — High — Observation-unconfirmed take-up can continue far beyond nominal gap

**Confidence:** observed; source-confirmed mechanism.

**Status:** remediated and representative simulation rerun passed. Response
readiness is now per-axis, and tendon engagement primarily uses
marker-corrected distal bending projected onto the v171 gauge-fixed mode. The
previous target reached tolerance without an unconfirmed excursion or tendon
reversal in `20260916_111659_mppi_sim`. See E-167 and E-168.

Target 1 in `20260916_101403_mppi_sim` increased error from 12.690 to 31.284
mm during a tendon take-up transaction. The catheter and corrected interface
pose had already responded, but the observer did not mark shaft 2
`PROVISIONAL` or `ENGAGED`. A stopped shaft-1 residual of 0.094 rad was above
the 0.01-rad moving threshold but below the 0.10-rad response threshold. The
shared-window `any` gate therefore returned before joint fitting on every
camera update, zeroing evidence for the strongly responding shaft 2 while the
normalized residual grew to 16.778. After shaft-2 reversal reset the window,
its response evidence immediately reached 1.0 and the observer promoted it.

This is a response-observer false negative. `TakeupTransactionArbiter`
continued the fixed physical command because it correctly saw shaft 2 as
pending, but the pending state was stale. The post-reversal response model was
locally accurate and error fell to 9.446 mm, localizing the major excursion to
the shared response-window gate rather than ordinary MPPI rollout.

The implemented correction restricts accumulation waits to shafts that are
still actively moving and fits only the eligible subset. A stopped
provisional shaft's subthreshold residual therefore cannot block another
shaft. Shaft 2 also has an independent distal-bending response channel because
interface-body motion is not a reliable tendon observation in a retracted
proximal configuration. The existing maximum-width travel guard remains the
fail-closed bound; a lower-speed post-nominal confirmation phase remains a
possible later hardening step if the corrected observer still misses genuine
responses.

See `session-20260916-101403-sparse-tendon-remanence-audit.md`.

## F-047 — High — Marker-corrected distal strain is inconsistent with rollout history

**Confidence:** observed proximate failure; source-localized internal cause.

In `20260916_112646_mppi_sim`, exact-zero executed commands on upper-circle
points predicted approximately +1.1 mm z motion over the response interval,
while the exact-model truth plant measured zero motion. The planner therefore
stopped 4.5--8.2 mm below upper targets while rating zero/hold rollouts roughly
1.7--1.9 mm closer than the live observed tip. Lower-circle points in the
second trial reached tolerance, and published marker tips exactly matched
ground truth, excluding a constant perception or frame offset.

UKF corrects interface pose and flexible strain but leaves the learned distal
history state unchanged. Encoder replay and MPPI subsequently relax that
corrected strain toward the unchanged history-conditioned equilibrium. The
truth plant's continuously evolved strain/history pair is self-consistent and
does not produce the same repeated relaxation. The resulting phantom +z
response also suppresses the predicted benefit of releasing inherited tendon
bend: points 2--4 proposed 32--34 axis-2 reversals but none cleared the 0.25 mm
terminal-improvement gate, and axis 2 remained exactly stationary.

Reconcile the corrected strain and history/equilibrium latent before planner
publication, and add a stationary zero-command posterior-rollout regression.
Only after that bias is removed should tendon-reversal admission be retuned or
extended to a longer macro-action value. See
`session-20260916-112646-negative-z-offset-audit.md`.

## F-048 — High — Global closest-path projection deadlocks a nearly complete closed path

**Confidence:** observed; source-confirmed mechanism.

**Status:** remediated and verified in simulation.

In the 3 mm/s `20260916_152629_mppi_sim` run, progress reached
80.599/80.811 mm before the governor entered `PAUSED`. The tip was only
0.505 mm from the circle and 0.522 mm from the active reference, but global
closest-point projection selected the coincident circle-start branch at arc
17.842 mm. The governor interpreted the resulting 62.76 mm backward arc
difference as real lag, exceeding its 5 mm pause gate. Its existing ambiguity
guard handles a closest point ahead of progress but not an earlier coincident
branch. Frozen progress then could not reach the endpoint, while the artificial
lag could not satisfy the 1.5 mm resume gate.

Use an arc-continuous, progress-local projection for phase/lag control and
retain the unrestricted global projection only for geometric path-distance
reporting. See
`session-20260916-152434-152629-grouped-cost-and-endpoint-audit.md`.

The corrected `20260916_154350_mppi_sim` run completed the full 80.811 mm
closed path without entering `PAUSED`, ending at 0.338 mm reference/geometric
error. See
`session-20260916-154350-branch-fix-hardware-readiness.md`.

## F-049 — High — Inconclusive observations revoke credible engagement

**Confidence:** observed; source-confirmed mechanism.

**Status:** remediated and recorded-stream replay passed.

In hardware session `20260916_163300_mppi_demo`, axis 0 produced multiple
correctly directed response estimates with response evidence approximately
1.0 and entered `PROVISIONAL`. Two subsequent observations were inconclusive,
not contradictory: one inferred a same-direction -0.074 rad increment below
the 0.10-rad confirmation floor, and one inferred zero. The observer counted
both as rejection votes, returned the shaft to `TAKEUP`, and immediately
faulted because retained travel exceeded the calibrated maximum-width bound.

The observer now separates `CONFIRMED`, `INCONCLUSIVE`, and `CONTRADICTORY`
responses. Inconclusive evidence preserves provisional engagement and its
confirmation count. Only repeated, sufficiently large opposite-direction
responses fail closed, and they do not restart full-rate take-up. The original
maximum-travel guard remains unchanged before the first credible response.
See E-177 and `session-20260916-163300-axis0-provisional-fault.md`.

## F-050 — High — One marker rejection bypasses the configured rejection tolerance

**Confidence:** observed; source-confirmed mechanism.

**Status:** remediated in source; hardware verification pending.

Hardware session `20260916_165943_mppi_demo` faulted on
`estimator_degraded` immediately after a `marker_outlier` warning reported
rejection 1/3. The runtime marks its global estimator health `DEGRADED` for
every rejected live marker observation. Controller readiness tests this health
before testing whether consecutive rejections reached the configured maximum,
and the active heartbeat converts either degraded result directly into a
latched fault. The separately implemented three-rejection threshold is
therefore ineffective for this path.

Define one explicit transient-rejection policy. Below the configured rejection
threshold, preserve the last valid posterior for control while reporting the
rejected sample and monitoring marker age. Fault at the threshold, or
immediately for a separately represented fatal/nonfinite estimator state.
Do not hide fatal model validity behind the transient allowance. See E-178 and
`session-20260916-165943-hardware-path-audit.md`.

Lifecycle readiness now allows only an explicitly rejected latest marker
update with a nonzero, below-threshold rejection count to use the transient
budget. The third rejection still fails closed, while a degraded accepted
update or another internal degradation remains immediate. Focused tests pass
59/59, the full package suite passes 235/235, and the package rebuild passes.

## F-051 — High — Fixed-rate take-up dominates hardware path execution

**Confidence:** observed behavior; inferred-high-confidence speed mechanism.

**Status:** rotation-rate mitigation implemented; matched hardware verification
pending. Reversal-demand remediation remains open.

The same 162 s active interval opened 162 take-up transactions and froze path
progress for 3,667/4,855 samples. Rotation alone opened 81 transactions; its
transaction direction alternated every time, median transaction duration was
0.899 s, median measured tip motion was 1.516 mm, and 44.4% of sampled
transactions moved the tip by more than 2 mm. Approximately half worsened the
sampled closest-path error. Insertion showed smaller typical but still
multi-millimetre transaction motion.

The response observer accumulates evidence, but actuation remains at one fixed
rate until the first usable camera/UKF response. Motion between physical
engagement and that observation is consequently open-loop. Reducing shaft-1
take-up velocity from 40 to 20 logical units/s is a reasonable isolated first
experiment and should reduce this delay-induced travel. It will not fix the
underlying reversal demand: reversal admission must also be retuned against
transaction cost and actual path-scale benefit. See E-179--E-180 and
`session-20260916-165943-hardware-path-audit.md`.

The hardware profile now uses `[8.0, 20.0, 4.5]`, halving rotation take-up
only. Scheduler thresholds are intentionally unchanged for the matched run.

## F-052 — High — Engagement detection rejects large off-model cross-axis response

**Confidence:** observed false fault; inferred-high-confidence torsional-release mechanism.

**Status:** open.

Hardware session `20260916_172704_mppi_demo` faulted on
`backlash_takeup_unconfirmed:axis_0` even though the terminal insertion
transaction moved the measured tip 7.312 mm and reduced closest-path error
from 6.230 to 1.420 mm. Axis 0 contributed about 95.7% of absolute encoder
travel, while the interface rotated approximately -0.0566 rad about local z.
The response therefore proved physical engagement but did not align with the
nominal insertion Jacobian closely enough to satisfy the current directional
confirmation gate. Raw travel then crossed the unchanged maximum-width bound
and faulted.

Mechanical engagement and model-consistent identification must be separated.
For an isolated pending shaft, a dominant causal input and marker-derived
response clearly above noise should terminate take-up and force a fresh plan
from the UKF-corrected state. Directional consistency should remain required
for backlash-width or Jacobian learning, and the off-model residual should be
recorded separately rather than absorbed into a fixed Jacobian. See E-182 and
`session-20260916-172704-axis0-cross-axis-release-audit.md`.

## F-053 — High — Encoder travel can fault before delayed tendon response arrives

**Confidence:** observed; source-confirmed mechanism.

**Status:** open.

Hardware sparse-point session `20260916_180903_mppi_demo` faulted on
`backlash_takeup_unconfirmed:axis_2`.  The terminal tendon transaction moved
the measured tip about 4.95 mm toward the target, but accepted camera/UKF
updates arrived 91--139 ms after their source stamps.  The nominal gap was
sampled as exhausted only about 89 ms before the fault.  A clearly
suprathreshold distal-bending response was published about 55 ms after the
fault, when the shaft was already `FAILED` and no longer eligible for response
confirmation.

At nominal gap exhaustion, stop the pending physical shaft and enter a bounded
`AWAITING_RESPONSE` phase instead of continuing full-rate take-up through the
sensor/estimator delay.  A response inside the grace window should trigger the
existing provisional zero/replan handoff; absence of response must still fail
closed.  See E-183 and
`session-20260916-180903-sparse-point6-axis2-audit.md`.

## F-054 — High — Simultaneous insertion/rotation take-up does not prevent stored twist

**Confidence:** observed transmission mismatch; inferred-high-confidence
torsional-storage interpretation.

**Status:** open.

Hardware session `20260916_180903_mppi_demo` contained five take-up
transactions that drove insertion and rotation concurrently for approximately
0.5--0.8 s.  Four expressed only 0.5--8.2% of the fixed-Jacobian material-roll
prediction during that overlap.  In the fifth, material roll moved opposite
the requested rotation direction.  Separate insertion transactions with an
effectively stationary rotation encoder produced 0.58--2.46 degrees of
material roll and 1.41--3.06 degrees of off-model local-z residual.

The combined arbiter transaction therefore does not establish rotation
transmission and does not prevent insertion from releasing history-dependent
roll.  Schedule torsion management as its own response-confirmed phase rather
than assuming concurrent take-up makes rotation and insertion jointly
engaged.  Exact elastic windup remains unobservable without an additional
torque/twist state or a calibrated hysteresis model.  See E-184 and
`session-20260916-180903-sparse-point6-axis2-audit.md`.

## F-055 — High — A limit transition can strand an active position transaction

**Confidence:** observed and source-confirmed.

**Status:** remediated in source; Teensy compile, flash, and hardware retest
pending.

During the point-4 home in hardware session `20260919_154754_mppi_demo`, the
tendon reached its lower software boundary while insertion still had
14.886 mm remaining.  The firmware's limit-transition handler stopped all
motors, but the active position tracker did not rebase its progress window or
schedule the residual insertion move.  It consequently reported
`POSITION_TIMED_OUT` 500 ms later even though the command had reached the
device intact.

The firmware now explicitly rebases the progress detector after this known
external stop and schedules only active axes still outside tolerance through
the existing validated correction path.  The original transaction start time
and hard timeout remain unchanged.  Native regression tests reproduce the
interrupted two-axis home and pass; full sketch compilation and a guarded
boundary-crossing hardware retest remain required.  See E-185 and
`session-20260919-154754-coordinate-split-and-home-interruption-audit.md`.

## Unknowns after instrumented shadow planning

- Root cause of the out-of-envelope ENC/POS values: firmware reference state,
  hardware counter state, transport interpretation, or another source.
- Why the previously flashed Teensy processed `CONNECT` only after motor-driver
  power was applied. The new pre-CONNECT predicate-`B` heartbeat and blinking
  LED distinguish main-loop failure from a receive/handshake failure after the
  diagnostic firmware is reflashed.
- The internal phase responsible for rare estimator and planner tails. New
  marker rewind/correction/replay and MPPI phase metrics are implemented, but
  no post-instrumentation live capture exists yet.
- Whether the 127-thread controller pool contributes materially to tail latency
  after runtime-state ownership is fixed; current data proves pool expansion
  but not CPU saturation.
- End-to-end command age and stop latency; passive mode intentionally produced
  no command traffic.
- Root cause of the UKF-session one-rig capture/pairing outage; current summary
  data identifies the boundary but not whether USB, ZED SDK, capture worker, or
  timestamp synchronization initiated it.
