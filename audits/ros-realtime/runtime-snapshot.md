# Runtime snapshot

Captured read-only on 2026-09-11. The first query found no application nodes;
the user then launched the stack and a second bounded passive audit captured the
runtime below. No controller was armed and no messages or operational service
requests were sent by the audit.

## Host

```text
Kernel: 6.8.0-136-generic, SMP PREEMPT_DYNAMIC
CPU: Intel Core i7-12700K, 12 physical / 20 logical CPUs, one NUMA node
CPU scaling governor (cpu0): powersave
ROS: Humble
Controller environment: torch 2.5.1+cu121; intra-op 12; inter-op 12
Controller environment: OpenCV 4.11.0; OpenCV threads 20
```

This is not a PREEMPT_RT kernel. `powersave` under Intel P-state does not prove
that the CPU stays at a low clock, but it does mean frequency behavior was not
explicitly configured for this experiment.

## Initial ROS graph

```text
$ ros2 node list
<empty>

$ ros2 topic list -t
/parameter_events [rcl_interfaces/msg/ParameterEvent]
/rosout [rcl_interfaces/msg/Log]

$ ros2 service list -t
<empty>
```

## Live graph and launch identity

The live graph contained exactly these application nodes:

```text
/marker_tracking
/device_serial_com
/manager
/catheter_mppi
/rosbag2_recorder
```

Important launch/runtime settings were:

```text
marker_tracking: minimum_valid_rigs=2, HD720, primary+oblique
catheter_mppi: command_output_enabled=false, adaptation_enabled=false
catheter_mppi interpreter: /home/chen-lab/Yifan/cr-venv/bin/python3
manager: catheter=imricor_test, source timeout=0.5 s
serial bridge: /dev/ttyACM0 at 115200 baud
rosbag output: .../catheter_sessions/20260911_125616_mppi_demo
```

QoS at runtime matched the static model: markers and the controller's device
subscription were best-effort; manager/device command paths were reliable;
manager safety was reliable and transient-local. The recorder added one
subscriber to every control-critical topic.

## Passive 30-second measurement

Raw data:

```text
/media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions/
  20260911_passive_ros_audit/passive_monitor_30s.json
```

The observer was one subscription-only rclpy node with depth-1 queues. It adds
some DDS/executor load, so maxima may include observer/discovery effects.

| Signal | Count | Rate | Period P50 / P95 / P99 / max |
|---|---:|---:|---:|
| markers | 899 | 29.966 Hz | 33.342 / 34.445 / 35.440 / 66.156 ms |
| marker status | 899 | 29.966 Hz | 33.348 / 34.667 / 35.643 / 66.094 ms |
| encoder | 2728 | 90.909 Hz | 10.986 / 11.866 / 12.427 / 13.266 ms |
| position | 2729 | 90.910 Hz | 10.991 / 11.568 / 11.924 / 12.768 ms |
| controller status | 300 | 10.000 Hz | 100.004 / 100.310 / 100.506 / 100.871 ms |
| manager safety | 150 | 5.000 Hz | 200.024 / 200.233 / 200.328 / 200.354 ms |

Marker image-to-observer age was 63.890 ms P50, 65.305 ms P95, 66.417 ms
P99, and 67.122 ms maximum. Marker processing itself reported 4.422 ms P50,
5.535 ms P95, 6.576 ms P99, and 8.155 ms maximum. Rig timestamp skew remained
below 5.809 ms. Every marker diagnostic was `TRACKING`; rejected-frame count
stayed at 4 during the window.

Raw POS/ENC pairing at the observer was tight: arrival skew was 0.204 ms P50,
0.603 ms P95, 1.036 ms P99, and 1.893 ms maximum. Header skew was below
0.562 ms. The controller diagnostic nevertheless had one excursion to
70.564 ms encoder age and 65.990 ms POS/ENC freshness skew; P99 values were
27.659 ms and 21.501 ms. This proves a controller-side freshness tail exists,
but not whether it came from executor wait, the runtime lock, or observer
perturbation.

All 300 controller statuses were `DISARMED`; estimator health was `TRACKING`,
accepted observations increased normally, and consecutive rejections stayed at
zero. No `/teleop/control` or `/manager/control` message was observed.

## Live feedback range anomaly

A separate two-second subscription-only probe observed constant values:

```text
ENC = [1197348480, -1032295180, -24, -122289928,
       -928671744, -882800638]
POS = [503783.1875, -3483996.5, 0.003571875, -51453.367,
       -3134267.5, 154815.0]
```

The configured `imricor_test` position envelope is:

```text
lower = [0, -270, 0, 0, -180, -360]
upper = [40, 270, 15, 80, 180, 360]
```

Despite this mismatch, manager safety was `MANAGER_READY`, serial transport was
ready, and the learned runtime reported `model_valid=True`. Its first three
motor angles were approximately `[940395, -810763, -0.01885]` rad and its
downstream state included approximately `-60807`. The cause of the live values
is unknown; the audit did not alter encoder state.

## Process scheduling during the window

All application threads used normal `SCHED_OTHER`, priority 19/nice 0, with
affinity to CPUs 0–19 and no process isolation.

| Process | Threads | CPU (one core = 100%) | RSS |
|---|---:|---:|---:|
| marker tracking | 82 | 136.1% | 451 MiB |
| catheter MPPI, disarmed | 39 | 88.4% | 482 MiB |
| manager | 33 | 12.1% | 57 MiB |
| serial bridge | 15 | 7.8% | 42 MiB |
| rosbag recorder | 14 | 5.0% | 52 MiB |

The total observed application load was about 2.5 CPU cores, so this disarmed
run does not demonstrate CPU saturation. It does confirm large thread pools and
unrestricted CPU migration. Planning load was absent.

## Tracing availability

`ros2 trace` is not installed as a ROS CLI verb. An instrumented follow-up needs
the ROS 2 tracing packages (or application-level monotonic timestamp probes)
before making scheduler or PREEMPT_RT decisions.

## Non-actuating shadow-planning measurement

Raw data:

```text
/media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions/
  20260911_155049_shadow_planning_audit/passive_monitor_shadow_60s.json
```

Command output and online adaptation remained disabled. The 60-second observer
window contained 133 `ACTIVE`, 464 `DISARMED`, two `READY`, and one `DEGRADED`
controller diagnostic samples, so only about 13.3 seconds of active planning
were captured. No `/teleop/control` or `/manager/control` messages were seen.

MPPI solve time was 22.082 ms P50, 29.059 ms P95, 33.465 ms P99, and
34.508 ms maximum. All 600 diagnostic samples reported zero consecutive
deadline misses. This is comfortably inside the configured 80 ms deadline and
the 66.7 ms nominal planner period for the captured warm-planning window.

Raw device delivery remained healthy: POS/ENC arrival skew was 0.213 ms P50,
0.490 ms P95, 0.702 ms P99, and 1.156 ms maximum. Controller-reported freshness
did not track that transport behavior. Feedback-pair skew reached 54.361 ms at
P99 and 87.493 ms maximum, while encoder age reached 60.306 ms at P99 and
95.001 ms maximum. Both stayed within the current 150 ms gate, but the tail is
materially worse than the passive baseline and requires callback/lock timing to
attribute.

Marker tracking remained 29.833 Hz and all 1,790 marker diagnostics were
`TRACKING`. Processing time was 3.199/4.210/4.779/8.304 ms
P50/P95/P99/max. One controller diagnostic was `DEGRADED`, and the maximum
consecutive rejection count was one; the aggregate monitor did not preserve
the corresponding text reason.

The PID discovery expression captured marker and serial processes but missed
the MPPI process. Consequently this run has no controller CPU, thread-count,
CPU-migration, or context-switch comparison. The next run should resolve the
PID from the active process command line before starting the observer.

## Deployment identity

The manager and serial bridge installed scripts are symlinks to their source
files. The catheter and marker entry points exist in the install tree. The
source worktree contains uncommitted changes, which this audit preserved.

## Instrumented 180-second shadow-planning measurement

Raw data:

```text
/media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions/
  20260911_162940_instrumented_shadow_audit/instrumented_shadow_180s.json
```

The capture contained 1201 `ACTIVE`, 594 `DISARMED`, three `READY`, one
`DEGRADED`, and one `FAULTED` controller status sample. No `/teleop/control` or
`/manager/control` traffic was observed. Device feedback stayed constant at
the validated post-reflash values, and raw POS/ENC arrival skew was
0.204/0.548/0.840/1.517 ms P50/P95/P99/max.

Marker correction was the dominant runtime critical section. Its
per-diagnostic-window maximum lock hold was
36.926/45.883/51.489/96.469 ms P50/P95/P99/max. Encoder runtime-lock wait was
26.648/41.926/48.537/87.700 ms, and plan snapshot wait was
22.160/39.107/44.279/52.142 ms. Plan snapshot hold was only 1.362 ms P95 and
3.390 ms maximum. Encoder timer-start lateness reached 74.750 ms, and complete
encoder callback duration reached 90.856 ms.

The causal symptom occurred twice as `observation_from_future`. Estimator
health was `DEGRADED` twice; aggregate state/reason counts establish one
`DEGRADED` and one `FAULTED` controller status with reason
`estimator_degraded`. This occurred despite prompt controller device
subscription delivery and tight raw observer pairing.

Complete plan callback window maxima were 46.406/67.664/74.363/86.648 ms
P50/P95/P99/max. The controller reported zero deadline misses because the
planner's internal timing excludes pre-solve runtime-snapshot wait and wrapper
work; its maximum reported solve time was 66.832 ms. Thus the current deadline
does not bound the complete callback or enforce the 66.7 ms release period.

The controller process ranged from 39 to 127 threads, averaged 109.4% of one CPU
over the mixed window, migrated across all 20 logical CPUs, and accumulated
262,450 voluntary and 4,340 involuntary context switches. Marker tracking
remained near 29.8 Hz; all but one marker diagnostic were `TRACKING`.

## Post-remediation instrumented shadow measurement

Raw data:

```text
/media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions/
  20260911_165927_instrumented_shadow_audit/instrumented_shadow_180s.json
```

This 180.001-second run used the causal single-owner estimator, replace-only
planner snapshot, 60 ms command-commit budget, and reported PyTorch thread
limits of four intra-op and one inter-op. It contained 1,201 `ACTIVE`, 597
`DISARMED`, two `READY`, and one `WAITING_FOR_MANAGER` status samples. There
were no `DEGRADED` or `FAULTED` states, zero consecutive deadline misses, all
1,801 estimator-health samples were `TRACKING`, and all reported marker updates
were accepted (`accepted`: 1,794; `accepted_noop`: 7). No command traffic was
observed on `/teleop/control` or `/manager/control`.

| Diagnostic-window maximum | Before P50 / P95 / P99 / max | After P50 / P95 / P99 / max |
|---|---:|---:|
| Planner snapshot wait | 22.160 / 39.107 / 44.279 / 52.142 ms | 0.00024 / 0.00061 / 0.00089 / 0.134 ms |
| Plan callback duration | 46.406 / 67.664 / 74.363 / 86.648 ms | 19.013 / 32.792 / 38.942 / 72.949 ms |
| Plan timer lateness | 0.559 / 3.001 / 9.507 / 22.623 ms | 0.306 / 0.506 / 0.609 / 7.168 ms |
| Marker correction work | 36.926 / 45.883 / 51.489 / 96.469 ms | 35.498 / 46.371 / 52.433 / 93.101 ms |

The snapshot exchange therefore removed the measured planner/runtime
contention: P95 snapshot wait fell by more than 99.99%, plan callback P95 fell
by 51.5%, and plan timer-lateness P95 fell by 83.1%. Marker correction cost did
not materially change, as expected.

Encoder freshness tails improved: reported encoder age P99/max changed from
85.838/107.700 ms to 50.492/92.364 ms, and feedback-pair skew P99/max changed
from 76.689/98.735 ms to 43.866/90.699 ms. Median feedback skew increased from
10.722 to 21.374 ms. Marker correction freshness also shifted later:
marker-age P50/P95 increased from 14.691/22.669 ms to 25.919/54.907 ms, while
the maximum remained nearly unchanged at 102.924 ms. All remained within the
configured 150 ms gates during this run.

The single owner still has no 20 ms worst-case execution bound. Its callback
window maximum was 41.720/54.134/60.952/107.173 ms P50/P95/P99/max, and timer
lateness was 22.665/36.980/47.542/87.849 ms. These are not directly comparable
to the old encoder-only callback because the new callback also performs marker
correction and may drain encoder twice. They do show that synchronous marker
correction overruns the nominal estimator period, even though it no longer
delays planning through a shared lock.

One plan callback reached 72.949 ms, above both the 60 ms command-commit budget
and 66.7 ms release period, while the aggregate status reported zero deadline
misses. The capture cannot correlate that peak with callback phase or armed
state; it may be a non-planning return path or post-commit publication. This is
not evidence that an overdue command was accepted, but it leaves complete
callback bounding unresolved. Event-correlated phase timing or a controlled
delay injection is the least invasive next measurement.

The controller process thread maximum fell from 127 to 66, CPU use from 109.4%
to 95.2% of one core, voluntary context switches from 262,450 to 230,634, and
involuntary switches from 4,340 to 3,581. It still migrated across all 20 CPUs.
Marker tracking remained about 29.76 Hz and raw POS/ENC arrival skew remained
healthy at 0.193/0.516/0.845/1.450 ms P50/P95/P99/max.
## EKF and UKF shadow comparison — 2026-09-11

The post-fix EKF capture is
`20260911_174958_instrumented_shadow_audit/instrumented_shadow_180s.json`.
It included 1,201 `ACTIVE` status samples with command output disabled. All
1,800 status samples reported estimator and marker `TRACKING`, every reported
marker update was accepted, and the rejection streak remained zero. Marker
owner diagnostic-window maxima were 25.797/38.262/50.696/95.140 ms
P50/P95/P99/max. Combined estimator callback maxima were
31.788/47.268/63.593/112.845 ms. One isolated planner deadline miss was
visible (`consecutive_deadline_misses` maximum one); no repeated miss or fault
latched.

The UKF capture is
`20260911_175523_instrumented_shadow_audit/instrumented_shadow_180s.json`.
Every reported correction was accepted and estimator health remained
`TRACKING`. Marker owner maxima improved to 13.444/20.291/26.615/47.277 ms and
combined estimator callback maxima to 18.046/25.324/33.724/61.557 ms. This run
did not exercise MPPI: status remained disarmed or ready with `target_missing`,
and it recorded no `ACTIVE` samples or plan metrics. Its process CPU and
contention therefore cannot be compared directly with the active GN/EKF runs.

The UKF session also contained a separate 14.900 s synchronized-marker outage.
Twenty `FEEDBACK_STALE` diagnostics reached 14.780 s age; synchronizer drops
rose by about 454 while detector rejections rose by one. This is consistent
with one 30 Hz rig continuing while its peer stopped producing pairable frames.
The downstream controller correctly entered `marker_feedback_degraded`; the
dataset cannot identify whether USB, the SDK, a capture worker, or timestamp
pairing initiated the outage.

Both filters retained sub-millimetre residuals with no correction rejection.
The sequential sessions are not a controlled accuracy comparison: EKF marker
RMS-after was 0.373/0.390/0.397/0.416 mm P50/P95/P99/max, while UKF was
0.477/0.511/0.525/0.541 mm, but camera timing and physical observation noise
differed and UKF lost vision for almost 15 s. Both covariance traces grew far
above their initial value while observable rank was usually nine, consistent
with isotropic process noise accumulating in a weak or unobservable direction;
time-series covariance is needed before calling this divergence.

### Replacement matched UKF run

The later capture
`20260911_181555_instrumented_shadow_audit/instrumented_shadow_180s.json`
supersedes the two non-planning UKF attempts for estimator/planner comparison.
It contained 1,201 `ACTIVE` samples, 1,800 estimator and marker `TRACKING`
samples, only accepted corrections, and no command messages. Vision remained
continuous at 29.072 Hz with a 101.331 ms maximum marker gap.

Compared with the matched EKF run, UKF reduced marker-owner window maxima from
25.797/38.262/50.696/95.140 ms to 16.540/29.619/41.213/80.965 ms
P50/P95/P99/max. Combined estimator callback maxima changed from
31.788/47.268/63.593/112.845 ms to 21.732/37.901/53.792/112.414 ms. Typical
execution improved, but the worst tail did not. Plan-callback window maxima
were 16.423/36.193/46.250/83.475 ms for UKF versus
22.704/33.904/43.248/84.756 ms for EKF. Both had isolated deadline misses and
neither latched a repeated-miss fault.

UKF process CPU was 72.3% of one core versus EKF 86.9%. Absolute marker RMS
after correction was higher for UKF (0.477 mm median versus 0.373 mm), but its
pre-correction RMS was also higher (0.522 versus 0.390 mm). These sequential
stationary sessions do not hold physical observation and target geometry
constant, so they establish robust sub-millimetre tracking, not a statistically
controlled accuracy ranking.

## Post-comparison source remediation and verification

After the matched UKF audit, the 20 Hz marker limiter was changed from
completion-relative scheduling to an absolute release phase with skipped late
releases. EKF/UKF diagnostics now separate conventional pre-update innovation
NIS from the legacy post-fit normalized residual, expose covariance eigenvalue
bounds, and prevent random-walk process noise from accumulating in the current
unobservable gauge. Marker tracking now reports per-rig liveness and an
explicit `PAIRING_STALE` state. Marker correction and MPPI each expose internal
phase times to localize the remaining 80--112 ms tails.

These source/test changes were subsequently measured in the fresh-start,
non-actuating UKF shadow run below.

### Post-remediation UKF verification run

The capture
`20260911_185444_ukf_post_remediation_shadow_audit/instrumented_shadow_180s.json`
contained 1,202 `ACTIVE` samples, no command traffic, continuous estimator and
marker `TRACKING`, and no marker rejection. Accepted observations advanced by
3,596 over 180.007 seconds, approximately 19.98 Hz, verifying the scheduler
fix. Marker output was 29.933 Hz and both rig frame-age maxima were 7.353 ms;
no worker error or pair-starvation diagnostic occurred.

Covariance trace remained 7.275–8.307, versus 90.8–102.7 in the matched prior
run. The new conventional innovation NIS per degree of freedom was
0.0147/0.0193/0.0225/0.0269 at minimum/P50/P95/maximum, demonstrating that the
filter/noise model remains substantially over-dispersed despite removing gauge
growth.

UKF correction itself measured 8.428/15.089/19.028/31.574 ms and causal replay
7.161/12.944/16.612/26.016 ms at P50/P95/P99/max. Total marker work reached
60.434 ms. The one planner deadline miss is localized to learned rollout:
90.978 ms of its 92.390 ms total. Command output remained disabled and the
isolated miss produced the intended zero-plan state without a repeated-miss
fault.

### Offline rollout-contention follow-up

An isolated 500-plan sweep did not reproduce the live tail: all 1/2/4-thread
rollout maxima stayed below 19 ms. A second non-actuating benchmark ran the
same deployed MPPI artifact concurrently with periodic UKF correction and
causal replay in one process. With the previous four-thread intra-op pool,
rollout P50/P95/P99/max increased to 13.60/27.60/31.84/41.26 ms. Two threads
reduced this to 11.89/22.66/25.01/26.31 ms; one thread was similar at
12.02/22.46/24.83/27.67 ms. The default is therefore reduced to two intra-op
and one inter-op thread. This supports a shared-pool contention mechanism but
does not reproduce or close the 90.98 ms live scheduling tail.

### Live 2/1-thread verification

The capture
`20260911_191342_ukf_post_remediation_shadow_audit/instrumented_shadow_180s.json`
verified the selected pool with 1,106 `ACTIVE` samples and 694 disarmed samples.
All 1,800 estimator statuses were `TRACKING`, every marker update was accepted,
no deadline miss or rejection occurred, and command traffic remained zero.

Plan P50/P95/P99/max was 25.326/33.123/39.205/43.050 ms and learned rollout
23.626/31.222/36.014/40.829 ms. The complete plan-callback maximum fell from
93.040 ms in the preceding 4/1 run to 60.334 ms; timer lateness remained below
1.306 ms. Marker-total maximum fell from 60.434 to 34.322 ms and combined
estimator-callback maximum from 83.036 to 61.069 ms. The change therefore
removed the observed deadline failure and improved both planner and estimator
tails over this window.

Accepted observations advanced by 3,597 over 180.003 seconds (19.98 Hz).
Vision remained `TRACKING` at 29.889 Hz with no worker errors. One marker gap
reached 134.320 ms but remained below the controller's 150 ms feedback gate.
Covariance trace remained 7.095–8.432 and NIS/DoF remained far below one.

Controller thread count fell from 64 to 46, while aggregate measured CPU rose
from 80.2% to 130.0% of one core. Because active durations and system load were
not controlled between these sequential sessions, this CPU increase is
observed but not attributed to the thread-cap change. Both controller and
marker processes still migrated across every logical CPU; marker tracking
retained 82 threads.
