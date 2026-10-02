# O2 paced evolving-snapshot qualification

## Step 1: phase-5 trace failure resolved

Observed root cause: the earlier test command put `/usr/lib/python3/dist-packages`
before the sourced Humble paths. ROS 1 `std_msgs.Header` then constructed
`genpy.rostime.Time` (fields `secs/nsecs`) where ROS 2 expects
`builtin_interfaces.msg.Time` (`sec/nanosec`). Merely preloading
`builtin_interfaces` did not resolve the wrong `Header` binding.
Putting system packages first also selects old system NumPy, missing
`numpy.linalg.vector_norm`. Neither issue requires a production estimator fix.
An explicit ROS 2 Header/Time regression test now diagnoses the import conflict.

Correct package test environment, with venv dependencies first, then ROS,
then system packages used for optional test dependencies:

```bash
cd /home/chen-lab/Yifan/robot-infra
source /opt/ros/humble/setup.bash
source install/setup.bash
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
export PYTHONPATH="src/catheter_control:src/runtime_supervision:src/bringup:src/experiments:/home/chen-lab/Yifan:/home/chen-lab/Yifan/cr-common:/home/chen-lab/Yifan/control:/home/chen-lab/Yifan/cr-venv/lib/python3.10/site-packages:$PYTHONPATH:/usr/lib/python3/dist-packages"
/home/chen-lab/Yifan/cr-venv/bin/python -m pytest -q \
  src/catheter_control/test src/runtime_supervision/test src/bringup/test
```

Use a fresh sourced shell if inherited `PYTHONPATH` already puts ROS 1 paths
ahead of Humble. No packages were installed and no runtime fallback was added.

## Step 2: replay implementation and scope

`runtime_supervision.qualify_paced_compute` extends the existing O0 replay,
not the UKF mathematics. Prefix history is continuous and unpaced; the measured
receipt-time window is paced against a monotonic clock. An isolated CPU float64
reference and concurrent CPU float64 / GPU float32 case consume identical input
events. Encoder thinning, latest pending markers, causal deferral and correction
rate limits retain the O0 semantics. Numerical checks include complete final
state and marker acceptance/reason/observable rank.

The planner consumes replace-only **evolving owned snapshots**, sampled at the
recorded planning rate. Transfer happens outside the snapshot-exchange lock,
is blocking, and counts toward the existing planner deadline. The same planner
persists between solves to retain warm-start/capture memory. Warmup precedes
the measurement epoch; missed timer slots are skipped and reported, not queued.
Recorded trial starts supply targets; accepted observations supply the tip.

MppiConfig is populated from the recorded controller parameters, including
512 samples, four-step 0.8-second coarse prediction, capture settings and gain
risk weights. Recorded diagnostic mean/lower/upper directional gains populate
the existing `EngagedGainSnapshot.scenarios` API. All-engaged transmission is
an explicitly declared planner **workload fixture**: this does not reconstruct
the online take-up arbiter, gain estimation, direction blocking, or permission
to execute commands. RLS remains manifest-selected off. The run is open-loop
recorded input, not a counterfactual reaching experiment.

Measured outputs include stage P50/P95/P99/max/count, input-processing lateness,
owned snapshot publication, transfer, planner duration, planner timer lateness,
source-time snapshot age, invalid plans, gain scenario count, deadline misses
and skipped planner slots. Every plan stays local: no node, publisher, serial
link, arm request or actuator command is constructed.

This is not ROS executor/DDS, camera, disk-recording, heartbeat or manager
qualification. Those still require separately authorized full-stack shadow
measurements. Source imports are used; the installed model wheel is unchanged.

## Reproduction

After the environment setup above, choose a new external output directory:

```bash
/home/chen-lab/Yifan/cr-venv/bin/python -m runtime_supervision.qualify_paced_compute \
  /media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions/attribution_twoaxis_points_grouped_n512_mount01_repeat01_20261002T211138237441Z \
  --model-manifest /home/chen-lab/Yifan/cr_meta_lnn/artifacts/manifests/20260929_175554_grouped_no_rotation_v2.json \
  --limits-file src/control_interface/config/catheter_limits.yaml \
  --start-offset-s 40 --duration-s 10 \
  --output /media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions/compute_optimization_o2_paced_repeat
```

The first target starts at receipt offset 39.335 seconds, so 40 seconds gives
a populated planner snapshot before warmup. The CLI refuses existing output
directories, missing recorded gain/target inputs and detected local controllers.
Earlier incomplete output directories are retained, not overwritten.

## Recorded results

Both bounded windows passed the declared offline gate. Complete final CPU64
state matched the isolated CPU64 reference within the original O2 tolerance
(`rtol=3e-5, atol=2e-6`); camera source timestamps, acceptance, reason and
observable-rank sequences matched. All **240 plans were valid**, each executed
three gain scenarios; no 60 ms deadline misses or skipped planner slots.

| Metric, milliseconds | 40–50 s window P50 / P95 / P99 / max | 72–78 s, third target P50 / P95 / P99 / max |
| --- | --- | --- |
| Planner including handoff | 18.29 / 26.06 / 29.27 / 36.49 | 18.26 / 25.32 / 27.49 / 29.68 |
| Snapshot source age at solve start | 14.27 / 27.67 / 32.10 / 46.63 | 14.38 / 27.44 / 32.32 / 32.37 |
| Handoff | 0.431 / 0.670 / 0.777 / 0.801 | 0.497 / 0.694 / 1.090 / 2.725 |
| Encoder propagation | 1.25 / 2.65 / 3.77 / 6.01 | 1.43 / 2.73 / 3.67 / 6.21 |
| Marker rewind/correction/replay | 9.05 / 17.22 / 21.38 / 24.31 | 9.26 / 15.47 / 17.78 / 20.20 |
| Input event-processing lateness | 0.065 / 9.64 / 15.26 / 25.42 | 0.074 / 9.50 / 14.71 / 20.92 |

Counts: 150/90 plans, 452/272 encoder updates and 181/108 marker updates.
Owned snapshot publication P95/P99 was 0.195/0.252 ms and 0.193/0.283 ms.
Planner timer lateness P99 was 0.282 and 0.204 ms.

Results are in the external session root:

- `compute_optimization_o2_paced_final_20261002/report.json` (40–50 s);
- `compute_optimization_o2_paced_point3_20261002/report.json` (72–78 s).

The preliminary `compute_optimization_o2_paced_40s_20261002` run also passed
numerical/decision checks, but predates explicit publication/skipped-slot
accounting. Incomplete `compute_optimization_o2_paced_20261002` and
`compute_optimization_o2_paced_complete_20261002` directories respectively
record development attempts before correcting velocity-minimum semantics and
before moving the window past the first target; they are not qualification
evidence.

These counts are finite-window evidence, not a zero-failure-rate guarantee.
The marker-update maximum still exceeds the 20 ms estimator period; this run
does not prove every complete owner callback meets 50 Hz. Receipt pacing also
does not model subscription replacement during overrun, DDS/executor queues,
camera capture, logging, recording, manager or heartbeat scheduling. No change
to safety thresholds or deployment defaults was needed to obtain this result.

## Verification and next gate

289 combined controller/runtime-supervision/bringup tests pass, including the
new ROS 2 timestamp-binding check, recorded-config/gain tests and prefix hook
ordering test. `runtime_supervision` rebuild and installed-module help smoke
test pass. This work changed only offline profiling/test tooling and maintained
audit documents; no learned equations, artifacts or production scheduling.

Proceed to **O3 development** using the qualified eager workload as the
reference. Keep runtime compilation outside readiness, and compare compiled
or graph paths against the same evolving-state/gain scenarios and plan
decisions. O2 remains an opt-in candidate until separately authorized
full-stack shadow timing qualification; these replay results do not authorize
hardware actuation or promotion of the installed model wheel.
