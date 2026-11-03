# Real-time deployment gap audit: reduced filter + batched MPPI

2026-11-02. Static, read-only audit of source, configuration and evidence.
Nothing was launched, built or run on hardware. Claims are marked **O**
(observed in source or evidence) or **I** (inferred). Line numbers refer to the
working trees on 2026-11-02, which include uncommitted changes.

## Bottom line

Every piece has been validated **offline and separately**:
- the reduced filter (whole recorded session, step P99 49.7 ms without a planner);
- the ROS shadow observer (bag replay, exact equivalence);
- the batched GPU forecast (518×4 candidates, P99 3.8 ms);
- the planner adapter (102/102 valid plans offline, P99 about 22 ms);
- the camera pipeline and tendon start-up characterization.

**Nothing connects them live.** `catheter_mppi` can only load the October v171
runtime. The reduced filter runs only in the shadow observer process, which
publishes status and nothing else. The October planner configuration, freshness
limits and arming path are incompatible with the new backend at 5 Hz. The
filter's priors and fault handling are tied to one October recording.

Most of the remaining work is offline software. Lab time is needed for three
things: live timing under full load, re-fitting the camera-frame priors (axis
and R) on a new session, and qualifying the closed-loop command path.

## Intended live path and where it breaks

```text
ZED pair -> marker_tracking -> /shape_tracking/markers ─┐
Teensy -> device_serial_com -> /device/state ───────────┤
                                                        v
            [MISSING] reduced filter inside catheter_mppi's estimator owner
                  (today: separate shadow process, String status only)
                                                        |
            [MISSING] plan-time root -> ReducedFilterRolloutBackend
                  (today: planner gets a v171 RuntimeState snapshot)
                                                        |
            CatheterMppi (refused mask supported, uncommitted)
                  [CONFLICT] lambda guard, take-up, gain scenarios, 15 Hz/0.15 s limits
                                                        |
            /teleop/control -> manager -> /manager/control -> firmware
                  [OPEN] legacy firmware restarts all motors on any start;
                         tendon start dead time 42–55 ms
```

## Gaps

Severity: **S** = blocks a live non-actuating shadow session; **C** = blocks
closed loop; **N** = nice to have. "Offline" means it can be done before the
next lab session.

### A. Controller composition (robot-infra `catheter_control`)

| ID | Gap | Evidence | Sev | Offline |
| --- | --- | --- | --- | --- |
| A1 | No selector for the reduced filter/backend. Loading goes only through `load_runtime_pair(model_manifest)`, which accepts `v171_streaming` or `experimental_joint_engagement` | O: node.py:309-314; orchestration/runtime.py:67-115; cr_meta_lnn deployment/bundle.py:118-121, 214-221 | S | Yes |
| A2 | The filter state is in the wrong process. `root(stream, plan_ns)` needs the live stream object; the controller publishes a v171 `RuntimeState` snapshot. `ProcessPlanner` cannot carry CUDA roots and loads a v171 bundle in its child | O: reduced_filter_shadow.py:63-136; node.py:1026-1036, 1435; reduced_filter_stream.py:214-230; planner_process.py:140-175 | S | Yes |
| A3 | The plan path assumes v171 roots: `markers_for_state`, `motor_angle_rad`/`clone_to`, `root_state.lambda_value`, `tip_readout_offset_m` | O: node.py:~1013-1024, ~2121, ~2163; mppi.py:1650-1653, 1698 | S | Yes |
| A4 | Planner configuration conflicts. The lambda guard 8.0 fails every plan (`engaged_gain_lambda_prediction_missing`). Gain scenarios plus `refused` gives `invalid_rollout`. `always_engaged_rollout` makes the backend raise. Tail steps must be 0 | O: controller/base.yaml:44; mppi.py:1636-1644, 1677-1682; node.py:551; reduced_filter_stream.py:243-251 | S | Yes (stack layer) |
| A5 | Shadow planning requires arming. `_plan_tick` returns unless armed, and arming needs `MANAGER_READY` with fresh device feedback and starts task-GC deferral | O: node.py:1949, 1652; safety/lifecycle.py:144-192 | S | Yes (dry-run path) |
| A6 | Kernel `prepare()` is not wired. It must run on the first tracking root before readiness | O: node.py:~2153; reduced_filter_stream.py:228 | S | Yes |
| A7 | Play is modelled twice. The October backlash snapshot, take-up transaction and direction lease would stack on the filter's own play memory | O: node.py:2142-2149; reduced_filter_stream.py:194-198 | C | Decision, then yes |
| A8 | Estimator health gates read the October runtime. A status mapping is needed (tracking → TRACKING; hold → rejection streak with zero hold; fault → latched; not_initialized → INITIALIZING). `root()` refuses any consecutive rejection, which is stricter than readiness's transient tolerance | O: safety/lifecycle.py:144-192; node.py:1117, 1283-1295 | S | Yes |

### B. Timing, freshness and cadence

| ID | Gap | Evidence | Sev | Offline |
| --- | --- | --- | --- | --- |
| B1 | No 5 Hz profile. All performance profiles are 15 Hz with a 60 ms deadline | O: config/performance/*.yaml | S | Yes |
| B2 | `command_timeout_s = 0.15` is shorter than the 0.2 s plan period, so `planned_command_stale` latches. `mode_claimed` is set even with output disabled. No check ties the timeout to the plan period | O: parameters.py:183; node.py:213-227, 1993, 2524-2536, 2635 | S | Yes (profile plus guard) |
| B3 | `marker_timeout_s = 0.15` against images every 100–133 ms, steps up to 75 ms, and held images. I: trips `accepted_marker_stale` | O: parameters.py:179; I | S | Yes, reviewed parameter |
| B4 | At 30 fps the 100 ms image rule (`<`) takes every 3rd or 4th frame irregularly (100 or 133 ms). The filter was validated at exactly 100 ms | O: reduced_filter_stream.py:71; simulated selection on jittered 30 fps stamps | S | Yes, atomic replay check on the original 30 fps markers |
| B5 | Live timing is unmeasured. Step P99 49.7 ms (max 75) comes from a replay without the planner, camera or tracking load. Cold start takes 390 ms, and the first step 108 ms. A queue of 3 dropped an image at startup | O: GATE3_REDUCED_FILTER_SHADOW.md | C | Warm-up offline; measurement needs the lab |
| B6 | In-flight command latency is not compensated. Compensation gives −27 to −71% error, but the declared 0.8 s criterion failed because of start dead time | O: GATE3_BATCHED_FORECAST_PLAN.md §18 | N | Separate atomic change |

### C. Filter readiness for a new session (cr_meta_lnn)

| ID | Gap | Evidence | Sev | Offline |
| --- | --- | --- | --- | --- |
| C1 | The world axial-direction prior was fit in the October camera frame (Oct 8 images 445–485), with prior std 0.05. A new registration or mount rotates it | O: real_native_memory_bootstrap.py:65, 84, 95-98 | C (biases the shadow) | Procedure offline; re-fit needs session data |
| C2 | Marker covariance R (12×12, world frame) was learned on the October camera setup and at a different image rate | O: real_native_memory_bootstrap.py:62, 69-70 | C (biases the shadow) | Procedure offline; re-fit needs held-pose data |
| C3 | Source identity is rigid and partial. Geometry and R are hash-checked (so a re-fit is refused); profiles, widths and axis are not (so a swap passes silently) | O: :67; real_observed_jacobian_bootstrap.py:26-29 | N | Yes: per-session manifest |
| C4 | Profiles and the tendon support are absolute in encoder coordinates. Each firmware boot restarts the counters at the current pose, so the board must boot at home | O: project memory (2026-11-02 reflashes); STATUS.md | S | Procedure: pre-session home check |
| C5 | No marker quality gate. Confidence, reprojection error and rig count are discarded | O: reduced_filter_shadow.py:92; estimator_owner.py:18-47 | C (log it for the shadow) | Yes |
| C6 | Faults are final, with no reinitialization. Triggers: 3 rejections, an encoder gap over 0.1 s, tendon outside [0.000893, 11.27] mm, any rotation change above 1e-12 rad, or history loss (4096 samples, about 45 s without a commit) | O: reduced_filter_stream.py:26, 61-63, 84, 116-118, 142-145; reduced_law_filter.py:588-603 | C | Yes: policy and tests |
| C7 | No readiness criterion for startup at an arbitrary pose. Tip-J was not learned until about image 120 (about 12 s) and needs tendon excitation | O: GATE3_FORECAST_SPIKE_DIAGNOSIS.md:54; distal_deformation_runtime.py:146-148 | C | Specify and test on recorded segments |
| C8 | NaN in the K=2 pair cost when both law weights underflow (0/0). `argmin` then picks the NaN pair first | O: reduced_law_filter.py:325-330 | N | Yes, with an equivalence check |
| C9 | `ReducedFilterStream.process` status transitions (fault latch, history-lost fault, initialization retry, drop then tracking) are untested | O: tests/test_reduced_filter_stream.py, test_reduced_rollout_backend.py (19 pass) | S | Yes |

### D. Command path, safety and firmware

| ID | Gap | Evidence | Sev | Offline |
| --- | --- | --- | --- | --- |
| D1 | Tendon range. The `imricor_test` bend axis is 0–15 mm, and the filter faults above 11.27 mm. A controller-only limits profile with an upper limit of about 10.8 mm needs no safety-code change. Passing it to the manager as well is a safety-configuration change that needs review | O: control_interface/config/catheter_limits.yaml; control.launch.py:214-215; hardware_contract.py:232 | C | Yes |
| D2 | Velocity mode republishes the first 0.2 s action at 100 Hz, and stop-confirm-replace does not apply. Stops still come from zero decisions, deadline misses (a whole 200 ms period at 5 Hz), `vel_min` rounding and holds. The legacy firmware restarts all active motors on any start or reversal | O: node.py:2062-2070; teensy_tekceleo.ino:2288-2312 | C | Analysis offline |
| D3 | Firmware. Production (`driver_native_evidence_v1`, 2026-11-02 17:38) was built from an uncommitted tree. The on-the-fly velocity image has one session of evidence (one transient STALL, one 99 ms speed change, no legacy baseline, no closed loop) and needs separate qualification | O: TENDON_MOTOR_START_TEST_PLAN.md:623-633; git status | C | Commit provenance offline; qualification needs the lab |
| D4 | The manager's deadman input is not wired: the flag stays `False` and its callback is commented out. Pre-existing; noted, not changed | O: manager.py:107, 334-335 | N | Review only |
| D5 | Task GC is correct: `base.yaml` is true, 300 s, 256 MiB, and every stack includes it. An in-controller filter inherits it; the separate observer does not | O: controller/base.yaml:3-5 | — | — |

### E. Build, provenance and sensing

| ID | Gap | Evidence | Sev | Offline |
| --- | --- | --- | --- | --- |
| E1 | The shadow and replay entry points are in setup.py but not installed (`install` is from Oct 11), and there is no launch wiring | O: runtime_supervision/setup.py:33-34 | S | Yes |
| E2 | Everything is uncommitted across five repositories. The `mppi.py` refused mask is already live through the symlink install | O: git status 2026-11-02 | S (provenance) | Yes, when the user asks |
| E3 | Camera: simultaneous capture stalls at 60 fps are undiagnosed. Diagnostics are deployed (wheel 2026-11-02). The fused-stamp correction gave no measurable filter benefit | O: CAMERA_PIPELINE_PROFILE.md | N at 30 fps | Lab |
| E4 | Inputs differ between paths. The shadow applies only `marker_measurement`; the controller also applies `CausalMarkerSchedule` and marker diagnostics | O: reduced_filter_shadow.py:89-97; node.py:672, 754-756 | S | Yes |

## Decisions recorded (user, 2026-11-02)

| # | Decision |
| --- | --- |
| 1 | The filter runs inside `catheter_mppi`'s estimator owner, with `planner_process_enabled=false` |
| 2 | The filter's play memory alone owns play and take-up. October take-up, backlash compensation and engagement are disabled in the Gate 3 stack |
| 3 | Lambda first-step guard set to 0 at first |
| 4 | Approved: a dry-run plan path, allowed only with `command_output_enabled=false`, claiming no mode and needing no manager |
| 5 | Approved: 5 Hz limits (deadline 0.05 s, command timeout 0.25–0.30 s with a guard above the plan period, marker timeout tied to the root age). Control frequency may increase if the end-to-end delay turns out much lower |
| 6 | The 10.8 mm tendon upper limit stays controller-only for now. With full-range data, a single limit will be maintained |
| 7 | 90 ms image-selection threshold for now |
| 8 | Try to integrate the on-the-fly velocity firmware. These decisions may change after hardware tests |

## Decisions needed from the user (as originally posed)

1. **Where the filter runs.** Recommended: inside `catheter_mppi`'s estimator
   owner, with `planner_process_enabled=false`, publishing an immutable plan
   root under the existing snapshot lock (A2). The alternative, building roots
   in the planner child, needs IPC of the filter state.
2. **Owner of play and take-up.** Recommended: the filter's play memory alone,
   with October take-up, backlash compensation and engagement disabled in the
   new stack (A4, A7).
3. **Lambda guard.** Disable it (0) for the first sessions, or define a guard
   in Gate 3 lambda units (A4).
4. **Shadow planning without arming.** Recommended: a new dry-run plan path,
   allowed only when `command_output_enabled=false`, that claims no mode and
   needs no manager (A5). It would not reuse the arm path.
5. **5 Hz limits.** For example `plan_rate_hz=5`, deadline 0.05 s,
   `command_timeout_s` 0.25–0.30 s with a guard above the plan period, and
   `marker_timeout_s` tied to the 250 ms root age (B1–B3). These are reviewed
   parameter changes.
6. **Tendon limit.** A controller-only limits profile at about 10.8 mm. Should
   the manager get it too, as a reviewed safety-configuration change (D1)?
7. **Image selection at 30 fps.** A 90 ms threshold or every 3rd frame, tested
   as one atomic replay check (B4).
8. **Firmware for closed loop.** Legacy, with its all-motor restarts, or a
   qualification campaign for the on-the-fly image (D2, D3).

## Proposed offline work, in order

Each item is atomic and short-segment first, with its formulation shown
before implementation as required.

1. **Shadow observer deployable (E1, C9, C5-logging, E4).** Rebuild the
   install, add an opt-in research-session launch argument for the observer,
   add stream status-transition tests, log marker quality, and use the same
   marker admission as the controller. Non-actuating.
2. **Image selection at 30 fps (B4).** Replay check on the session's original
   30 fps markers (3620 images): the 100 ms rule (actual 100/133 ms) against
   a 90 ms threshold.
3. **Startup warm-up and queue (B5, A6).** Warm the kernels and the first step
   before ready, and use a startup-only queue allowance. Equivalence against
   the 16-queue replay.
4. **Controller integration, non-commanding (A1–A3, A5, A6, A8, B1–B3).**
   - A backend selector in `orchestration/runtime.py`, with the filter inside
     the estimator owner.
   - The dry-run plan path.
   - A Gate 3 stack layer and a 5 Hz profile.
   - Tests using the real backend inside `CatheterMppi` and the node.
   - Verified by bag replay, non-actuating.
5. **Fault and reinitialization policy (C6), readiness criterion (C7), NaN
   cost (C8).** Each is a separate formulation with recorded-segment checks.
6. **Procedures for the lab:** a home check at boot (C4), axis and R re-fit
   (C1, C2), and the controller-only tendon limit profile (D1).

## Step 1 results (2026-11-02): shadow observer deployable

Done, uncommitted, non-actuating.

**Changes:**
- `cr_meta_lnn/tests/test_reduced_filter_stream_status.py` (new, 6 tests).
  It checks `ReducedFilterStream.process` transitions against a scripted
  estimator (C9):
  - initialization retries after a rejected root;
  - a held image's motion stays in the next interval, and tracking commits;
  - the third rejection latches a fault, and later images never step;
  - history loss faults every pending image;
  - an encoder gap is `not_initialized` before initialization and `holding`
    after;
  - a dropped image is counted and tracking continues.
- `reduced_filter_shadow.py`:
  - per-image marker quality (confidence, reprojection error, rig count) and
    the latest `marker_status` diagnostic level, written to report rows.
    These are logged only; the filter still receives points alone, enforced
    by a test (C5, logging part);
  - code identity (module file, commit, dirty flag) in the report.
- `research_session.launch.py`:
  - opt-in `reduced_filter_shadow:=false|true`, writing to
    `<session>/reduced_filter_shadow/`, capped at 3600 s;
  - `research_code_root` (default `/home/chen-lab/Yifan`), prepended to
    `PYTHONPATH` for the observer process only;
  - the controller command is unchanged.
- `session_recording.py`: an optional observer role. It is started after
  the recorded stack and stopped first, so its report is written. Its exit
  is recorded in `observer_exits` and never fails the recording.
- `colcon build --symlink-install` of runtime_supervision, bringup and
  experiments. The `reduced_filter_shadow` and `reduced_filter_replay`
  entry points are now installed (E1).

**Matched marker admission (E4) is deferred to step 4.** The controller adds
newest-only buffering, a 20 Hz causal schedule and a marker-diagnostic fault,
and has no ROS-level quality threshold. Copying that admission into the
observer would change the filter's inputs, and step 4 moves the filter into
the controller.

**Tests:**
- cr_meta_lnn stream and backend tests: 25 passed.
- Shadow tests: 16 passed. The suite has to run from an isolated copy,
  because the sibling `test_engagement_g4.py` imports a missing
  `cr_meta_lnn.tests` module. That is pre-existing.
- Research-session launch and recording supervisor tests: 46 passed.
- flake8: no new findings beyond the files' existing continuation-indent
  style.

**Installed smoke run.** Recorded replay in domain 214 with the
`/reduced_filter_replay` prefix, images 3–30, running the observer command
exactly as the launch composes it:
- 28 images received, 1 initialized and 18 tracking, no faults;
- marker quality present in the rows;
- code identity recorded: cr_meta_lnn `3fb5551`, dirty.

**New findings:**
- **P1, packaging (blocks step 4).** The `cr-venv` `cr_meta_lnn` is the
  promoted deployment wheel (2026-10-05). It contains only `deployment`,
  `networks` and `artifacts`, so the controller cannot import the reduced
  filter. The observer works around this with an explicit source-tree
  `PYTHONPATH`. The controller needs a decision: promote the filter's
  runtime modules into a deployable wheel, or run the controller from the
  source tree as well.
- **B4 is worse than first recorded.** Even on the 10 Hz recording, the
  strict 100 ms rule skipped 9 of 28 images, because recorded spacing
  jitters just under 100 ms. Several intervals became 200 ms. Step 2 (the
  90 ms threshold) addresses this.

## Step 2 results (2026-11-02): image selection at 30 fps

Approved and run in 41 s. Evidence is in
`catheter_sessions/gate3_image_period_check_20261102/`: the script, `results.json`
and `run.log`.

**Setup:**
- Both arms select from the Oct 8 session's original 30 fps valid markers
  with the aligner's rule, starting at the checkpoint image: strict 100 ms
  (current) against 90 ms.
- Packets come from the archived preparation owner.
- Both arms continue from the same deep-copied baseline checkpoint at
  images 430 (segment C) and 956 (segment B), with settings as in
  `reduced_filter_session.py --gain physical`.
- Score: marker-only readouts exist only for the archived 10 Hz images. So
  each arm's prediction at every archived image in the segment comes from
  its latest committed root. The targets are the same for both arms (79).

| Segment | Arm | Images | Gap min / median / max ms | Holds | Step P50 / P95 / max ms | RMS at targets, mm | Median horizon ms |
| --- | --- | ---: | --- | ---: | --- | ---: | ---: |
| C | 100 ms | 33 | 100.0 / 116.6 / 133.5 | 0 | 35.2 / 41.1 / 42.6 | 0.273 | 100.0 |
| C | 90 ms | 39 | 99.9 / 100.0 / 100.1 | 0 | 36.1 / 41.4 / 42.9 | 0.284 | 100.0 |
| B | 100 ms | 34 | 100.0 / 100.1 / **166.5** | 0 | 35.9 / 41.3 / 43.9 | 0.296 | 66.6 |
| B | 90 ms | 40 | 99.9 / 100.0 / 100.1 | 0 | 37.3 / 41.7 / 43.8 | 0.295 | 100.0 |

Pooled: RMS 0.2849 (100 ms) against 0.2895 mm (90 ms), a difference of
+0.0047 mm. 43 targets were better and 33 worse with 90 ms (sign test
p = 0.15).

| Declared check (90 ms arm) | Result |
| --- | --- |
| No new hold or fault | Pass |
| No gap over 134 ms | Pass: max 100.1 ms. The 100 ms arm had a 166.5 ms gap |
| Paired RMS not worse than +0.005 mm | Pass, narrowly (+0.0047 pooled). Segment C alone is +0.011 mm |
| Step time | Unchanged per step. About 18% more images, so about 18% more estimator CPU |

**Reading:**
- 90 ms gives the regular 100 ms spacing the filter was validated at.
- The error metric favours the 100 ms arm. Its irregular frames often put
  a root closer to the target: median horizon 66.6 against 100 ms in B.
  The small RMS differences are therefore not evidence that either arm
  estimates better.
- Over the whole session, the 90 ms selection overlaps the archived 10 Hz
  frames by 40%. The archive used a different picking rule, so the two are
  not identical.

**Applied:** the observer's `--minimum-image-period-ms` default is now 90
(test added; shadow tests 17 passed). The `load_stream` default (100 ms) is
unchanged for existing callers. Step 4 will select 90 ms for the controller
explicitly.

## Step 3 results (2026-11-02): start-up without dropped images

Approved. Evidence is in `catheter_sessions/gate3_startup_queue_check_20261102/`.

**Diagnosis: warm-up would not help.** Three fresh `load_stream` streams in
one process, each fed recorded images in arrival order, without ROS:

| Stream | Initialization ms | First step ms | Later steps ms |
| --- | ---: | ---: | --- |
| First (cold process) | 354.8 | 102.8 | 30–35 |
| Second | 358.3 | 98.2 | 31–36 |
| Third (image 400 start) | 349.3 | 102.5 | 32–36 |

The cost is per stream, not process cold start, so warming up before ready
was dropped. The first step is probably the readout factory's one cold fit;
that is inferred and not separately timed.

**Change:**
- `EncoderImageAligner(..., startup_pending_images=None)` raises the
  pending bound until the stream has tracked its first image and the queue
  fits the steady bound.
- `ReducedFilterStream.process` calls `end_startup_if_caught_up()` after
  each commit.
- `load_stream` passes the option through. The default is `None`, so
  existing callers keep a single bound.
- The observer adds `--startup-pending-images` (default 8; 0 disables) and
  records both bounds in its report.
- 3 new stream tests; the cr_meta_lnn stream and backend tests pass (28 in
  total), and the shadow tests pass (17).

**ROS replay check.** Domain 214, recorded images 3–40 at the recorded rate,
90 ms selection, 66 s for all three arms:

| Arm | Dropped | Outputs | Max arrival-to-output | Against queue 16 |
| --- | ---: | ---: | ---: | --- |
| a: queue 3, no allowance | 1 | 36 | 402 ms | Image missing; up to 0.025 mm and 0.027 lambda different (reproduces the earlier failure) |
| **b: queue 3, start-up 8** | **0** | 37 | 388 ms | **Bit-identical** |
| c: queue 16 (reference) | 0 | 37 | 369 ms | — |

All arms skipped the same 1 image under the 90 ms rule, from recorded
spacing below 90 ms.

**Still true:**
- The first outputs arrive up to about 0.4 s late, which is intrinsic to
  initialization. Readiness for control must wait for catch-up; that is
  part of step 4's readiness mapping and C7.
- If the filter never catches up (overload), the start-up bound persists.
  Live timing in L1 will show whether that can happen.

## Step 4 contract (declared 2026-11-02, before implementation)

**Packaging (user):** the source tree is used for now. The dry-run
controller gets the workspace on `PYTHONPATH` for its process only.

**The single authorized addition** is a non-commanding dry-run controller:
- It is selected by `estimator_backend: reduced_filter_dry_run` in a new
  stack, launched by `control.launch.py` as the `catheter_mppi` node with a
  separate executable class.
- It runs the reduced filter in its own estimator-owner timer.
- It builds the plan-time root under a stream lock and plans with
  `CatheterMppi` over `ReducedFilterRolloutBackend` at 5 Hz.
- It publishes private status and plans only.

**Unchanged, verified as listed:**
- **The October `CatheterControlNode` path.** It is byte-identical apart
  from two startup guards: refuse a non-`v171` backend, and require
  `command_timeout_s` to exceed the plan period. All existing stacks
  satisfy the second, which a test checks. Verified by the `catheter_control`
  suite.
- **Filter mechanics.** Same `load_stream` owners, 90 ms selection, queue 3
  with start-up allowance 8, and owner tick as the observer. The check is
  that dry-run filter rows are bit-identical to the observer's on the same
  recorded replay.
- **Safety.** No `/teleop/control`, `/teleop/position_control`, manager
  clients, arm service, mode claim or `SET_ZERO`. The node refuses
  `command_output_enabled=true` and `planner_process_enabled=true`. A static
  test checks this.

**Declared choices:**

| Choice | Detail |
| --- | --- |
| Play | The filter's play only: `transmission_state=None`, no lease, plain sampling (decision 2) |
| Planner settings | Lambda guard 0 (decision 3); gain scenarios off; 4 × 0.2 s coarse steps, tail 0 |
| Joint position | From the newest encoder counts through the existing `counts_to_logical` owner, so it matches the filter's inputs |
| Tendon limit | Controller-only upper limit of 10.8 mm (decision 6) |
| Rates | 5 Hz, deadline 0.05 s (decision 5) |
| Kernels | Prepared for both candidate counts on the first tracking root, before ready |
| Readiness | Tracking, no pending images, kernels prepared |
| Status mapping | tracking → TRACKING; holding → HOLDING; fault → FAULT (latched); otherwise INITIALIZING |
| Root failures | A root failure (stale, holding, rotation) is recorded as a refused plan, not a fault |
| Control point | Assumed to be the same tip as `/catheter_mppi/target_tip` (inferred; to confirm on L1) |

## Step 4 results (2026-11-02): non-commanding dry-run controller

Done, uncommitted. Evidence is in
`catheter_sessions/gate3_dry_run_integration_check_20261102/`.

**What was added:**

| Piece | Owner |
| --- | --- |
| Filter ownership, health mapping, plan-time root, per-law-count kernel preparation (ROS-independent) | `catheter_control/orchestration/reduced_filter_owner.py`. Also the single owner of `encoder_sample` and the ordered inbox, which the observer now imports |
| Dry-run node `catheter_mppi`: owner timer at 5 ms, plan timer at 5 Hz, private `/catheter_mppi/status` and `/catheter_mppi/dry_run_plan`; refuses command output, planner process and October play options | `catheter_control/reduced_filter_dry_run.py`, entry point `catheter_mppi_reduced_dry_run` |
| Encoder input QoS: best-effort, depth 512, for history-consuming estimators | `estimator_owner.encoder_history_qos`, used by the dry run and the observer |
| `CatheterMppi.candidate_counts()` for plain sampling | `planning/mppi.py` |
| Selection by stack and launch; source-tree `PYTHONPATH` for this process only; report directory from the session | `control.launch.py` (`research_code_root`) |
| Stack `hardware_reduced_filter_dry_run_no_rotation`: standalone controller layer (lambda guard 0, no take-up/backlash/gain/scheduler, plain sampling, 4 × 0.2 s, 10.8 mm, task GC), platform v175 no rotation, `performance/gpu_512_5hz` | `catheter_control/config` |
| 6 parameters with units and validation | `estimator_backend` plus `reduced_filter_*` |
| October node guards | Backend must be `v171`; `command_timeout_s` must exceed the plan period |
| `/catheter_mppi/dry_run_plan` | Added to the recorded topics |

**Defects found and fixed during integration:**
1. **Kernels prepared for one law count.** The reduced-filter snapshot's
   shapes depend on the number of active laws: 1 at initialization, up to 2
   later. Kernels prepared at one count failed every plan at the other
   (`accelerator signature changed`). The 3d harness hid this by preparing
   late. Fix in `ReducedFilterRolloutBackend`:
   - `ReducedPlanRoot.law_count`;
   - kernels keyed by (candidate count, law count);
   - `prepared_for()`;
   - an unprepared count fails closed.

   The owner prepares each new law count in the estimator thread after a
   step, never during a solve. No forecast mathematics changed. A backend
   test was added.
2. **Encoder samples lost in transport.** With `qos_profile_sensor_data`
   (depth 5), the observer received 438 of 455 encoders during its slow
   first step at a mid-session start. The resulting history gap rejected
   three images and faulted the filter. The October controller was
   unaffected because it keeps only the newest sample. Fix: depth-512
   best-effort input for both filter nodes; a test was added.
3. **Start-up allowance ended too early.** It ended at the first tracked
   image, but a mid-session start has a slow first tracking step
   (146–413 ms, against about 100 ms at image 3). Images arriving during it
   met the steady bound. It now ends at the second tracked image that
   leaves the queue within the bound.
4. **Dry-run start-up queue too small.** It also prepares kernels: 393 ms
   for law count 1 and 269 ms for law count 2. Its start-up allowance is 16
   images, start-up only, set in the stack.
5. **Shutdown race.** SIGINT raised `RCLError` from a worker's wait set
   after context shutdown. Both nodes now treat that as a normal stop,
   and other `RCLError`s still propagate. Exit code 0 is verified.

**Integrated replay check.** Domain 214, recorded images 800–850 at the
recorded rate, the trial 3 target active at that time. The arms ran
sequentially: the dry run alone, then the observer alone.

| Check | Result |
| --- | --- |
| Filter equivalence | **Pass: bit-identical.** Same 49 images, max difference 0 in position and lambda, statuses equal; 455/455 encoders and 0 drops in both arms |
| Graph | Pass. Publishers: `/catheter_mppi/status`, `/catheter_mppi/dry_run_plan` and standard ROS topics. Service clients: none. Action clients and servers: none. Only the standard parameter services |
| Plans | 26 plans, 25 valid, 25 would command motion. 1 deadline miss: the first plan at 76 ms, a first-call cost |
| Plan tick | P50 20.6, P95 34.8, P99 65.6, max 75.7 ms (deadline 50 ms) |
| Rollout | P50 4.8, P99 7.0 ms |
| Root build | P50 2.3, P99 3.3 ms |
| Root age | P50 37 ms; up to 133 ms after the replay ended |
| Filter step (dry run) | P50 41.7, P95 49.7, max 50.7 ms |
| Tests | `catheter_control` 782 passed, 2 skipped (October suite unchanged; 12 new). cr_meta_lnn stream/backend 29. Observer 17. Bringup and supervisor 76. Lint clean |

**Behaviour changes, by intent:**
- the October node's two startup guards;
- the observer's encoder QoS;
- the start-up allowance rule, used by the observer and the dry run;
- the backend's kernel bookkeeping.

Filter mathematics, priors, settings and update order are unchanged.

**Known limits:**
- Plan ticks in-process (P50 20.6 ms) are about twice the offline P50 of
  10 ms, because they share the GIL with the filter.
- The first plan misses its deadline. A warm-up plan before ready would
  remove that, but it is not added.
- The planner uses plain sampling, not the grouped sampling of the 3d
  harness.
- The control point is assumed to match `/catheter_mppi/target_tip`.
- Live timing under camera load is unmeasured (L1).
- Nothing qualifies closed loop.

**Running it in L1:**
`research_session.launch.py stack_config:=hardware_reduced_filter_dry_run_no_rotation`,
with `command_output_enabled` left false. Publish a target on
`/catheter_mppi/target_tip`. The plans are recorded in the bag, and the
report is written to the session directory.

## Pre-lab tools (2026-11-02)

Done, uncommitted, non-actuating. Evidence is in
`catheter_sessions/gate3_lab_tools_check_20261102/`.

| Tool | Entry point | What it does | Safety |
| --- | --- | --- | --- |
| Dry-run targets | `ros2 run experiments dry_run_targets --target X Y Z \| --targets-file EXP.yaml --dwell-s 10 --log NEW.jsonl` | Steps tip targets (metres) on `/catheter_mppi/target_tip` at 2 Hz and logs each interval | Publishes only while `/catheter_mppi/status` is fresh and from `reduced_filter_dry_run` with command output off. It stops on any other controller or a stale status |
| Dry-run analysis | `ros2 run runtime_supervision dry_run_analysis --report <session>/reduced_filter_dry_run.json --targets-log LOG` | Filter statuses, drops, step distribution, faults/holds, kernel preparations, warm-up; plan validity and reasons; tick, planner, rollout, root-build and root-age distributions; deadline misses; per-target plans and predicted terminal error | Offline |
| Home check | `ros2 run runtime_supervision home_check [--reference REF.json] [--save-reference NEW.json]` | After each boot: first three encoder channels within 67 counts of zero and stationary (≤ 3 counts); optionally every marker within 0.5 mm of a saved home reference. A reference is saved only if the encoders pass | Subscribes only |
| Prior re-fit | `ros2 run runtime_supervision prior_refit axis\|covariance --session DIR \| --sensors NPZ --output NEW` | Axis: engaged chassis-only response direction compared with the current prior. Covariance: restored 2026-10-20 owner on the longest still window, with held-out scores including the current October R. Nothing is installed | Offline |
| Warm-up plan | Dry-run node | One discarded plan at first readiness, aimed at the target or the filter's tip, then a planner reset | No output |

**Owners added or restored (cr_meta_lnn):**
- `training/hybrid/marker_covariance_validation.py` is restored from
  `repository_archives/cleanup_20261102`, byte-identical to the copy shipped
  with the October R evidence. It gains optional training and held-out
  windows; the defaults are the October contract.
  - Re-running the October fit regenerates `learned_full_covariance_mm2`
    bit-for-bit.
  - Its archived tests are restored: 10 passed.
- `training/hybrid/engaged_axial_direction.py` (new) fits b in dp = b·dc on
  engaged, chassis-only image intervals: tendon and knob still, and travel of
  at least the 7.05 mm play width since the last reversal. It returns
  direction, gain and a bootstrap spread. 2 synthetic tests.

**Validation on the Oct 8 session:**
- **Warm-up and tools replay** (images 800–850, driven by `dry_run_targets`):
  - plans: 37/37 valid, 0 deadline misses (the first-plan miss is gone);
  - plan tick: P50 13.7, P95 18.8, P99 25.2, max 28.0 ms;
  - warm-up: 50 ms; kernel preparations: 425 and 286 ms;
  - root age during the replay: 108–118 ms. After the replay ended it stays
    at 132.7 ms, a replay artifact.
- **Covariance re-fit:**
  - still window found automatically: 27.2 s, 273 images;
  - OAS shrinkage 0.085; spatial criterion passes; whiteness not rejected
    (p = 0.055);
  - October R on this held-out data: mean Mahalanobis 11.9 against the
    expected 12, 96% coverage.

  The tool runs end to end and the October R is calibrated on its own
  session. On a new session, the decision quantity is October R's held-out
  calibration there.
- **Axis re-fit: finding, not acted on.**
  - 94 engaged chassis-only intervals; direction (0.036, 0.034, 0.999), gain
    0.93, bootstrap 95th-percentile spread 1.7°.
  - All four markers agree: per-marker gains 0.92–0.94, directions within
    about 1.5°.
  - That is **10.1° from the current prior (0.156, 0.160, 0.975)**. The
    prior's component std is 0.05, about 2.9°.
  - So, on this recording's own engaged chassis motion, the prior is
    confidently off by about 3.5 prior standard deviations.
  - Its provenance is a `trusted_vector` passed explicitly to the Oct 27 run
    `gate3_cartesian_gain_cli_445_485_20261027`; no code in either repository
    derives it.
  - Possible explanations, none tested: it represents a different quantity
    (such as the distal or interface body axis rather than chassis travel),
    or the fit's assumptions (fully engaged after 7.05 mm, still tendon) do
    not hold.
  - Changing the prior would be a separate, explicitly approved experiment
    with a matched replay.

**Tests:** lab tools 6, target publisher 3; cr_meta_lnn 41 (restored
covariance, axial direction, stream and backend); `catheter_control` 782
passed, 2 skipped. Lint clean.

## L1 runbook (draft, non-actuating unless separately authorized)

1. Boot the firmware at home. Run `home_check`, saving a reference the first
   time.
2. Run the camera stall diagnosis: two 3-minute captures with diagnostics on
   (see `CAMERA_PIPELINE_PROFILE.md`).
3. Run `research_session.launch.py
   stack_config:=hardware_reduced_filter_dry_run_no_rotation`, with command
   output false, which is enforced. Optionally add
   `reduced_filter_shadow:=true` for a second filter, but it doubles the
   filter load.
4. With the robot still, run `dry_run_targets --targets-file
   <experiment>.yaml --dwell-s 10 --log <session>/targets.jsonl`. This
   measures live timing under camera load, and the still period also feeds
   `prior_refit covariance`.
5. Stop the session. Run `dry_run_analysis` and `prior_refit covariance` on
   the session.
6. With separate motion authorization: chassis travel beyond the 7.05 mm
   play width in both directions, with the tendon still, for
   `prior_refit axis`.

## L1 results (2026-11-03, non-actuating)

**Camera stall diagnosis (runbook step 2):** done; see
[`CAMERA_PIPELINE_PROFILE.md`](CAMERA_PIPELINE_PROFILE.md) runs 1–3. At 30 fps, 0.33 % of periods
lose a frame on both cameras at once, always a single frame. The loss is inside the ZED SDK, not GC,
the GIL or the controller. 60 fps is parked.

**Dry-run session (steps 3–5):** session
`catheter_sessions/reduced_filter_dry_run_l1_20261103T161930172017Z`.
- **Setup:** stack `hardware_reduced_filter_dry_run_no_rotation`, 30 fps, `video:=false`, domain 42.
  The motor interface was up, driver power unqualified, manager inhibited, robot still.
- **Targets:** `dry_run_targets` ran the 4 targets of `farther_tendon_12_grouped.yaml`, 10 s dwell × 5
  repeats, and completed.
- **Session:** `complete`, all processes returned 0, qualification passed. `/device/command_tx` and
  `/manager/control` had 0 messages.

**Results:**

| Quantity | Value |
| --- | --- |
| Images received / tracked / skipped by the 90 ms selection / dropped | 6,525 / 2,177 / 4,347 / **0** |
| Filter status | Tracking throughout; no holds or faults |
| Plans | 1,076 at 5.0 Hz (interval P50 200.0 ms, max 218.5 ms); **1,076 valid; 0 deadline misses** |
| Plan tick P50 / P95 / P99 / max | 16.2 / 39.1 / 45.7 / **49.2 ms**, against the 50 ms deadline |
| of which rollout P50 / P99 | 4.9 / 6.8 ms |
| Root build P50 / P99 | 3.1 / 6.1 ms |
| Root age (image to plan) P50 / P95 / max | 92.5 / 116.7 / 153.7 ms |
| Warm-up plan; kernel preparation (1 law, 2 laws) | 52.3 ms; 573 ms, 414 ms (both before the first counted plan) |

**Planner tail.** 870 of 1,076 plans took 10–20 ms. About 10 % took 25–49 ms, and the maximum left
only 0.8 ms of margin. The non-rollout part of planning rose from 10.9 ms (P50) to 33.5 ms (P95), while
the rollout stayed at about 5–7 ms. Offline replay gave a planner P99 of about 22 ms. The extra time
is in the Python side of the planner. A likely, unconfirmed cause is GIL contention with the filter
step on the same executor. Confirming it needs the per-image filter step times (below). The deadline
was not changed.

**Report lost at shutdown (code defect).** `reduced_filter_dry_run.py` writes
`reduced_filter_dry_run.json` only after `executor.shutdown()` in the `finally` block. The session
stop delivers a second SIGINT (supervisor and `ros2 launch`), which raised `KeyboardInterrupt` inside
`executor.shutdown()`, so the report was never written. Plan rows and the status summary were
rebuilt from the bag (`reduced_filter_dry_run.rebuilt_from_bag.json`, summary `dry_run_summary.json`).
The per-image filter step times are not in the bag and are lost for this session.

**Marker covariance re-fit (step 5)** on the 219 s still window; output `covariance_refit/`, **not
installed**. On the held-out window, October R is too tight for today's mount:

| | Mean Mahalanobis (expected 12) | Nominal-95 % coverage |
| --- | --- | --- |
| October R | 15.6 | 78.6 % |
| Re-fit, full | 10.6 | 99.0 % |

- Re-fit minus October log density: +2.28 per frame.
- The white-noise check does not reject iid (lag-1 p = 0.98).
- Full beats diagonal and baseline in every window.
- The tool's `iid_noise_model_qualified` is false by design: no physical truth or cross-session check.
- Adopting the re-fit is a separate decision.

### Report fix and dry-run repeat (2026-11-03)

**Fix in `reduced_filter_dry_run.py`.**
- Shutdown now goes through `finish()`, which ignores SIGINT before `executor.shutdown()` and then
  writes the report.
- The report is written atomically: `.json.partial`, then renamed.
- Each filter row now carries a wall-clock `wall_ns`. This is observational only; filter behaviour is
  unchanged.
- New regression test: a real SIGINT is sent during `shutdown()`. The dry-run and lab-tool tests pass
  (21); lint is clean; `catheter_control` was rebuilt and the installed copy matches the source.

**Repeat session** `catheter_sessions/reduced_filter_dry_run_l1_20261103T163153883420Z`. Same setup
and targets as the first. Session `complete`; the report was written; `/device/command_tx` and
`/manager/control` had 0 messages.

| Quantity | Run 1 (16:19Z) | Run 2 (16:31Z) |
| --- | --- | --- |
| Images tracked / dropped | 2,177 / 0 | 2,168 / **2** |
| Filter step P50 / P95 / P99 / max | lost | 41.6 / 50.0 / 55.9 / 142.9 ms (1 step over 100 ms) |
| Plans valid / total; deadline misses | 1,076 / 1,076; 0 | 1,068 / 1,071; **3** |
| Plan tick P50 / P99 / max | 16.2 / 45.7 / 49.2 ms | 17.9 / 43.0 / **73.2 ms** |
| Rollout P50 / P99 | 4.9 / 6.8 ms | 5.2 / 8.3 ms |
| Root age P50 / P95 | 92.5 / 116.7 ms | 93.6 / 128.7 ms |

**Attribution of the planner tail:**

| Band | Plans | Overlapping a filter step | Median non-rollout time | Reading |
| --- | --- | --- | --- | --- |
| Under 20 ms | 700 | 110 | 11.0 ms | Normal |
| 20–30 ms | 259 | **229** | 16.0 ms | Explained by contention with the concurrent filter step (GIL) |
| Over 30 ms | 111 | 33 | 31.4 ms | **Not** explained by filter overlap |

The plans over 30 ms come in sustained windows, not isolated spikes. Fraction of plans over 30 ms per
10 s bin:

- Run 1: zero until 170 s, then 0.26, 0.90 and 0.78 over 170–200 s.
- Run 2: 0.32 → 0.08 over the first 30 s, then 0.12, 0.88 and 0.61 over 130–160 s.

They do not follow the target or the blocked-candidate count: the same targets plan in about 16 ms on
other repeats. Within a window the GPU rollout is unchanged while the Python part of planning about
doubles. That suggests host-level CPU contention or a frequency or thermal drop. These logs cannot
attribute it; system-wide CPU, frequency and process sampling during a run would.

The planning deadline was not changed.

### Dry-run run 3 with host sampling (2026-11-03)

Session `catheter_sessions/reduced_filter_dry_run_l1_20261103T164058758124Z`.
- **Setup:** same as runs 1–2, plus a 1 Hz host sampler (`sysample.jsonl`: top-10 processes by CPU,
  per-core utilization and frequency, package temperature, load) and `nvidia-smi` at 1 Hz (`gpu.csv`).
- **Session:** `complete`; `/device/command_tx` and `/manager/control` had 0 messages.

| Quantity | Run 3 |
| --- | --- |
| Images tracked / dropped | 2,174 / 0 |
| Filter step P50 / P99 / max | 40.9 / 54.3 / 112.4 ms |
| Plans valid / total; deadline misses | 1,075 / 1,075; 0 |
| Plan tick P50 / P95 / P99 / max | 15.9 / 20.2 / 24.3 / 28.8 ms |
| Rollout P50 / P99 | 5.1 / 6.7 ms |

**No slow window occurred.** No 1 s bin had more than 10 % of plans over 30 ms. The host was lightly
loaded throughout:

- **Load average:** median 2.4, P95 4.5 on 20 cores.
- **CPU:** mean core utilization 15–19 %; no core above 50 % in the median second.
- **Frequency and temperature:** maximum core 4.7 GHz; package 41–45 °C.
- **Processes:** the camera process averaged about 138 % CPU and the dry-run node about 61 %. Desktop
  applications (VS Code renderers, Chrome, NoMachine `nxnode`, gnome-shell) were present at low load.

The sustained slow windows of runs 1 and 2 (2 of 3 runs, 20–30 s each, at different times) therefore
remain unattributed. They are intermittent, and whatever causes them was absent here. The sampler
stays in the dry-run wrapper so that the next occurrence is captured. Until then the planner tail is
an open risk for closed loop at the 50 ms deadline.

### CPU-pinning test (2026-11-03): placement is not the cause; the slowdown is state-dependent GPU work

**Hypothesis:** the slow windows come from controller threads running on E-cores (CPUs 16–19 of the
hybrid i7-12700K) or on a hyperthread shared with the camera.

**Test:** run A unpinned, with per-thread core sampling; run B with the controller pinned to CPUs 4–7
and the camera to CPUs 8–15 (`taskset -a`).

**Run A** (`catheter_sessions/reduced_filter_dry_run_l1_20261103T165513844564Z`, `complete`,
`/device/command_tx` 0). This was the worst session so far:

| Quantity | Run A |
| --- | --- |
| Plans valid / total; deadline misses | 745 / 1,074; **340** |
| Plan tick P50 / P99 / max | 44.0 / 71.0 / 80.9 ms |
| Rollout P50 / P99 | 5.1 / 7.1 ms (unchanged) |
| Filter step P50 / P99 | 41.7 / 57.2 ms (unchanged) |
| Images dropped | 1 |

**Placement does not explain it:**
- The controller's busy threads were on E-cores 26 % of the time in slow seconds and 23 % in normal
  seconds.
- Plans whose busiest thread was on a P-core were as slow as those on an E-core (P50 43.0 ms vs
  46.1 ms).
- Load, core frequency (P-core median 4.6 GHz) and package temperature did not change between
  windows.

**What does explain it:**

| Window (time from first plan) | Median plan tick | GPU utilization | Controller CPU | Filter lambda | Blocked candidates |
| --- | --- | --- | --- | --- | --- |
| 0–90 s | 15–18 ms | 5–6 % | 62 % | +0.11 → +0.02 | 120–370 |
| 90–150 s | **46–47 ms** | **10 %** | 62 % | +0.01 → −0.04 | 120–350 |
| 150–220 s | **53–58 ms** | **10–12 %** | 62 % | −0.05 → −0.11 | **0** |

The slowdown is a persistent step, aligned to the second with a step in GPU utilization. Controller CPU
use is unchanged, so the planner waits on additional GPU work outside the timed rollout. The steps
follow the filter state: the lambda readout drifted steadily through zero while the robot was still,
and the blocked-candidate count fell to 0. This explains why the windows in runs 1–2 appeared at
"random" times and why run 3, with a different lambda history, had none. CPU pinning cannot remove it.

The steady lambda drift of about −0.1 per 100 s on a still robot is itself unexplained.

**Run B (pinned) did not run.** The oblique camera (S/N 26080456) dropped off USB while the SDK
opened it (11:59:39, `CAMERA MOTION SENSORS NOT DETECTED`), before the pinning step.
`/device/command_tx` was 0. It is the second camera-USB fault today, on the other camera.

**Next isolated step (proposed):** record the planner's existing phase timings
(`sample_projection_ms`, `cost_weighting_ms`, `update_projection_ms`) in each dry-run plan row, then
rerun. That locates the extra GPU work.

### Root cause (2026-11-03): plan ticks block on the owner lock during filter steps

**Correction.** The "additional GPU work" inference above is wrong; the GPU-utilization alignment in
run A was coincidental.

**Run with phase timings.** The plan rows now record all four planner phases (an observational change
to `reduced_filter_dry_run.py`; 21 tests pass). Session
`catheter_sessions/reduced_filter_dry_run_l1_20261103T170452148247Z`, `complete`,
`/device/command_tx` 0. 90 of 1,077 plans missed the deadline.

**Phase breakdown:**
- In slow bins (0–90 s), median tick is 42–47 ms. Sampling (about 4 ms), rollout (about 5 ms), cost
  weighting (about 3 ms) and update (0 ms) are unchanged. The **unphased remainder** is 28–33 ms.
- In fast bins it is 3–4 ms.
- Neither GPU utilization nor lambda tracks the switch.

**Mechanism.**
- `ReducedFilterOwner.process()` holds `owner.lock` for the whole filter step (P50 42.5 ms).
- Each plan tick starts its deadline clock, then calls `owner.health()`, which takes the same lock,
  and then `plan_root()`.
- A tick that fires during a filter step waits for the step to finish before planning.

**Evidence:** in all 416 slow plans, root building begins 0.12 ms (median) after a filter step ends.
In fast plans, only 1 % do. The waiting costs no CPU, which is why the controller's CPU use was flat.

**Timing:** filter steps follow the selected images every 100.0 ms (camera clock); plan ticks run on a
200 ms ROS timer. The near-2:1 ratio makes the relative phase drift slowly. That produces sustained
windows of tens of seconds in which most ticks land inside a filter step, starting at unpredictable
times. This explains runs 1, 2 and A and the clean run 3.

The 20–30 ms band is the residual GIL overlap when the tick misses the lock but runs concurrently
with the filter's Python work.

**A second, one-off stall:** the maximum filter step, 524 ms, is the two-law kernel preparation. It
runs inside `process()` under the lock the first time a second law appears.

**Consequence.** CPU pinning, priorities or an RT kernel cannot fix this: the planner is waiting by
design on a lock, not being starved. Run B (pinned) is not needed for this question.

**Proposed fix (a scheduling change, needs approval).** Trigger planning from filter commits instead
of an independent timer: plan once after every second committed step (5 Hz on the camera clock).
- Filter step plus plan (about 42 + 16 ms) fits inside the 100 ms image period, so the planner neither
  waits on the lock nor overlaps the filter's Python work.
- Root age at planning becomes minimal.
- Separately: prepare both kernel law counts at warm-up, so no preparation runs under the lock
  during operation.

### Commit-triggered planning (Pattern 1, 2026-11-03)

**References.** This is the image-triggered visual-servoing structure (ViSP, `visp_ros`). The
alternative is a fixed-rate controller reading the latest estimate without blocking (ros2_control
`realtime_tools`, Autoware MPC). That remains the route for higher rates, together with a separate
planner process. Both satisfy the same rule: the controller never blocks on the estimator.

**Change (`reduced_filter_dry_run.py`, approved).**
- The 200 ms plan timer and its callback group are removed.
- `_owner_tick` runs `_plan_tick` in the same thread right after a filter result, when
  `plan_due(last, now, period)` holds: the first result, or at least 0.9 × the plan period since the
  last plan.
- New diagnostic count `plan_overdue`: no plan for more than 2 periods while the owner is ready.
- Unchanged: filter, planner, roots, deadline check, readiness gate, warm-up, safety refusals.
- Two-law kernel preparation at warm-up is deferred (separate step).
- Tests: 24 pass, including the plan cadence (commits at 100 ms give plans at exactly 200 ms; never
  faster than 0.9 × period) and the absence of an independent plan timer. `catheter_control` was
  rebuilt; the installed copy matches the source.

**Session** `catheter_sessions/reduced_filter_dry_run_l1_20261103T172355579318Z`. Same setup, targets
and still robot as runs A and phase. Session `complete`; `/device/command_tx` and `/manager/control`
had 0 messages; `plan_overdue` 0.

| | Run A (timer) | Phase run (timer) | **Commit-triggered** |
| --- | --- | --- | --- |
| Deadline misses | 340 / 1,074 | 90 / 1,077 | **0 / 1,075** |
| Plan tick P50 / P95 / P99 / max | 44.0 / 62.4 / 71.0 / 80.9 ms | 18.7 / 52.4 / 59.4 / 86.0 ms | **13.9 / 18.7 / 20.8 / 25.0 ms** |
| Unphased remainder P50 / P99 | (not recorded) | 4.3 / 44.8 ms | **2.4 / 5.9 ms** |
| Root age (image to plan) P50 / P95 / max | 35.4 / 110.4 / 120.0 ms | 105.0 / 115.0 / 146.3 ms | **34.0 / 40.2 / 43.2 ms** |
| Plan interval P1 / P50 / P99 | 185 / 200 / 216 ms | 184 / 200 / 217 ms | 184 / 200 / 220 ms |
| Filter step P50 / P99 | 41.7 / 57.2 ms | 42.5 / 59.3 ms | 42.6 / 58.5 ms |

**Reading.**
- The lock wait is gone by construction: the unphased P99 is 5.9 ms, against 44.8 ms.
- The GIL overlap band is also gone, since plans never run during a filter step.
- The worst plan, 25.0 ms, leaves 25 ms of margin to the 50 ms deadline.
- Image-to-plan age fell from about 105–115 ms (P50/P95 in the phase run) to 34–40 ms, because each
  plan uses the commit that triggered it.
- The plan interval follows the camera clock: 200 ms median, 184–220 ms at P1–P99.

**Not covered:**
- One run. Live, non-actuating, still robot; no motion or closed loop.
- The one-off two-law kernel preparation stall (about 0.3–0.5 s, once) still runs inside the first
  two-law filter step and would delay that plan.

## Lab sessions after that

| Session | Content | Authorization |
| --- | --- | --- |
| L1, non-actuating | Camera stall diagnosis (10 min). Live shadow observer plus dry-run planning under full camera and controller load: timing distributions and deadline misses. Held-pose data for the R re-fit | Hardware on, no motion |
| L1b, motion | Pure-insertion and tendon excitation for the axis re-fit and the readiness criterion | Explicit motion authorization |
| L2, closed loop | First closed loop with the filter, at 5 Hz, inside the reduced tendon range | Separate authorization; firmware decision first |

## Unknowns and the least invasive way to resolve each

- **Live estimator and planner concurrency (GIL, GPU contention with
  cameras):** L1 dry-run timing.
- **Whether October profiles and widths transfer to the current mount:** the
  L1b shadow fit on a new session.
- **Stopping overshoot near the tendon limit:** existing `tendon_start` data
  near the upper range, or an L1b run.
- **Whether the simulator can host an offline end-to-end loop** (it has no
  marker generator consistent with the filter model): unknown; inspect
  `simulation` before relying on it.
