# Modeling and control research experiment plan

Date: 2026-10-02. Status: planned research protocol. This document is not
hardware authorization.

## Scope and evidence boundary

This is the separate experiment plan accompanying
[`ROBUST_TENDON_ENGAGEMENT_AND_INTRINSIC_MODEL_PLAN.md`](../../ROBUST_TENDON_ENGAGEMENT_AND_INTRINSIC_MODEL_PLAN.md).
The local, intentionally Git-ignored literature collection is
`catheter_modeling_control.bib` in this directory.

Immediate priorities, confirmed by the user:

1. **Clean controller attribution:** separate the effects of grouped sampling,
   response-confirmed take-up compensation, online adaptation, and other
   implementation differences in matched two-axis reaching trials.
2. **Two-axis continuous hardware paths:** test repeated insertion/tendon
   reversals and sustained tracking with rotation disabled.

Sampling-budget comparisons support item 1; they are not a substitute for
isolating the controller components. Remount robustness is deferred.

Three-axis hardware path following and torsion/insertion coupling are deferred.
Do not present successful three-axis simulation as successful hardware control.

Existing experiments motivate these tests but do not replace a matched,
repeated comparison. In particular, the apparent advantage of grouped MPPI
at small sample budgets and the success of plain MPPI at 512 samples are
working observations, not an established statistically general result.

## 1. Clean controller attribution

### Research questions

- Does grouped sampling improve success or convergence at a limited compute
  budget, rather than universally outperform plain MPPI?
- Does response-confirmed take-up compensation improve robustness, or can
  its interruptions and reversals reduce performance?
- How does the existing adaptive extended Jacobian controller compare with
  the model-based controllers under the same observations and constraints?

### Controllers

| Variant | Purpose |
| --- | --- |
| Grouped MPPI + compensation | Current combined method. |
| Plain MPPI + compensation | Remove grouped proposals/selection while retaining the response-confirmed take-up layer. |
| Plain MPPI | Conventional ungrouped controller without explicit take-up compensation. |
| AdapJ | Existing adaptive extended Jacobian comparison baseline. |

Reuse `control.adapj.AdapJController`; do not reimplement its update algebra.
It is not the `AdaptiveForwardJacobian` used inside the learned forward model.
The algorithm already exists; verify the current task/ROS integration before
calling it a ready-to-run hardware baseline.

An optional AdapJ + compensation ablation can follow the primary comparison,
but should not expand the first implementation slice.

### Separate system comparison from component attribution

The current semantic profiles are **not** single-factor ablations:

- `grouped.yaml` versus `plain_with_takeup.yaml` changes grouping, the
  best-candidate guard, take-up risk cost, and reversal scheduler.
- `plain_with_takeup.yaml` versus `plain.yaml` also changes engaged-gain
  adaptation and gain-scenario rollout, not only compensation.

First reproduce these deployed variants and label the result a **whole-system
comparison**. It cannot establish that grouping or compensation alone caused
an improvement. Save the complete resolved parameter diff and rollout-model
semantics for each pair.

Then construct reviewed attribution profiles with one changed component at a
time, holding the estimator, model/artifacts, gain treatment, horizon, limits,
and task protocol fixed:

1. Grouped versus ungrouped proposal/selection with shared compensation,
   costs, response rules, and candidate-guard policy. If a feature cannot be
   applied consistently to both, explicitly test it as an additional factor.
2. Compensation enabled versus disabled with shared ungrouped planning and
   gain treatment. Define whether rollout uses nominal transmission or
   effective engaged dynamics; do not silently switch models with the flag.
   If an identical rollout is invalid for one execution mode, report the
   necessary model change as part of a compensation/model bundle instead of
   claiming a compensation-only comparison.
3. Online gain adaptation enabled versus frozen, retaining the same
   response-confirmation safety rules and initialization data budget.

Do not require a full combinatorial grid immediately. Use these paired tests
to identify the important contributors. A grouped-without-compensation cell
is optional only after confirming that its execution semantics are valid;
never bypass safety gates to complete a factorial design.

For AdapJ, match available feedback and initialization effort, and document
whether it consumes measured tip positions or estimator output. Sharing an
estimator that contains the proposed model can otherwise obscure attribution.
Include existing baseline behavior faithfully rather than silently adding the
proposed belief or compensation to it.

### Matched protocol

1. Disable rotation for all variants. Use the same qualified encoder and
   velocity bounds, observations, target tolerance, timeout, and settling rule.
2. Start each independent reaching trial from encoder `[20, 0, 0]` with the
   existing guarded homing procedure. Record the measured home shape and tip.
   Encoder homing does **not** imply a reset of physical catheter history.
3. Freeze one reviewed Cartesian target set per mount and use it for all
   controllers. Never generate a different target set from each controller's
   own preview. Include insertion-dominated and coupled insertion/bending
   targets, not only targets already selected for one controller's success.
4. Use local relative targets when the mounting geometry changes. Store the
   absolute target coordinates, the measured reference pose/shape, and the
   relative definition. Evaluate mount-to-mount differences separately from
   controller-to-controller differences within a mount.
5. Keep the MPPI point horizon at the reviewed four-step, 0.8 s trial setting
   for the comparison; verify installed effective parameters, not source
   filenames. Match planning rate, deadline, noise distribution, temperature,
   and warm-start rules wherever the algorithm permits.
6. Use 48, 128, and 512 stochastic candidates as initial budget levels. Report
   deterministic probes, gain scenarios, and total candidate-step evaluations
   separately: equal `samples` does not necessarily mean equal computation.
7. Run a primary matched-sample comparison and, where possible, a secondary
   matched wall-time/rollout-work comparison. Do not silently increase one
   controller's deadline to accommodate more computation.
8. Predeclare any temperature/noise tuning using a separate tuning target set
   and equal tuning effort. Freeze settings before evaluation. Poorly tuned
   plain MPPI is not a meaningful baseline.
9. Counterbalance controller order within a mount and record the actual order.
   Use the same preconditioning schedule, but preserve and record continuous
   encoder/shape history across trials. Do not claim order-independent state
   reset where none exists.

Suggested pilot: four targets, three repeats per controller/budget on one
mount. AdapJ has no sampling-budget factor; run it once per repeat block rather
than creating artificial sample-count variants. Use pilot variance and session
duration to determine the confirmatory repeat count before final collection.
Allow partial blocks to be saved rather than silently restarting a dataset.

### Outcomes

Report each trial, not just a mean over successful trials:

- Success fraction and final measured tip error for every trial, including
  timeouts; separately report closest approach and overshoot.
- Time to settled success. Timeouts are censored at the common timeout, not
  substituted as successful reaching times.
- Encoder reversal count per axis with a fixed noise/deadband definition;
  report commanded reversals separately.
- Motor travel, time in take-up, transaction count, saturation count, and
  fraction of the trial holding zero command.
- Planner p50/p95/p99/max latency, deadline misses, accepted-observation ages,
  and fault causes. Distinguish a controller failure from transport, homing,
  or perception failure without omitting the latter from the run accounting.

Pair comparisons by mount, target, and repeat block. Treat mounts/session
blocks as clusters, not independent video frames. Show error and reaching-time
distributions alongside success counts and reversal counts.

### First implementation gate

Implement one mount, one reviewed target set, all three existing MPPI variants,
and the same machine-readable trial manifest and parameter-diff report.
Verify the AdapJ integration without powered motion before adding it to
hardware blocks. Add the paired attribution profiles before expanding the
sampling-budget matrix; do not introduce new model mechanics in this gate.

## 2. Two-axis continuous hardware paths

### Research questions

Can the controller maintain progress through repeated insertion/tendon
reversals without chatter, prolonged take-up holds, or overshoot? Does the
grouped method provide a benefit on continuous tasks that successful isolated
point reaching does not expose?

### Collection protocol

1. Keep rotation velocity zero, preserve encoder zero, and use the same guarded
   `[20, 0, 0]` initialization and logged continuous history as item 1.
2. Begin with an insertion-dominated out-and-back path, then a coupled
   insertion/bending out-and-back path, and finally a smooth closed loop with
   repeated reversals. Avoid abrupt corners in the first trial.
3. Construct one reachable Cartesian path per task, inside the observed
   two-axis workspace. A Cartesian circle in an arbitrarily chosen plane is
   not necessarily reachable: the tip's reachable surface can be curved.
   Use safe joint-space seeds and model preview as a proposal, then validate
   with slow observed waypoint checks and freeze the same Cartesian knots
   for every controller. A model preview is not proof of hardware reachability.
4. Exclude joint-limit-boundary paths from the initial tracking comparison.
   Store the seed trajectory, absolute knots, registration, observed
   waypoint residuals, and accepted safety margins. Do not make paths easier
   separately for individual controllers after inspecting their results.
5. Start at a proposed nominal speed of 1 mm/s; progress to 3 mm/s only after
   slow-path completion and timing/observation qualification. These are
   study settings to review, not authorization to actuate.
6. Start with 512 candidates and the matched controllers from item 1. Set the
   **path** rollout horizon explicitly; do not assume the point-only 0.8 s
   horizon setting applies to path tasks. Use the same horizon and reference
   preview coverage for all MPPI variants.
7. Match preview, reference advancement, pause/resume thresholds, terminal
   tolerance, and timeout. Include the measured-start approach segment in
   logging but report its performance separately from the evaluation path.
8. Run three counterbalanced repeats per controller/path as a pilot. Keep
   interrupted runs, homing failures, and faults in the session accounting.
   Do not reset hidden physical state between laps or label laps independent
   samples. A later remount study will use a separate protocol.

### Reference policy and comparison fairness

The existing path client supports a governed reference that can pause when
tracking error grows. Use this shared policy initially and report how much it
slows or stops: completing a path with unlimited pauses is not equivalent to
tracking it at nominal speed. AdapJ must receive the same governed reference,
not a different waypoint schedule.

Measure geometric path error, error to the current governed reference, and
progress lag relative to the original nominal schedule separately. A later
fixed-time-reference trial may test tracking bandwidth, but it must retain
safety stop behavior and be labeled a different experiment. Freeze tuning on
a separate path before evaluating the comparison paths.

### Outcomes

In addition to item 1's control metrics, report:

- Path completion fraction, lap completion count, elapsed completion time,
  and achieved speed versus nominal speed.
- RMS, p95, and maximum measured Cartesian path/reference error, including
  reversal windows; report partial-path errors alongside incomplete progress.
- Total/maximum pause duration, pause count, progress lag, and time spent in
  take-up versus estimator/freshness holds or planner-invalid holds.
- Encoder and commanded reversals per axis, motor travel per completed path
  length, closest approach, and overshoot after response confirmation.
- Horizon prediction errors at recorded snapshots, separated into engaged
  motion and take-up/reversal windows; no aggregate MAE-only conclusion.

### First implementation gate

Reuse `control_tasks.path_file` and `TrackTipPath` for a single slow out-and-back
hardware path, with a session manifest and pause/progress telemetry. Review
path feasibility and preview coverage offline first. Add the coupled closed
loop only after this gate; then run the controller comparison, not a new
three-axis circle.

Do not infer intrinsic mechanics from noisy image-fitted curvature alone.
Use Cartesian distal points and rigid-motion-invariant shape comparisons for
offline checks. Any interface pose used for alignment is an estimated latent;
report its role and uncertainty. Offline full-shape reconstruction is
evaluation evidence, not an online controller input.

## Implementation ownership

New **research suite/session orchestration belongs in `experiments`**. Reusable
single reaching/path tasks stay in `control_tasks`. This distinction follows
the existing package layout rather than the presence of "experiment" in an
executable's historical name.

| Responsibility | Owner |
| --- | --- |
| Trial matrices, counterbalancing, preconditioning protocol, mount/repeat labels, manifest/results aggregation | `experiments` |
| Generic sparse reaching/path action clients and reusable guarded home/task execution | `control_tasks` |
| Controller policy, ROS lifecycle, and command/feedback integration | `catheter_control` |
| AdapJ algorithm | Existing `control.adapj.AdapJController` |
| Learned mechanics, estimator mathematics, model rollout | `cr_meta_lnn` |
| Stack/session launch composition | `bringup` |
| Generic runtime identity and finalized-session qualification | `runtime_supervision` |
| Online visual observations and capture integration | `perception` and `catheter-shape-tracking` |

Concretely, reuse
`control_tasks/control_tasks/sparse_point_experiment.py` and its current
`catheter_sparse_point_experiment` entry point for item 1, and
`control_tasks/control_tasks/path_file.py` with `catheter_tip_path_file` for
item 2. Do not move these modules just to add a
study, or copy its homing/manager logic into a new collection node. Add a small
research protocol/runner under `src/experiments/experiments/` to sequence
existing task entry points or action interfaces and collect trial metadata.
It should not implement controller updates or publish a competing command
stream while a task/controller owns motion.

Controller changes that require restart must occur between disarmed trials,
with ownership released and runtime identity captured again. Compose launches
in `bringup`, not inside the controller core. Declare any new package
dependencies explicitly and preserve one-way dependencies: the controller and
generic task packages must not depend on research-suite orchestration.

For the first slice, reuse the existing semantic stack resolver and canonical
task definitions under `src/catheter_control/config/experiments/`. Do not create
a parallel configuration loader or migrate configuration ownership as part of
this behavior change. Protocol metadata can be owned by `experiments`; any
later configuration relocation should be a separately reviewed cleanup.

Offline study-specific analyses should be executable without starting ROS or
connecting devices. Reuse finalized-bag validation and existing reconstruction
tools; do not duplicate their parsers or mathematical implementations.

## Required analysis and result-generation scripts

### Limit-aware tendon prehome correction (2026-10-02)

The guarded sparse task observes manager-approved POS commands, matched to
its own source and exact publication timestamp. During tendon prehome only,
axis-0 projection is allowed and convergence is checked against that approved
endpoint. This handles physical chassis holds that would require logical
insertion below the manager limit (the observed request was -12.5139 mm;
the manager approved -10 mm). The log explicitly reports when physical chassis
hold cannot be maintained. All other axes must match the request, and the final
home still requires the exact configured six-axis endpoint. Missing manager
acknowledgement times out; an unexpected projection fails closed. Existing
manager limits, shared home deadline, settling and feedback freshness checks
remain unchanged. No limit values are duplicated in task configuration.

Regression tests cover the observed projection, source/stamp matching,
unsupported-axis projection rejection and exact-final-home enforcement.
Hardware verification remains pending; the interrupted fourth target must
not be counted as a controller reaching failure.

### Recording readiness correction (2026-10-02)

The initial coordinated session failed with rosbag SQLite `database is locked`.
The supervisor no longer queries the active database. Readiness now combines
the owned recorder's process liveness, storage-file existence, its required
ROS graph subscriptions, fresh telemetry and camera recording diagnostics.
The manifest stores recorder evidence and missing subscriptions; a readiness
loss preserves the failing snapshot. Database integrity/content validation
remains mandatory after all owned processes stop. This changes no controller,
manager or firmware safety gate and does not permit resuming a failed session.
See `audits/ros-realtime/RECORDING_READINESS_20261002.md` for evidence and tests.

### Implemented bag-evidence gate

After the coordinated recorder is finalized, use a sourced ROS Humble terminal:

```bash
ros2 run experiments reaching_analysis \
  --session "$RESEARCH_SESSION" --bag-metrics
```

This command is offline: it initializes no ROS node and opens no device links.
It reuses the ROS storage/CDR APIs and the canonical marker-ID-3 validation.
Target-file identity is checked before analysis. Source timestamps define each
trial window from reaching admission to result/fault/interruption; homing is
excluded. Missing terminal timestamps are reported rather than guessed.

Metric definition version 2 adds:

- Observed maximum and sample-RMS Cartesian marker error, and a source-time
  final marker error only if the last sample is within 250 ms of trial end.
  Journal endpoint and controller-reported error remain separate columns.
- Separate reversal counts for planned velocities, transmitted device VEL
  commands, and raw encoder counts. Physical-axis velocity deadbands are
  `[0.02 mm/s, 0.2 deg/s, 0.02 mm/s]`; encoder deadband is 2 counts per axis.
  Two non-deadband directional samples confirm direction, zeros do not reverse
  it, and gaps longer than 250 ms reset direction evidence. Initial direction
  acquisition is not a reversal.
- Motor travel from physical POS telemetry in `[mm, deg, mm]`, excluding pairs
  separated by gaps over 250 ms. This is partial observed travel, not a lower
  uncertainty claim about unobserved motion.
- Per-stream sample counts, maximum gaps including trial boundaries and
  coverage flags; missing/wrong message types, invalid marker samples/source
  timestamps and non-monotonic source timestamps are reported explicitly.
- `trial_NN.png` plots of measured XYZ with target lines, motor position and
  transmitted velocity; missing spans are broken instead of interpolated.
  Figures are linked from the generated Markdown report.

The maximum error is the maximum **observed** error, not a bound across missing
spans. RMS is sample-weighted, not time-weighted. No UKF shape estimate is used
as control ground truth. There is no assumption that all streams are precisely
synchronized: source-time membership and coverage are exposed for review.
Unattempted/incomplete trials and missing metrics remain explicit. Full timing,
take-up-state attribution, matched study comparison and continuous-path tasks
remain subsequent gates. Validation includes deterministic metric tests and a
synthetic finalized-bag decode/plot smoke test; no hardware was operated.

### Implemented next gate: journaled frozen-target reaching

The coordinated recorder remains non-actuating by default. Once an operator
has reviewed the target YAML and explicitly enabled/qualified the hardware
controller using the existing procedure, a separate opt-in task command binds
the reaching run to the ready recording:

```bash
# In a ROS Humble terminal with the rebuilt workspace sourced.
ros2 run experiments reaching_session \
  --session "$RESEARCH_SESSION" \
  --targets "$REVIEWED_TARGET_YAML" --execute
```

`--execute` authorizes actual guarded homing and reaching; do not use it just
to test recording. The task snapshots the YAML and SHA-256 identity, refuses
session reuse and non-ready/disabled recordings, and invokes the canonical
`control_tasks/catheter_sparse_point_experiment` client. Its existing live
startup, manager, controller identity, homing and rotation guards remain
authoritative. Recorder readiness does not qualify motor power. There is no
automatic controller arming by the recorder itself.

The optional `--trial-records PATH` on the generic sparse task writes exclusive,
flushed JSONL events for task start, homing, frozen targets, reaching, outcomes,
faults and interruption. ROS and monotonic timestamps are both recorded.
After stopping the coordinated recording normally:

```bash
ros2 run experiments reaching_analysis --session "$RESEARCH_SESSION"
```

This offline command produces `analysis/trial_metrics.csv`, JSON, integrity
evidence and `report.md`. Time to result excludes homing and includes action
admission/settling. Latest received marker position is explicitly labeled as
an unsynchronized endpoint snapshot; controller-reported final error remains
separate. Incomplete and unattempted trials are retained. Missing bag-derived
maximum errors, reversals, travel, timing and stream coverage are **null**, not
zero when running in journal-only mode. The bag-evidence gate above adds
source-time metrics and plots with `--bag-metrics`. Cross-controller comparison,
full timing attribution and continuous-path protocols remain later gates.

Verification is non-actuating unit/package tests and installed CLI checks;
the first operator-reviewed hardware session remains pending.

The suite must include usable, tested offline analysis commands, not only
collection scripts. Research-specific analysis belongs in `experiments`;
reuse existing bag readers, reconstruction tools, and runtime/session
qualification instead of duplicating them. Reading finalized data must not
start ROS nodes, open devices, or require a running controller.

Proposed interfaces (**not yet implemented**):

```text
python -m experiments.analysis.session --session SESSION_DIR
python -m experiments.analysis.compare --study-manifest STUDY_MANIFEST
python -m experiments.analysis.report --study-manifest STUDY_MANIFEST
```

Required outputs under each session's `analysis/` directory:

- Integrity report: finalized bag, required topics/types, trial boundaries,
  timestamp coverage, stream gaps, configuration/artifact identity, and optional
  video coverage. Preserve failed and incomplete trials.
- `trial_metrics.csv` and JSON: success/fault/censoring, final/maximum errors,
  time, encoder and commanded reversals, motor travel, pauses, take-up, and
  full-stack timing metrics specified in items 1 and 2.
- Matched comparison tables and figures: sample counts, distributions,
  uncertainty definitions, missing trials/exclusions, and parameter diffs.
- Diagnostic figures: measured and target trajectories, error/progress versus
  time, encoder/command traces, belief/take-up states, and reversal windows.
- A generated Markdown report linking provenance, integrity findings, tables,
  figures, and optional overlay videos. Export publication figures to
  PNG/PDF/SVG, with consistent labels, units, and controller names.

Version the metric definitions: reversal deadband/persistence, reaching
tolerance/settling, event windows, observation-gap rules, and clock handling.
Do not interpolate across long missing spans or replace missing metrics with
zeros. Use measured marker positions for independent Cartesian control error;
label estimator-derived metrics separately. Distinguish source timestamps
from bag receipt times, and use recorded camera indices/timestamp mappings
rather than file creation times or hand-picked offsets for video alignment.

Record analysis revision and parameters; never modify raw recordings. Test
synthetic success, timeout, fault, reversal, pause, and missing-data cases,
then characterize the outputs on historical sessions. A single report command
must regenerate the study tables/figures from its manifest without manual
notebook edits. Optional full-shape reconstruction/overlay should call the
existing shape-tracking tools after collection, not run inside control.

## Coordinated recording and descriptive names

### Current state

Implementation update (2026-10-02): `bringup/research_session.launch.py` now
coordinates video/tracking, one bag recorder, and the controller launch through
`experiments.session_recording`. Task execution and analysis remain pending
gates; hardware output defaults to false. See
[`bringup/README.md`](../../src/bringup/README.md) for commands and lifecycle
semantics. Existing direct launches remain supported as described below.

Verification: 64 focused experiment/bringup/perception tests passed, with the
opt-in ROS smoke test separately passing against a real sqlite3 recorder and
synthetic localhost-only telemetry. `experiments`, `perception`, and `bringup`
built successfully; installed launch arguments and console entry points were
checked. No cameras/device links were opened and no hardware was actuated.
The broader perception suite could not collect its legacy estimator test
because `ros2_igtl_bridge` is unavailable in the test environment. Actual video
capture/finalization and representative recording-enabled timing remain
hardware qualification gates.

`bringup/control.launch.py` already starts the optional rosbag recorder, but
uses a timestamp plus `_mppi_demo`. Native video recording is separately
enabled in `perception.marker_tracking` through `recording_enabled` and
`recording_session_dir`. It shares the existing camera-owner process with
online tracking. `marker_overlay.launch.py` does not expose these recording
settings, and the direct control launch does not start camera recording. Use
the new research-session launch to coordinate these processes.

### Required suite behavior

Provide one research-session launch in `bringup` coordinating tracking/video,
controller, rosbag, task execution, and final qualification. Keep separate
processes and one camera owner; never run a second standalone camera recorder
alongside marker tracking. Reuse existing composition and prevent a second
default bag recorder. Recording coordination does not authorize automatic
arming, driver-power qualification, or physical motion.

Allocate a shared content-based session ID with a timestamp suffix, for example:

```text
attribution_twoaxis_points_plain_with_takeup_n512_mount01_repeat02_20261002T143000-0400
continuous_twoaxis_loop_grouped_with_takeup_n512_speed1_mount01_repeat01_20261002T150000-0400
```

These are illustrative identifiers, not scheduled times. Use sanitized semantic
tokens, explicit timezone/UTC metadata, and collision detection; refuse to
overwrite recordings. Keep full settings in the manifest rather than endlessly
extending filenames. Assign trial IDs within multi-trial sessions.

Use a single parent directory for bag and video:

```text
SESSION_ID/
  session_manifest.json
  session_metadata.json          # camera metadata, when enabled
  primary_SESSION_ID.svo2
  oblique_SESSION_ID.svo2
  primary_frame_index.csv
  oblique_frame_index.csv
  camera_frame_pairs.csv
  registration.json
  robot_bag/                     # metadata.yaml and storage files
  trials.jsonl                   # protocol events/results
  analysis/                      # metrics, figures, report
  processed_full_shape/          # optional offline reconstruction
```

First-slice implementation uses a `SESSION_ID_video/` subdirectory containing
the camera artifacts above, plus a `robot_bag` link to the parent's recorded
bag. This preserves the camera's exclusive directory-creation API and the
existing shape-tracking bag discovery contract without changing either.
The parent additionally contains `controller_manifest.json` and process logs.
Recording does not yet generate `trials.jsonl`, analysis, or reconstruction.

Colocating `robot_bag` matches the existing shape-tracking bag discovery
contract. Respect the camera node's current requirement for a nonexistent
recording directory: specify one directory-creation owner or review that API
before integration, rather than racing launch against camera initialization.
The bag recorder creates its own storage directory. Non-video runs retain the
same parent/manifest structure. Do not rename historical recordings; link
their old separate paths explicitly when analyzing them.

Lifecycle requirements:

1. Validate study definition, free storage, output paths, camera mode, and
   runtime identity while disarmed.
2. Start recording and verify actual camera/bag readiness and required
   telemetry before admitting a trial; process spawn alone is insufficient.
3. Require explicit operator authorization and normal manager gates for
   guarded initialization and task execution. Record trial boundaries.
4. On task completion/failure, release motion authority and verify the stopped
   state. Retain terminal feedback, then finalize bag and both video streams
   gracefully. Do not claim their starts/stops were physically simultaneous.
5. Qualify finalized artifacts and mark the session complete, partial, or
   failed. Preserve evidence and failure reasons after Ctrl-C, faults, or
   recorder failure; never automatically resume powered motion.

Request 720p/30 fps native capture and record achieved settings/frame drops.
Generate overlays offline; live preview is optional. Video is selectable and
declared required/optional before a run. Required video failure must not allow
a trial to continue silently. Qualify recording-enabled full-stack timing
separately: coordinated launch does not remove encoding, I/O, or GPU overhead.

## Provenance, safety, and readiness gates

Write generated bags, images, traces, manifests, and analysis output under:

```text
/media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions
```

Each manifest should link study/mount/trial IDs to session paths, Git revisions,
resolved configuration, artifact hashes, random seed, controller order,
targets, homing result, and termination reason. Keep completed and interrupted
trials. Recording uses the coordinated contract above when video is enabled;
observation telemetry and trial results are always required.

Implementation gates:

1. Pure protocol/manifest and metric tests, characterization of reused task
   behavior, and reproducible reports on historical recordings.
2. Non-actuating composition/replay checks: identity, interfaces, lifecycle,
   recorder readiness/finalization, naming, result handling, interrupted-session
   recovery, and no duplicate camera or command authority.
3. Representative full-stack timing qualification before new powered runs;
   existing performance evidence does not automatically qualify a new stack.
4. Explicit operator authorization for each hardware collection block.

Preserve manager/firmware limits, freshness checks, watchdogs, and fault
latching. Never issue `SET_ZERO`; preconditioning is a position move, not an
encoder-reference change. No physical hidden-state reset is assumed. Do not
automatically recover and resume powered motion after a fault.

Deferred: remount/within-session robustness as a dedicated adaptation study,
insertion/rotation torsion-release characterization, a coupled torsional state
model, three-axis hardware paths, and changed-catheter generalization. These
require separate protocols after the two-axis attribution and continuous-path
comparison stack are reliable. For a future remount study, distinguish shared
catheter intrinsic mechanics from mount-dependent transmission and compare
frozen versus adaptive beliefs with identical calibration budgets.
