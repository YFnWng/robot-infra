# Parallel engagement-model deployment for MPPI

Status: implementation plan; no hardware authorization or deployment promotion.
Date: 2026-10-04.

## Step 4 revision — simplify the existing implementation in place

Planning decision: 2026-10-06. Status: implemented in source and scoped offline
verification passed; active learned-package promotion and step 5 remain pending.
This section supersedes the G3/G5 requirements below for explicit bootstrap
probes, all-enabled-axis engagement before planning, and a separate coordinator.
Those paragraphs describe the historical implementation, not the target design.
This is step 4 of the ordered plan in
[path repair plan](../../../repository_archives/cleanup_20261102/files/robot-infra/audits/ros-realtime/PATH_REFERENCE_COMMIT_OFFLINE_REPLAY.md).

**Modify the existing new pipeline; do not reimplement it.** Keep its runtime
family, joint filter, state representation, profile artifacts and estimator /
rollout tests. Parallel deployment remains an explicit runtime selection, not
another controller stack or another command owner. The intended difference
from the baseline is unified belief inference and distal input/modeling only.

### Retain, remove, and reconcile

| Existing component | Step 4 disposition |
| --- | --- |
| `cr_meta_lnn/deployment/experimental/engagement_runtime.py` | Retain and modify its supported runtime methods and snapshot contract in place. No second runtime class or disguised baseline state. |
| `joint_engagement_filter.py` | Retain joint Cartesian UKF, modal spatial constraints, covariance, shape-response confirmation, reversal memory, delayed correction/replay. No new filter, independent shape correction, or online window refit. |
| Existing engaged profile / prior / mechanics artifacts | Retain selected bytes and hashes. No retraining or mechanics tuning in this step. |
| `engagement_proximal.py` | Retain only the necessary joint-state composition; align artifact initialization, units, interface transmission and propagation with the actual baseline. Reuse existing mathematical owners. |
| `planning/engagement.py::reconcile_engagement` | Keep as a thin derived tendon-belief-to-snapshot conversion, not another estimator or command policy. Preserve interface axes 0/1. |
| `EngagementControl`, `begin_probe`, `tick`, `validate_control_trial` | Remove their special control policy after migrating callers/tests to shared owners. Preserve artifact/family validation in the existing loader/configuration boundary. No replacement coordinator. |
| G5 branches in `catheter_control/node.py` | Remove bootstrap, all-axis engagement requirements, point-only restriction and experimental post-plan command scaling. Keep only necessary runtime API conversion and identity differences. |
| Baseline MPPI, process worker, take-up arbiter, projection, tasks | Reuse the current repaired implementations unchanged; no experimental copies. |
| G4 observer / qualification tools | Retain as non-commanding research surfaces; update them to exercise the same runtime/decision contract, not a retired bootstrap policy. |

The existing `EngagementState` currently exposes some compatibility views
(`lambda_value`, Jacobian and health metadata). Audit their consumers and make
these views physically explicit rather than copying baseline history into the
new state. In particular, do not leave rejection/health fields hard-coded to
healthy values. Preserve deep clone/device transfer and the causal replay buffer.

### 4A — Characterize the seams before changing behavior

Record the current G5 decision flow and direct callers of the coordinator,
probe parameter, registration/phase gates and uncertainty scaling. Extend
existing tests with a finite, initialized, visually corrected root in which
insertion or tendon is UNKNOWN/TAKEUP. Capture seeded proposals, projected
commands, selected intent, transaction ownership and stop/replan behavior.

Treat the current repaired baseline as the control-policy reference, not an
older pre-optimization tag. Match candidate inputs, sample budget, temperature,
point/path horizons, gain policy, compensation caps and physical limits.
Different model outputs are expected; identical lifecycle/arbitration semantics
are required. Keep characterization fixtures for existing filter and rollout
outputs so structural changes cannot silently change inference mathematics.

### 4B — Separate model validity from observed engagement

Modify the existing runtime readiness contract: finite initialized state,
valid covariance, accepted Cartesian observations and baseline freshness /
health checks establish whether a prediction is usable. Observed ENGAGED is
evidence for executing through the transmission, not a prerequisite for asking
MPPI what motion is useful. The current `prediction_ready` combines registration
and ENGAGED; split its meaning without adding another startup state machine.

Use the initialized joint posterior and its existing uncertainty when motion
has not yet confirmed engagement. Do not invent a zero input, future profile
registration or oracle pose. If a valid input posterior cannot be constructed,
report a specific model-validity failure; removing a phase gate is not license
to plan from invalid state. Retain intrinsic passive dynamics at zero command.

Keep live encoder inference history-conditioned. With external compensation,
MPPI samples hypothetical engaged response from the current posterior even
when the real transmission has not yet engaged. This distinction is already
supported by the experimental engaged rollout; do not create a second profile
law or change ENGAGED solely because a sampled plan predicts shape motion.

### 4C — Reconnect the existing runtime to shared control flow

The baseline flow is the target:

```text
fresh valid joint posterior -> shared MPPI motion intent
 -> existing take-up arbiter for the requested axes
 -> either projected MPPI command or sole-owner take-up command
 -> observed response / completion -> discard old plan -> fresh shared solve
```

Warm-up and planning must not wait for every enabled joint to engage. Remove
the fixed probe vector, point-only startup branch, and all-axis engagement
rechecks at commit. An insertion-only intent must not be blocked because the
tendon is unconfirmed. Retain baseline snapshot/reference consistency checks,
deadlines, sensor freshness, manager readiness, cancellation and fault latching.
During an actual transaction, preserve the existing exclusive take-up owner
and zero/replan completion boundary; do not sum compensation and MPPI velocity.

Remove the extra experimental multiplier applied to the complete command after
planning. The executable candidate must be the candidate that was scored and
projected; do not silently push it below coupled minimum speeds. Uncertainty
remains in the filter covariance and the existing documented belief/risk
interfaces, without a second overlapping uncertainty penalty or speed policy.
Keep step 2's bounded take-up scoring and its full physical feasibility reserve.

Audit `reconcile_engagement`: Gaussian width quantiles and remaining travel
are derived views with explicit mm-to-shaft-radian conversion. Distal tendon
confirmation has one authority: direct observed shape response in the joint
filter. Do not duplicate tendon confirmation in the ROS backlash estimator.
Preserve the separate interface-axis evidence and the arbiter's existing
saturation/failure behavior. Any additional adapter-generated inhibition must
be mapped to the shared contract, not silently bypassed or removed as cleanup.

### 4D — Match proximal composition and trim trial-specific configuration

Compare the existing experimental proximal transition against the currently
selected baseline for identical motor increments: full J initialization state,
coordinate scaling, interface play, chassis/knob coupling, right-body SE(3)
update and adaptation policy. Reuse its loaded artifact and mathematical owner;
do not copy baseline UKF/history merely to reuse pose prediction. The new joint
filter still updates pose and strain/input together from Cartesian observations.

Remove hard-coded n512/four-point-step/probe requirements from the runtime-
specific validator where normal shared configuration already owns them. Keep
CPU estimation / isolated GPU planning as the initial matched deployment
assignment, explicit experimental manifest selection and no implicit fallback.
Use the same task clients, frozen targets/path and coordinated recording.
Do not expand rotation, enable gain adaptation, alter compensation caps or tune
costs during this migration. If baseline/proximal parity reveals an artifact
or unit mismatch, fix and qualify that explicitly rather than hiding it behind
a new adapter or declaring it a harmless structural change.

### Acceptance and handoff to step 5

Step 5 suite is now implemented; see
[installed offline integration qualification](../../../repository_archives/cleanup_20261102/files/robot-infra/audits/ros-realtime/ENGAGEMENT_OFFLINE_INTEGRATION_QUALIFICATION.md).
The overlay/isolated-wheel CPU smoke passed 236 tests and three matched windows.
The representative 512-sample CUDA run and any active-package promotion remain
pending and separate from this non-actuating verification.

Implementation record (2026-10-06): the existing runtime/filter were modified,
not replaced. Removed the separate `EngagementControl` coordinator, fixed probe
parameter, all-axis engagement gates, point-only startup restriction and
experimental post-plan confidence scaling. Feature validation remains at the
configuration boundary, without fixed sample-budget or point-horizon policy.
The node and G4 observer now distinguish posterior validity from engagement;
actual transmission execution still belongs to the shared take-up arbiter.
The tendon adapter retains its prior-derived failure/saturation contract and
preserves interface-axis evidence. Rejection counts and health now reflect the
joint posterior rather than placeholder healthy values.

Proximal artifact loading retains the selected physical Jacobian and interface
transmission, loads its action/state normalization scales, and exposes a
unit-consistent shaft-radian Jacobian diagnostic view. Physical Jacobian output
is unchanged. Fixed proximal adaptation remains required for this slice; no
model artifact, fitting procedure, governor, tracking cost, scheduling or
command projection was changed.

Verification:

- 150 focused model, adapter, owner callback, configuration, path commit,
  arbitration and shadow tests passed; the optional real-artifact case was run
  separately and passed with a spawned CPU worker and matching local solve.
- `catheter_control`, `runtime_supervision` and `bringup` rebuilt successfully.
- The 23-second recorded-input startup replay accepted 72 marker frames and
  produced 33 valid plans while tendon phase remained UNKNOWN: zero invalid
  plans and zero deadline misses. It used eight samples on CPU, executes no
  commands, and is not representative CUDA/live timing. The first two seconds
  contained no markers and therefore could not test planning readiness.
- Startup report:
  `/media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions/step4_simplified_engagement_cpu_startup_20261006/report.json`.

Tests/replay explicitly selected workspace `cr_meta_lnn` source. The active
venv learned-model package was **not** replaced; do not start a hardware trial
with mismatched ROS/model artifacts. Step 5 must qualify an isolated installed
wheel and matched CPU-estimator/GPU-worker multiwindow integration, including
reversals, take-up, marker gaps, cancellation and completion, before promotion.

Extend existing tests, rather than build another qualification framework:

- New backend produces useful insertion-only intent with unconfirmed tendon;
  unconfirmed insertion can enter the existing take-up transaction through
  sampled intent, without a fixed bootstrap probe.
- Unknown/TAKEUP/PROVISIONAL/ENGAGED and reversal transitions have one belief
  authority; static Cartesian observations do not confirm engagement.
- Same input/seed yields identical candidate/projection/cost/arbiter behavior
  when supplied a common mock backend; real backends may differ only in the
  declared belief/distal model outputs and their propagated uncertainty.
- Clone, restore, CPU-to-GPU snapshots and delayed replay preserve the existing
  joint mean/covariance, widths, reversal memory and proximal state; no target
  boundary resets, baseline-latent initialization or future data.
- Point and path interfaces, coupled minimum-speed commands, take-up ownership,
  saturation, marker gaps, cancellation and final hold use shared branches.
- Existing baseline regressions remain unchanged; new-runtime characterization,
  installed wheel identity and ROS builds pass before promotion to offline replay.

Run the existing non-actuating replay with startup and reversal windows and
source-matched Cartesian residuals. Report invalid plans, actual model-validity
failures, transaction decisions and timing separately. This slice is complete
when the simplified existing runtime uses shared control plumbing and passes
its scoped tests/replay; the comprehensive matched integration suite is step 5.
Neither gate authorizes hardware or establishes live timing. Retire unused
coordinator/probe imports, tests and parameters only after all maintained
consumers migrate; retain historical session manifests unchanged.

G1 implementation is available in the explicitly non-commanding experimental
runtime. See [implementation, offline commands and verification](../../../cr_meta_lnn/EXPERIMENTAL_ENGAGEMENT_RUNTIME.md).
49 focused tests and a real-artifact synthetic-marker smoke test passed. Actual
recorded sparse-marker qualification remains required before a hardware trial;
G2/G3 component implementation is now available (2026-10-04): joint proximal
prediction, optional gated Jacobian adaptation, batched engaged rollout and a
non-publishing planner/takeup coordinator. The baseline remains selected.
G4 preparation/replay and the separate ROS shadow observer are now implemented;
representative recorded-data/full-stack timing qualification is still pending.
These components are not authorization or readiness for a hardware run.

## Decision and scope

Add an explicitly selected experimental runtime beside the existing qualified
v171 streaming pipeline. Keep the baseline manifest, runtime exports, launch
defaults, estimator behavior, planner costs and regression fixtures unchanged.
Parallel means independently selectable pipelines, not two controllers issuing
commands simultaneously. Start with no-rotation two-axis reaching.

Keep frozen v171 distal spatial mechanics:

$$
D\dot q=-K(q-q_*)+Bu.
$$

Replace the baseline motor-history-to-tendon-input law with the fixed
directional engaged profile and joint Cartesian engagement filter. Raw bending
encoder coordinate is $x$; effective tendon coordinate is $u$. Profile dynamics
apply only to engaged actuation. Width distributions, reversal memory and
shape-response confirmation belong to the belief, not a second tendon filter.
Do not add oracle registration, future E-step poses, posterior-strain updates,
or recent-window refits to the online estimator.

The dataset-wide assessment supports a control trial, not full model
qualification. Engaged-motion predictions improve moving-shape tails, while
near-static false motion and transition errors remain. Perfect 0.8 s Cartesian
prediction is not the entry requirement; safety, consistent causal state,
useful directional response and bounded closed-loop behavior are.

## Existing seams and gaps

- `cr_meta_lnn/deployment/bundle.py` currently accepts only the baseline runtime
  family and constructs `V171StreamingCatheterRuntime`. Extend selection through
  explicit family dispatch; never redirect its current family to the new model.
- `robot-infra/.../orchestration/runtime.py` already selects manifests and verifies
  estimator/planner identities. Reuse this boundary, not another node-local loader.
- Experimental `JointEngagementFilter` already owns joint covariance, prediction,
  Cartesian correction, clone and delayed replay. Reuse its mathematics; do not
  copy them into ROS or run a separate shape filter followed by input correction.
- The offline filter currently consumes a complete centerline and requires
  timestamp-aligned observations. Hardware supplies sparse markers at delayed
  source timestamps. This is a material interface gap, not a configuration fix.
- Reference-pose evaluation does not test online interface estimation or proximal
  dynamics. Reuse the v174-initialized `control.adapj.AdaptiveForwardJacobian`
  composition, but incorporate its pose prediction into the new joint filter.
- Existing ROS transmission/engagement logic must not maintain a competing
  estimate of the new tendon state. Adapt its planner/compensator interface to
  consume one authoritative runtime belief.

## Ownership and runtime contract

`cr_meta_lnn` owns the new semantic runtime, state, sparse observation operator,
joint UKF, physical transition, batched rollout and artifact validation.
`robot-infra` owns composition, snapshots, message conversion, MPPI policy,
takeup command arbitration, lifecycle and final command projection.

Expose the same narrow operations needed by the existing controller:
initialize, advance encoder, observe markers, clone/restore state, delayed
correction/replay, current markers/centerline, immutable diagnostics and batched
control-sequence prediction. Use a separately identified experimental state
type; do not disguise $u$ as the old runtime's tendon-history state. Share
interfaces and FK, not baseline-specific state assumptions.

The new snapshot contains joint mean/covariance for $q,T_I,u$ and the configured
input/profile parameters; discrete engagement mode, direction, reversal encoder
anchor, width distribution, confirmation evidence, and source timestamps.
Derived travel/gap quantities are computed from this memory, not independently
filtered. Clone/device transfer includes all causal memory and covariance.
No reinitialization at target changes or dataset episode labels. Homing changes
state through measured motion; it is not a physical-history reset.

Artifact manifest records a new runtime-family identifier, schema/API versions,
experimental status, hashes of mechanics/profile/prior/proximal initialization,
marker material coordinates, geometry, encoder calibration and units. Missing
or mismatched artifacts fail closed; no baseline fallback. Explicitly record
whether profile gain adaptation is disabled (initial trial) or enabled later.
Do not silently transfer the phase2e 55.5/14.5 mm geometry to another mounting.

## G1 — Parallel runtime and sparse-marker joint inference

First implementation slice, non-actuating:

1. Add the experimental runtime family/manifest while preserving baseline family
   loading and public default exports. Limit loader changes to family dispatch.
2. Give the joint filter an explicit observation geometry: full centerline for
   offline tests, four known material marker locations for hardware. Use the
   authoritative PCS/rigid-tip FK for both. Support per-marker visibility and
   quality covariance; absent observations are not zero-valued measurements.
3. Initialize online pose/strain/input with the existing validated spatial
   fitting approach and modal/gauge regularization, using marker geometry only.
   No stored E-step pose is available online. Initialize uncertain input from
   this fit; do not pretend four markers uniquely determine all 24 strains.
4. Preserve shape-only engagement evidence. With sparse points use rigid-invariant
   inter-marker geometry, calibrated uncertainty and sufficient visible support.
   Interface translation alone must not confirm distal tendon engagement.
5. Handle non-aligned delayed timestamps through the existing causal timeline
   contract: source-time correction then replay, including discrete memory.
   Never relabel arrival time as acquisition time.

Tests: baseline loading unchanged; new hashes/schema rejected when incorrect;
synthetic sparse-marker correction; pose-only motion does not confirm tendon;
missing/occluded markers; clone/restore/device transfer; chronological versus
delayed replay; continuous reversal memory; uncertain initialization.

Exit: deterministic non-actuating replay works through the runtime API. Full
centerline and four-marker replay are reported separately; neither is replaced
by an E-step reference-pose score.

## G2 — Online proximal composition and belief consistency

Implementation: `cr_meta_lnn/deployment/experimental/engagement_proximal.py`
and the existing joint filter. A hash-bound optional v175 interface artifact
initializes the physical-coordinate Jacobian; provenance must match the selected
spatial mechanics. Interface play predicts pose only. Delayed correction replays
raw three-shaft encoders, transmission state and conditional RLS state together.

Adaptation is off by default and explicitly selected in the manifest. It uses
consecutive joint-posterior poses, not a separately corrected pose innovation.
The existing `AdaptiveForwardJacobian` owns RLS and bounds. Fixed trial gates:
contiguous increments <=0.5 s, tendon engaged, normalized innovation <=4,
combined translation variance <=(1 mm)^2, rotation variance <=(0.03 rad)^2,
and meaningful physical excitation. These are conservative conditional gates,
not a claim of independent observed pose or a calibrated Bayesian J posterior.

Use the existing interface-transmission and adaptive-Jacobian implementation for
insertion/bending pose increments. Keep chassis/knob insertion coupling separate
from distal tendon $u$; do not apply interface takeup a second time to tendon
actuation. Preserve calibration and adaptation sign/frame conventions.

Predict pose inside the joint state transition and correct $q,T_I,u$ together
from marker positions. Update the proximal Jacobian through its existing owner,
with quality/engagement gating and documented uncertainty; no second pose UKF.
Do not update it from an innovation already removed by another pose correction
without accounting for that attribution. Unobservable modes remain uncertain.

Replay qualification uses recorded four-marker-equivalent observations and
encoders. Report marker innovations, pose/shape ambiguity and world-frame
short-horizon errors alongside reference-pose distal-only scores. The latter
remain a diagnostic, not online deployment evidence.

## G3 — MPPI engaged rollout and slow compensation

Implementation: `EngagementRuntime.predict_control_sequence[_coarse]` and
`catheter_control/planning/engagement.py`. The latter is a pure, non-publishing
composition surface using the existing `CatheterMppi` and
`TakeupTransactionArbiter`; it is not yet wired into the baseline ROS node.
Both plain and grouped planners are tested against the actual new backend.

The adapter overwrites only tendon axis 2 in an independent backlash snapshot;
axes 0/1 retain their distinct interface evidence. Width quantiles and remaining
travel are derived from the tendon Gaussian-chart distribution, never separate
learned interval states. First shape response maps to PROVISIONAL and a zero
confirmation hold; the joint filter alone promotes ENGAGED. Exceeding the width
95th-percentile bound without response inhibits motion rather than declaring
engagement. Saturation stays held for explicit recovery; successful confirmation
discards the old plan and requires a fresh rollout.

An explicit initial probe intent is required when any enabled axis is not yet
engaged. MPPI is inactive during that bootstrap and throughout transactions.
Disabled rotation does not participate. Trial compensation is capped at
[1 mm/s, 10 deg/s, 0.5 mm/s] in motor-axis coordinates. The adapter rejects
faster compensators and second raw-transmission/gain-scenario branches. Commands
are projected by the existing hardware contract, but a future ROS composition
must still supply lifecycle/freshness/watchdog checks and manager publication.

Effective-input uncertainty is expressed once as relative std over profile
support: it lowers the existing reversal-risk confidence and conservatively
scales the complete coupled executable action. No new cost weight or shape
objective is introduced; width bounds retain their physical meaning.

Verification: 160 focused model/bundle/filter/planner/backlash tests passed
before final documentation changes. A synthetic two-direction 0.8 s fixture
comparison of 4 versus 24 steps gave terminal disagreement 0.029/0.010 mm,
response-direction cosine 0.9988/0.9989, and passive motion 0.224 mm. Synthetic
CPU 512-sample/four-step tip rollout (20 measured repetitions after 3 warmups)
was 6.59 ms median, 6.99 ms p95, 7.40 ms max. These are component smoke
measurements, not recorded-data accuracy or live full-stack timing qualification.

Reuse grouped and plain planners and existing simplified costs. The initial
comparison changes the runtime, not sample count, temperature, horizon or target
set. Keep the previous qualified 512-sample configuration as the comparison
anchor; qualify the new runtime's actual compute load before selecting it.

MPPI samples **effective engaged dynamics**. It receives the runtime belief's
directional reversal-width mean/std and registration uncertainty to penalize
reversals and reduce aggressive commands when uncertain. No immediate-engagement
hypothesis after reversal; predicted engagement is not an observed confirmation.
Use the existing reversal-risk cost owner, not additional overlapping uncertainty
and confidence penalties. Keep costs for tip targets; no shape target is added.

At an actual reversal, the compensator becomes sole command owner. MPPI does not
issue motion while takeup is active. Slow bounded compensation stops/settles at
shape-response confirmation, saturation or safety failure. Resume MPPI only
when all enabled motion channels it will use are confirmed engaged; disabled
rotation must not block two-axis operation. The distal runtime is the authority
for tendon confirmation; existing interface-axis evidence retains its distinct
physical meaning. Define an explicit reconciliation contract rather than a
duplicate generic engagement estimator.

Compensation commands pass through the same manager, limit projection, freshness
and watchdog paths. Never add velocities from planner and compensator. Width
priors guide bounded travel, but cannot substitute for observed confirmation.

At rest, retain physical passive dynamics. Do not blindly freeze $q$ to remove
false drift; ensure joint $q/u$ uncertainty and equilibration are consistent.
Log predicted passive displacement and observed displacement so false-motion
forecasting is visible. No automatic gain adaptation is added in the first trial.

Batched rollout must reuse the same mechanics/profile transition as inference.
Check the configured four-step, 0.8 s rollout against fine source-time rollout;
do not assume the offline assessment used four 0.2 s steps. Report directional
and magnitude disagreement before the hardware trial.

## G4 — Non-commanding integration and full-stack timing gate

Implementation available (2026-10-04). Commands and qualification limits are in
[G4 shadow qualification](../../../repository_archives/cleanup_20261102/files/robot-infra/audits/ros-realtime/G4_ENGAGEMENT_SHADOW_QUALIFICATION.md).
This completes the implementation slice, **not** recorded-data or representative
full-stack qualification. G5 remains blocked until the resulting reports are reviewed.

`runtime_supervision engagement_g4 prepare` hash-binds the spatial mechanics,
directional prior, engaged profile and v175 interface artifact, and exports native
encoder/marker streams through the existing profiling bag reader. It preserves
receipt order and source timestamps without episode resets, E-step poses or baseline
latent initialization. `replay` runs the independent causal estimator and actual
grouped/plain planner, producing `report.json` and `timing.csv`.

`runtime_supervision engagement_shadow` is a separate bounded observer process in
`/experimental_engagement`. Its subscriptions retain latest images, positions and
targets plus bounded, ordered raw encoder history; one estimator callback
owns mutable history and exchanges deep snapshots with a separate planning callback
group. It publishes only a private String status: no command publisher, mode/arm
service, action client, device link or automatic startup of other nodes.
Following the first live failure, this owner mirrors the baseline's 50 Hz owner /
20 Hz causal marker scheduling using the same `CausalMarkerSchedule`. Each tick
processes at most four raw encoders and one eligible image, rather than draining
an entire mixed FIFO. Baseline diagnostics bypass the estimator handoff. Raw
encoder reversals are never smoothed or silently superseded; a 128-entry raw-history
overflow latches shadow planning inhibition and fails qualification. Latest-image
replacement is counted explicitly. No deadline or freshness threshold changes.
Burst scheduling tests and the ROS build pass; paced DDS replay and renewed
representative full-stack qualification remain required before G5.
MPPI is skipped while the observed enabled joints are unconfirmed or feedback is
stale. Proposed compensation is counterfactual: its transaction is not persisted
as though a command had actually executed. This observer cannot qualify closed-loop
compensation; that remains G5.

Recorded target messages are used when present. Until one arrives, a home-tip plus
5 mm probe is explicitly labeled as a timing probe, not a matched control target.
The four-versus-24-step comparison runs once, with separately reported cost.
In live ROS shadow mode it is deferred until executor shutdown using one retained
independent snapshot; `post_run_rollout_check` and `post_run_coarse_fine_check`
report its result and duration separately from planner callback timing. Offline
compute replay retains its original inline comparison. Stop the reaching task
before ending the shadow so post-run diagnostics do not compete with active control.
Reports include source-matched marker/tip residuals and phases, passive/coarse-fine
predictions, encoder prediction, UKF rewind/correction/replay, snapshots, planner
subcomponents, callback duration/period lateness and queue wait. Undefined zero-motion
direction cosines are JSON null. Posterior residuals are **not** future rollout accuracy.
Baseline traces are comparison-only and never initialize the new filter.

The observer's queue wait is callback-entry-to-owner time, not DDS/kernel latency.
G4 compute optimization now avoids discarded encoder-history returns and uses
history-free prediction snapshots, while the estimator retains its complete replay
buffer. Delayed corrections use copy-on-write suffix/rollback records. Full snapshots
remain the default API; no physical history, model dynamics or safety gate was altered.
The matched CPU 25 s replay improved about 19x with exactly matching traces/counters;
this is not CUDA/full-stack qualification.
No marker-detection timing is invented: camera/tracking latency must be supplied by
the running perception stack and its existing full-stack recording qualification.
Missing samples, unconfirmed phases, rejected observations, overflow, stale inputs
and deadline misses remain visible; no safety threshold is widened.

Run recorded replay first, then an explicitly non-commanding shadow instance
with distinct namespace/diagnostics and no command publisher or mode claim.
Only the baseline may command during any separately authorized baseline run.
Compare marker estimates, engagement decisions, planned response and cycle timing.
Do not clone the baseline's latent history into the new model; each estimator
must process the same observations continuously under its own state semantics.

Reuse existing compute profiling and full-stack timing qualification. Include
UKF correction/rewind/replay, encoder prediction, snapshot transfer, batched
rollout and planner time under tracking/recording load. Keep existing safety
thresholds; do not hide misses by widening them. Sparse-marker observability and
timing can block deployment even if the full-centerline offline model looks good.
Scheduling changes, if needed, require the ROS realtime audit workflow.

Sparse engagement correction (2026-10-04): per-marker covariance still enters the
joint UKF likelihood, but no longer imposes a separate 3 mm deformation floor.
Confirmation uses the same direct pair-distance descriptor as phase2e: a 0.3 mm
minimum or three-sigma pre-reversal descriptor noise, whichever is larger. Passive
continuation, visibility-change resets, two supported response observations and
reversal memory remain unchanged. `report.json` now records raw motor coordinate and
descriptor noise; `engagement.png` plots motor/evidence/threshold/mode without using
posterior strain as confirmation evidence.

Full recorded CPU replay of the selected research bag (83.9 s of data) accepted
1,893 markers and produced 219 valid 512-sample plans with no deadline misses.
Thresholds were 0.300–0.338 mm; planner p95 was 10.78 ms, max 13.98 ms.
This is non-commanding component qualification, not full-stack or hardware approval.

## G5 — Bounded hardware reaching comparison

Implementation status (2026-10-05): the explicitly selected command-trial path
is implemented, **not yet hardware-qualified**. See
[G5 commands and qualification](../../../repository_archives/cleanup_20261102/files/robot-infra/audits/ros-realtime/G5_ENGAGEMENT_CONTROL_TRIAL.md).
The successful G4 repeat supports this bounded pilot; it does not establish
closed-loop compensation performance or a clean controller attribution result.
The historical blockers below describe earlier attempts and are superseded by
the latest dispositions in the maintained G4 and optimized-eager audits.

G5 derives a separate hash-bound `joint_engagement_control_trial` manifest from
the verified G4 bundle. G4 remains non-commanding and is never overwritten.
`engagement_control_enabled` must explicitly match this selection. The parent
uses the same joint Cartesian UKF on CPU, and a spawned GPU worker performs
512-sample, four-by-0.2-second engaged rollouts. Frozen v171 spatial mechanics
and the fixed proximal Jacobian remain unchanged; no second gain estimator or
gain-scenario branch is enabled.

The existing ROS node owns lifecycle, mode claims, heartbeat, faults and final
projection. The existing compensator/transaction arbiter owns slow take-up
(motor-axis caps 1 mm/s insertion, 0.5 mm/s tendon; rotation disabled). Until
insertion and tendon are confirmed engaged, an explicit bounded probe intent
is routed through that arbiter and MPPI is inactive. Width priors cannot
confirm engagement. A coherent snapshot is used for planning; engagement and
registration are rechecked at commit. Increased input uncertainty scales the
whole coupled action. All source-age, marker-health, deadline and manager
gates remain in force. No automatic retry or baseline fallback is introduced.

The modest pilot freezes four target positions once from a guarded home,
then invokes the existing journaled `experiments/reaching_session`. Recording
remains coordinated, with content-named directories and artifact/config hashes.
Pilot targets are not yet evidence of reachability; review them before executing.
For a scientific baseline comparison, reuse exactly those absolute target
coordinates with a baseline-specific status contract, not regenerated targets.

Optimized eager installed promotion has passed (32 CUDA/CPU tests, zero historical
conformance differences). First live baseline still has 24/491 deadline misses
and failed point-4 arming because a FAILED axis-0 belief created during disarmed
homing persisted. External-motion ownership repair now invalidates transaction
evidence only for moved disarmed axes, retains learned widths/history, and excludes
externally owned visual samples from engagement confirmation. Active failure
checks remain unchanged. Next qualify the repaired shared-CUDA baseline, then
the opt-in CPU-float64 estimator/CUDA planner trial via research-session arguments;
no default device/precision promotion or shadow comparison yet. See the maintained
optimized eager promotion report for evidence, tests and trial settings.

Follow-up arithmetic repair: method ablation isolates the five historical
differences to batched marker frame multiplication. Preserving its original
per-marker product ordering while retaining static maps and batched local
exponentials makes source and rebuilt-wheel CPU conformance pass without
fixture/tolerance changes. The user-run CUDA eager benchmark now passes all
300 planner solves with zero deadline misses (reference grouped P50/P95
15.01/15.73 ms). The guarded promotion script repeats CUDA marker/FK tests,
backs up existing packages, verifies wheel/import/source identity and runs
installed historical conformance. Run it from a host terminal with controllers
stopped; active installation and representative full-stack timing remain pending.
See the promotion report below for the command and qualification boundary.

Initial installed-package promotion check (2026-10-04): optimized eager wheels pass
58 focused CPU tests, but the historical conformance fixture reports five
small numerical mismatches while the old installed packages pass. Active
`cr-venv` packages were not replaced. See
`audits/ros-realtime/OPTIMIZED_EAGER_PACKAGE_PROMOTION_20261004.md` for hashes,
scope and the pending promotion gate; source benchmarks are not evidence that
the current hardware runtime uses optimized model code.

G4 currently remains blocked by baseline-only estimator freshness failure and
a separate tendon-prehome device transaction failure. The 2026-10-04 repair
removes redundant covariance and scalar-duration synchronization from v171
encoder replay, and makes existing sparse homing fail promptly on a fresh
manager-forwarded device timeout/rejection. The earlier firmware bounded
endpoint-progress fix is already present in source and has an exact-recording
regression; its deployment to the connected Teensy is not established.
See `audits/ros-realtime/G4_ENGAGEMENT_SHADOW_QUALIFICATION.md`. These repairs
do not authorize hardware motion or promote the experimental model.

Requires explicit user authorization at execution time. Use the existing
`experiments` coordinated recording/reaching suite and frozen targets;
`control_tasks` remains the task primitive owner. Do not create another recorder
or reaching implementation. Run baseline and experimental runtime sequentially,
with one manager command source, matched homing, sample budget and controller
variant. Record runtime family/artifact hashes in session names/manifests.

Begin with the previously successful modest no-rotation targets. Gate expansion
to farther targets on bounded behavior, reliable engagement and clean timing.
Report success, final error, time, overshoot, reversals, compensation travel/time,
static pauses, marker loss and deadline/freshness faults—not just prediction MAE.
Log $u$, its uncertainty, width priors, mode, predicted/observed passive motion,
and command owner to make attribution possible.

Success means useful/safe closed-loop behavior comparable to the baseline, with
no unexplained false motion or degraded safety/timing. It does not require every
offline 0.8 s forecast below 2 mm. A failed trial triggers a targeted trace review,
not another unconstrained model family. Rotation and continuous three-axis paths
are out of initial scope because torsional storage remains unmodeled.

## Non-goals and rollback

No replacement of baseline manifests/artifacts; no automatic model switching;
no C++ rewrite; no safety-gate relaxation; no SET_ZERO path; no hardware startup,
arming or motion during implementation. Return to the baseline by selecting its
unchanged manifest and restarting the controller—never switch state families
mid-command. Promotion needs an explicit later decision and updated handoff.
