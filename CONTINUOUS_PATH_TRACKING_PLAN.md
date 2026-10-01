# Continuous Catheter Tip Path Tracking Plan

## Purpose

Replace the current stop-and-settle waypoint behavior with continuous,
time-parameterized tip-path tracking while preserving the existing manager,
firmware, estimator, joint projection, freshness gates, watchdogs, and fault
latching.

The immediate target is the existing approach-plus-circle experiment. The
design must also support arbitrary smooth paths in `robot_base`, simulation and
real hardware, fixed or adaptive proximal Jacobians, and the existing
backlash-state estimator/feedforward compensator.

This plan does not change encoder calibration and must not introduce any route
that issues `SET_ZERO`.

## Why the Current Action Is Not Continuous

`TrackTipTrajectory` currently sends one `PointStamped` target at a time. The
`TrajectorySequencer` holds that target until a tolerance/settle test or a
timeout, then makes an instantaneous jump to the next point. The MPPI node
therefore sees a constant target across all rollout steps even though
`CatheterMppi.plan()` already accepts an `(horizon_steps, 3)` future reference
sequence.

The 18-point trial reduced the frequency of target switches, but it did not
remove their underlying effects:

- each switch creates a new position regulation problem;
- the optimizer can reverse or make fine corrections around every point;
- backlash compensation and stiction can turn those corrections into
  take-up/release events and overshoot;
- waypoint timeout advances are unrelated to continuous path progress;
- frame-wise target error is not the desired geometric closest-path error.

Continuous tracking should exploit the existing sequence-capable MPPI cost,
not increase the rollout horizon merely to bridge coarse target jumps.

## Proposed Runtime Structure

```text
YAML path specification
        |
        v
path-file client -- TrackTipPath action --> path reference server
                                                |
                     full path for display -----+----> /reference_path
                     current/preview reference -+----> /reference_horizon
                                                |
                                                v
                                      catheter_mppi planner
                                                |
                           existing estimator, rollout, MPPI, projection
                                                |
                           existing backlash compensator and manager route
                                                |
                                                v
                                      firmware safety authority
```

The action/reference server owns path geometry, phase, completion, cancellation,
and optional auto-arm/disarm. The MPPI node owns only reference validation,
time alignment, cost evaluation, command planning, and existing control safety.
The learned model and estimator remain in `cr_meta_lnn`; ROS lifecycle and
scheduling remain in `robot-infra`.

## 1. New Interfaces Without Breaking Point Control

Keep `TrackTipTrajectory` and `/catheter_mppi/target_tip` for single-target and
legacy waypoint tests. Add a separate action and reference topic.

### `control_interface/action/TrackTipPath.action`

The goal should contain:

- a `std_msgs/Header`; only `robot_base` is accepted initially;
- geometric path knots as `geometry_msgs/Point[]`;
- a nominal tangential speed in mm/s;
- total action timeout;
- final-point tolerance and settle time;
- soft and hard cross-track thresholds;
- an `auto_arm` flag.

Feedback should report:

- normalized path progress and arc length;
- elapsed time and phase lag relative to nominal time;
- current reference point and tangent;
- point-to-reference, cross-track, along-track, and closest-path errors;
- whether the progress governor is running, slowed, or paused;
- current controller state.

The result should distinguish completion, user cancellation, controller fault,
stale feedback/reference, hard path-deviation abort, and overall timeout. It
should include final, RMS, and p95 closest-path errors. A path timeout is an
action failure, unlike an intentional per-waypoint transition in the legacy
action.

### `control_interface/msg/TipReferenceHorizon.msg`

Publish an explicitly timed preview rather than overloading `PointStamped`:

- header stamp: time represented by preview sample zero;
- frame ID: `robot_base`;
- path ID and monotonically increasing sequence number;
- sample period;
- `geometry_msgs/Point[] positions`;
- `geometry_msgs/Vector3[] tangents`;
- current arc-length phase, total length, and nominal speed;
- final-hold and progress-paused flags;
- expiry duration.

The preview must cover reference transport age plus the full MPPI horizon.
With the current four 40 ms rollout steps, publish at least 0.5 seconds of
future reference. The MPPI node resamples it at its actual snapshot time plus
`[1, ..., H] * rollout_step_s`; it must not assume callback arrival time equals
the source timestamp.

Use reliable, keep-last-one QoS for this low-rate control reference. Do not put
path generation or file I/O in the MPPI planner callback.

### Source arbitration

The MPPI node must have exactly one active source: `POINT` or `PATH`.

- Arming with a fresh path horizon selects `PATH`.
- Arming with a valid point and no path selects `POINT`.
- A point publication cannot silently replace an active path.
- A new path goal is rejected while another goal is active.
- Path end, cancel, or abort invalidates the horizon before disarming.
- Any ambiguity fails closed with zero command output.

This prevents a utility such as `catheter_target_offset` from changing an
active path accidentally.

## 2. Smooth Path and Motion Law

Represent path geometry by arc length `s`, separately from execution time.
Construct a continuously differentiable interpolant `p(s)` and unit tangent
`T(s)`. A monotone cubic Hermite or cubic B-spline implementation is adequate;
it must avoid overshooting the supplied spatial knots.

For the hardware circle:

1. sample the measured tip once at goal acceptance;
2. construct the circle relative to that accepted start state, rather than a
   stale recorded tip embedded in YAML;
3. connect the start to the circle entry with a bounded-curvature blend whose
   terminal tangent matches the circle tangent;
4. traverse the analytic YZ circle at constant nominal arc speed;
5. decelerate near the final point and apply a settle requirement only there.

The circle itself remains centered at `(x0 + 10 mm, 0, z0)` with 10 mm radius.
The 18/36-point count is no longer a control parameter: spatial samples only
approximate and visualize the curve. Accuracy is controlled by interpolation
error and reference sample period.

### Progress governor

A purely wall-clock reference can run away while stiction prevents motion. A
nearest-point phase can jump or rewind under noise and on self-intersections.
Use a monotonic governed phase instead:

- advance at nominal speed while cross-track/reference error is below a soft
  threshold;
- continuously reduce phase speed through a configurable transition band;
- pause, but never rewind, above the pause threshold;
- resume with hysteresis after the error recovers;
- abort and disarm on a hard error, stale feedback, controller fault, or total
  action timeout.

The governor changes only how quickly the reference advances. It does not
bypass MPPI constraints or manufacture actuator motion. Log nominal and
governed phase separately so pauses cannot masquerade as good time tracking.

## 3. MPPI Reference and Cost

### Minimal first implementation

> Superseded for the next correction by the revised post-take-up arbitration
> plan below. In particular, measured backlash state must no longer affect
> MPPI candidate response, cost, or sampling constraints.

Pass the resampled `(H, 3)` position preview directly to the existing
`CatheterMppi.plan()` API. Its present pointwise tip cost, terminal weight,
joint projection, slew cost, reversal cost, and backlash rollout model can all
remain intact. This provides a small, testable change and demonstrates whether
removing target jumps is sufficient.

The current `H=4`, `dt=0.04 s` 160 ms prediction window remains the initial
baseline. Continuous reference does not require a long rollout horizon merely
to see past backlash because take-up is handled by the persistent estimator and
feedforward compensator.

### Path-aware refinement

After the minimal version passes simulation, optionally decompose each
predicted error using the supplied tangent:

```text
e_parallel = dot(tip - p, T) T
e_cross    = (tip - p) - e_parallel
cost       = w_cross ||e_cross||^2 + w_phase ||e_parallel||^2
```

Start with `w_phase < w_cross` so the controller prioritizes staying on the
curve without making aggressive reversals solely to recover schedule lag. Keep
a terminal position term so completion is well defined. Tune existing slew and
reversal weights using measured reversal count, take-up time, and closest-path
error; do not add a duplicate smoothing mechanism.

Path progress must not be inferred from motor encoders alone. The feedback
quantity is the measured tip in `robot_base`; the estimator and proximal
Jacobian remain responsible for mapping motor action to interface/distal
motion.

## 4. Scheduling and Freshness Contract

Use these initial scheduling boundaries:

- reference update: 30 Hz or faster;
- preview duration: at least 0.5 s;
- MPPI planning: existing 15 Hz baseline;
- command heartbeat: existing 100 Hz;
- controller reference-age limit: explicitly configured and shorter than the
  action feedback timeout;
- clocks: monotonic time for elapsed durations and governor state; ROS source
  stamps for inter-node alignment and recorded evidence.

The reference callback stores a validated immutable snapshot and returns. The
planner atomically snapshots state, position, prior command, and one reference
version. It then interpolates the preview without holding the main controller
lock. If the preview is stale, expired, too short, non-monotonic, in the wrong
frame, or changes path ID during an active plan, command zero and enter the
existing fault/release sequence.

Reference generation must run in the action server's separate process and
callback group. Its timer must not wait on service calls or marker callbacks.
Service calls remain asynchronous with bounded waits, as in the current action
server.

## 5. Safety and Completion Semantics

Preserve all current safety layers:

- estimator and marker health gates;
- POS/ENC validity and freshness;
- manager readiness and exclusive mode claim;
- MPPI planning deadline handling;
- per-rollout and heartbeat-time joint projection;
- command freshness and firmware watchdog;
- manager/firmware fault latching.

Add path-specific fail-closed checks:

- finite points, positive speed, nonzero path length, bounded knot spacing;
- smooth-path interpolation error below a configured limit;
- bounded reference speed and acceleration;
- valid `robot_base` frame and fresh source stamp;
- no discontinuity between accepted current tip and approach start;
- reference preview long enough for every planner rollout timestamp;
- soft progress hold and hard geometric deviation abort;
- total-duration watchdog independent of the progress governor;
- disarm on every action exit path, including exceptions.

Cartesian path validation is not proof of reachability. Joint constraints in
the rollout remain authoritative. Initial hardware paths must stay away from
known workspace and joint boundaries; repeated boundary projection or near-zero
progress should abort rather than allow indefinite saturation.

Do not physically home between path samples. That would destroy continuity,
add large unnecessary travel, and replace the history problem with repeated
history-reset transients.

## 6. Recording and RViz

Record enough information to distinguish path design, optimization, model,
backlash, and execution errors.

Add a per-update `PathTrackingTrace` topic containing:

- source and arrival timestamps, path ID, and sequence;
- nominal and governed phase/speed;
- current reference point, tangent, and preview endpoint;
- measured tip and closest point on the full path;
- closest-path, cross-track, along-track, and point-reference errors;
- progress-governor state and accumulated paused time;
- controller state and fault reason;
- desired post-backlash command, compensated command, joint position, encoder
  counts, backlash phase/remaining take-up, and Jacobian/adaptation summary.

Continue recording estimator traces, MPPI plans/predictions, response traces,
manager commands, confirmed serial writes, POS/ENC, markers, diagnostics,
parameter events, and ROS logs. Add the action feedback/status, reference
horizon, full reference path, and path trace to both hardware and simulation
bag topic lists and manifests.

Publish for RViz:

- the full curve as a line strip or `nav_msgs/Path`;
- the governed reference point and future MPPI preview;
- measured tip trail;
- closest point and cross-track error segment;
- visible `robot_base` axes;
- text for progress, error, pause state, and controller state.

Use the same visualization node and topic schema in simulation and hardware,
with namespace remapping only.

## 7. Implementation Phases

### Phase A: deterministic path library and interfaces

- Add `TrackTipPath`, `TipReferenceHorizon`, and `PathTrackingTrace` to
  `control_interface`.
- Implement a ROS-independent `ContinuousPath` and `ProgressGovernor` library.
- Support polyline knots with monotone interpolation and the analytic YZ-circle
  generator with a tangent-matched approach blend.
- Extend the YAML loader with `mode: continuous`, nominal speed, total timeout,
  final settle, and progress thresholds.
- Keep the legacy action unchanged.

### Phase B: minimal future-reference MPPI wiring

- Add the horizon subscription and explicit source arbitration to the MPPI
  node.
- Resample the horizon at the exact rollout times and pass `(H,3)` into the
  existing planner.
- Extend lifecycle readiness with path reference presence, freshness, expiry,
  and coverage reasons.
- Record and display the full path, active reference, and preview.
- Do not change the MPPI cost or adaptive Jacobian in this phase.

### Phase C: simulation comparison

- Run the existing waypoint controller as the frozen baseline.
- Run continuous reference with identical model, estimator, limits, backlash,
  circle geometry, and random seeds.
- Sweep path speed, preview publication rate, governor thresholds, MPPI horizon,
  slew weight, and reversal weight.
- Add path-aware cross/along-track weighting only if the minimal pointwise
  horizon still produces avoidable schedule-catch-up reversals.

### Phase D: full-stack shadow run

- Run real cameras, manager, estimator, and MPPI with command output disabled.
- Verify reference freshness, source arbitration, CPU scheduling, bag content,
  frame consistency, and predicted command/Jacobian behavior under full load.
- Verify cancel, stale reference, marker loss, manager inhibit, and controller
  fault all terminate the action and disarm cleanly.

### Phase E: staged hardware promotion

- Begin with fixed Jacobian and online adaptation disabled, while retaining the
  validated backlash estimator/compensator.
- Track the smooth approach and a short interior circle arc at conservative
  speed.
- Promote to half and full circles only after joint margin, closest-path error,
  reversal rate, and fault metrics pass.
- Enable adaptive Jacobian updates in a later controlled comparison; adaptation
  must retain the accumulated-motion, response-SNR, directional-purity,
  reversal-holdoff, and bounded-column gates.

## 8. Tests and Acceptance Evidence

### Unit and integration tests

- curve interpolation, arc length, tangents, closure, and no overshoot;
- tangent-continuous approach-to-circle transition;
- progress advance, slowdown, pause hysteresis, resume, timeout, and no rewind;
- horizon timestamp validation and resampling at planner times;
- `CatheterMppi` response to a changing `(H,3)` reference;
- source arbitration between point and path targets;
- stale/short/wrong-frame/changed-ID reference fail-closed behavior;
- action success, cancel, controller fault, total timeout, and guaranteed
  disarm;
- launch remapping, rosbag topic inclusion, and RViz topic availability.

### Simulation scenarios

Run nominal and robustness cases with fixed seeds:

- ideal model/no backlash;
- measured asymmetric backlash and stiction;
- marker noise, delay, dropouts, and timestamp skew;
- proximal Jacobian gain/direction perturbations;
- combined perturbations near, but not at, safe joint margins.

Compare waypoint and continuous modes using:

- closest-path median/RMS/p95/max error;
- fraction within 1.8 mm;
- along-track lag and total paused time;
- completion time;
- command sign reversals per axis;
- backlash take-up events and distance spent in take-up;
- joint-limit projection frequency and minimum margin;
- MPPI plan p50/p95/max time and deadline misses;
- stale-data, watchdog, and controller fault counts.

The continuous version should pass all safety/freshness tests, introduce no
deadline regression, materially reduce reversal/take-up events, and improve
closest-path RMS/p95 relative to the frozen waypoint baseline before hardware
promotion. Numerical thresholds beyond the existing 1.8 mm tolerance should
be chosen from the simulation sweep and the current hardware baseline rather
than guessed in code.

## Recommended First Slice

Implement Phases A and B with the existing MPPI pointwise cost, `H=4`, and
40 ms rollout step. In simulation, start with 1, 2, and 3 mm/s path speeds and
the same circle geometry. This isolates the benefit of continuous future
references from cost-function, horizon, and Jacobian changes. Only after that
A/B comparison should path-aware weighting or hardware output be introduced.

## Implementation status (2026-09-14)

Phases A and B are implemented. The deployed slice includes:

- `TrackTipPath`, `TipReferenceHorizon`, and `PathTrackingTrace` interfaces;
- a chord-length PCHIP path, closest-path projection, and monotonic
  slow/pause/resume governor;
- a 30 Hz action server with 0.2 s history, 0.4 s future coverage, explicit
  expiry, total timeout, hard-error abort, and unconditional exit disarm;
- explicit `POINT`/`PATH` arbitration and estimator-time `(H,3)` resampling in
  the MPPI node;
- YAML clients for current-tip-relative simulation and hardware circles;
- rosbag topic coverage and RViz display of the full path, active reference,
  and future preview.

The legacy waypoint action is unchanged. Path-aware cross/along-track cost
weighting is deliberately deferred until the pointwise future-reference
baseline is measured. The next gate is the Phase C simulation comparison at
1, 2, and 3 mm/s using the healthy 1024-sample CUDA route.

The path-file client now accepts `--speed-mm-s` and `--total-timeout-s`
overrides. This keeps one frozen path geometry/governor YAML for the Phase C
speed sweep and avoids untracked manual configuration edits. Each speed trial
must use a freshly restarted simulation so plant, estimator, and backlash
initial state are comparable.

### Phase C initial sweep result

The 1, 2, and 3 mm/s CUDA trials all completed without a controller fault, so
the continuous-reference action, governor, MPPI route, and simulation-only
deadline recovery are operational. The comparison is not suitable for speed
selection: every run reached the autonomous joint-0 lower boundary near 67% of
path progress, and error in the constrained portion dominated the aggregate
metric. The next Phase C slice is therefore an interior-path generator or
offline reachability preflight, followed by the same speed sweep. The current
full circle must not be promoted directly to hardware.

## Revised post-take-up arbitration plan (2026-09-15)

The simulations after the initial sweep show that the planner and backlash
compensator currently have overlapping authority. MPPI sends desired
post-engagement rates to the learned rollout, but measured take-up state also
changes candidate costs, locks rotation sample signs, and can replace the
selected first action. The correction is to make the boundary explicit:

```text
continuous reference + estimated catheter state
                 |
                 v
      post-take-up MPPI (no backlash state)
                 |
           desired effective command
                 |
                 v
       transmission transaction arbiter
          /                         \
 engaged in requested directions    one or more shafts pending
          |                          |
 execute freshly planned vector      execute take-up shafts only
                                     hold other active shafts at zero
                                     freeze path progress
                                     |
                          all pending responses confirmed
                                     |
                           command zero and replan
                                     |
                           execute fresh MPPI vector
```

### A. Planner contract

MPPI optimizes only desired post-take-up motion. Remove take-up width,
remaining gap, engagement phase, direction latch, take-up-delay cost, and
transmission-derived first-step reversal cost from candidate generation and
scoring. Retain joint-position/velocity projection, the ordinary desired-input
slew/reversal costs, and the learned distal/proximal rollout.

Separate optimizer-state update from executable-action selection. The weighted
MPPI sequence updates the sampling nominal, but execution uses the
minimum-total-cost trajectory that was actually scored. Deterministic zero is
candidate 0, so the selected sample cannot be costlier than hold within the
evaluated batch. This avoids another learned rollout and prevents a diffuse
weighted mean from being executed when no sampled response supports it. Record
zero, selected, best, and weighted-approximation costs.

### B. Transaction state machine

Add one stateful arbiter downstream of MPPI and upstream of the existing
manager/safety projection:

```text
READY_TO_PLAN -> TAKEUP_ACTIVE -> REPLAN_REQUIRED -> READY_TO_PLAN
                      |       \
                    FAILED   SATURATED_REPLAN
```

At `READY_TO_PLAN`, convert the selected logical first action to physical motor
shaft rates before deciding engagement. The active set contains only nonzero
physical shafts after deadband and safety projection. For each active shaft:

- `READY`: response-confirmed engaged in the requested direction;
- `PENDING`: unknown direction, active remaining gap, or a requested reversal;
- `FAILED`: take-up exceeded the existing bounded-travel/error contract.

If all active shafts are `READY`, execute the fresh desired vector. If any are
`PENDING`, start a transaction and latch its physical active mask and direction
vector. Do not depend on subsequent stochastic MPPI calls returning the same
sign.

During `TAKEUP_ACTIVE`:

- command only pending shafts at their bounded directional take-up rates;
- hold active shafts that are already engaged at zero;
- do not execute, blend, or partially release the desired MPPI vector;
- retain partial take-up travel and confirmation state across frames;
- let upstream encoders advance the transmission estimator and let accepted
  UKF interface motion terminate take-up early when response is observed;
- fault through the existing fail-closed route if a shaft exceeds its bounded
  take-up travel, feedback becomes stale, or another safety gate fails.

When every transaction shaft has its first strongly attributed response,
output zero for the transition and enter `REPLAN_REQUIRED`. Such shafts enter
`PROVISIONAL`: high-rate take-up has no further authority, while repeated
response observations still decide whether the state becomes `ENGAGED` or
falls back to `TAKEUP`. Discard the pre-take-up plan and run a fresh solve from
the newest estimator state, joint positions, path reference, and joint
margins. This prevents the confirmation tail from driving through a nearby
target while retaining fail-closed statistical confirmation.

Only physical shafts in the transaction participate in the barrier. A shaft
whose requested physical rate is zero neither moves nor delays release, even
if it retains an old partial-gap estimate. Because logical bend/insertion
commands can map to multiple physical shafts, the active and pending masks
must never be formed in logical coordinates.

### C. Continuous-reference behavior during take-up

Add an explicit `TRANSMISSION_HOLD` input to the path governor. Freeze path
phase and future reference progression throughout `TAKEUP_ACTIVE` and
`REPLAN_REQUIRED`; continue publishing the fixed horizon for observability.
The measured tip may move slightly because real take-up is not an ideal dead
zone, so the estimator continues normally. After the fresh post-engagement
plan is committed, resume progress using the existing hysteresis rules.

### C.1 Causal saturation boundary

Every take-up command is re-projected through the complete logical-position,
coupling, integer-RPM, and physical-shaft contract before publication. If any
pending shaft loses its requested physical direction, or projection moves a
shaft that the transaction intended to hold, the transaction is atomic: send
zero, enter `SATURATED_REPLAN`, and discard the old plan. Record the measured
encoder position, requested/realized physical rates, blocked mask, and leakage
mask at this event.

The saturated physical direction remains blocked in subsequent MPPI candidate
selection. Direction tests use the candidate's physical intent *before*
position clipping, since clipping can erase the saturated shaft while leaving
coupled motion on another shaft. A candidate touching the registered direction
is ineligible; deterministic zero remains available. The block is released
only when current measured joint feedback makes an isolated take-up command in
that direction survive the final projection without coupled leakage. Thus
unknown take-up travel never becomes an invented MPPI margin, while the
encoder position observed at saturation becomes a causal directional boundary.

MPPI may run in shadow during take-up for diagnostics, but its output has no
actuation authority. The efficient default is to suspend expensive planning
until engagement changes or the transaction completes. Target cancellation or
a changed path ID aborts the transaction safely, commands zero, preserves the
measured partial-gap state, and follows the normal disarm/reinitialization
route.

### D. Reversal arbitration

Once backlash state is removed from MPPI, noisy replans must not repeatedly
start opposite-direction transactions. Keep ordinary desired-input slew cost
inside MPPI, then require a proposed physical reversal to persist for a small
number of fresh plans and to improve on zero by a configured cost margin before
opening a new take-up transaction. While reversal intent is being confirmed,
command zero rather than continuing motion in the old direction. This policy
belongs to the transaction arbiter, not the learned rollout.

### E. State ownership and stale-plan protection

Keep the estimator callback as the sole owner of transmission state. Give each
snapshot a monotonically increasing transmission generation. A plan commit is
valid only if its estimator timestamp and generation still match the current
snapshot. A take-up transition invalidates any in-flight plan. Maintain four
separate diagnostics instead of overwriting one `last_effective_command`:

- selected post-take-up MPPI command;
- latched transaction direction/active mask;
- physical take-up command;
- final safety-projected command sent to the manager.

### F. Verification order

1. Unit-test all state transitions, mixed ready/pending axes, logical-to-motor
   coupling, response-confirmed early engagement, bounded failure, target
   cancellation, stale plan generation, and zero fallback.
2. In deterministic simulation, deliberately mismatch controller and plant
   take-up widths. Assert that no productive component is released while any
   active shaft is pending and that the path reference remains fixed.
3. Repeat the current continuous circle with the same seed and initial state.
   Require no initial divergence, no invalid weighted command, no repeated
   reversal transaction, and completion without a hard path error.
4. Repeat with asymmetric width error, small pre-clearance response, marker
   noise/delay, and mixed-axis commands.
5. Run a non-actuating full-camera hardware shadow before enabling output.
   Hardware motion remains a separate explicitly authorized step.

## 2026-09-15 response-handoff and path-tube correction

The `20260915_183741_mppi_sim` audit showed that 31 of 35 rotation reversals
followed a real target crossing during full-rate response confirmation. The
closest-path error remained small, but the exact moving reference stayed
behind the tip and requested reversal into another take-up transaction.

The implemented correction has two coupled parts:

1. The first response that passes transmitted-motion magnitude, sign,
   direction-cosine, and leave-one-column-out evidence gates changes the shaft
   from `TAKEUP` to `PROVISIONAL`. The transaction stops, publishes zero for
   its existing transition period, and requests a fresh MPPI plan. Repeated
   evidence still commits `ENGAGED`; two inconsistent moving observations
   revert to `TAKEUP`. Width learning remains restricted to confirmed
   engagement.
2. The path governor treats `soft_error_mm` as a geometric capture tube. If
   the measured tip is slightly ahead along the path and remains in that tube,
   the monotonic reference catches up by at most `pause_error_mm` in one
   update. Slow/pause decisions use cross-track error plus positive lag;
   along-path overshoot no longer asks the controller to return to an obsolete
   point. The bounded catch-up also prevents a closed curve's coincident end
   point from skipping an entire lap.

Testing proceeds in this order:

1. Unit-test first-response handoff, provisional confirmation and fallback,
   mixed-axis transaction release, path-tube catch-up, external-hold freezing,
   and closed-path endpoint ambiguity.
2. Run the deterministic causal-v2 simulation with controller and truth
   backlash widths matched. Require no hard path error and inspect that
   `PROVISIONAL` immediately replaces full-rate `TAKEUP` after first response.
3. Repeat with controller widths intentionally high and low relative to the
   truth model. Require bounded take-up, no repeated reversal limit cycle, and
   monotonic path progress.
4. Run a recorded, non-actuating hardware shadow and estimate the false
   provisional-response rate under stationary and ordinary marker-noise
   conditions. Do not interpret response evidence as a calibrated
   probability.
5. Only after those gates pass, repeat the explicitly authorized low-speed
   hardware path with the existing manager, freshness, limit, and watchdog
   protections unchanged.

## 2026-09-15 post-take-up planner-continuity correction

The `20260915_191324_mppi_sim` run established that the remaining oscillation
had moved from rotation to insertion: 56 insertion reversals occurred in 108
plans even though the Cartesian error direction normally did not cross the
target. The controller was discarding both the shifted MPPI nominal sequence
and the previous desired input whenever a response completed an ordinary
take-up transaction. Each post-take-up solve consequently restarted around
zero with no first-step direction memory.

The implemented correction separates invalidating an action from invalidating
the planner distribution:

- `REPLAN_REQUIRED` commands zero for the transition and runs a fresh rollout,
  but preserves the shifted nominal sequence and `last_effective_command`;
- `SATURATED_REPLAN` still clears both because saturation changes the feasible
  command set;
- target changes, disarm, faults, and planner failures retain their existing
  hard-reset behavior;
- MPPI candidate cost now includes a soft, per-shaft first-step reversal cost
  in physical motor coordinates. The deployed ROS default is `0.10`.

The next deterministic run should keep the same plant, seed, path, speed, and
take-up settings as `20260915_191324_mppi_sim`. Compare path progress,
`TRANSMISSION_HOLD` occupancy, insertion/rotation reversal counts, and
closest-path error. The correction passes the first gate only if insertion
reversals and hold occupancy fall without increasing path error or hiding a
genuinely beneficial reversal.

## Sparse independent-target sanity test

Before further controller arbitration changes, isolate point reachability from
continuous path history. Sample eight unique points uniformly on the existing
10 mm-radius circle in the base YZ plane. Freeze the absolute circle from the
measured tip after reaching joint home `[20,0,0,0,0,0]`. For every point:

1. keep MPPI disarmed;
2. return through the manager's guarded position mode to the same joint home;
3. require fresh position feedback inside per-axis tolerance and a settling
   interval;
4. release position mode and submit exactly one tip target;
5. let the trajectory action reach or time out, then disarm before the next
   home.

This produces a consistent initial mechanical history and one fixed Cartesian
error vector per trial. Compare reachability, final error, joint excursion,
take-up transactions, and selected command direction point by point. It is not
a path-tracking score: no arc-order or timing requirement connects the target
points.

The `20260915_195840_mppi_sim` run reached the first two points, then exposed
a separate feasibility-contract defect at point 3. MPPI drove positive
insertion until raw axis-0 feedback reached 109,839 counts, immediately below
the learned model's configured 110,000-count validity ceiling. Candidate
projection honored logical joint bounds but not this tighter coupled
motor/count envelope. The simulated plant then rejected the next state,
feedback stopped, and the manager correctly inhibited with
`FEEDBACK_NOT_QUALIFIED`.

The shared hardware contract now projects the first three physical motor axes
against the raw model-count envelope one guard horizon ahead, including RPM
quantization. Both scalar executable projection and batched MPPI sampling use
the same coupled constraint. An unreachable sparse target should therefore
settle at the feasible boundary and consume its waypoint time budget instead
of invalidating model feedback and faulting the stack.

## 2026-09-15 sparse-target reversal scheduling correction

The `20260915_203808_mppi_sim` rerun established that reversal churn persists
with a fixed target and identical home history. Point 2 alternated rotation
take-up direction across generations 3--11 and timed out about 6.43 mm from
target. Point 3 repeated direction churn on bend until the bounded take-up
guard faulted axis 2. F-042 behaved correctly; the missing layer is the
reversal arbitration already specified in section D above.

Implement that arbitration as a per-axis response-clocked direction lease.
After credible engagement, same-sign motion and zero remain available. A
reverse sign must beat an explicitly scored constrained plan, persist for
multiple fresh solves, and satisfy productive-response and cooldown gates.
Until then, constrain that physical shaft to zero before rollout and continue
planning the other shafts. Never delete a motor component after scoring.
Diagnostics and tests must expose lease direction, pending reversal direction
and count, constrained-versus-reverse cost margin, and the decision reason.

The next simulation gate is the same eight-point sparse experiment. Passing
requires no controller fault, no every-transaction sign alternation, and no
regression of maximum-width, saturation, limit, freshness, or manager safety
behavior. Only after that gate should continuous-path testing resume.

## Torsional-windup management after hardware session 20260916_172704

The hardware record shows that an insertion-dominant transaction can release
interface rotation comparable to the preceding rotation transaction. This is
history-dependent transmission behavior, not a fixed insertion Jacobian
column. Track two candidate designs without conflating them with backlash:

1. **Prevent accumulation:** estimate encoder rotation not expressed as UKF
   material roll, stop rotation take-up on any credible causal response,
   penalize further windup, and prefer insertion before final rotation.
2. **Unwind before insertion:** when estimated windup exceeds a threshold,
   pause progress and execute a bounded, response-terminated unwind followed
   by zero and replanning. Encoder home alone is not proof of neutral torsion.

The immediate engagement fix should accept a dominant isolated response as
mechanical engagement while retaining Jacobian-direction checks for learning.
Off-model release samples must be logged, not fitted into the nominal local
Jacobian. Preserve all manager, projection, timeout, and firmware gates.

Before either design is enabled on a continuous path, use the hardware sparse
point experiment. It returns the logical joints to `[20,0,0,0,0,0]` before
each independent target but deliberately preserves physical transmission
history. It therefore measures whether point reachability remains repeatable
under real remanence; it is not an unwind/reset experiment.

The direction-lease scheduler is now implemented. MPPI evaluates the
unrestricted candidate and a scored alternative that holds leased axes against
unapproved reversal. A reversal must persist for three plans, beat the
constrained solution by both configured margins, follow at least three accepted
observations, and satisfy a one-second cooldown. The approved sign is consumed
by exactly one take-up transaction. Unit/package verification is complete; the
eight-point sparse simulation remains the acceptance test.

The `20260915_223716_mppi_sim` acceptance run failed despite completing without
a fault. Only point 1 reached tolerance. The implemented constraint means
“do not oppose the lease,” which permits continued motion in the old direction;
it does not hold a pending reversing shaft at zero as specified. In addition,
normalizing reversal benefit by the complete horizon cost kept stable useful
reversals at only 2--7% versus the configured 15% threshold. Replace this with
explicitly scored physical-shaft hold branches and per-axis ablation evidence
before another sparse or continuous-path run. See F-044 and the corresponding
session audit.

The F-044 correction is now implemented. Pending reversal branches neutralize
the affected physical shafts exactly, are reprojected and rerun through the
learned model, and fall back to deterministic zero unless they predict descent.
Single-axis ablations provide causal reversal evidence, with a default 0.25 mm
terminal-benefit floor. The first implementation used a second small learned
rollout and caused repeated 60 ms deadline misses in
`20260916_095434_mppi_sim`. Ordinary, combined-hold, and per-axis-ablation
candidates now share one fixed-size bank and one learned-model call without
exceeding the configured sample population. Local tests and build pass; the
sparse GPU experiment must be repeated to validate tracking and live deadline
behavior before returning to continuous paths.

The `20260916_100502_mppi_sim` rerun passed the timing gate and completed the
first two sparse points, but exposed an axis-2 coordinate-sign bug in the
fixed-size hold bank. Direction leases use physical shaft radians/s whereas
the bank classified firmware motor-axis units; axis 2 has a negative
units-per-RPM conversion. As a result, diagnostics claimed an unapproved
reversal was held while the corresponding physical shaft actually reversed.
Classification now uses physical-rate signs, and execution requires projected
zero physical rate on every held axis. Repeat the sparse test before any
continuous-path or hardware run.

### `20260916_150511_mppi_sim` grouped-mode cost correction

The CUDA/grouped route was active (1,024 total samples and eight proposal
groups), but 50 take-up holds consumed 44.85 seconds, about 42% of the active
path run. Rotation changed nonzero command sign 35 times. Many reversing modes
won for only hundredths of a millimetre of predicted terminal improvement
because the configured take-up-delay cost was zero and the discrete first-step
penalty was only 0.10.

Grouped MPPI now prices the physical transaction inside every complete
candidate objective. A response-confirmed shaft-direction switch contributes
one cost unit per switched axis, and estimated take-up duration contributes
four cost units per second. The switch event is measured against the direction
lease rather than the immediately previous command, which is intentionally
zero during a take-up hold. Before a lease exists, the previous physical
command remains the direction reference. These are soft costs: MPPI still
executes an unmodified reversing plan whenever its predicted benefit exceeds
the complete transaction cost. Diagnostics expose the configured weights and
the selected candidate's switch count, switch cost, take-up duration, and
take-up cost.

## 2026-09-16 UKF-history and grouped-MPPI correction

The post-hoc hold/ablation bank is superseded by grouped MPPI. For every shaft
with a response-confirmed direction lease, one rollout batch covers the two
proposal choices requested by the hardware study: unconstrained (U) and
continue/zero (C). With three active leases this yields eight proposal groups.
After projection, candidates are classified by their actual complete reversal
mask and the minimum-cost complete plan in each mask is retained. Reversal
diagnostics compare the best reversing mode with the best no-reversal mode.
Grouped MPPI executes the globally best feasible complete plan; the legacy
response-clocked scheduler is not a second execution gate. No element of any
chosen plan is zeroed, blended, or replaced after scoring. A selected reversal
is still realized by the bounded, response-confirmed take-up transaction.

MPPI remains defined in useful post-take-up coordinates, while candidate
feasibility now includes take-up encoder travel. Directional learned widths,
or the active remaining gap, are converted from physical shaft radians through
the firmware transmission and coupling into a logical joint-position offset.
The full horizon is projected from that offset position. This closes the prior
gap in which useful motion respected limits but the compensator travel needed
to reach useful motion did not.

The UKF now reconciles its distal strain posterior with the learned scalar
tendon history. An accepted strain correction is projected onto the scalar
equilibrium coordinate; the difference from the prior strain's equilibrium is
stored as a bounded history correction and carried through causal rewind,
replay, cloning, and batched rollout. It therefore preserves learned rate and
memory effects while removing the false zero-command relaxation produced by a
marker-corrected strain paired with stale history.

The reviewed fixed-J profile enables this reconciliation, grouped sampling,
take-up limit reservation, CUDA, and 1024 total samples. These changes require
the sparse simulation and full-stack timing gates before hardware output is
enabled.

### `20260916_145549_mppi_sim` grouped-bank failure

The controller moved for approximately four seconds and then selected hold
for the rest of the run. This was not a governor pause: progress reached
6.146 mm, the governor remained `SLOWED`, and the closest-path error grew to
3.444 mm. The plant and model response agreed closely before the stop.

The run exposed a fixed-population partition bug. With 32 total samples and
three active leases, the eight U/C groups should each receive four distinct
candidates. Instead every group copied the same first four candidates and the
remaining 28 samples were discarded. Those four were deterministic hold,
warm-start, and positive/negative insertion, so rotation, tendon, and all
late exploration candidates disappeared. Once the required correction was no
longer insertion-dominant, hold was the lowest-cost remaining candidate.

Groups now receive disjoint round-robin slices of the complete population.
This preserves the fixed rollout count while retaining every deterministic
probe and random sample exactly once. New status fields expose total samples,
active group count, and minimum samples per group. The next CUDA simulation
must use 1,024 total samples, giving 128 candidates per group when all three
leases are active.
