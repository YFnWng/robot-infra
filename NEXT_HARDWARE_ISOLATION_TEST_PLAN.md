# Next hardware isolation tests

Date: 2026-09-19

## Objective

Determine which remaining boundary causes the rotation-disabled tracking
failures:

1. insertion response magnitude;
2. tendon response, relaxation, and remanence;
3. unequal insertion/tendon motor onset and stopping time;
4. the combined two-axis forward transition;
5. MPPI selection and cost, after the plant/model boundary has passed.

The v174 Jacobian remains the hardware baseline. Rotation stays disabled and
online Jacobian adaptation stays disabled throughout these tests. No test
changes encoder zero or bypasses the manager, serial bridge, firmware limits,
freshness gates, watchdog, or fault latch.

## Common initialization

Every following actuating isolation schedule begins with a guarded absolute
position transaction to `[20 mm insertion, 0 deg rotation, 0 mm bend]`. After
reaching that target, the runner releases position mode and requires a second
stable POS/ENC and estimator preflight before entering the first episode. The
recorded `run_start` and causal generator origin therefore describe the
achieved initialized configuration rather than the arbitrary launch position.

Initialization failure aborts without entering an excitation episode. This is
an absolute position move only; encoder zero remains read-only.

## Why this order

The 2026-09-19 axial A/B showed that both Jacobians can reach simple base-Z
targets, and that the v174 insertion direction is correct. It did not isolate
the tendon column because the negative target used both insertion and tendon
actuation. It also did not reset physical tendon history between trials.

The next experiment must therefore separate:

```text
direct motor realization
  -> interface response
    -> distal response
      -> closed-loop planning
```

Do not use MPPI to identify the first three boundaries. A feedback optimizer
can compensate for a poor model and make the identification result ambiguous.

## Phase 0: recording and runtime identity

Implement these prerequisites before further motion:

1. Fix the launch manifest so it records the effective post-profile ROS
   parameters. The present `controller_parameters` and top-level Jacobian
   artifact describe pre-overlay defaults.
2. Continue recording `/parameter_events`; treat it as the runtime-identity
   authority until the manifest fix is verified.
3. Record an episode identifier, intended command time, command axis mask,
   commanded endpoint/speed, starting POS/ENC, starting tip, interface pose,
   distal strain, take-up state, and profile/artifact hashes.
4. Record `/teleop/control`, `/manager/control`, `/device/command_tx`, POS, ENC,
   markers, estimator trace, marker diagnostics, and manager safety status.
5. Use SQLite3 storage, finalize each bag, and run an automated completeness
   check before accepting a session.

## Phase 1: stationary noise and response threshold

At the reviewed encoder position `[20,0,0]`, with the controller disarmed and
motors enabled but stationary, record at least 10 seconds.

Measure distributions for:

- raw marker-tip displacement per camera interval;
- UKF tip, interface translation, interface rotation, and distal-strain
  increments;
- accepted-observation spacing and camera-to-UKF correction latency;
- POS and ENC jitter.

Define a credible physical response as both:

- above the measured 99th-percentile stationary increment; and
- directionally persistent for two accepted camera observations.

Keep the present covariance-normalized evidence as a second gate. The absolute
noise floor prevents covariance tuning from declaring static noise to be
engagement.

## Phase 2: direct single-basis response

Use guarded manager position transactions, not MPPI. Resolve all amplitudes
from the current hard limits and reviewed interior margins at runtime.

### 2A. Insertion basis

- Hold rotation and logical bending fixed.
- Start near insertion 20 mm.
- Execute positive and negative insertion excursions at the existing reliable
  slow and fast speeds.
- Return to the same encoder position between excursions.
- Use an order-balanced sequence rather than all-positive followed by
  all-negative.

This estimates insertion gain, direction, onset delay, stopping delay, and
history dependence without MPPI or tendon-command contamination.

### 2B. Raw tendon-motor basis

Bidirectional tendon testing must start from an interior tendon bias, not the
zero boundary. Use the existing guarded bias near 7.5 mm when limit preflight
accepts it.

Use the firmware coupling deliberately:

- choose equal logical insertion and bending motion to cancel the raw
  insertion-motor request and excite primarily raw shaft 2;
- verify the cancellation from recorded ENC, rather than assuming it from the
  command;
- execute both directions, two speeds, and repeated returns to the bias.

This isolates the tendon shaft's effect on interface pose, distal bending, and
tip motion. It also measures how much distal state remains after returning the
encoder to the bias.

### 2C. Production compensated-bending basis

- Hold logical insertion fixed and command logical bending around the same
  interior bias.
- This excites raw shafts 0 and 2 through the actual firmware compensation.
- Compare its response with the superposition of the measured 2A and 2B
  responses.

If 2A and 2B pass individually but 2C does not match their causal
superposition, the defect lies in coupling/timing rather than either static
Jacobian column alone.

## Phase 3: insertion–tendon timing isolation

Implementation status: **implemented in causal runner v5** as
`schedule:=timing` / `schedule:=phase_3`. The default positive-direction run
uses one 100 Hz publisher, order-balanced conditions, three repetitions, and
the existing guarded initialization and return path. Negative-direction
timing is a separate `timing_direction:=-1` session so preconditioning history
is not mixed within one fit. Each active physical shaft receives one constant
reliable-speed pulse. Flooring and onset scheduling occur in raw shaft
coordinates before a single conversion to logical commands. A pre-publication
manager-projection check aborts to zero if the requested raw mask, magnitude,
or ordering would be changed.

Add a short dedicated episode generator; do not approximate these episodes
with two independent shell publishers. It must generate one monotonic command
sequence and record the actual command forwarded by the manager.

The evaluator must qualify source, manager, and transmitted waveforms before
using their response windows: inactive shafts remain zero, each active shaft
has exactly one pulse, lead/lag error remains within tolerance, integrated raw
travel matches the planned excursion, and encoder polarity is applied in
physical motor coordinates.

Use identical safe endpoints and speeds with these schedules:

1. insertion only;
2. tendon-motor basis only;
3. simultaneous insertion and tendon command;
4. insertion leading tendon by 20, 40, and 80 ms;
5. tendon leading insertion by 20, 40, and 80 ms.

Repeat at slow and fast speed. Use an order-balanced schedule and at least
three repetitions after a common directional precondition. Do not interpret
encoder homing as a reset of tendon remanence.

For each physical shaft measure:

```text
source command stamp
 -> manager-forwarded stamp
 -> device TX
 -> first credible ENC motion
 -> first credible UKF interface/distal response
 -> settled endpoint
```

Primary statistics are median, P95, and bootstrap confidence intervals for:

- command-to-encoder onset;
- shaft-0 versus shaft-2 encoder-onset skew;
- encoder-to-interface response delay;
- endpoint and path residual relative to the sum of single-basis responses;
- excess logical insertion caused by imperfect compensation.

Interpretation:

- a stable encoder-onset skew identifies actuator/firmware timing;
- aligned encoders but delayed interface response identifies transmission or
  mechanical lag;
- aligned interface response but delayed markers identifies perception/UKF
  timing;
- speed- or direction-dependent skew requires a dynamic actuator model, not a
  single fixed delay.

## Phase 4: reachable-plane MPPI isolation

Only after Phases 1--3 pass, run MPPI with v174, rotation disabled, adaptation
disabled, and the existing safety stack.

Generate targets from the controller's own forward model at the measured
run-start state instead of using an arbitrary circle. Freeze the targets for
the block.

Run three profiles:

1. insertion-only MPPI: rotation and logical bending velocity limits zero;
2. bending-only MPPI: rotation and logical insertion limits zero, starting
   from the reviewed interior bend bias;
3. two-axis MPPI: insertion and bending enabled, rotation zero.

For each profile test:

- one positive model-generated direction;
- one negative/relaxation direction when the hard limits allow it;
- two diagonal targets requiring both bases;
- an intentionally unreachable target to verify closest-point stabilization.

Home or return to the reviewed bias before each independent point, but record
the measured tip and hidden-history proxies because physical history is not
reset. Use sparse targets and a single approach direction; continuous path
tracking is outside this isolation scope.

Evaluate:

- final Cartesian error and error projected into/out of the predicted
  two-DOF reachable plane;
- time and motor travel to tolerance;
- reversal count per logical and physical shaft;
- planned, transmitted, and measured response;
- model endpoint error and direction cosine;
- distance to position and raw-encoder limits;
- estimator rejections, deadline misses, and manager interventions.

### 2026-09-20 entry gate after target-correction simulation

The `20260920_163629_mppi_sim` run completed the full continuous circle after
the forward-corridor target correction, but carried approximately 6 mm
closest-path residual through the known unreachable arc. Rotation ranged over
about 116 degrees. This is evidence that the reference deadlock is removed;
it is not evidence that the same geometry is suitable for rotation-disabled
hardware.

The next hardware run remains a sparse-point Phase-4 isolation and must use a
dedicated v175/no-rotation profile:

1. Load frozen v171 distal mechanics, v175 interface transmission, and the
   v174 interface Jacobian; keep adaptation disabled.
2. Set the rotation velocity limit exactly to zero and verify the effective
   runtime parameter and every recorded planned/manager command.
3. Start each independent target with the existing guarded absolute return to
   `[20,0,0]`; do not interpret this as a hidden-state reset.
4. First repeat the already-qualified `+5/-5 mm` base-Z axial pair as a
   regression against `20260919_171058_mppi_demo`.
5. If both axial targets pass, test one insertion-dominant, one tendon-dominant,
   and two diagonal targets generated by the frozen forward model from that
   run's accepted home state. Reject targets whose predicted trajectory uses
   rotation or approaches a joint reserve.
6. Use independent point captures, not the continuous circle. Require final
   error at most 1.8 mm, no rotation command, no hard-limit event, no manager
   intervention, no estimator degradation, and no repeated planner-deadline
   fault. Stop the block at the first failed target.
7. Compare in-plane residual, out-of-plane residual, model endpoint error,
   command reversals, physical shaft travel, and take-up occupancy. Only a
   passing two-axis sparse block authorizes construction of a planar
   continuous path for a later hardware test.

Implementation status (2026-09-20): the first gated axial block is available
as `v175_grouped_hardware_no_rotation.yaml` plus
`axial_points_v175_hardware_no_rotation.yaml`. The sparse runner now verifies
the effective controller diagnostic identity before arming and throughout an
active target, and cancels on any nonzero logical-axis-1 planned or
manager-forwarded velocity command. The model-generated two-axis block remains
deliberately gated on the axial result; it must not be substituted with
arbitrary Cartesian offsets.

Hardware result: `20260920_165501_mppi_demo` passed the axial gate. Both
targets reached with reported errors `0.671/0.637 mm`; planned,
manager-forwarded, POS, and ENC rotation were exactly zero; estimator rejection
and planner deadline-miss counts were zero. The maintained audit is
`audits/ros-realtime/session-20260920-165501-v175-no-rotation-axial-audit.md`.
Proceed to model-generated, reserve-qualified two-axis sparse targets; do not
advance directly to a continuous hardware path.

Implementation status: the disarmed controller-owned preview service and the
two-axis sparse runner configurations are now implemented. Target prediction
uses a cloned accepted recurrent snapshot, current observed tip, fitted v175
interface play, v171 tendon history, the normal hardware projection, and
estimated response-free take-up travel in the endpoint reserve. The four
frozen candidates are insertion-dominant, tendon-dominant, and two coupled
diagonals. Simulation uses a full hidden-state reset between independent
points; hardware preserves physical history. Both variants require exact zero
rotation velocity and independently abort on any nonzero planned or
manager-forwarded rotation command.

Hardware result: `20260920_175149_mppi_demo` passed the four-target
rotation-disabled closed-loop gate with final errors
`0.874/1.232/1.450/1.270 mm`, zero rotation at every recorded command and
feedback boundary, no fault, and no deadline miss. It is not yet a clean
four-direction model-validation block: targets were frozen from the first
home while physical distal history was preserved, and the positive diagonal
was reached without tendon motion. The next isolation revision must generate
each fixed joint-displacement candidate from the accepted estimator/history
snapshot after that candidate's own guarded home. See
`audits/ros-realtime/session-20260920-175149-v175-two-axis-hardware-audit.md`.

Implementation update: model-generated isolation profiles now regenerate one
absolute Cartesian target after each candidate's own guarded home from the
current accepted UKF/history snapshot. The standard profile retains the
qualified joint-displacement set. Separate farther simulation/hardware
profiles add `[8,0,0]`, `[0,0,4.5]`, `[12,0,3]`, and `[-6,0,4.5]` logical
displacements with the same exact-zero rotation guard and endpoint-reserve
service gate. Run standard per-home simulation first, then farther simulation;
only passing simulations authorize their corresponding hardware blocks.

Functional simulation verification (2026-09-20): the standard per-home set
reached all four targets with final errors `0.285/1.279/1.209/1.050 mm`.
An initial farther diagonal `[10,0,4.5]` exposed near-cancellation and timed
out at `2.992 mm`; it was replaced rather than waived. The corrected farther
set `[8,0,0]`, `[0,0,4.5]`, `[12,0,3]`, `[-6,0,4.5]` then reached all four
targets with initial Cartesian errors `6.46/7.31/5.66/11.91 mm` and final
errors `0.098/0.352/1.121/1.194 mm` in a CPU functional run. The existing
1,024-sample CUDA result remains the timing/performance qualification; repeat
the corrected profiles with CUDA when producing recorded comparison data.

Hardware interruption update: the first farther run reached points 1 and 2,
then its atomic point-3 return timed out after the tendon lower-bound event
stopped insertion. The September 19 residual-axis firmware repair was active,
but the resumed insertion segment did not satisfy the endpoint retry gate.
Hardware isolation profiles now use a decoupled tendon prehome: tendon is
returned to zero while physical chassis axis 0 is held fixed, and only then is
the final `[20,0,0]` insertion home issued. Firmware limits and retry rules are
unchanged. See
`audits/ros-realtime/session-20260920-181027-far-two-axis-home-interruption-audit.md`.

## Decision tree

```text
2A fails
  -> insertion units/frame/history/encoder association problem

2A passes, 2B fails
  -> tendon column, remanence, or tendon engagement model problem

2A and 2B pass, 2C fails
  -> firmware coupling or insertion/tendon timing problem

2C passes, Phase 4 single-axis MPPI fails
  -> planner cost/sampling/limit projection problem

single-axis MPPI passes, two-axis MPPI fails
  -> mixed rollout timing/state cloning or mode-selection problem

two-axis sparse points pass, continuous path fails
  -> reference progression/reversal policy problem rather than static model
```

## Safety and abort criteria

Every episode remains fail-closed. Immediately terminate the current episode,
command manager mode `NONE`, and preserve the bag on any of:

- manager not ready or latched fault;
- invalid/stale POS or ENC;
- estimator fault, sustained degraded health, or repeated marker rejection;
- unexpected non-commanded rotation-shaft motion;
- position or model-encoder reserve violation;
- position transaction timeout;
- tip response inconsistent with the commanded basis beyond the reviewed
  maximum excursion;
- planner fault or repeated deadline miss during Phase 4.

Do not automatically continue to the next episode after an abort.

## Recommended implementation order

1. Correct effective-parameter recording and add the session completeness
   checker.
2. Add the stationary and selectable single-basis episode modes to the causal
   experiment runner.
3. Add the timestamped staggered-pair generator and analyzer.
4. Add model-generated reachable-plane target construction and three
   rotation-disabled MPPI profiles.
5. Verify all generators in simulation, including exact limits, episode
   continuity, axis masks, timestamps, and abort paths.
6. Run hardware Phases 1 and 2; audit before authorizing Phase 3.
7. Run Phase 3; update the actuator/transition model only if the measured skew
   is repeatable.
8. Revalidate in simulation, then run Phase 4 on hardware.
