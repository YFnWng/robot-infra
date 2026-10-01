# Session 20260915_183741 MPPI simulation audit

Session:
`/media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions/20260915_183741_mppi_sim`

## Corrected outcome

The run did not encounter the newly added causal saturation boundary. No
status sample entered `SATURATED_REPLAN`, all
`planner_blocked_motor_direction` values were `[0,0,0]`, and every recorded
plan reported zero blocked candidates.

A first-pass transaction-count audit suggested independent-plan chatter. A
follow-up comparison of the actual Cartesian target-minus-tip vector before
each take-up and at the next MPPI solve shows that this is not the primary
mechanism. The take-up/response-confirmation interval usually moved the tip
through the nearby target. MPPI then correctly selected the opposite rotation
direction, starting another take-up transaction and forming a limit cycle.

## Runtime evidence

- Armed path window: 43.401 s, 434 status samples.
- Transaction state duration:
  - `TAKEUP_ACTIVE`: 36.592 s;
  - `REPLAN_REQUIRED`: 2.308 s;
  - `READY_TO_PLAN`: 4.501 s.
- Transaction generations: 50.
- Rotation transaction signs: 29 negative, 19 positive, 2 zero.
- Rotation sign changes between generations: 39.
- Only 68 MPPI plans were published; rotation was nonzero in 54 and reversed
  sign 36 times.
- Path progress: 0.042 mm to 1.480 mm of 76.918 mm.
- Governor states: 1,254 `TRANSMISSION_HOLD`, 44 `RUNNING`.
- The path tangent was effectively constant throughout this interval:
  mean `[0.71967,-0.69432,0]`, maximum angular change 0.00964 degrees.
- Raw rotation encoder total variation was 364,249 counts while net travel was
  -9,487 counts. It reversed increment sign 36 times.
- Estimated transmitted rotation total variation was 28.560 rad while net
  travel was -2.091 rad, with 35 sign changes.
- Closest-path error remained bounded (median 0.224 mm, P95 0.366 mm, maximum
  0.426 mm), but reference progression was almost completely suppressed by
  transmission hold.

### Cartesian error-direction test

For each transaction, the recorded `tip_error_xyz_mm` was compared at the
first `TAKEUP_ACTIVE` status and at the first status belonging to the next
transaction. Among 35 consecutive transactions whose rotation sign reversed:

- 31/35 target-minus-tip vectors had negative dot product;
- median direction cosine was -0.544 (about 123 degrees);
- only 1/35 remained strongly aligned (cosine greater than 0.8).

Examples:

```text
generation 1, rotation - -> +:
  [ 0.422,-0.407, 0.000] mm -> [-0.456, 0.208,-0.143] mm

generation 6, rotation + -> -:
  [-0.428, 0.170, 0.045] mm -> [ 0.185,-0.283,-0.062] mm

generation 12, rotation + -> -:
  [-0.415, 0.152,-0.121] mm -> [ 0.526,-0.510, 0.013] mm
```

The MPPI reversal is therefore usually consistent with a real reversal of the
Cartesian error. It is not merely choosing an opposite motor direction for an
unchanged error vector.

## Source-supported mechanism

During `TAKEUP_ACTIVE`, pending shafts run at their fixed take-up velocity
until several direction-consistent visual observations confirm interface
response. Once the simulated plant's fixed gap has been exhausted, motion is
already transmitted, but the macro continues at the same take-up velocity
during the response-detection and confirmation delay. With sub-millimetre tip
errors, that post-engagement travel is large enough to cross the reference.

The fresh solve then observes the crossed target and legitimately commands a
reversal. That reversal opens a new take-up transaction, whose full-rate
post-engagement confirmation tail crosses the target again. The path governor
correctly freezes progress during every transaction, so the system stays near
one path location instead of advancing.

The controller also calls `planner.reset()` and zeroes
`last_effective_command` after take-up completion. This removes continuity and
can add sampling variance, but the recorded Cartesian error reversal is strong
evidence that it is secondary in this session.

## Finding

### F-039 — High — Full-rate response confirmation drives the tip through nearby targets and creates a reversal limit cycle

**Confidence:** directly observed; source-supported mechanism.

Response observation is necessary because the controller does not know the
true take-up width. However, engagement detection currently doubles as the
only stopping mechanism while the fixed macro continues at full rate. The
resulting confirmation tail is too coarse for sub-millimetre path tracking.

## Required correction

1. Separate gap traversal from engagement confirmation. Use the bounded fast
   rate while no response exists, then immediately transition to zero or a
   much lower probe rate on the first credible response sample.
2. Confirm engagement from subsequent observations while holding, rather than
   accumulating full-rate post-engagement motion. If evidence decays or is
   inconsistent, resume only the bounded low-rate probe.
3. Make the first-response threshold and confirmation threshold explicit and
   hysteretic so marker noise does not chatter between traverse and hold.
4. Keep the path frozen and require a fresh MPPI plan after confirmed
   engagement, as today.
5. Preserve MPPI nominal/previous desired state across normal completion as a
   secondary continuity improvement, but do not use that to suppress a
   reversal when the measured Cartesian error truly crossed the target.
6. Add a deterministic near-target regression: mismatched take-up width,
   three-frame confirmation, and fixed reference must not create repeated
   error-vector crossings.

The causal saturation guard from F-038 remains valid, but this session did not
exercise it and cannot validate that remediation.

## First-response credibility and target semantics

The present response detector is not a raw marker-displacement threshold. It
requires accumulated raw shaft motion, at least 0.1 rad inferred transmitted
motion with the commanded sign, a constrained fit to the local 6x3 interface
Jacobian, direction cosine at least 0.5, and leave-one-column-out evidence at
least 0.5. In this simulation, 99 stationary pre-path estimator traces had
exactly zero evidence and inferred motion on all axes. Across 40 rotation
transactions with a threshold crossing, first evidence was 0.9984 to 1.0 and
the median inferred rotation increment was 0.296 rad. Simulation separation
was therefore strong. This evidence value is a relative model-attribution
score, not a calibrated probability, and existing hardware bags predate these
diagnostic fields; hardware false-positive probability remains unmeasured.

Actuation handoff should consequently be separated from parameter confidence:
the first calibrated credible response should stop full-rate take-up and hand
authority to a fresh MPPI solve, while repeated observations establish a later
`CONFIRMED_ENGAGED` state for width learning and persistent transmission state.

The target should also be a reachable set rather than an exact moving point.
This run's closest-path error remained below 0.426 mm even while point-reference
tracking oscillated. A continuous-path controller should use a cross-track
tube plus monotonic forward progress/lookahead, and should advance the path
reference after along-track overshoot instead of requesting a backlash-causing
reverse correction inside the tube. The tube size must be identified from the
hardware distribution of first-response tip displacement, not selected from
simulation alone.

## Implemented correction (2026-09-15)

The controller now distinguishes `PROVISIONAL` from `ENGAGED`. A first
credible response immediately removes full-rate take-up authority and causes
the existing zero-command/fresh-replan transition. The unused bounded gap is
retained internally so two inconsistent moving observations can return safely
to `TAKEUP`; only repeated consistent evidence clears it and commits width
learning. The continuous path governor now treats `soft_error_mm` as a
capture tube and catches its monotonic reference up to a measured along-path
overshoot bounded by `pause_error_mm`. This prevents a small productive
response from turning an obsolete exact point into an unnecessary reversal.

Validation remains staged: matched-width deterministic simulation,
controller/truth width mismatch in both directions, stationary hardware
shadow false-response measurement, then an explicitly authorized low-speed
hardware path run.
