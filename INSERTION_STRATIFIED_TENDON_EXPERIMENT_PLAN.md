# Insertion-stratified tendon experiment plan

Date: 2026-09-21

## Question

Test whether the distal tendon response depends on catheter insertion because
more proximal catheter is exposed outside the sheath at the experiment
configuration than in the original identification data.  The specific
hypothesis is:

```text
same physical tendon-shaft motion and history
  + greater exposed proximal length
    -> more proximal bending / tendon travel absorbed proximally
      -> later and smaller distal-curvature response
```

This is a plant-identification experiment, not a controller test.  MPPI stays
disarmed, online Jacobian adaptation stays disabled, and the guarded causal
experiment command path remains the only actuator command source.

## Coordinate definition

Use the firmware coupling explicitly.  For logical catheter coordinates
`q = [q_lin, q_rot, q_bend]`, the relevant physical proximal coordinates are

```text
raw chassis position = q_lin - q_bend
raw tendon/knob position = q_bend
catheter insertion carried by chassis + knob = q_lin
```

The primary experiment changes the insertion plateau and then holds logical
insertion fixed while sweeping logical bending.  Through the production
firmware coupling, the chassis shaft moves oppositely to the knob so catheter
insertion remains approximately fixed.  The measured insertion deviation
caused by unequal motor timing is expected to be small relative to the
0--40 mm insertion span and will be recorded as a covariate rather than used
to disqualify the experiment.

Do not interpret an episode boundary, a return to an encoder position, or the
initial move to `[20,0,0]` as a reset of tendon history.  The entire session is
one continuous mechanical trajectory.

## Primary schedule: `compensated_bend_insertion_sweep`

Add a causal-runner schedule named `compensated_bend_insertion_sweep`, with alias
`phase_2e`.  Its configurable defaults are:

- four insertion plateaus at 0, 13.333, 26.667, and 40 mm;
- tendon bias at 7.5 mm;
- tendon half-excursion 7.5 mm, spanning the full 0--15 mm bend range;
- slow and fast physical tendon speeds of 2 and 4 mm/s;
- two visits to every insertion plateau;
- 3 seconds stationary at each plateau before excitation;
- 2 seconds at each tendon endpoint.

The tendon bias is the midpoint of the commanded tendon/bending coordinate.
The bend axis is one-sided (`0..15 mm` in the current profile), so starting at
`7.5 mm` permits approximately equal loading and relaxing motion without
starting on a hard boundary.

The tendon half-excursion is the distance from that midpoint to either signed
endpoint.  A 7.5 mm half-excursion therefore commands

```text
0.0 mm <- 7.5 mm bias -> 15.0 mm
```

and spans a 15 mm peak-to-peak tendon cycle.  It is a complete experimental
excursion, not a per-control-cycle step.

During the primary compensated sweep, logical insertion is held at the
selected plateau while logical bend traverses 0--15 mm.  The complete planned
logical insertion envelope is therefore exactly `[0,40] mm`, leaving 10 mm
reserve to the exact `[-10,50] mm` insertion limits.

Implementation must therefore use an explicit schedule-specific reserve for
this experiment rather than silently clipping the 0 and 40 mm plateaus.  The
0 and 15 mm bend endpoints are exact hard-boundary commands, so the generator
must use a zero-velocity smooth arrival, retain the configured feedback
tolerance, and reject any measured boundary violation.  The complete logical
and derived raw trajectory must be previewed against the live limits.  If the
live safety configuration does not permit an exact endpoint, the launch must
reject the plan and report the required reserve; it must not silently shrink
the sweep.

### Session sequence

1. Perform the existing guarded initialization to `[20,0,0]` and repeat the
   stationary POS/ENC and estimator preflight.
2. Enter the common tendon bias at `[20,0,7.5]` through the existing
   compensated setup path.
3. Visit the four insertion plateaus in forward and reverse order:

   ```text
   pass 1:  0.000 -> 13.333 -> 26.667 -> 40.000 mm
   pass 2: 40.000 -> 26.667 -> 13.333 ->  0.000 mm
   ```

   The reversed pass balances early/late occurrence while preserving one
   continuous mechanical history.
4. At each plateau:
   - move insertion while holding the tendon bias fixed;
   - dwell for 3 seconds and record the equilibrium posterior;
   - execute one compensated-bending cycle at the speed assigned to that
     visit while holding logical insertion fixed;
   - balance the two visits as one slow and one fast cycle;
   - alternate which signed tendon excursion occurs first between visits.
5. Return to `[20,0,7.5]`, exit the tendon bias to `[20,0,0]`, record a final
   stationary interval, and use the normal guarded return/collection seal.

Each measured cycle follows `7.5 -> 15 -> 7.5 -> 0 -> 7.5 mm`, or the
order-reversed equivalent, and returns to the bias.
Changing the branch order prevents one direction from always inheriting the
same history.  No unrecorded preconditioning motion is permitted.

With the current quintic move profile, the implemented duration is
**413.3125 seconds (6 minutes 53.3 seconds)**:

```text
8 tendon cycles (1 slow + 1 fast at each of 4 levels)     232.8 s
forward/reverse insertion transitions at 2 mm/s            112.5 s
3 s plateau dwell x 8 visits                                24.0 s
enter/exit the 7.5 mm tendon bias                            14.1 s
initial and final stationary intervals                      30.0 s
                                                              -----
total                                                       413.3 s
```

This calculation assumes transition endpoint holds are represented by the
listed plateau dwells rather than added a second time.  The implemented
generator must print its exact duration from the generated segments before an
actuating launch and reject a plan exceeding that value plus a small explicit
runtime allowance.

## Secondary schedule: raw-tendon isolation

Retain a separate optional schedule, `tendon_insertion_sweep`, only if the
primary result later requires single-shaft attribution.  It uses the same
plateaus, order, speeds, amplitudes, and history policy, but commands equal
logical insertion and bending increments `[dq,0,dq]` to hold the physical
chassis shaft fixed.

The secondary schedule changes logical catheter insertion as the knob moves,
so it is not the preferred dataset for training the insertion-conditioned
distal response.  Its purpose is only to distinguish a production coupling
effect from physical tendon/knob transmission if the primary data makes that
distinction necessary.

## Required implementation changes

### Generator and launch

1. Add both schedule names and `phase_2e` alias to
   `experiments/causal_experiment.py`.
2. Add launch arguments for absolute insertion plateaus, plateau dwell, and
   visit count.  Do not overload the existing insertion half-amplitude.
3. Add a dedicated episode builder that labels every transition, dwell, and
   measured branch with:
   - insertion plateau;
   - raw or compensated excitation basis;
   - speed tier;
   - pass/repetition;
   - signed branch order.
4. Preserve fixed logical insertion during primary excitation.  Record the
   actual transient insertion error produced by unequal chassis/tendon timing
   without post-hoc retiming or smoothing.
5. Validate every planned logical waypoint and the derived raw coordinates
   before enabling motion.  Reject rather than clip a plateau or tendon
   excursion.
6. Include the insertion schedule and all resolved endpoints in the session
   manifest and runtime identity.

### Tests

Add generator tests that prove:

- all episode boundaries are position/velocity continuous;
- the primary compensated excitation holds logical insertion fixed;
- the optional raw-tendon schedule has zero raw-chassis velocity;
- every insertion level occurs twice with balanced temporal order and
  speed/branch order;
- no hidden reset is inserted between episodes;
- every logical and derived raw endpoint respects its reserve;
- dry-run metadata and episode labels reconstruct the complete schedule.

Run the experiment package unit tests and a model-in-loop dry run before generating
hardware commands.

## Recorded signals

Retain the existing causal trace, POS/ENC, manager-forwarded commands, device
TX, marker observations, marker diagnostics, estimator trace, parameter
events, safety state, runtime identity, and episode events.  For this test the
post-run dataset must expose, per accepted estimator frame:

- logical POS and raw ENC;
- derived raw chassis and tendon positions;
- measured UKF interface pose;
- UKF distal strain/curvature and reconstructed distal centerline;
- measured 3-D markers and acceptance diagnostics;
- insertion plateau, speed, direction, pass, and episode-local time;
- v171 one-step and coherent-rollout prediction from the recorded history.

The measured UKF distal strain is the main response variable.  Marker-space
error remains the independent check that the posterior curvature estimate is
supported by observation.

## Analysis and acceptance criteria

First qualify encoder realization and the offline camera reconstruction. For
the SVO-first acquisition, online marker rejection is diagnostic and does not
abort motion; the recorded images must instead pass the post-run pairing and
full-shape quality audit. Do not fit an insertion effect from a branch with a
timed-out position transaction, stale POS/ENC feedback, failed offline shape
qualification, or an unintended raw-chassis command.

For each insertion level, speed, and direction, report:

- raw tendon travel before the first credible distal-curvature response;
- incremental distal-curvature gain versus raw tendon travel;
- equilibrium curvature change at both endpoints;
- tip displacement and maximum marker-space error;
- return residual/remanence at the common tendon bias;
- v171 onset error, maximum Cartesian error, and reversal-window error;
- between-pass variability and dependence on the preceding plateau.

Use paired contrasts within each continuous pass, then bootstrap by complete
visit rather than by frame.  The insertion-dependence hypothesis is supported
only if the response change is repeatable across pass order and remains after
conditioning on actual raw tendon motion, speed, direction, and starting
distal state.

The result should distinguish three outcomes:

1. **Insertion-dependent distal transmission:** actual tendon motion and the
   small measured insertion deviation are accounted for, but onset travel or
   distal-curvature gain still changes systematically with the insertion
   plateau.
2. **Actuator/coupling effect:** the primary compensated response changes with
   insertion, but a later raw-tendon isolation is invariant across insertion.
3. **History/estimation effect:** differences follow predecessor state or
   marker quality rather than insertion level.

Only outcome 1 justifies adding insertion/exposed-length conditioning to the
distal tendon model.  Outcome 2 belongs in the proximal transmission model;
outcome 3 requires a history or observation-model correction instead.

## Execution gate

Do not run this on hardware until:

- generator/unit tests pass;
- the exact plan and duration are printed in a non-actuating dry run;
- a model-in-loop run completes and its episode labels reconstruct correctly;
- the hardware manager is ready, motors are requalified, the estimator is
  tracking, MPPI is disarmed, adaptation is disabled, and firmware fault
  status is clean;
- an operator remains present for the full actuating session.

The experiment never changes encoder zero and does not bypass the manager,
firmware limits, watchdogs, freshness gates, or fault latch.
