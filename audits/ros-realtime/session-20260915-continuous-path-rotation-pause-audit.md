# Continuous-path rotation/pause audit (2026-09-15)

## Scope

This is a passive, non-actuating audit of the hardware continuous-path runs:

- `20260914_203226_mppi_demo`;
- `20260915_110326_mppi_demo`;
- the readable portion of `20260915_111843_mppi_demo`.

The last session had no finalized `metadata.yaml` at analysis time, so it is
corroborating evidence rather than a complete experiment artifact. The audit
aligns `path_tracking_trace`, `reference_horizon`, `planned_control`,
`predicted_tip`, the controller diagnostics, the command heartbeat, and
predicate-69/80 device feedback by recorded time. No hardware commands were
issued.

## Outcome

The reference governor is behaving as designed. It freezes progress when the
reference error reaches 5 mm, but the controller then fails to reduce the
frozen error because its rotation prediction assumes immediately transmitted
motion while the real catheter exhibits delayed torsional transmission,
wrong-way release, and stick-slip.

Two runs repeat the failure at 3.74--3.91% path progress; one run reaches
8.87% before the same pause. The initial path tangent is almost invariant, but
the measured tip can jump across the reference after a long interval of
little response. The Cartesian error therefore reverses even though the path
direction does not. MPPI reverses the rotation command in response. The
feedforward compensator then commands full-speed take-up in the new direction,
which reloads the torsional transmission and can initially move the distal tip
in the old direction.

## First-pause alignment

Logical inputs are ordered `[insertion_mm_s, rotation_deg_s, bend_mm_s]`.
Predicted and measured displacements use a 160 ms interval, equal to the
deployed four-step, 40 ms rollout horizon. The measured interval is an aligned
observation, not a clean open-loop impulse response, because the receding
controller can update within it.

| Session | Pause progress | Frozen reference minus tip (mm) | Path tangent | Desired first input | Compensated input | Model terminal displacement (mm) | Measured displacement (mm) | Rotation shaft response |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| `20260914_203226` | 3.91% | `[+0.64,+4.85,-1.44]` | `[+0.837,-0.548,0]` | `[+2.18,+23.42,0]` | approximately `[0,+40,0]` | `[+0.71,+3.03,-1.84]` | `[+0.40,-0.64,+0.45]` | `+5.72 deg` |
| `20260915_110326` | 8.87% | `[+1.75,+3.48,-3.48]` | `[+0.782,-0.623,0]` | `[0,+9.95,0]` | approximately `[0,+40,0]` | `[+1.91,+3.25,-4.02]` | near zero on the aligned interval | approximately `+6 deg` |
| `20260915_111843` | 3.74% | `[+0.65,+4.71,-1.69]` | `[+0.853,-0.523,0]` | approximately `[0,+24.04,0]` | `[0,+40,0]` | approximately `[+0.28,+2.71,-1.55]` | approximately `[+0.06,+0.05,+0.02]` | `+5.21 deg` |

In the first run, the model displacement has cosine `+0.97` with the target
error while the observed displacement has cosine `-0.78`. In the latest run,
the shaft executes the commanded reversal but the tip response over the same
horizon is only about 3% of the predicted norm.

The device-side rotation response is therefore present. The dominant mismatch
lies downstream of the measured motor shaft, between motor rotation and
material-interface/distal motion.

## Repeated causal sequence near 3.8% progress

The two repeat runs show the same sequence:

1. The approach tangent stays fixed to within 0.06 degrees over the second
   before the pause. It points approximately in base `+x,-y`, with zero `z`
   component. At this progress the robot is still on the straight approach to
   the circle, not traversing the circle.
2. MPPI commands negative rotation while the `y` error is negative. The shaft
   follows, while the tip response is initially much smaller than predicted.
3. A delayed distal displacement drives the tip farther in `-y`; the `y`
   error crosses positive. In `20260915_111843`, for example, the target error
   changes from approximately `[+2.33,-0.39,-0.40]` mm to
   `[+2.02,+0.62,-0.54]` mm.
4. MPPI consequently changes the first rotation command from negative to
   positive. Backlash feedforward raises the physical command to `+40 deg/s`.
5. The motor shaft reverses immediately, but the tip initially continues in
   the old direction. One aligned interval in the latest run moved the shaft
   `+6.64 deg` while the tip moved approximately
   `[+0.17,-0.40,+0.57]` mm. The model predicted positive `y` and negative `z`.
6. The governor reaches its 5 mm reference-error threshold and pauses. The
   frozen target is still supplied to MPPI, so the pause does not disable
   control.
7. Subsequent delayed releases and stochastic first actions repeat the loading
   and unloading cycle instead of converging.

This explains why a geometrically consistent path direction can coexist with
rotation reversals: the controller reacts to the evolving **tip-to-reference
error**, not just the path tangent. Stick-slip makes that error change after a
delay. Later in the pause, first-step rotation can also reverse while the `y`
error retains one sign, as described below.

## Model/plant mismatch during the pause

For rotation-only, sign-consistent 160 ms windows within the first five seconds
after pausing:

| Session | Median model/target cosine | Median measured/target cosine | Wrong-way measured windows | Median measured/model displacement norm |
|---|---:|---:|---:|---:|
| `20260914_203226` | 0.985 | 0.124 | 41.2% | 0.028 |
| `20260915_110326` | 0.996 | 0.021 | 50.0% | 0.017 |
| `20260915_111843` | 0.992 | 0.087 | 44.8% | 0.027 |

The model is internally confident in the desired direction, whereas the
short-horizon hardware response is nearly uncorrelated with that direction and
roughly 37--59 times smaller in median norm. These ratios include measurement
noise and should not be interpreted as calibrated gains, but their magnitude
and repeated wrong-way signs rule out marker noise as the sole explanation.

Over the complete actions, planned rotation reversed 84 times in 117.5 s, 88
times in 179.9 s, and 14 times in the readable 57.9 s run. Corresponding shaft
travel was approximately 479, 650, and 217 degrees, despite net changes of only
-0.8, -22.9, and -25.6 degrees. This is repeated loading/unloading, not useful
monotonic rotation.

## Why the present controller permits the oscillation

### The planner horizon is shorter than rotation take-up

All three bags record `samples=32`, `horizon_steps=4`,
`rollout_step_s=0.04`, `mppi_noise_std=[4,20,2]`, and CPU execution. They did
not use the 1,024-sample CUDA profile.

The directional rotation widths are about 7.02 and 7.09 motor-shaft radians.
At the compensated `40 deg/s` logical rotation command, firmware quantization
corresponds to approximately 9.32 shaft rad/s, so a full reversal take-up is
about 0.75--0.76 s. The rollout sees only 0.16 s. In the latest pause the
estimated rotation remainder was 4.485 rad, still about 0.48 s at full take-up
speed—three complete planning horizons.

### Backlash is outside the model being optimized

With feedforward compensation enabled, rollout backlash is configured as zero
to avoid double counting. MPPI therefore optimizes a desired post-engagement
logical velocity and predicts that velocity as immediately effective. Only
after planning does the compensator replace it with a take-up command. This is
not double counting, but it is a causal disconnect: the rollout does not
propagate the estimator's current `remaining_rad`, engaged direction, or
torsional loading state.

The current play-state estimator can represent a dead travel budget. It cannot
represent elastic torsional windup, direction-dependent delayed release, or
stick-slip after it declares `ENGAGED`. In `20260914_203226`, rotation was
already labeled `ENGAGED` at the pause even though the aligned tip response was
opposite the model prediction.

### First-step reversal is almost unpenalized

The planner includes a reversal term only between future horizon steps. A
change from the previously executed rotation to the new first action receives
only the generic normalized slew cost. With the fixed defaults, that cost is
orders of magnitude smaller than squared millimetre tip error. Every sample
batch also reserves both positive and negative rotation basis probes, and the
rotation noise standard deviation is 20 deg/s.

After the gross `y` overshoot is corrected, several first-action signs have
nearly equivalent predicted costs because the remaining `x/z` error dominates
and only the weighted terminal prediction is recorded. With 32 stochastic
samples, the selected first sign can alternate even while the `y` error keeps
one sign. The bag does not record candidate costs or the complete weighted
control sequence, so this last selection mechanism is strongly supported by
source inspection but is not directly observable candidate-by-candidate.

### Critical model validation data are suppressed

Every bag has zero `MppiResponseTrace` messages. Forecast registration is
skipped whenever the backlash compensator changes the command, precisely the
intervals needed to distinguish desired post-engagement motion from physical
take-up motion. The recorded `predicted_tip` is the rollout for the desired
logical sequence, not a prediction of the compensated shaft command through a
torsional plant.

## Disposition

Continuous hardware path tracking is not qualified. Raising the governor pause
threshold or forcing progress would hide the divergence and increase the
excursion.

Before another circle trial:

1. Put the persistent transmission state inside the rollout boundary. A
   candidate must propagate current direction, remaining take-up, and at least
   a rotation lag/windup state; the optimized prediction must correspond to
   the command ultimately sent to the manager.
2. Add an explicit first-step direction-change cost or hysteretic direction
   latch. Permit a reversal only after a sustained, model-predicted benefit
   exceeds the cost of reloading the measured rotation gap.
3. Give take-up a multi-rate or macro-action representation. Extending every
   expensive distal rollout beyond 0.75 s is unnecessary if the planner can
   represent a bounded take-up phase followed by the short distal horizon.
4. Record both desired and compensated full control sequences, candidate sign
   statistics, the current transmission state used by each rollout, and
   predictions for the actual compensated command. Do not suppress response
   traces during compensation.
5. Validate first with an interior, rotation-dominant target that requires one
   monotonic direction. Require predicted-versus-observed response agreement
   after engagement before attempting the continuous circle.

The 1,024-sample GPU route may reduce stochastic first-action variability, but
it does not fix the missing torsional state. The recorded hardware trials used
the 32-sample CPU route, and the previously observed shared-GPU estimator
starvation remains a separate qualification issue.

## Implemented remediation (2026-09-15)

The first bounded correction is implemented in the ROS controller and enabled
by `config/causal_v2_fixed_hardware.yaml` while hardware output remains off in
that profile:

- every transmission-aware MPPI candidate propagates the measured direction,
  directional gap, and take-up phase, including reversals from `ENGAGED`;
- model rollouts receive predicted transmitted shaft velocity, while the
  candidate-local physical velocity uses the same bounded feedforward rate as
  the executable command;
- rotation samples are direction-latched throughout measured `TAKEUP`;
- engagement requires three consistent interface responses rather than gap
  exhaustion, and excessive unconfirmed travel fails closed;
- the previous-command to first-action rotation reversal has an explicit
  cost; and
- status and response diagnostics distinguish transmission-aware prediction,
  direction latch, confirmation count, and compensated versus transmitted
  sequences.

This corrects the discrete play/takeup mismatch demonstrated by the bags. It
does not claim to identify continuous elastic windup or friction state. The
next qualification must therefore be simulation followed by an interior,
single-direction, rotation-dominant hardware target before another circle.
