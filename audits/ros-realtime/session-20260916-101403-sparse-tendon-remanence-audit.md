# Sparse target tendon-remanence audit — 20260916_101403

## Scope and evidence

This is a passive analysis of the completed simulation bag:

```text
/media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions/
20260916_101403_mppi_sim/20260916_101403_mppi_sim_0.db3
```

No ROS process, simulator state, controller parameter, or hardware state was
changed. Evidence came from fixed targets, trajectory-action feedback,
ground-truth tip, projected and realized control, raw POS/ENC feedback,
simulator transmitted state, MPPI response traces, and controller status.

## Result

Target 1 first diverged, then partially recovered. Its target error started at
12.690 mm, peaked at 31.284 mm 6.20 s into the action, and ended at 9.446 mm
when the 10.06 s budget expired. Physical tendon shaft 2 executed 657 nonzero
samples: 329 in the initial direction and 328 after exactly one reversal.
Thus the latest direction-sign correction prevented rapid reversal chatter,
but the first unconfirmed take-up transaction still produced a very large
plant excursion.

Targets 2--8 had **no tendon actuation** during their target windows:

- projected logical bend/tendon command: exactly zero;
- realized logical bend/tendon command: exactly zero;
- raw tendon encoder change: zero;
- transmitted tendon-state change: zero.

Their failure to straighten was therefore not caused by continued tendon
commands. They inherited the transmitted tendon state left by target 1.

## Target 1 sequence

At action start, the tip and target were:

```text
tip:          [23.601, 17.813, 60.976] mm
target:       [33.601, 10.000, 60.976] mm
target - tip: [+10.000, -7.813, 0.000] mm
```

The physical shaft-2 transaction first ran negative. The estimator's nominal
remaining gap reached zero, but the response observer failed to promote the
direction to `PROVISIONAL` or `ENGAGED`. This was a false negative, not an
absence of physical response. Between 1.41 and 4.52 s, the corrected interface
pose moved approximately `[+3.12,+2.10,+9.16]` mm, and its incremental SE(3)
motion aligned with the shaft-2 Jacobian column at approximately unit cosine.

The false negative came from the joint response window. Shaft 1 was still
`PROVISIONAL` with a residual raw movement of 120 encoder counts, or about
0.094 rad. That exceeded `minimum_motor_increment_rad=0.01`, so shaft 1 was
classified as moving, but remained just below
`minimum_transmitted_increment_rad=0.10`. The accumulation condition returns
early when *any* moving pending shaft is below that threshold. It therefore
zeroed response evidence for every shaft and retained the same global
pose/motor baseline. Shaft 2's increasingly large, well-aligned response was
never passed to joint fitting; the normalized unexplained residual grew from
0.049 to 16.778.

Because the transaction arbiter treats only an observation-confirmed
direction as ready, it continued commanding the fixed take-up rate. It
eventually saturated/replanned after roughly 3.3 additional seconds of
physically engaged but estimator-unconfirmed motion.

The next transaction reversed physical shaft 2. The reversal reset the shared
response baseline and removed the stale shaft-1 residual. Shaft-2 response
evidence then jumped to 1.0 near 6.12 s, the phase became `PROVISIONAL` near
6.13 s, and the tip began returning toward the target. The local
post-engagement model was reasonably consistent with the measured response:
for 33 response traces, endpoint error was 0.276 mm median and 1.028 mm P95;
direction cosine was 1.000 median. The dominant target-1 failure therefore
occurred during a response-observer false negative in the take-up macro,
outside ordinary post-engagement MPPI execution.

Selected target-1 states were:

| Time in action | Tip (mm) | Target error (mm) |
|---:|---|---:|
| 0.04 s | `[23.601, 17.813, 60.976]` | 12.690 |
| 6.20 s | `[37.501, 28.031, 35.710]` | 31.284 |
| 10.06 s | `[42.419, 12.206, 58.406]` | 9.446 |

During this target, raw tendon ENC ranged from 0 to -99,574 counts and ended
at -617 counts. The simulator's transmitted tendon state ranged from 0 to
-95,007 counts and ended at -40,432 counts. Returning the measured encoder
near zero therefore did not return the transmitted tendon state or shape to
the original home.

## Later targets

The sparse experiment declared home using only measured logical position.
After target 1 it accepted approximately 0.092 bend position / -617 raw tendon
counts, while the hidden transmitted tendon state remained -40,432 counts.
That transmitted value then stayed constant throughout targets 2--8.

The resulting target-start tips demonstrate that the experiment did not reset
the plant to one common initial condition:

| Target | Start tip (mm) | Raw tendon ENC | Transmitted tendon state |
|---:|---|---:|---:|
| 1 | `[23.601, 17.813, 60.976]` | 0 | 0 |
| 2 | `[32.978, 27.875, 49.163]` | -617 | -40,432 |
| 3 | `[32.959, 28.291, 48.427]` | -617 | -40,432 |
| 4 | `[32.779, 28.534, 48.026]` | -617 | -40,432 |
| 5 | `[32.344, 28.841, 47.813]` | -617 | -40,432 |
| 6 | `[32.042, 29.066, 47.522]` | -617 | -40,432 |
| 7 | `[31.161, 29.264, 47.750]` | -617 | -40,432 |
| 8 | `[30.791, 29.476, 47.518]` | -617 | -40,432 |

The slight evolution of the later start tips came from the other two axes;
the tendon channel remained fixed. The later points are consequently tests
from one remanent bent state, not independent tests from `[20, 0, 0]` home.

## Source mechanism

`TakeupTransactionArbiter` sends a fixed physical take-up rate to every
pending shaft and keeps it pending until the backlash observer reports the
requested direction as `PROVISIONAL` or `ENGAGED`. Exhausting nominal
`remaining_rad` alone does not make the shaft ready.

The observer accumulates one shared motor/pose window. Its current threshold
gate waits until every shaft that is both moving and `TAKEUP`/`PROVISIONAL`
has at least 0.10 rad in that shared window. A stopped provisional shaft with
a residual between the 0.01-rad moving threshold and 0.10-rad response
threshold can therefore block evaluation of a different shaft indefinitely.
That exact condition occurred here.

The sparse experiment's `home()` routine checks only whether reported logical
position is within `home_tolerance`. The simulated actuator separately
integrates raw motor angle and transmitted motor angle. A position transaction
can therefore restore the reported encoder coordinate while the simulated
transmission retains tendon history.

## Conclusions and next gate

1. Target 1 did not fail because of rapid MPPI reversal chatter. It suffered
   one large physically engaged excursion that the response observer failed
   to confirm, then one reversal and partial recovery.
2. Tendon actuation was exactly zero for targets 2--8.
3. Tendon remanence from target 1 remained in simulator truth, so encoder-only
   homing did not create independent sparse trials.
4. This bag cannot determine whether targets 2--8 are reachable from the true
   common home condition.

Before using the sparse experiment as a controller sanity gate, add a
simulation-only full-plant reset between points, including transmitted motor
state and perturbation history. Keep hardware semantics separate: physical
hardware cannot reset hidden transmission state in software, so a hardware
home must be confirmed by observed shape/response rather than encoder position
alone.

Correct the shared-window gate first: a stopped shaft's sub-threshold residual
must not prevent fitting other shafts that have accumulated enough movement.
Freeze or clear inactive per-axis residuals, or evaluate the observable subset
per shaft. Add a regression with shaft 1 `PROVISIONAL` at 0.094 rad residual
and a strongly responding shaft 2; shaft 2 must become `PROVISIONAL` on its
first strongly attributed response.

Also bound travel after nominal take-up exhaustion. If engagement remains
unconfirmed, enter a zero or low-speed confirmation phase with a hard
additional-travel budget; do not continue a full fixed take-up rate until a
joint limit or large maximum-width fault intervenes.

## Implemented follow-up

The response-window correction is now implemented. Readiness is evaluated per
axis using fresh source-matched encoder motion, so an inactive residual below
the transmitted-response floor is excluded rather than blocking another
ready shaft. The regression reproduces a stopped 0.09-rad shaft-1 residue and
verifies that a responding shaft 2 is confirmed.

For tendon engagement, the observer now projects the marker-corrected distal
strain onto the normalized v171 gauge-fixed bending mode. A projected change
of 0.05 is sufficient primary evidence for shaft 2 even with negligible
interface-pose motion. Analysis of this session's recorded posterior placed
ordinary stationary per-frame variation around 0.02 or below at P95, while
post-engagement increments were materially larger. Distal-only evidence can
change engagement phase but cannot update the metric backlash-width prior.
New trace/status fields expose the increment, evidence, and confirmation bit.

Verification: `control_interface` and `catheter_control` build successfully;
all 221 `catheter_control` source tests pass; touched Python files pass
`ament_flake8`. Repository-wide `control_interface` lint remains red on
pre-existing formatting/docstring issues and offline XML schema retrieval,
none in the changed message definition.
