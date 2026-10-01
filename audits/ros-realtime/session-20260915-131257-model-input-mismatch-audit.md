# Session 20260915_131257 model-input mismatch audit

Status: observed failure; F-033 source remediation implemented, rerun pending.

## What changed relative to the prior run

The F-032 patch was active. Maximum take-up commands ended when geometric
remaining travel reached zero, and the controller resumed the smaller MPPI
commands. The repeated hard-path error was therefore not stale installation or
continued take-up-speed overshoot.

## Failure timeline

- The path began at `(20.40,19.95,55.26)` mm and initially advanced toward
  approximately `(+x,-y,0)`.
- The first plan was rotation-dominant. Before any downstream response, raw
  rotation and later coupled bend/linear shaft motion were integrated directly
  into the controller's v171 state.
- At 0.613 s, controller motor state was
  `[-1.198,-3.168,-1.901]` rad. Simulator transmitted state still showed no
  shaft-0 or shaft-1 motion. Despite this dynamic mismatch, post-fit marker RMS
  was 0.304 mm.
- The commanded motion settled near `[-2.21,-15,2.16]` logical units/s. The
  truth tip moved predominantly `(+x,+y,-z)`, opposite the required y direction
  and with a large undesired z component.
- Progress paused at 1.55 mm; closest-path error reached 15.04 mm after 3.65 s,
  activating the unchanged hard-error abort.

## Correction and replay evidence

The backlash estimator now constructs an estimated transmitted motor angle
from raw shaft increments. Travel inside the directional gap updates physical
memory but does not advance v171. Travel beyond the gap is passed through.
Marker observations remain the independent engagement/confidence evidence.

An offline replay over all 1,758 raw/transmitted pairs reproduced simulator
truth with maximum absolute motor-angle errors of 0.000526, 0.000575, and
0.000699 rad. Subsampling to the controller's effective 50 Hz cadence did not
increase those maxima for this run.

Raw encoder counts remain the input to hard-limit validation and are explicitly
retained in `EstimatorStateTrace`; the learned runtime's virtual motor angle is
recorded separately. No manager, watchdog, command-freshness, or path-safety
gate was weakened.
