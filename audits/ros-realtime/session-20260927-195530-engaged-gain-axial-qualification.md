# Session 20260927_195530 engaged-gain axial qualification

## Decision

**PASS for the near, no-rotation two-axis sparse block.** This result does not
authorize farther targets, rotation, or continuous-path tracking.

Point 1 reached with 0.691 mm recorded terminal feedback error. Point 2 reached
with 1.146 mm. Both are below the configured 1.8 mm gate.

## Observed control sequence

### Point 1

- Axis-0 take-up command began at 1790553340.301 with logical
  `[2,0,0,0,0,0]`.
- Response became provisional and the command went to zero at 1790553342.581.
- The zero confirmation/replan barrier was retained until 1790553342.781.
- A fresh post-engagement MPPI command then executed.

### Point 2

- Coupled tendon take-up began at 1790553348.041 with logical
  `[2,0,2,0,0,0]`. This held physical chassis shaft 0 and moved physical
  tendon shaft 2.
- Diagnostics recorded configured physical request
  `[0,0,-2.618] rad/s` and integer-RPM realized response
  `[0,0,-10.577] rad/s`.
- Observed response entered confirmation hold at 1790553348.222.
- A zero replan barrier remained until 1790553348.441, when a fresh MPPI plan
  began. The pre-take-up plan was not resumed.

This confirms mutual exclusion between take-up compensation and MPPI execution
for the tested branches.

## Estimator and gain belief

During both armed intervals the estimator remained `TRACKING`, with maximum
consecutive rejection count zero. The only `DEGRADED` state was one
`observation_before_rewind_buffer` sample at controller startup, about 8.7 s
before point 1 was armed.

At the end of point 2, the engaged tendon negative-direction posterior was:

- mean 0.9798;
- 90% credible interval `[0.9058, 1.0600]`;
- status `CONFIDENT`;
- 12 accepted updates.

The belief continued receiving accepted settling/static updates after disarm
and ended at mean 0.8839 with 95 updates and `LEARNING`. This is not a failure
of the completed targets, but the next two-axis audit must check whether
post-action updates remain informative rather than allowing static correction
noise to drift the carried belief.

## Safety and timing

- Planned VEL rotation: exactly zero.
- Manager-forwarded VEL rotation: exactly zero.
- POS rotation feedback: exactly zero.
- ENC rotation feedback: exactly zero.
- The apparent `25` on `/teleop/control` occurred only in guarded POS home
  messages, where `joint_vel` carries per-axis position-transaction speed
  limits; it was not a rotation velocity command.
- Planner elapsed time over 23 distinct plans: p50 44.60 ms, p95 54.89 ms,
  p99 55.87 ms, maximum 56.09 ms against the 60 ms deadline.
- Consecutive deadline misses: zero.
- No controller, manager, or estimator fault was observed.

The 3.91 ms maximum deadline margin is materially smaller than the isolated
CUDA preflight margin, so full-stack deadline timing remains a gate in the
next block.

## Next gate

Run only `two_axis_points_v175_hardware_no_rotation.yaml`. Stop at the first
failed point, any nonzero rotation, deadline miss, estimator degradation while
armed, unconfirmed take-up, or gain interval that widens while nominal engaged
excitation is informative. Audit post-disarm gain updates separately before
promoting the belief across longer idle periods.
