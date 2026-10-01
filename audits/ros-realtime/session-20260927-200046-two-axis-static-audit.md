# Session 20260927_200046 Two-Axis Static Audit

## Outcome

The hardware session did not primarily fail because of estimator degradation,
planner deadlines, or command transport. Points 2 and 4 were held at zero by
an unbounded `CONFIRMATION_HOLD`. Point 3 lost substantial time to a repeated
coupled take-up saturation/replan loop.

The recorded bag is:

`/media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions/20260927_200046_mppi_demo/20260927_200046_mppi_demo_0.db3`

All observations below were made read-only from that bag.

## Point-by-point evidence

| Point | Result | Dominant controller behavior | Evidence |
|---|---:|---|---|
| 1 | reached, 0.764 mm | normal engaged MPPI execution | 11/11 recorded plans nonzero; estimator `TRACKING`; zero deadline misses |
| 2 | timed out, 5.819 mm | tendon reversal entered permanent confirmation hold | `CONFIRMATION_HOLD` from 0.625 s through the 15.025 s armed endpoint; tendon phase remained `PROVISIONAL`; confirmation count stopped at 2/3 |
| 3 | timed out | repeated saturation/replan followed by several take-up transactions | 31 status samples reported `replanning_after_takeup_saturation`; 56 reported `takeup_active`; only 50 reported `post_takeup_command_ready` |
| 4 | timed out, 8.014 mm | multi-axis transaction reduced to a permanent tendon confirmation hold | transaction started at 0.247 s; axis 0 confirmed, but axis 2 remained `PROVISIONAL` at 2/3 from 1.146 s through the 15.047 s armed endpoint |

Across all points the estimator remained `TRACKING`, and
`consecutive_deadline_misses` remained zero. Thus the static periods were
deliberate zero commands from the controller state machine, not missing
heartbeats or an overloaded planner.

## Root cause 1: provisional confirmation compares against the wrong anchor

The hardware profile requires three engagement confirmations. After the first
credible response, the estimator stores `engagement_anchor_motor` and the
arbiter stops take-up motion. The persistence path currently increments the
confirmation count only when

```text
abs(current_motor - engagement_anchor_motor) < minimum_motor_increment_rad
```

The threshold is 0.01 rad. This is not a stationarity test. Any ordinary
motor settling or coast after the first detected response permanently moves
the encoder away from the first-response anchor. Once the held motor rests at
that new value, it is stationary but can never satisfy the condition again.
The regular response-confirmation path also cannot help because the hold is
commanding zero and therefore supplies no new causal motor increment.

That exact failure is visible twice:

- Point 2: tendon confirmation progressed 0 -> 1 at 0.625 s and 1 -> 2 at
  0.826 s, then remained 2 until timeout.
- Point 4: tendon confirmation progressed 0 -> 1 at 1.046 s and 1 -> 2 at
  1.146 s, then remained 2 until timeout.

`TakeupTransactionArbiter.advance()` has no bounded timeout or alternate exit
from `CONFIRMATION_HOLD`, so the controller publishes zero indefinitely until
the enclosing point action times out.

### Required correction

Preserve the first-response anchor as the estimated engagement boundary, but
do not use distance from that anchor to test stationarity. Confirmation hold
should instead use consecutive accepted observations whose *inter-frame*
encoder displacement is below the motor-motion threshold (and whose visual
updates remain accepted). A bounded hold timeout must fail closed with an
explicit reason rather than silently command zero for the rest of the target.

## Root cause 2: coupled saturation is forgotten by an isolated-axis check

During the first approximately 6.2 s of point 3, the transaction repeatedly
requested this physical take-up vector:

```text
requested motor rad/s = [+1.885, 0, -2.618]
```

Final logical-coordinate projection produced:

```text
realized motor rad/s  = [0, 0, -10.577]
saturated mask        = [1, 0, 0]
```

The arbiter correctly rejected that atomic transaction and requested a fresh
plan. However, `_refresh_blocked_motor_directions_locked()` tests the blocked
axis by itself. Axis 0 is feasible in isolation, so the block is immediately
cleared even though the original *joint direction combination* remains
infeasible. MPPI can then reproduce the same coupled transaction, causing the
observed replan loop and repeated zero barriers.

### Required correction

Saturation memory must represent the infeasible pending direction vector (or
mode), not only independent per-axis signs. It must remain active until either
feedback position changes enough to make the full vector feasible or the
planner selects a different mode. The planner/arbiter should test the same
coupled pending set that will be executed through final projection.

## Recommended verification gates

1. Unit-test confirmation with a first credible response followed by a small
   one-time encoder coast greater than 0.01 rad and then stationary accepted
   observations. It must reach `ENGAGED` without further actuation.
2. Unit-test a confirmation hold with no accepted observations. It must leave
   the hold through a bounded fail-closed outcome, not remain there forever.
3. Unit-test a direction pair that is feasible axis-by-axis but infeasible as
   a coupled transaction. The coupled block must survive replanning.
4. Re-run the same no-rotation sparse-point profile in simulation with injected
   post-response encoder coast, then on hardware only after the above tests and
   normal build/preflight pass.



## Implemented remediation (2026-09-27)

The two observed defects were corrected without weakening manager, firmware,
freshness, limit, or estimator-health gates.

1. Provisional persistence now uses consecutive accepted inter-frame encoder
   displacement. The original first-response motor position remains unchanged
   as the learned engagement boundary.
2. `CONFIRMATION_HOLD` has an explicit 1.0 s fail-closed timeout exposed as
   `takeup_confirmation_hold_timeout_s`. Timeout latches the controller fault
   reason `takeup_confirmation_timeout`; it never resumes MPPI without confirmed
   engagement.
3. Saturation state now records the complete physical direction vector. It is
   cleared only when that same coupled vector becomes feasible.
4. MPPI interprets a multi-axis blocked vector as one coupled mode: the exact
   combination is excluded while independently feasible alternatives remain
   available.

### Verification

- Focused backlash/MPPI tests: **89 passed**.
- Complete `src/catheter_control/test` suite: **281 passed**.
- Python and launch syntax compilation: passed.
- `colcon build --packages-select catheter_control --symlink-install`: passed.
- Installed `control.launch.py --show-args` and `simulation.launch.py
  --show-args`: both expose `takeup_confirmation_hold_timeout_s` with default
  `1.0`.
- The ROS `colcon test` target for `catheter_control` currently registers zero
  tests; the tests above were therefore run directly with pytest. The global
  `colcon test-result` also reports pre-existing failures from the unrelated
  dirty `control_interface` package and is not evidence against this change.
