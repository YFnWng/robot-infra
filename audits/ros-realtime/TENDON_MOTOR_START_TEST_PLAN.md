# Tendon-motor start-up characterization: hardware test plan

2026-11-02. **Plan only. No hardware has been run; every hardware step needs
explicit authorization at run time.** It follows from
[GATE3_BATCHED_FORECAST_PLAN.md, section 17](GATE3_BATCHED_FORECAST_PLAN.md#17-where-the-tendon-start-up-delay-comes-from-and-why-it-matters-2026-11-02).

## 1. Question

Raw motor 2 (catheter bend, the tendon) takes 60–80 ms from a velocity
command to 90% of its commanded speed. The insertion motor 0 takes 16–34 ms.
The firmware decouples insertion as motor 0 = insertion − bend and assumes
both motors are at speed. During every tendon start, the executed joint
direction therefore deviates from the plan. The cause decides the remedy:

| Hypothesis | Prediction | Remedy it points to |
| --- | --- | --- |
| H1. Load: tendon tension resists the start | Unloaded starts are much faster | State-dependent start model |
| H2. Driver acceleration limit | Rise time grows with commanded RPM, independent of load | Fixed ramp model, or driver tuning |
| H3. Start-path latency: the full re-initialization (O0, L1, S, C, O1) in `sendMotorCmds` | Speed changes on a running motor are fast; only starts from rest are slow | Firmware start-path change, or a planner that avoids stops |
| H4. History: dwell or preload after a reversal | Rise time depends on dwell time and on the previous direction | Dwell- and history-aware model, or planner policy |
| H5. Coupled starts | Starting both motors adds delay and an insertion overshoot | Lead compensation, or firmware compensation ramp |

The hypotheses can hold together. The tests are designed to separate them.

## 2. What existing recordings already show (offline, done)

Recorded `/device/command_tx` and `/device/state` ENC data. Encoder telemetry
runs at about 11 ms intervals, so values are resolved to about one telemetry
period.

| Source | Motor and condition | n | 90% of speed, median (P90) |
| --- | --- | ---: | --- |
| 2026-09-20 `tendon_motor` | Motor 2, 101 RPM, negative direction, alone | 32 | 75 ms (153) |
| 2026-09-20 `tendon_motor` | Motor 2, 101 RPM, positive direction, alone | 28 | 71 ms (94) |
| 2026-09-20 `tendon_motor` | Motor 2, 101 RPM, insertion motor also starting | 13 | 88–91 ms |
| 2026-09-20 `tendon_motor` | Motor 2, same direction as previous motion | 54 | 70 ms (89) |
| 2026-09-20 `tendon_motor` | Motor 2, after a reversal | 6 | 106 ms (109) |
| 2026-10-08 control session | Motor 2, 101 RPM | 10 | 78 ms |
| 2026-09-20 `insertion_rotation` and 2026-10-08 | Motor 0, 36–178 RPM | 9 | 16–34 ms |

**Consequences for the test design.**
- Direction is not the explanation. Skip any direction-only test.
- Every recorded tendon start was at 101 RPM, so H2 is untested.
- Speed changes on a running motor, H3, are absent from the data.
- After-reversal starts are suggestive but too few (n = 6 + 2).
- No unloaded data exist.
- The 2026-09-19 Phase 3 timing run was rejected (archived audit
  `session-20260919-194147-causal-phase3-timing-audit.md`). The corrected v5
  timing schedule was verified offline but **never run on hardware**.

## 3. Preconditions (operator, before any session)

1. **Authorization.** The user explicitly authorizes each session below and
   confirms an operator is present at the emergency stop.
2. **Hardware and zero state.** Confirm the encoder reference is the normal
   training zero after the 2026-10-14 chassis-mark recovery
   (`firmware/ENCODER_RECOVERY_20261014.md`). Confirm the flashed firmware is
   the normal control image, not the recovery seed image, which rejects
   velocity. Check the runtime identity, firmware identity and limits profile
   (`imricor_test`). **Never `SET_ZERO`.**
3. **Software.** Rebuild `robot-infra` and source the overlay; record installed
   entry points and artifact hashes in the session. The planner change in
   `mppi.py` is uncommitted and not used by these tests.
4. **Stack.**
   - The manager, serial bridge, marker tracking and the **disarmed**,
     adaptation-disabled `catheter_mppi` estimator run, as required by the
     causal launch.
   - MPPI command output stays disabled.
   - Rotation velocity limit 0.
   - Task GC enabled per `robot-infra/AGENTS.md`.
   - Native position streaming disabled: it is not qualified.
5. **Dry run.** Run each schedule first with `use_sim:=true` and with
   `start_motor:=false` on hardware. Inspect the generated plan and its
   raw-shaft topology checks.
6. **Stop criteria, any one ends the session.**
   - A manager or firmware fault, a latched motion monitor, a watchdog expiry
     or an encoder-integrity latch.
   - A `MOTION_SUSPECTED:STALL` that does not recover.
   - A tendon position outside 0.5–14.5 mm, or insertion outside the
     schedule's margins.
   - Any unexpected operator observation.

   Do not retry inside the session; stop and review.

## 4. Sessions (revised 2026-11-02: one compact schedule, run loaded and unloaded)

The original Sessions B–D took about an hour. Most of that was excursions,
recovery moves, endpoint dwells, static blocks and repeated setup; a start
needs only about 0.4 s of motion. They are replaced by one new schedule,
`tendon_start`, run twice with identical commands.

The schedule is implemented offline in
`src/experiments/experiments/causal_experiment.py` but not yet run. Episodes
gain an optional piecewise raw-shaft profile; only this schedule uses it.

```bash
# loaded (catheter attached, markers and disarmed estimator running)
ros2 launch bringup causal_experiment.launch.py schedule:=tendon_start start_motor:=true
# unloaded (motor pack detached; no catheter or markers)
ros2 launch bringup causal_experiment.launch.py schedule:=tendon_start start_motor:=true \
  require_estimator_tracking:=false
```

**Plan.** 168 episodes, 153.8 s of schedule, from home [20, 0, 0], with
`imricor_test` reliable speeds. Pulses last 0.3 s; the rest before each pulse
is 0.3 s unless stated.

| Block | Where | Content | Pulses |
| --- | --- | --- | ---: |
| Tendon starts | bend 2.0, 6.0, 9.5 mm | 2.0 and 4.5 mm/s (101 and 227 RPM) × both directions × same direction or after reversal × 3 repeats, ordered ++−− or −−++ toward the middle of the range | 72 |
| Rest time | 6.0 mm | 0.1 s and 1.0 s × 4 combinations × 3 repeats, plus 3.0 s same-direction × 2 directions × 3 repeats | 36 |
| Intermediate speed | 6.0 mm | 4.0 mm/s (202 RPM), 3 repeats | 12 |
| Speed change while running (H3) | 6.0 mm | 2 → 4 → 2 mm/s without stopping, alternating sign | 6 |
| Insertion reference | 6.0 mm | raw shaft 0 at 2 and 10 mm/s (36 and 178 RPM), both directions, 3 repeats | 12 |
| Combined starts (H5) | 6.0 mm | raw (shaft 0, shaft 2) = (+2, +2), (−4, +2), (−2, −2), (+4, −2) mm/s × tendon lead 0, 40, 80 ms × 2 repeats | 24 |

- The tendon maximum in `imricor_test` is 4.5 mm/s, so 250 RPM is replaced by
  227 RPM.
- In combined starts every segment gives logical insertion zero or at least
  2 mm/s, so the manager's speed floor never alters a raw command.
- Bend stays within 0.5–10.7 mm during every pulse (checked when the plan is
  built). Bend 0 is the home start. Insertion stays within 20–29.5 mm.
- Raw shaft 0 is held for every tendon-only pulse and setup move.

**Offline verification (done).**
- The collection-node guard (raw command plus manager projection, real
  `imricor_test` contract) holds on all 12,396 pulse command samples.
- All ten existing schedules are bitwise identical to the committed
  generator: state, speed ceilings and raw velocity at every 10 ms sample.
- `src/experiments/test/test_causal_tendon_start.py`: 11 tests pass. The
  package suite has 79 passed and 1 skipped.
- The package builds in a private overlay; `robot-infra/install` was not
  rebuilt.

**Simulation run (done 2026-11-02).** `simulation.launch.py` (isolated domain,
estimator disarmed) plus `causal_experiment.launch.py schedule:=tendon_start
use_sim:=true start_motor:=true`, from the private overlay. Session in the
scratchpad, not the session root.
- All 168 episodes ran in order; no topology abort, warning or fault.
- During every pulse the commanded logical velocity equals the requested
  one, raw shaft 0 is exactly 0 in tendon-only pulses, rotation is 0, and the
  manager forwarded the teleop velocity unchanged. Only the smooth setup moves
  were floored by the projection, as in the existing schedules.
- Bend stayed within 1.92–9.33 mm and insertion within 21.8–29.1 mm.
- Per-pulse bend travel matched the commanded travel within one 10 ms tick
  (median 0.03–0.05 mm, max 0.13 mm at 4.5 mm/s).

Two findings, both pre-existing and not caused by this schedule, fixed
2026-11-02 (uncommitted; built in the private overlay only):
1. **The launch never stopped the recorder.** `causal_experiment.launch.py`
   (since `dd8d048`, 2026-10-01) signalled it with
   `matches_name("causal_rosbag_record")`, but Humble names the process
   `causal_rosbag_record-N`. The recorder ran until the launch was killed,
   and the completeness check never ran. **Fix:** the SIGINT now matches the
   recorder action itself. A minimal Humble launch with the same handlers
   confirmed that the signal reaches the recorder and the launch shuts down.
   Run by hand, the completeness check fails in simulation only because the
   simulated device publishes no `/device/command_tx`. All other critical
   topics were present.
2. **The return to start timed out in simulation.** Velocity dead reckoning
   left the bend 0.14 mm off. The simulated POS servo stopped within a
   hand-set 0.1 mm on bend, but the collection node requires 0.05 mm, so it
   stopped at 0.097 mm. **Fix:** the simulated device now uses the firmware
   rule, `|jointPRate| × POSITION_TOLERANCE_MOTOR_DEG` (3 motor degrees):
   0.0281 mm on insertion, 0.225° on rotation and 0.0099 mm on bend.
   Parameter `position_tolerance_motor_deg` replaces `position_tolerance`,
   which no launch or configuration set. Two tests were added in
   `test_sim_plant.py`; the simulation suite has 39 passed.

**Rerun with both fixes (2026-11-02).** All 168 episodes again; no abort.
- Fix 1 confirmed: the launch stopped the recorder, ran the completeness
  check and exited by itself. The check fails only on the simulation-absent
  `/device/command_tx`.
- Fix 2 was necessary but not sufficient. The return still timed out, with
  bend stuck at 0.0862 mm (insertion settled within its 0.028 mm tolerance).
  The real cause: the simulated plant passes its POS-servo velocity through
  the contract's VEL projection. Within `velocity_min × position_guard_horizon_s`
  = 2 mm/s × 0.05 s = 0.1 mm of a joint limit, that projection zeroes motion
  toward the limit. The return target is bend 0, the lower limit, so the
  simulation can never get closer than 0.1 mm. The firmware executes POS
  transactions itself, outside the manager's VEL projection, so this is a
  simulation departure from the hardware path.

**Simulator POS path rewritten to mirror the firmware (2026-11-02,
uncommitted, overlay only).** Requested by the user: the simulator should
mirror hardware behavior. `sim_plant.py` and `sim_device.py` now follow
`teensy_tekceleo.ino` and `position_move_tracker.h`:
- The logical target is decoupled into motor coordinates (motor 0 =
  insertion − bend). Each driver gets one relative move of
  `round(|residual| / jointPRate)` whole motor degrees at
  `round(speed / |jointVRate|)` RPM, capped at 250, using the plant's existing
  driver-native endpoint move. The manager's VEL projection (speed floor,
  limit guard horizon, model-count guard) no longer applies to POS moves.
  Hard position limits and the plant feedback checks still do.
- A moving axis with zero RPM, or a move beyond 65,535 degrees, rejects the
  whole transaction. A repeated target is a heartbeat, not a new move.
- `FirmwarePositionTracker` ports the firmware tracker in motor coordinates:
  per-axis 100 ms settle with an individual axis stop, residual corrections
  after 0.5 s idle when progress is at least one tolerance (at most 4), and
  the 45 s hard timeout reported as `POSITION_TIMED_OUT`. The limit-switch
  resume path is not modelled; the simulator has no limit switches.
- The plant's shaft-RPM ceiling now allows the firmware's integer-RPM
  rounding of the speed limit (4.5 mm/s tendon = 227 RPM). This also applies
  to the simulated native stream.
- The old logical servo `position_servo_velocity` is removed. Tests:
  simulation suite 45 passed, including the return from bend 0.0862 mm to 0
  on the real `imricor_test` contract, which the VEL projection refuses.
  Bringup simulation-launch and package-boundary tests: 25 passed.

The earlier unit test passed only because its contract fixture has no speed
floor and ±100 limits; the new regression uses the deployed contract.
A simulation rerun with this change is still to do.

### Loaded hardware run (done 2026-11-02, user-authorized, operator present)

Session `catheter_sessions/20261102_155924_causal_proximal` (104 MB).
- Stack: production firmware `tkctl:boot-v1|…|Oct 14 2026T18:18:58`, built from
  the current source 4 s after the recovery flag was restored to 0. The
  controller ran `research_session` with
  `stack_config:=hardware_grouped_no_rotation_farther_tendon_12` and
  `video:=false`, on ROS domain 42, with per-session camera registration.
- Read-only checks before motion:
  - controller DISARMED, command output and adaptation off, live
    `task_gc_enabled=true`;
  - position streaming and the native live probe off on the manager and the
    serial bridge;
  - `MANAGER_READY`;
  - home ENC [2, 0, −1] counts;
  - markers TRACKING on both rigs, 0 rejected frames.
- Initialization to [20, 0, 0], error 0.0013 mm. All 168 episodes ran; no
  warning, fault, stall or abort.
- Return to start succeeded with a 0.0022 mm maximum error. The recorder
  stopped by itself, and the completeness check passed.
- Recorded:
  - 15,893 `/device/command_tx` frames;
  - 28,972 `/device/state` messages;
  - 4,805 marker frames;
  - 15,377 trace rows;
  - 110,278 `DRIVER_UART` events. These are firmware-timestamped
    driver-transaction evidence, not faults, and usable to time each start's
    O0/L1/S/C/O1 sequence.
- End state: run start [20.003, 0, −0.0006] mm, ENC [47544, 0, 4]. The return
  to home is the operator's choice.

Analysed: see section 8.

### Unloaded hardware run (done 2026-11-02, user-authorized)

Session `catheter_sessions/20261102_160759_causal_proximal`. Run right after
the loaded run, with the stack unchanged and the cameras left running as a
check that nothing reached the catheter. The pack was detached at the run start
[20.003, 0, −0.0006] mm.
- Before the run: ENC [47544, 0, 4], steady over 182 samples.
- Launched with `require_estimator_tracking:=false`. All 168 episodes ran; no
  warning, fault or stall. The measured run-start bend of −0.0006 mm was
  snapped onto the 0 limit for the plan.
- Return succeeded within the firmware's 3-motor-degree tolerance; the
  completeness check passed.
- **Encoder check failed on motor 0.** After the run, ENC was
  [47531, 0, 1]: Δ = [−13, 0, −3] counts against the user's ±11-count
  threshold. Motor 0 is 13 counts short, 0.29 motor degrees or 0.0055 mm of
  insertion. A sub-degree correction is not reliably achievable: position
  moves are whole motor degrees, and a velocity jog would last about 3 ms.
- **The user accepted the 13-count residual before reattachment.** After
  reattachment, the insertion encoder-to-mechanism relation differs from the
  training zero by about 0.0055 mm. No `SET_ZERO` was used. The evidence is in
  the session's `encoder_detach_check.json`.
- **Post-reattachment check.** One research-session registration covers the
  loaded run's end, detachment, the unloaded run and reattachment, so marker
  steps there carry no registration change. Median marker 0 position along
  the insertion axis (z), in 10 s bins:
  - **detachment** (about 255 s after the loaded run): −0.05 mm;
  - **during the unloaded run**: flat within about 0.01 mm while motor 0
    travelled 44,960–54,515 counts. The pack was fully decoupled.
  - **reattachment** (about 640 s): +0.10 mm;
  - net: about +0.05 mm at a constant encoder reading.
- The home tip differs from the pre-run home by 0.9 mm on markers 0–2 (mostly
  z) and 1.6 mm on marker 3. The two registrations explain at most about
  0.1 mm of this. The rest is insertion play from the approach: homing from
  20 mm moved marker 0 by 17.3 mm, and the initial move out by 18.2 mm,
  against 20 mm commanded. It is not a detachment shift.
- Also seen: after the loaded run, the tip marker drifted about 0.25 mm in y
  over 4 min at a fixed encoder reading, consistent with relaxation after
  cycling.
- For any future detached run: the firmware return tolerance (about 67 counts)
  is wider than an encoder-restoration threshold, so it cannot guarantee
  restoration. Plan for it, or accept a declared residual beforehand.

Analysed: see section 8.

### Unloaded run: motor pack detached, protecting the encoder zero

The encoders are on the motors and the zero must stay consistent with the
training data. Running motors detached from the robot changes their angles
relative to the catheter mechanics.

1. Home. Record the exact ENC counts and logical positions, and keep them in
   the session.
2. The operator detaches the motor pack.
3. Run `tendon_start` unloaded. The schedule finishes at its run-start
   relative zero.
4. Verify the ENC counts equal the recorded counts. If they do not, stop:
   do not reattach and do not `SET_ZERO`; decide with the user.
5. The operator reattaches the pack. Home again and compare the measured tip
   at home with the last homed tip before any further motion.

## 5. Measurements

- Recorded topics are those of the existing causal launch: commands at each
  stage (`/teleop/control`, `/manager/control`, `/device/command_tx`), ENC and
  POS (`/device/state`), device events, transport status and markers.
  Sessions are written to the external session root.
- Per pulse, from the raw motor-2 encoder (polarity-aware):
  - onset: the first sample at or above 5% of the commanded speed;
  - t50 and t90;
  - whether a plateau exists: more than 30 ms between 20% and 70% of speed;
  - travel deficit against an ideal step over the commanded duration (mm).
- Combined starts: logical insertion overshoot (mm, and ms above
  110% of commanded insertion speed), and the lead time that minimizes it.
- Resolution limit: about 11 ms from ENC telemetry, and host-receipt time
  stamps. Firmware-side time stamps or faster telemetry would be a firmware
  change, so they are out of scope.

## 6. Decision rules (declared before running)

| Result | Conclusion |
| --- | --- |
| Unloaded t90 within 15 ms of loaded | Not load (H1 rejected); go to H2 and H3 |
| Unloaded t90 under half of loaded | Load-dominated (H1); model start time as a function of tendon state |
| t90 rises roughly in proportion to RPM | Acceleration-limited (H2); a ramp model or driver tuning |
| Running speed-change rise under half of the from-rest t90 at the same target | Start-path dominated (H3); firmware start path or a policy avoiding stops |
| Median t90 differs by over 20 ms between dwell 0.05 s and 3.0 s, or between same-direction and after-reversal starts | History effect (H4) |
| Some insertion lead removes the coupled insertion overshoot to under 20% | Lead compensation is viable (H5) |

Each remedy, whether a firmware change, a model mechanism or a planner
policy, is proposed separately afterwards. None is part of this plan.

## 8. Results (2026-11-02, both hardware runs)

The outputs and scripts are in `catheter_sessions/tendon_start_analysis_20261102`.

**Method.**
- Time zero is the host transmit of the first velocity frame commanding the
  axis (`/device/command_tx`).
- The firmware driver-UART log (`driver_native_evidence_v1`) records firmware
  microseconds. These were mapped to host time with a lower-envelope linear
  clock fit (residual P95 0.05 ms, drift −4.3 ppm), because the log reaches
  the host in batches during motion.
- Speed is the encoder count difference between ENC samples (about 11 ms
  apart, host-receipt stamped), normalized by the RPM the firmware sent the
  driver.
- Crossing times are interpolated. Differences under about 10 ms are within
  the telemetry resolution.
- 0.14% of log records were lost between firmware and bag (the firmware
  reported none dropped). For those starts, and where the firmware skipped
  `$C` because the RPM was unchanged, the RPM is the firmware's rounding of
  the commanded speed and transmit → `$O1` is unavailable.
- Coverage: all 120 tendon-only starts in each run, plus 12 speed changes,
  12 insertion starts and 24 combined starts per run.

**Tendon starts from rest** (median, with P90 for t90):

| RPM (mm/s) | Loaded t90 | Unloaded t90 | Transmit → `$O1` | `$O1` → t90 (loaded / unloaded) | Plateau (loaded / unloaded) | Deficit (loaded) | Equivalent dead time (loaded / unloaded) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 101 (2.0) | 82.6 (103) | 79.9 (92) | 19.3 | 66.0 / 60.2 | 66/72 / 72/72 | 0.109 mm | 54.5 / 48.1 ms |
| 202 (4.0) | 73.7 (80) | 67.2 (71) | 18.5 | 53.3 / 47.5 | 0/12 / 0/12 | 0.187 mm | 46.7 / 44.3 ms |
| 227 (4.5) | 71.1 (79) | 66.4 (72) | 19.5 | 52.3 / 47.6 | 0/36 / 0/36 | 0.199 mm | 44.3 / 42.3 ms |

The steady speed matched the commanded RPM (median ratio 0.997). The
equivalent dead time is the travel deficit divided by the commanded speed.

**Comparisons.**
- Running speed change, a single `$C` transaction: t90 28.5 ms loaded and
  30.3 ms unloaded for 2 → 4 mm/s. Transmit → `$C` is about 7 ms; the motor
  responds in about 21 ms.
- Insertion motor 0 from rest: t90 14–20 ms (loaded) and 17–27 ms (unloaded)
  at 36–178 RPM. It is at speed within about 5 ms of `$O1`.
- The firmware start sequence takes about 19 ms from host transmit to `$O1`:
  a stop, a 5 ms settle, then acknowledged `O0`/`L1`/`S`/`C`/`O1`
  transactions of about 1.4 ms each. A running speed change takes about 7 ms.

**Decision rules (section 6):**

| Rule | Result | Conclusion |
| --- | --- | --- |
| Unloaded t90 within 15 ms of loaded | 2.7 / 6.5 / 4.7 ms slower when loaded at 101 / 202 / 227 RPM | **H1 rejected:** not load |
| t90 roughly proportional to RPM | t90 falls as RPM rises (82.6 → 73.7 → 71.1 ms) | **H2 rejected:** not acceleration-limited |
| Running speed-change rise under half of from-rest t90 at the same target | 28.5 vs 73.7 ms at 202 RPM (ratio 0.39) | **H3 supported:** start-path dominated |
| Dwell 0.1 s vs 3.0 s, or same direction vs after reversal, differing by more than 20 ms | Dwell 6.8 / 6.2 ms. Direction history +16.1 ms loaded at 2 mm/s (same direction slower), −5.6 ms at 4.5 mm/s, ≤ 0.6 ms unloaded | **H4 not met.** A load-related 16 ms effect at 2 mm/s is below threshold. The plan said 0.05 s dwell; the schedule used 0.1 s. |
| Some lead brings the coupled insertion error under 20% | Peak insertion error 0.106 / 0.169 / 0.207 mm loaded and 0.091 / 0.147 / 0.173 mm unloaded at lead 0 / 40 / 80 ms | **H5 rejected:** a lead makes it worse |
| Position × direction difference over 20 ms | Range of cell medians: 17.1 / 9.3 ms loaded and 13.6 / 4.0 ms unloaded at 2 / 4.5 mm/s | None |

**Interpretation.**
- The tendon start-up is intrinsic to the motor-2 start path, not the catheter
  load. It has two parts:
  - the firmware's re-initializing start sequence, about 19 ms against about
    7 ms for a running speed change;
  - the motor-2 driver's response after enable, about 50–66 ms against about
    21 ms for a running speed change. Motor 0 responds within about 5 ms of
    enable.
- At 101 RPM nearly every start is two-stage (the plateau), and at 202–227 RPM
  none is.
- The net effect is an equivalent dead time of about 42–55 ms per start from
  rest, nearly independent of load and history. It costs 0.10–0.20 mm per
  start at the tested speeds.
- Combined starts: the logical-insertion error is the tendon lag mirrored by
  the decoupling (motor 0 = insertion − bend, and motor 0 is fast).
- **New firmware finding.** Starting a motor stops and restarts every motor
  already running: `sendMotorCmds` from rest re-initializes the active set.
  With a 40 or 80 ms lead the running tendon was restarted in 16/16 pulses
  and paid its start delay twice. The same happens in normal MPPI operation
  whenever one axis starts while another runs.

**Remedies** are separate proposals and none is authorized. The candidates:
- a planner policy that avoids stopping the tendon, preferring running speed
  changes;
- a firmware start path that does not restart motors already running and
  shortens the from-rest sequence;
- a deterministic start-delay model in the forecast, about 45–55 ms
  equivalent dead time depending on RPM.

## 9. On-the-fly velocity firmware for the test (2026-11-02, uncommitted, not flashed)

Requested by the user: change only the firmware, so that speed and direction
can be changed on the fly in a test. No planner, schedule or host change.

**Change.**
- A compile-time flag, `TKCTL_ONTHEFLY_VELOCITY_ENABLED`, **defaults to 0**,
  so production is unchanged. With the flag at 1, `sendMotorCmds` classifies
  each active axis with the new pure header `velocity_update_plan.h`:
  - running in the same direction: only `$C` when the RPM changes, as before;
  - running and reversing: `$S` (direction), then `$C`, **without disabling
    the driver**. This is the behaviour under test.
  - not running (disabled, or mode or direction cache unknown): the existing
    stop, 5 ms settle, `O0`/`L1`/`S`/`C` and coordinated `O1`, **for these
    axes only**. Running axes are no longer stopped and restarted when another
    axis starts.
- Running axes are updated first, then starts.
- Any failure (including the rollback of a failed start) stops every active
  axis, as before.

**Unchanged.**
- Zero speed still disables (`$O0`).
- The watchdog, fault latching, stall detector, limit checks, position and
  streamed paths, `SET_ZERO` rejection and the recovery image are all
  unchanged. The recovery image refuses the flag with `#error`.
- The stall detector already treats a reversal or a large speed step as a new
  start-up phase.

**Identity.** A test image appends `onthefly_velocity_v1|` to
`FIRMWARE_BUILD_ID`. The host matches capabilities by substring, so this is
safe, and the 161-byte status frame fits.

**Verification (offline).**
- The sketch translation unit was rebuilt with the cached IDE prototypes and
  compiled with the cached Teensy 4.0 command (600 MHz, `-O2`): 0 warnings
  with the flag at 0 and at 1.
- With the flag at 0, every `.text` section is byte-identical to a build of
  the committed `HEAD` sketch. `sendMotorCmds` is 1,808 bytes in both; with
  the flag at 1 it is 2,004 bytes.
- Tests:
  - 13 firmware component tests pass with `-Wall -Wextra -Werror -pedantic`
    and UBSan, including the new `test_velocity_update_plan.cpp` (6 cases;
    legacy policy equivalence included);
  - source-policy tests 11 passed, including the default-off flag and the
    preserved legacy block;
  - host identity and streamed-position tests 31 passed.
- Nothing was flashed and no hardware ran.

**To build the test image (operator).**
1. Set `#define TKCTL_ONTHEFLY_VELOCITY_ENABLED 1` in
   `teensy_tekceleo.ino`.
2. Compile and flash with driver power off.
3. Confirm the boot diagnostic contains `onthefly_velocity_v1|`.
4. Restore 0 and reflash production after the test.

**Open before the hardware test.**
- Whether the Tekceleo driver accepts `$S` while enabled is exactly what the
  test determines. A failed acknowledgement takes the existing fault-stop
  path.
- The existing `tendon_start` schedule covers running speed changes and
  starts while another axis runs (the combined leads). Reversals without an
  intermediate rest are in the new `tendon_reversal` schedule below.
- The hardware run needs its own authorization.

**Reversal schedule `tendon_reversal` (2026-11-02, uncommitted).**
- `causal_experiment.launch.py schedule:=tendon_reversal`. It has 60
  episodes and lasts 76.2 s at bend 6.0 mm (the middle knob position), with
  the `tendon_start` pulse, rest and speed settings. No new configuration
  field.

| Block | Profile after a 0.3 s rest | Episodes |
| --- | --- | ---: |
| `tendon_reversal` | tendon +v for 0.3 s, then −v for 0.3 s, no zero command between; v = 2.0 and 4.5 mm/s, both orders, 3 repeats | 12 |
| `tendon_reversal_rest` | matched in-session reference: the same with a 0.3 s rest between | 12 |
| `tendon_reversal_speed_change` | reversal with a speed change: 2 → −4.5 and 4.5 → −2 mm/s (orders cancel), both signs, 3 repeats | 12 |
| `tendon_reversal_insertion` | raw shaft 0 reversal at 2 and 10 mm/s, both orders, 3 repeats | 12 |
| `tendon_reversal_with_insertion` | tendon ±2 mm/s reversal while raw shaft 0 keeps running at ±4 mm/s (logical insertion 6 or 2 mm/s), 2 repeats | 8 |

- On production firmware every direct reversal is a coordinated restart. On
  the on-the-fly image it is `$S` then `$C` without disabling. The schedule
  therefore gives a legacy baseline and the test result on the same commands.
- The static holds, setup moves, shaft-0 return check, exit and bend-band
  check are shared with `tendon_start` through
  `CausalExperimentGenerator._raw_profile_session`.
- **The refactor is behaviour-preserving.** All 11 existing schedules,
  including `tendon_start`, hash identically before and after: state, speed
  ceilings and raw shaft velocity at every 10 ms sample, plus the episode
  metadata.
- Tests: `src/experiments/test/test_causal_tendon_reversal.py` adds 5 tests:
  - bounds and rest at the ends;
  - a direct sign change against the rest-separated reference;
  - held shafts, and insertion constant through a tendon reversal;
  - direction and speed coverage;
  - raw command unchanged by the manager projection with the real
    `imricor_test` contract over all pulse samples. This is the collection
    node's `_timing_command_error` guard.

  The experiments suite has 84 passed and 1 skipped.
- **Simulation check (2026-11-02, passed).**
  - Isolated domain, estimator disarmed, `install` only (current source).
  - All 60 episodes ran, with no abort or fault.
  - The commanded velocity equalled the requested one on all 5,400 pulse
    samples. Raw shaft 0 was 0 in the tendon-only blocks, and raw shaft 2 was
    0 in the insertion block.
  - Bend stayed within 4.56–7.27 mm and insertion within 22.9–30.7 mm.
  - The manager forwarded exactly the planned direct reversals, with no zero
    frame between: 32 on shaft 2 and 12 on shaft 0.
  - The return to start **succeeded** with a 0.0024 mm error in 0.38 s, the
    first end-to-end confirmation of the firmware-mirroring simulated
    position path.
  - The launch stopped the recorder and ran the completeness check itself.
    It fails only on the `/device/command_tx` topic the simulator does not
    publish.
  - The simulator does not model driver reversal behaviour. That is what the
    hardware test measures.
  - Not yet run on hardware.

**Hardware test on the on-the-fly image (2026-11-02, user-authorized,
operator present).**
- Firmware: `…|driver_native_evidence_v1|onthefly_velocity_v1|Nov  2 2026T17:18:16`.
  It was built 12 s after the sketch was saved with the flag at 1, and its
  diff against `HEAD` is exactly the reviewed change. The working-tree sketch
  still has the flag at 1, so the source-policy test fails until it is
  restored.
- The reflash restarted the encoder counters at home: [−1, 0, 0] became
  [0, 0, 0]. Home markers matched the post-reattachment home within 0.12 mm,
  against a declared limit of 0.3 mm.
- **`tendon_start`** (session `20261102_172930_causal_proximal`):
  - all 168 episodes; return succeeded (0.0017 mm); completeness check passed;
  - one firmware `MOTION_RETRYING:STALL` on axis 2, recovered by the stall
    retry;
  - the stall came during the setup move `tendon_start_enter_p1`, about 1.1 s
    into steady 101 RPM cruise, with a constant command and no driver
    transaction to that axis for 1.5 s (no log records missing). The motor
    stopped by itself for about 650 ms. Two full start sequences had run
    since the move's first frame. That first frame was an unplanned direct
    reversal (−4.5 → +2 mm/s at the episode boundary), which the test image
    executed as `$S0` + `$C101` and the motor followed within one encoder
    sample.
  - Neither earlier legacy run had a stall event.
- **`tendon_reversal`** (session `20261102_173447_causal_proximal`): all 60
  episodes, no warning, fault or stall; return succeeded (0.0030 mm);
  completeness check passed.

First look at the reversals. Times are from host transmit; ENC samples are
about 11 ms apart, so roughly ±5 ms resolution.

| Block | n | Zero crossing, median | t90, median (P90) | Driver disabled (`$O0`) | Other shaft restarted |
| --- | ---: | ---: | ---: | ---: | ---: |
| `tendon_reversal` | 12 | 8.7 ms | 15.8 (23.1) ms | 0/12 | 0/12 |
| `tendon_reversal_speed_change` | 12 | 6.9 ms | 15.7 (26.7) ms | 0/12 | 0/12 |
| `tendon_reversal_insertion` | 12 | 5.5 ms | 13.6 (15.7) ms | 0/12 | 0/12 |
| `tendon_reversal_with_insertion` | 8 | 7.1 ms | 12.2 (17.4) ms | 0/8 | 0/8 |
| Reference: start from rest after a rest (`tendon_reversal_rest`) | 12 | — | 80.9 ms | — | — |

- **The Tekceleo drivers accept `$S` while enabled.**
- An on-the-fly reversal reaches 90% of the new speed in about 12–16 ms,
  against about 81 ms for a start from rest.
- A tendon reversal no longer restarts the running insertion shaft.
- **`tendon_start`, test image against the legacy loaded session**
  (identical schedule, episode-matched; outputs in
  `scratchpad/startup/rows_onthefly.json`, `combined_onthefly.json` and
  `onthefly_compare.txt`):

| Measure (median) | Legacy | On the fly |
| --- | ---: | ---: |
| Tendon start from rest t90, 101 / 202 / 227 RPM | 82.6 / 73.7 / 71.1 ms | 82.7 / 73.0 / 71.6 ms |
| Paired tendon start t90 difference (n = 120) | — | −0.3 ms (IQR −6.6 to +6.5) |
| Start deficit, 101 / 227 RPM | 0.109 / 0.199 mm | 0.106 / 0.211 mm |
| Running speed change t90, 2 → 4 / 4 → 2 mm/s | 28.5 / 26.2 ms | 25.0 / 28.3 ms (one 99 ms outlier) |
| Insertion start while tendon runs: tendon restarted | 14/16 | **0/16** |
| Insertion transmit → `$O1` in those starts | 25.4 ms | 18.6 ms |
| Tendon t90 in combined starts, lead 40 / 80 ms | 129.8 / 174.1 ms | 95.2 / 90.4 ms |
| Peak bend error, lead 0 / 40 / 80 ms | 0.135 / 0.193 / 0.247 mm | 0.141 / 0.140 / 0.127 mm |
| Peak insertion error, lead 0 / 40 / 80 ms | 0.106 / 0.169 / 0.207 mm | 0.099 / 0.132 / 0.135 mm |
| Direct reversals at episode boundaries | 1, restarted | 1, live `$S` |
| Insertion-only start from rest t90, 2 / 10 mm/s | 14.0 / 20.3 ms | 18.8 / 25.2 ms |

**Reading.**
- Starts from rest and running speed changes are unchanged, as expected:
  they use the same firmware path.
- The test image removes the restart of running motors. When insertion
  starts during a tendon move, the tendon reaches speed as in a single start
  (about 90–95 ms instead of 130–174 ms), and the bend error no longer grows
  with lead.
- A tendon lead still does not reduce the coupled insertion error below
  lead 0. H5 stays rejected: the tendon start delay reaches logical insertion
  through the decoupling whatever the lead.
- Together with `tendon_reversal`, on-the-fly reversals take 12–16 ms
  against about 81 ms from rest. With this firmware, the remaining slow event
  is a start from rest: 42–55 ms equivalent dead time, 0.10–0.20 mm.
- Unexplained, small or single events:
  - Insertion-only starts were about 5 ms later from transmit to `$O1` on the
    test image (n = 6 + 6), on the same code path. This could be main-loop
    phase or build timing; not investigated.
  - One slow running speed change (99 ms) and one transient tendon stall
    (above) occurred in the test-image run. Both were on paths the change does
    not touch, and the legacy runs had neither. This may be run-to-run drive
    variation; watch for it.
- The legacy reversal baseline (`tendon_reversal` on production firmware) was
  not run. The in-session rest-separated pairs are the reference.
- **Production restored (2026-11-02).**
  - Identity `tkctl:boot-v1|…|driver_native_evidence_v1|Nov  2 2026T17:38:34`,
    with no `onthefly_velocity_v1`. It was built 5 s after the sketch was
    saved with the flag back at 0.
  - The reflash restarted the counters at [0, 0, 0]. The cameras confirm home:
    19 mm from the run-start pose, and 0.21–0.34 mm from the pre-test home,
    of which the new camera registration alone predicts 0.06–0.22 mm.
  - The encoder chain is bounded by the homing's firmware completion
    tolerance: at most 67 counts, 0.028 mm insertion and 0.0099 mm bend. The
    exact landing count was not recorded.
  - Source-policy tests pass again.

**Proposed hardware order** (each step needs authorization):
1. `tendon_reversal` on production firmware: the legacy reversal baseline.
2. Flash the on-the-fly image; then `tendon_start` and `tendon_reversal`.
   `tendon_start` compares directly with the 2026-11-02 loaded session.
3. Restore production firmware.

## 7. Order, cost and approvals

1. **Offline.** Schedule, tests, guard replay, regression of the existing
   schedules, the overlay build, the simulation run and the fixes for its two
   findings are done. A simulation rerun with the fixes is still to do.
2. **Loaded hardware run.** About 10–12 min including setup and homing; 2.6
   min of motion. Needs authorization.
3. **Unloaded hardware run** with the motor pack detached. About 15 min
   including the encoder-count check and reattachment. Needs authorization.

Analysis uses the decision rules in section 6. They gain one: a position ×
direction difference over 20 ms in median t90, attributed to load if the
unloaded run shows none.
