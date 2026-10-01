# Session 20260927_194659 engaged-gain axial hardware audit

## Outcome

Point 1 reached with 0.769 mm final error. Point 2 timed out at 9.273 mm
because a valid tendon take-up transaction was numerically suppressed at the
final publishability projection. This was not an engaged-gain, estimator,
planner-deadline, manager, or hardware fault.

## Evidence and classification

### F-001: coupled minimum-speed command collapsed to zero

Severity: high
Confidence: observed
Control path: MPPI -> take-up arbiter -> publishability projection -> manager

At 1790552866.448 the selected MPPI plan began with logical velocity
`[-9.5204, 0, 4.5, 0, 0, 0]`. The arbiter correctly marked physical shafts 0
and 2 active, shaft 0 already engaged, and only shaft 2 pending. It generated
the coupled logical take-up command `[2, 0, 2, 0, 0, 0]`, which holds physical
chassis shaft 0 while moving physical tendon shaft 2.

Coupling arithmetic represented the two 2.0 minimum-speed components as
`1.9999999999999998`. `HardwareContract.project_publishable_velocity` used a
strict comparison against the 2.0 floor, classified both components as
materially subminimum, and returned zero. `/teleop/control` and
`/manager/control` therefore remained zero from 1790552866.457 through the
point-2 timeout. Device position stopped at approximately
`[19.6902, 0, 0, 0, 0, 0]`.

The controller remained `TAKEUP_ACTIVE` with active mask `[1,0,1]`, pending
mask `[0,0,1]`, direction `[-1,0,-1]`, no deadline misses, estimator health
`TRACKING`, and no fault. No movement meant no tendon engagement evidence, so
this state could not terminate.

### F-002: successful transaction speed telemetry was stale zero

Severity: low
Confidence: observed

The diagnostic keys `takeup_requested_motor_rad_s` and
`takeup_realized_motor_rad_s` were updated only on saturation, so they reported
zero during an otherwise active transaction. This obscured F-001 but did not
cause it.

## Remediation

- Treat values within a relative 1e-9 / absolute 1e-12 numerical tolerance of
  the configured minimum reliable speed as equal to the floor. Genuinely
  subminimum commands created by limit projection still become hold.
- Added a regression using the recorded position and the one-ulp-low coupled
  `[2,0,2]` command; it verifies physical shaft 0 is held and shaft 2 moves.
- Update configured and realized physical take-up rates on every transaction
  tick for the next hardware audit.

## Verification

- 104 focused hardware-contract/backlash/MPPI tests passed after the functional
  fix.
- Full `catheter_control` suite: 277 passed.
- ROS package rebuilt successfully.
- Exact 384-candidate x 3-scenario CUDA preflight passed; report:
  `/media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions/20260927_195321_phase5_preflight.json`.

## Retest gate

Restart the controller so the rebuilt installed entry point is loaded, preserve
the no-rotation profile, and repeat only the two-point axial block. Do not
continue to the two-axis block unless both points complete and the recording
shows nonzero realized tendon take-up followed by response confirmation, a
zero replan barrier, and a fresh MPPI plan.
