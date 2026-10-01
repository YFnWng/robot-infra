# Far two-axis point-3 home interruption audit

Date: 2026-09-20

## Scope

Passive diagnosis of the hardware run started from ROS log directory:

`/home/chen-lab/.ros/log/2026-09-20-18-10-27-349403-BM2-Desktop-1-639817`

The run was not captured in a new rosbag. Evidence below therefore comes from
the sparse-point process log and the still-running control-interface launch
log. No hardware command was issued during this audit.

## Result

Points 1 and 2 reached their generated targets with final errors of 1.443 mm
and 0.609 mm. The run failed before point 3 during the return to `[20,0,0]`.
Firmware reported:

```text
POSITION_TIMED_OUT mask=0x05
errors=[3.3682, 0, 0.00104, 0, 0, 0]
```

The terminal POS was `[16.6307, 0, -0.0010, 0, 0, 0]`. Thus tendon was home
within the experiment tolerance, but insertion remained 3.3693 mm short.

## Causal timeline

- The running firmware was `tkctl:boot-v1|Sep 19 2026T16:36:33`, which includes
  the earlier residual-axis resumption repair.
- Point 2 commanded a tendon displacement and reached its tip target.
- The point-3 atomic home moved insertion and tendon together (`mask=0x05`).
- At `1789942251.947`, tendon entered its lower-bound state (`Lnnnan`). The
  firmware deliberately stopped every motor and resumed the still-needed
  insertion residual.
- At `1789942252.643`, the position tracker timed out. The resumed insertion
  segment had made some progress, but not the 25% fraction required to earn
  another endpoint correction.

This is not an MPPI planner fault, marker-estimator fault, stale command, or
failure to flash the September 19 firmware. It is a remaining interaction
between the logical insertion/tendon coupling, a deliberate global limit stop,
and the bounded endpoint retry gate.

## Implemented isolation-runner correction

The hardware isolation configurations now request a decoupled tendon prehome.
If logical axis 2 is away from its home tolerance, the runner first computes:

```text
logical_target_2 = home_target_2
logical_target_0 = current_logical_0 - current_logical_2 + home_target_2
```

Because firmware uses
`physical_axis_0 = logical_axis_0 - logical_axis_2`, this first transaction
moves the tendon to zero while holding physical chassis axis 0 fixed. After
that transaction settles, a second transaction moves to the final `[20,0,0]`
home. The tendon boundary event can no longer interrupt a simultaneously
moving insertion axis.

The change is intentionally above firmware safety. It does not alter hard
limits, encoder integrity, stop semantics, position retry thresholds, the
manager, or MPPI.

## Verification

- sparse-point unit tests: 15 passed;
- Python compilation: passed;
- `ament_flake8`: passed for source and test;
- `colcon build --packages-select catheter_control --symlink-install`: passed;
- installed far-hardware YAML resolves `decoupled_tendon_prehome: true` and
  preserves the farther candidate set.

## Next guarded test

Restart/source the rebuilt catheter-control overlay, then repeat the farther
rotation-disabled sparse-point experiment. Before target execution, confirm
the log contains `point N tendon prehome` whenever tendon is nonzero, followed
by `point N tendon prehome complete` and `point N home complete`. Stop if any
prehome or final home reports a position timeout; do not relax firmware limits
or retry gates for this experiment.
