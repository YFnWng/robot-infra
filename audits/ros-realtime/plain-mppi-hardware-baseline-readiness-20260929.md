# Plain-MPPI hardware baseline readiness — 2026-09-29

## Finding

The conventional plain-MPPI implementation already existed in
`src/catheter_control/launch/simulation.launch.py` as
`mppi_variant:=plain`. It uses a single ungrouped proposal population and
the weighted feasible MPPI mean. Relative to the grouped controller, it
disables grouped U/C proposal modes, scored-candidate execution,
take-up-delay cost, and the response-clocked reversal scheduler.

No reviewed hardware profile previously materialized that switch, and the
uncompensated baseline cannot load the v175 transmission artifact because the
runtime explicitly requires compensation whenever that artifact is active.

Confidence: **observed** from source and configuration.

## Implemented baselines

### A — plain MPPI plus take-up compensation

Controller profile:
`src/catheter_control/config/v175_plain_takeup_hardware_no_rotation.yaml`

Preserved:

- v171 distal mechanics and v174 local Jacobian;
- v175 motor-to-interface transmission belief;
- response-observed engagement belief and engaged-gain scenarios;
- slow atomic take-up transactions;
- four 0.2 s point-rollout steps, 48 stochastic samples, and rotation lock.

Disabled:

- grouped U/C sampling;
- best-scored-candidate execution guard;
- MPPI take-up-delay cost;
- reversal scheduler.

### B — plain MPPI only

Controller profile:
`src/catheter_control/config/v171_plain_hardware_no_rotation.yaml`

In addition to the plain-planner changes above, this disables:

- v175 transmission artifact;
- backlash-state compensation;
- take-up transactions;
- engaged-gain belief/scenario rollout.

The v171/v174 runtime consumes raw shaft encoders. Commands still pass through
controller projection, manager limits, and firmware limits.

## Matched Cartesian task

Both baseline experiment profiles contain the four exact `/target_tip`
coordinates recorded in successful grouped session
`20260929_104353_mppi_demo`:

1. [24.685867, 20.163964, 84.481336] mm
2. [33.203728, 29.865349, 63.246734] mm
3. [31.191053, 29.780382, 51.087875] mm
4. [31.725504, 29.091984, 50.746046] mm

This avoids asking different controller models to synthesize different target
sets. Each run retains guarded [20,0,0] encoder homing, physical-history
preservation, zero rotation velocity, and command monitoring.

## Runtime gates

The experiment refuses to arm unless diagnostics match the intended:

- compensated/raw-shaft estimator boundary;
- take-up transaction enable;
- plain-MPPI selector;
- ungrouped sampling;
- weighted-mean execution;
- zero take-up risk cost;
- rotation-disabled velocity contract.

## Verification

- Complete `catheter_control` unit suite: 299 passed.
- ROS package rebuilt successfully.
- Four controller/experiment profiles installed and parsed.
