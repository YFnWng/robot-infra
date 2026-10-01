# MPPI simulation robustness-test plan

## Objective

Extend the passing exact-model simulation into a controlled sim-to-real test
bench. Change one boundary at a time so failures can be assigned to sensing,
actuation, proximal-model error, or their interaction. The controller must
remain unchanged during a plant/sensor sweep.

The 2026-09-11 hardware evaluation supplies the first calibration targets:

- stationary tip-increment p95: approximately 0.17--0.20 mm;
- useful-response median actual/model gain: approximately 0.07--0.10;
- insertion interface gain relative to `J0`: approximately 0.55--0.57;
- rotation angular gain relative to `J0`: approximately 0.46--0.53;
- rotation linear direction inconsistent with `J0`;
- bending angular response not sufficiently identified.

These are sweep centers, not assumed ground truth.

## Required architecture

Keep three distinct boundaries:

```text
MPPI/controller nominal model
            |
            v
production manager -> simulated actuator -> perturbed truth model
                                             |
                                      uncorrupted truth
                                             |
                                      sensor perturbation
                                             |
                                  controller marker input
```

Ground-truth markers and tip must remain uncorrupted for scoring. Sensor
perturbations affect only `/sim/shape_tracking/markers`. The controller keeps
the nominal v171/v174 artifacts while the truth runtime receives simulation-
only Jacobian perturbations. This avoids accidentally giving the controller
knowledge of the injected mismatch.

## Implementation

Implementation status (2026-09-12): R0--R3 and the first per-trial R4 gates
are implemented in `catheter_control`. The launch exposes all perturbations
below, and `catheter_sim_scenario` runs and scores a single axis-separated
trial. The first scorer enforces final-error/no-fault, finite-command, and
post-disarm-zero gates. Automated limit-history, sustained-oscillation, and
fault-injection matrix gates remain R4 follow-up work.
Large multi-seed matrices remain an experiment campaign rather than being run
implicitly; each trial deliberately uses a fresh launch/domain so estimator,
actuator, and MPPI state cannot leak between cases.

### R0 — deterministic perturbation primitives

Add a ROS-independent `SimulationPerturbationConfig` and tested components:

- `MarkerSensorModel`: seeded Gaussian noise, common rigid bias,
  marker-specific bias, latency queue, timestamp jitter, dropout, and outlier;
- `ActuatorPerturbation`: per-axis gain, deadband, first-order lag, asymmetric
  reversal/backlash, and optional command-to-achieved delay;
- `JacobianPerturbation`: per-column gain plus separate angular/linear block
  transforms applied only to the truth runtime;
- deterministic seeds and a serialized effective configuration.

All defaults must reproduce the current ideal simulation bit-for-bit. Invalid
or non-finite parameters fail before nodes start.

### R1 — preserve truth and add the sensor boundary

Refactor simulated perception to publish:

- `/sim/catheter_sim/ground_truth_markers` and `ground_truth_tip` before noise;
- the perturbed controller observation on `/sim/shape_tracking/markers`;
- sensor queue age, injected bias/noise, dropped frames, and seed in
  diagnostics.

Do not alter ground truth when adding latency or dropout. Simulated marker
health should reflect the injected condition instead of always claiming
perfect tracking.

### R2 — actuator and truth-model mismatch

Apply actuator perturbations after manager projection but before encoder-count
integration. Publish requested, manager-projected, and achieved commands as
separate topics.

Allow the truth runtime to load a perturbed copy of `J0`. Never overwrite the
artifact or the controller runtime. Record both matrices and their hashes in
the bag and summary.

### R3 — repeatable scenario runner and scorer

Implement a non-interactive runner for seeded trials:

1. launch at home and wait for `MANAGER_READY` and estimator initialization;
2. command one target from `{+x,-x,+y,-y,+z,-z}`;
3. run for 20 seconds or until settled;
4. disarm and save a JSON result;
5. use a fresh ROS domain/process for the next trial so state is not silently
   carried between configurations.

Save bags, effective configurations, and summaries beneath
`/media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions`.

Each implemented result includes final, best, and maximum error; settling
time; integrated absolute error; command energy; estimator acceptance and
rejection state; deadline-miss diagnostics; terminal lifecycle/fault state;
effective perturbation configuration; command finiteness; and whether disarm
emitted zero. Limit-clamp and detailed lifecycle histories remain available in
the recorded ROS topics rather than being duplicated into the first version
of the JSON scorer.

### R4 — automated regression gates

Use these initial gates, revising only from recorded baseline distributions:

- ideal baseline: final error <= 0.5 mm and no fault;
- nominal robustness pass: final error <= 1.0 mm within 20 seconds, no fault,
  and no sustained oscillation;
- safety pass under every perturbation: finite bounded command, limits always
  respected, stale/invalid feedback fails to zero, and no late nonzero command
  can be committed after a fault;
- deterministic replay: same seed and configuration reproduce metrics within
  declared numeric tolerance.

An intentionally severe perturbation may fail tracking while still passing
the safety gate.

## Experiment matrix

Run each row independently before combining rows.

| Family | Sweep |
|---|---|
| Independent marker noise | 0, 0.05, 0.10, 0.20, 0.40 mm standard deviation |
| Common marker bias | 0, 0.25, 0.5, 1, 2 mm; each Cartesian axis |
| Marker-specific bias | 0, 0.1, 0.25, 0.5 mm |
| Latency | 0, 33, 66, 100, 150, 200 ms |
| Timestamp jitter | 0, 5, 15, 30 ms standard deviation |
| Dropout | 0%, 5%, 10%, 20%, 40% |
| Outliers | 0%, 0.1%, 1%, 5%; 2--20 mm magnitude |
| Actuator gain | 1.0, 0.75, 0.5, 0.25, 0.1 per axis |
| Actuator delay | 0, 20, 50, 100, 200 ms |
| Deadband/backlash | zero, then values fitted from reversal data |
| `J0` column gain | 1.0, 0.75, 0.55, 0.5, 0.25, 0.1 |
| Rotation linear block | nominal, zero, sign reversal, fitted diagnostic block |
| Bending angular block | nominal, zero, 0.25x, 0.5x, direction perturbations |

For stochastic rows, run at least ten seeds. For deterministic model and
actuator rows, run both target signs because backlash and local Jacobian error
need not be symmetric.

## Prioritized sequence

1. Establish ten-seed ideal repeatability.
2. Add marker noise and latency only. This tests estimator robustness without
   model mismatch.
3. Keep perfect observations and sweep actuator gain/delay. This isolates
   achieved-motion mismatch.
4. Keep perfect observations and sweep `J0`, starting with 0.55 insertion and
   0.5 rotation-angular gains and a zero rotation-linear block.
5. Combine the best hardware-like actuator/Jacobian configuration with the
   measured marker-noise and latency profile.
6. Compare fixed `J0` with online RLS only after the perturbed fixed-model
   baseline is repeatable. Score pre-update predictions so adaptation cannot
   hide instability or data leakage.

The first scientific question is whether sensing perturbations alone can
reproduce hardware behavior. If not, compare actuator-only and Jacobian-only
runs. Do not tune all perturbations simultaneously; many combinations can
produce the same low observed tip gain and would not be identifiable.
