# Handoff: hardware-compatible catheter identification generator

Date: 2026-08-27

Owner for this handoff: robot-side automation agent

## Outcome requested

Add a hardware-safe `identification` collection mode for the catheter robot.
It must generate one continuous, episode-labelled trajectory that separates
the effects of catheter insertion, axial rotation, and tendon/bend actuation,
then excites their interactions.  The output is intended for system
identification and recurrent dynamics training.  A separately collected,
fixed-seed sinusoidal run will be held out as validation and must not be mixed
into the identification data.

This is not a request to run an existing SOFA generator unchanged.  The useful
experiment design should be retained, while units, amplitudes, durations,
velocity floors, feedback tracking, safety checks, and metadata must follow the
real robot contract.

## Relevant existing code

Robot-side collection:

- `automation/collection/node.py`
- `launch/collection.launch.py`
- `config/catheter_limits.yaml`
- `test/test_collection_faults.py`
- `DATA_COLLECTION_DESIGN.md`

Simulation generators that informed the design:

- `simulation/data_collection/generators/proximal_curriculum.py`
  (`SlowProximalCurriculumGenerator`)
- `simulation/data_collection/generators/proximal_identification.py`
  (`ProximalIdentificationGenerator`)
- `simulation/data_collection/generators/sinusoidal.py`
  (`SinusoidalGenerator`)

The slow curriculum has the best identification structure: isolated ports,
matched paths at different rates, repeated hysteresis loops, opposite-history
revisits, settling intervals, and coupled motion last.  The shorter proximal
identification generator adds chirps, ring-downs, and pairwise interactions.
Use these ideas, not their literal numerical constants.

## Why the existing generators cannot be used directly

1. The slow curriculum documents metres/degrees/newtons and hard-codes an
   insertion cap of `0.040`.  With real limits expressed in millimetres this
   becomes only `0.04 mm`.
2. The proximal-identification trajectory is not actually speed-safe under the
   current `imricor_test` limits.  Sampling it at 100 Hz gives approximate peak
   rates of `26.46 mm/s`, `133.40 deg/s`, and `2.38 mm/s`, versus configured
   maxima of `10 mm/s`, `40 deg/s`, and `1 mm/s`.
3. The SOFA third coordinate denotes tendon force in several scenes.  On the
   robot it is catheter bend motor position in millimetres.  Do not call it
   force or tension in hardware metadata.
4. `automation.collection.node` currently builds only `SinusoidalGenerator`,
   and its reference tracking and finite-difference logic explicitly checks
   `mode == "sinusoidal"`.
5. The local SOFA sinusoidal class stores `_ramp_dur`, while the robot node logs
   `self._gen.ramp_duration`.  Remove this fragile private/public version
   mismatch while generalizing the generator interface.

## Coordinate, input, and state contract

The generated target is a relative physical-joint position reference

```text
delta_q(t) = [catheter_lin_mm, catheter_rot_deg, catheter_bend_mm]
```

relative to the measured POS state at `run_start`.  The robot remains in
`JOINT_VEL` mode.  The collection node realizes this reference with bounded
feed-forward velocity plus its existing floor-aware position tracking.

The run-start reference is

```text
q_ref(t) = q_start + delta_q(t).
```

Requirements:

- The generator should expose an **unwrapped** rotation reference internally.
  Apply circular differences only when computing rotation tracking error or a
  finite-difference velocity.
- Validate all planned extrema against the selected catheter profile before
  enabling motors.  Do not silently clip the reference: clipping changes the
  experiment and invalidates matched-path comparisons.
- Continue recording commanded velocity, POS, and raw ENC.  Learning may use
  either feedback representation later; the generator must not bake a nominal
  transmission ratio into the recorded data.
- The three catheter joints are actuated; sheath joints remain fixed.
- A subepisode boundary does **not** reset physical or learned hysteresis state.
  This must be one continuous run.  A zero-command dwell settles velocity but
  is not claimed to erase backlash or material memory.

## Generator interface

Prefer a robot-owned implementation, for example
`automation/collection/identification.py`, instead of importing a
hardware-specific subclass from the simulation repository.  A small generic
position-generator protocol is sufficient:

```python
class PositionReferenceGenerator(Protocol):
    @property
    def duration(self) -> float: ...
    @property
    def episode_names(self) -> list[str]: ...
    @property
    def episode_starts(self) -> list[float]: ...
    @property
    def metadata(self) -> dict: ...
    def relative_position(self, time_s: float) -> np.ndarray: ...  # shape (3,)
    def is_done(self, time_s: float) -> bool: ...
```

Generalize the existing finite-difference and floor-tracking path to operate on
any position-reference generator, not just on a string comparison with
`sinusoidal`.  Preserve `constant` mode as a separate bring-up path.

## Normalized paths and automatic timing

All episode paths must start and finish with zero position velocity.  One useful
out-and-back scalar path is

```text
p(tau) = 0.5 * (1 - cos(2*pi*tau)),  tau in [0,1].
```

For amplitude `A` and duration `T`, its peak speed is `pi*A/T`.  Select duration
from the path derivative rather than choosing a duration first:

```text
T >= pi*A / v_planned.
```

Use no more than approximately 70--80% of the configured maximum speed for the
feed-forward path, leaving authority for tracking correction.  Check the
sampled finite-difference trajectory at the actual command rate as a final
guard.

For two-rate experiments, choose reliable physical speeds rather than arbitrary
duration ratios:

```text
v_slow   = max(vel_min, approximately 0.3*vel_max)
v_medium = approximately 0.7*vel_max
```

Only label an episode pair as a rate pair if the hardware has a meaningful
reliable interval (suggested requirement: `v_medium >= 1.5*v_slow`). With the
direct-drive catheter-bending shaft, the current `imricor_test` profile uses a
2.0--4.9 mm/s reliable interval and supports a clean slow/medium damping
comparison. The 2 mm/s floor also keeps the coupled catheter-linear motor at
or above its reliable operating speed.

## Planned experiment suite

Build the episode list from configured amplitudes and automatically computed
durations.  The following order is recommended.

1. `settle_start`
   - Hold all relative references at zero for at least 5 s.
2. `insertion_out_back_slow`
   - Bend and rotation fixed at their run-start values.
3. `settle_after_insertion_slow`
4. `insertion_out_back_medium`
   - Identical position path and amplitude to the slow episode.
5. `settle_after_insertion_medium`
6. `rotation_positive_negative_slow`
   - Visit positive and negative rotation branches and return to start.
7. `settle_after_rotation_slow`
8. `rotation_positive_negative_medium`
   - Same position path at the second reliable rate.
9. `settle_after_rotation_medium`
10. `bend_out_and_back`
    - Execute the bend path at the slow reliable rate.
11. `dwell_after_bend_slow`
12. `bend_medium`
    - Repeat the identical bend path at the second reliable rate when enabled.
13. `dwell_after_bend_medium`
14. `bend_hold_unload_relaxation`
    - Smooth load, nonzero hold, smooth unload, then a zero-reference dwell.
      This is a relaxation experiment, not an instantaneous external-force
      ring-down.
15. `bend_repeated_loops`
    - At least two identical load/unload loops to expose repeatability,
      backlash, and rate-independent memory.
16. `bend_at_mid_insertion`
    - Establish and hold an insertion plateau before repeating bend loading.
17. `insertion_bend_interaction`
18. `rotation_bend_interaction`
19. `insertion_rotation_interaction`
20. `repeated_state_opposite_history`
    - Revisit the same three-joint target through at least two distinct paths.
21. `coupled_persistent_excitation`
    - Smooth, modest-amplitude, incommensurate motion in all three channels.
      This comes last so isolated identification remains available if the run
      is interrupted.
22. `settle_end`
    - Return the relative reference to zero and hold for at least 5 s before the
      existing return controller begins.

Do not add abrupt chirps until the speed-safe isolated suite has been verified
on hardware.  A later optional chirp may be useful for bandwidth identification,
but its derivative and acceleration must be checked explicitly.

## Default amplitude policy

Amplitudes must be launch/config parameters and must be reduced automatically
when run-start position leaves insufficient room.  Reasonable initial maxima
for the current catheter are:

```text
insertion amplitude: 20 mm
rotation amplitude:  100--120 deg on each safe branch
bend amplitude:      6 mm
```

These are suggested experiment amplitudes, not new safety limits.  Compute the
actual amplitude from `q_start`, `pos_lower`, `pos_upper`, a configurable margin,
and the requested maximum.  Abort before motor enable if the remaining safe
excursion is below a configured minimum useful amplitude.  Never obtain an
apparently safe experiment by clipping individual samples.

An optional rotation wrap-crossing episode may be added only if the configured
hardware range and firmware semantics genuinely permit continuous travel
through the reporting wrap.  It should be disabled by default.  Do not claim a
pi-crossing experiment when a position limit truncates the path below 180 deg.

## Episode events and provenance

Publish an event on `/collection/events` whenever an episode begins and ends.
Each JSON marker should include at least:

```json
{
  "event": "episode_start",
  "generator": "hardware_identification_v1",
  "episode_index": 3,
  "episode_name": "insertion_out_back_medium",
  "planned_start_s": 52.0,
  "planned_duration_s": 18.0,
  "stamp_ns": 0
}
```

The timer may step over a nominal transition; publish every crossed boundary
exactly once.  Do not infer episode boundaries offline only from elapsed time,
because scheduling delays, pauses, faults, and command clipping can alter the
real experiment.

Include the complete resolved generator plan in `run_start` metadata:

- generator name and schema/version;
- episode names, start times, durations, path types, amplitudes, and planned
  rates;
- run-start POS and raw ENC;
- position limits, reliable minimum speeds, maximum speeds, and safety margin;
- command rate and finite-difference convention;
- whether floor-aware tracking is enabled and its parameters;
- rotation wrapping convention;
- software commit identifiers when available.

There are more than ten episode events, while the current transient-local marker
publisher retains only ten messages.  Increase the marker history sufficiently
or otherwise ensure all early episode markers are retained for a late-starting
recorder.

## Launch integration

Add `mode:=identification` to `collection.launch.py`.  Expose at least:

- identification insertion/rotation/bend maximum amplitudes;
- safe position margin;
- settle duration;
- slow and medium speed fractions or explicit per-joint planned speeds;
- enable/disable flags for coupled, opposite-history, and optional wrap tests;
- generator version/seed (a seed is used only for coupled multisine phases);
- an optional maximum run duration that may reject but must not silently
  truncate the generated suite.

For identification mode, the generator's resolved duration should be
authoritative.  The current generic `duration_s=10` default must not terminate
the run early.  Either make `duration_s=0` mean "use generator duration" or
validate that a supplied duration is at least the generated duration.

The existing safe lifecycle remains mandatory:

- fresh stationary POS/ENC preflight;
- no motion unless `start_motor:=true`;
- bounded velocity commands;
- stop on confirmed device fault;
- explicit zero velocity before changing control mode;
- encoder-qualified return controller;
- zero command on shutdown and exception paths.

## Validation trajectory remains separate

The newly collected sinusoidal trajectory is a whole-session validation set.
Use a fixed seed not used for identification/training and preserve its exact
limits and generator parameters in metadata.  Do not add its frames to the
identification training split.

While touching generator integration, fix the `ramp_duration` API mismatch and
ensure `sofa_sim_path` cannot silently select an unknown external generator
version.  Prefer recording a generator version/hash, or own the sinusoidal
implementation in the robot package as well.

## Required tests

Add pure-Python tests that do not require connected hardware.

1. **Bounds and endpoints**
   - Sample the entire plan at 100 Hz.
   - Every value is finite and inside the planned relative/global bounds.
   - The first and final references are exactly zero relative displacement.
2. **Velocity safety**
   - Circularly difference rotation.
   - Every feed-forward rate stays below its planned margin and configured
     maximum before the node's final clamp.
   - Assert that the nominal `imricor_test` plan requires no clipping.
3. **Rate-pair identity**
   - Slow and medium paths have the same normalized position samples after
     phase normalization and differ only in timing.
   - Do not generate a false bend rate pair when `vel_min == vel_max`.
4. **Continuity**
   - References and intended velocities are continuous at ordinary episode
     boundaries; holds begin with zero intended velocity.
   - Rotation finite differences contain no wrap spike.
5. **Run-start rebasing**
   - Test starts near lower, middle, and upper limits.
   - Plans are reduced safely or rejected before enable; no sample clipping.
6. **Floor-tracker integration**
   - Simulate measured position with the existing floor-aware tracker.
   - Verify bounded commands, explicit stops before reversal, and acceptable
     turning-point tracking.
7. **Markers**
   - Every episode start/end is emitted once, in order, even when a timer tick
     crosses a boundary.
8. **Lifecycle/fault regression**
   - Existing preflight, fault, return, and shutdown tests still pass.
9. **Sinusoidal regression**
   - Existing sinusoidal collection still constructs with the current local
     generator API and records its resolved parameters.

## Acceptance criteria before a full collection

1. Unit tests pass using the checked-in `imricor_test` profile.
2. A dry run with `start_motor:=false` publishes the expected episode markers
   and finite, bounded commands.
3. A low-amplitude hardware run visually and numerically confirms reference
   tracking, direction reversals, and return-to-start.
4. The bag contains POS, raw ENC, velocity commands, faults/events, `run_start`,
   all episode markers, `return_start`, and `run_end`.
5. A post-run audit reports actual position/rate ranges per episode, command
   clipping counts, tracking RMSE, stale-feedback intervals, and any fault.
6. Only after this audit should amplitudes be raised to the identification
   defaults and synchronized ZED collection performed.

## Non-goals for the first implementation

- Do not estimate forces or tendon tension online.
- Do not assume zero motor command erases hysteresis.
- Do not add an external tip-release experiment to this run.
- Do not optimize the trajectory using the current learned model.
- Do not use the held-out sinusoidal validation session for fitting amplitudes,
  delays, hidden states, or model parameters.
