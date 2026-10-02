# Causal proximal identification experiment

This implementation executes the experiment designed in
`HYSTERESIS_AWARE_PROXIMAL_CONTROL_AND_CAUSAL_EXPERIMENT_PLAN.md` through the
existing guarded automation command path. It does not alter encoder zero and
does not issue a zeroing command. The v5 runner also exposes the Phase 1--3
isolation schedules from `NEXT_HARDWARE_ISOLATION_TEST_PLAN.md`:

- `stationary`: 15 seconds of zero command at the measured run start;
- `insertion`: Phase 2A only;
- `tendon_motor`: Phase 2B raw shaft-2 isolation only;
- `compensated_bend`: Phase 2C production coupling only;
- `phase_0_2`: stationary bookends plus 2A, 2B, and 2C;
- `timing` / `phase_3`: physical shaft-0/shaft-2 onset isolation with
  simultaneous and 20/40/80 ms staggered commands;
- `insertion_rotation` / `phase_2d`: signed torsional preloads followed by
  bidirectional insertion sweeps while the rotation motor is held fixed;
- `compensated_bend_insertion_sweep` / `phase_2e`: compensated full-range
  bending at fixed 0, 13.333, 26.667, and 40 mm insertion plateaus;
- `full`: the original insertion/rotation/tendon/compensated/mixed suite.

The Phase-2E insertion-stratified tendon session uses two continuous-history
passes.  The first visits the four insertion levels in increasing order and
the second in decreasing order.  Every level receives one 2 mm/s and one
4 mm/s bend cycle, with opposite branch order, while logical insertion is
held fixed.  Each cycle covers the complete `0 -> 15 mm` bend range through a
7.5 mm bias.  The generated trajectory duration is 413.3125 seconds
(6 minutes 53.3 seconds), excluding launch preflight and bag finalization.

The insertion/rotation schedule is specifically intended to measure whether
linear insertion releases stored torsion. Every actuating launch first moves
to `[20,0,0]`. Each labelled trial then rotates to either `+75` or `-75` deg,
holds the rotation command fixed, sweeps insertion through
`20 -> 26 -> 20 -> 14 -> 20 mm`, and returns rotation to zero. Positive and
negative preload order alternates across three repetitions at both slow and
fast speeds. Encoder zero is never changed, and returning the commanded joints
to `[20,0,0]` is not treated as proof that hidden torsion was reset.

## Initialization invariant

Every actuating causal isolation run (`start_motor:=true`) first uses the
manager's guarded absolute-position path to move the catheter joints to
`[20 mm, 0 deg, 0 mm]`. The experiment does not publish `run_start` or build
its trajectory until POS/ENC feedback is within
`[0.1 mm, 0.5 deg, 0.05 mm]`, remains settled, and passes the normal stationary
and estimator preflight again at that configuration. A rejected, timed-out,
faulted, or stale initialization aborts the run fail-closed.

This phase moves the mechanism but never changes encoder zero. Non-actuating
dry runs (`start_motor:=false`) do not perform initialization motion.

## What it runs

The runner builds its complete trajectory only after receiving fresh,
stationary POS and raw ENC feedback. It applies its own 15 mm insertion margin
to the selected hard-limit profile; with the current `[-10,50] mm` recovery
profile, purposeful excitation is confined to `[5,35] mm`. At the normal
20 mm insertion start it
runs 29 labelled episodes in about 423 seconds:

- 15 second static baseline;
- insertion `[v,0,0]`, two speeds and three repetitions;
- rotation `[0,v,0]`, two speeds and three repetitions, including relaxation;
- an interior bend-bias setup;
- raw bend-shaft isolation `[v,0,v]`, two speeds and three repetitions;
- production compensated bend `[0,0,v]`, two speeds and three repetitions;
- one held-out mixed validation episode;
- return from bend bias and a final 15 second static baseline.

The raw bend-shaft block enforces `u_lin == u_bend` after feedback shaping, not
only in the nominal trajectory. Since firmware computes raw shaft-0 velocity
as `u_lin-u_bend`, this preserves the intended single-shaft experiment.

The configured amplitudes are half-excursions about the experiment center,
not per-control-cycle steps. The default insertion block, for example, follows
`20 -> 22 -> 20 -> 18 -> 20 mm`, for a 4 mm peak-to-peak range. Motion is
smoothly interpolated at 100 Hz. The console prints each episode START/END,
its physical excitation basis, speed tier, repetition, and duration; the same
boundaries remain machine-readable on `/collection/events`.

The launch refuses to start unless:

- firmware fault status is clean and motors are disabled;
- POS and ENC are fresh, stable, finite, and in range;
- the catheter UKF is tracking with at least eight accepted observations;
- marker diagnostics report `TRACKING`;
- the MPPI controller is disarmed;
- online Jacobian adaptation is disabled.

During motion, confirmed firmware faults, stale estimator status, or repeated
marker rejection stop the run. The operator abort endpoint is:

```bash
ros2 service call /collection/abort std_srvs/srv/Trigger "{}"
```

For experiments that record synchronized dual-camera SVO and reconstruct the
shape offline, keep `marker_tracking` running as the camera owner and online
diagnostic, but launch collection with
`require_estimator_tracking:=false`. This makes accepted-marker count, UKF
health, and marker rejection non-fatal; fresh controller status, a disarmed
controller, and disabled online adaptation remain mandatory. POS/ENC,
manager, firmware, limit, watchdog, and fault gates are unchanged.

For a simulation run the remapped endpoint is `/sim/collection/abort`.

Abort stops without commanding an automatic return. Normal completion returns
to the measured run-start POS with the qualified position transaction.

## Build

```bash
cd /home/chen-lab/Yifan/robot-infra
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select control_interface catheter_control control_tasks bringup
source install/setup.bash
```

## Model-in-loop test

Use a separate ROS domain so simulation cannot contact hardware nodes. Start
the model-in-loop stack with MPPI left disarmed and recording disabled:

```bash
export ROS_DOMAIN_ID=42
ros2 launch bringup simulation.launch.py \
  record:=false \
  marker_estimator:=ukf \
  auto_qualify:=true
```

Then start the experiment in a second terminal in the same domain:

```bash
export ROS_DOMAIN_ID=42
cd /home/chen-lab/Yifan/robot-infra
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch bringup causal_experiment.launch.py \
  use_sim:=true \
  start_motor:=true
```

The simulated plant starts at zero insertion. The experiment runner first
moves it to the configured 20 mm interior center, runs all identification
episodes with the same `[5,35] mm` insertion envelope used for hardware, and
returns it to the measured simulation start. This centering move is enabled
only by `use_sim:=true`; a hardware launch never enables it implicitly.

The isolation recorder uses the Humble-standard `sqlite3` backend. Before the
runner starts, it snapshots the effective parameters and model artifacts of
the already-running MPPI estimator. After the runner exits, the launch sends
SIGINT to rosbag, waits for `metadata.yaml`, and runs the completeness checker.
Only a session whose `completeness.json` reports `PASS` is accepted.

Simulation perturbations such as actuator lag and reversal backlash are set on
`simulation.launch.py`; the experiment launch records the perturbed realized
motion and the accepted UKF posterior.

To verify that a deliberately larger finite excursion remains observable in a
perturbed simulation before selecting any hardware amplitudes, use a separate
simulation run such as:

```bash
ros2 launch bringup causal_experiment.launch.py \
  use_sim:=true \
  start_motor:=true \
  amplitudes:=4.0,40.0,2.5 \
  bend_bias_position:=4.0
```

This is an explicit test profile, not the hardware default. Its endpoint
ranges remain inside the configured interior envelope when centered at 20 mm
insertion and 4 mm bend.

## Hardware run

Start the manager/serial stack and marker tracker using the established
hardware procedure. Qualify driver power and require `MANAGER_READY`. Start a
separate estimator-only controller process:

```bash
ros2 launch bringup control.launch.py \
  controller_config:=$(ros2 pkg prefix catheter_control)/share/catheter_control/config/v174_fixed_hardware_no_rotation.yaml \
  marker_estimator:=ukf \
  adaptation_enabled:=false \
  command_output_enabled:=false \
  record:=false
```

Confirm that `/catheter_mppi/status` reports `armed=False`,
`estimator_health=TRACKING`, `marker_diagnostic=TRACKING`, and
`model_adaptation_enabled=False`. Then run:

```bash
ros2 launch bringup causal_experiment.launch.py \
  schedule:=stationary \
  static_s:=15.0 \
  start_motor:=true
```

Analyze the Phase-1 noise floor after the launch prints a passing completeness
result:

```bash
ros2 run runtime_supervision causal_stationary_analysis \
  /media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions/SESSION_NAME
```

Run Phase 2 as three independently reviewable sessions. Re-run the normal
manager readiness and stationary estimator checks before each command:

```bash
# Phase 2A: logical insertion / raw shaft 0.
ros2 launch bringup causal_experiment.launch.py \
  schedule:=insertion start_motor:=true

# Phase 2B: equal logical insertion+bend, cancelling raw shaft 0.
ros2 launch bringup causal_experiment.launch.py \
  schedule:=tendon_motor start_motor:=true

# Phase 2C: logical bending through the production compensation mapping.
ros2 launch bringup causal_experiment.launch.py \
  schedule:=compensated_bend start_motor:=true
```

The `tendon_motor` session is also the preferred raw-tendon model check. It
commands equal logical insertion and bend, so the firmware mapping holds raw
shaft 0 fixed and moves only raw shaft 2. Preserve the finalized bag and its
runtime identity for comparison against a v171 preview; use the recorded
encoder trajectory rather than the nominal command so actuator timing and
incomplete position transactions are not mislabelled as learned-model error.
The experiment retains continuous tendon history across every labelled
episode.

For a single uninterrupted reviewed run, `schedule:=phase_0_2` combines those
three blocks but deliberately excludes rotation and mixed validation.

Run the insertion-stratified compensated-bending experiment as its own
session:

```bash
ros2 launch bringup causal_experiment.launch.py \
  schedule:=phase_2e \
  require_estimator_tracking:=false \
  start_motor:=true
```

The defaults are the reviewed two-pass design: insertion plateaus
`0,13.333,26.667,40 mm`, bend sweep `0..15 mm`, 7.5 mm bias, one slow and one
fast visit per plateau, and 3 seconds of stationary data before each measured
cycle.  Override these only through the explicit `insertion_plateaus`,
`insertion_plateau_visits`, `insertion_plateau_dwell_s`, and
`tendon_sweep_limits` launch arguments.  The current implementation requires
exactly two visits so speed and branch order remain balanced.

## Insertion/rotation coupling isolation

Use the dedicated `insertion_rotation` (`phase_2d`) schedule to test whether
insertion releases stored torsion. It uses the ordinary manager-mediated
command path and the same guarded `[20,0,0]` initialization. For each slow/fast
speed tier and three repetitions, it performs both signed conditions:

1. rotate from zero to `+75` or `-75` deg with insertion and bend fixed;
2. hold the rotation motor reference fixed;
3. sweep insertion `20 -> 26 -> 20 -> 14 -> 20 mm`;
4. return the rotation reference to zero and dwell before the next condition.

Feedback shaping is projected back onto the intended basis before publication:
rotation and bend commands are exactly zero during insertion probes, while
insertion and bend commands are exactly zero during rotation setup/unwind.
Positive/negative order alternates between repetitions and speed tiers. The
default schedule has 38 labelled episodes and lasts approximately 712 seconds
(11.9 minutes). Remain at the robot until normal return and final bag
completion; do not leave an actuating experiment unattended.

```bash
ros2 launch bringup causal_experiment.launch.py \
  schedule:=insertion_rotation \
  amplitudes:=6.0,75.0,0.0 \
  minimum_amplitudes:=5.0,65.0,0.0 \
  repeats:=3 \
  start_motor:=true
```

Returning the encoders to `[20,0,0]` separates commanded trials but is not
interpreted as a reset of hidden torsional state. The recorded estimator trace,
raw markers, source/manager/device commands, POS/ENC, and episode boundaries
are sufficient to compare interface-roll and distal-shape response during the
fixed-rotation insertion probes remotely.

## Phase 3 timing isolation

Phase 3 uses the same guarded `[20,0,0]` initialization and the same interior
tendon bias as Phase 2. It is generated by one collection node at 100 Hz; do
not reproduce it with separate topic publishers. The v6 runner constructs one
constant, reliable-speed pulse per active **physical shaft**, converts that
raw command to logical coordinates once, and verifies before publication that
the manager-equivalent projection preserves it. A changed mask, magnitude, or
onset aborts the run to zero. For every slow/fast speed tier and three
order-balanced repetitions it records:

- insertion shaft only;
- tendon shaft only;
- simultaneous shaft onset;
- insertion leading tendon by 20, 40, and 80 ms;
- tendon leading insertion by 20, 40, and 80 ms.

Each measured move is monotonic and starts from the same bias after a known
opposite-direction recovery. The default run measures the positive direction.
Use a separate reviewed session with `timing_direction:=-1` when the negative
direction is needed; mixing directions inside one run would confound onset
timing with a different preconditioning history.

After reviewing the Phase 0--2 session, run the positive timing experiment:

```bash
ros2 launch bringup causal_experiment.launch.py \
  schedule:=phase_3 \
  timing_direction:=1 \
  timing_leads_ms:=20.0,40.0,80.0 \
  start_motor:=true
```

Evaluate the finalized session with the normal evaluator. Its
`phase3_timing` section reports each event chain and grouped median, P95, and
bootstrap median confidence intervals:

```bash
python3 audits/model-validation/evaluate_causal_proximal_experiment.py \
  /media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions/SESSION_NAME
```

The recorded chain is source command, manager forwarding, serial transmit,
credible encoder motion, and credible UKF interface/distal response. The
evaluator checks source/manager/transmit pulse topology and integrated raw
travel before qualifying a Phase 3 session. Encoder polarity is applied in
physical motor coordinates; encoder and estimator response thresholds are
derived from the session's static bookends and require persistence across two
samples.

Data are written under
`/media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions` in a timestamped
`*_causal_proximal` directory. The bag contains native commands, manager
projection, firmware command trace, POS/ENC, raw markers, per-command episode
labels, and the accepted observation-time UKF interface pose/strain/covariance
trace.

## Chassis backdrive from knob actuation

The short `chassis_knob_backdrive` schedule tests whether tightly driven knob
motion physically backdrives the rack-and-pinion chassis. It initializes at
`[20,0,0]`, then executes exactly four labelled moves:

1. chassis backward by 10 mm;
2. knob forward by 5 mm while commanding zero raw chassis-shaft motion;
3. chassis forward by 10 mm while the knob remains bent;
4. knob backward by 5 mm while commanding zero raw chassis-shaft motion.

The complete motion, including five-second visual dwell periods and static
bookends, takes about 58 seconds. The equal logical lin/bend commands during
the knob moves are enforced after feedback correction so
`raw0 = lin - bend = 0`; any visible chassis displacement during those two
episodes is therefore a mechanical backdrive response, not a requested chassis
move.

```bash
ros2 launch bringup causal_experiment.launch.py \
  schedule:=chassis_knob_backdrive \
  amplitudes:=10.0,0.0,5.0 \
  minimum_amplitudes:=8.0,0.0,4.0 \
  static_s:=5.0 \
  endpoint_dwell_s:=5.0 \
  start_motor:=true
```

Stop the launch immediately if the carriage approaches an obstruction or the
manager leaves `MANAGER_READY`. The schedule never issues an encoder-zero
operation and uses the ordinary manager-mediated position and velocity paths.

## Evaluate a completed run

```bash
cd /home/chen-lab/Yifan/robot-infra
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 audits/model-validation/evaluate_causal_proximal_experiment.py \
  /media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions/SESSION_NAME
```

The evaluator creates `causal_experiment_summary.json`. Its 0.25, 0.5, and
1.0 second response windows use only past accepted UKF states and never cross
an episode boundary. Results are grouped by physical excitation basis, speed,
repetition, direction, and horizon so complete repetitions can be reserved for
held-out validation.
