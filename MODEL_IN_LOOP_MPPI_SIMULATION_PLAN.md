# ROS model-in-the-loop MPPI simulation plan

## Objective

Build a non-hardware ROS 2 closed loop in which the deployed v171/v174 learned
catheter model is treated as the exact plant. Run the existing MPPI controller
and production control manager against that plant, with synthetic encoder,
position, marker, and marker-health feedback. Visualize the ground-truth
catheter, target, tracking error, and MPPI forecast in RViz.

The first milestone answers one narrow question: **can the current controller,
estimator, ROS lifecycle, and manager contracts reach targets when model error,
camera error, and physical actuator error are absent?** Later milestones add
controlled perturbations one at a time.

This is not a Gazebo dynamics project. The learned model is the dynamics model;
RViz is only the renderer. Adding another physics engine would confound the
baseline the simulation is intended to establish.

## Safety and isolation

The simulation must be incapable of reaching the physical serial bridge.

- Put every simulated endpoint under `/sim` with explicit remappings. The
  production nodes currently use absolute topic names, so a namespace alone is
  insufficient.
- Do not launch `device_serial_com.py` or either ZED marker node.
- Remap the controller's command and mode outputs to `/sim/teleop/*` before
  setting `command_output_enabled:=true`.
- Remap the manager's device side to `/sim/device/*`; the simulated device
  exposes no serial-port parameter and imports no serial package.
- Remap controller services and targets to `/sim/catheter_mppi/*` so an arm or
  target command cannot address a concurrently running hardware controller.
- Use a non-default `ROS_DOMAIN_ID` as a second barrier for experiments, but do
  not rely on domain isolation instead of topic remapping.
- Preserve all manager limit, freshness, watchdog, and fault behavior. The
  simulator must not expose an encoder-reference-changing operation.

Recommended experiment shell:

```bash
export ROS_DOMAIN_ID=42
```

## ROS graph

```text
 target helper / operator
           |
           v
 /sim/catheter_mppi/target_tip
           |
           v
 +----------------------+       /sim/teleop/control,event
 | existing catheter    |----------------------------------+
 | MPPI node            |                                  v
 | controller runtime A |                         +------------------+
 +----------------------+                         | existing manager |
     ^       |                                    +------------------+
     |       +-- predicted_tip, status, plan                |
     |                                                     | clamped
     | marker and device feedback                          | DeviceStream
     |                                                     v
 +-----------------------+ encoder counts +-------------------------+
 | simulated device      |---------------->| simulated perception   |
 | projection/integration|                 | independent runtime B   |
 +-----------------------+                 +-------------------------+
     |                                          |
     |                                          +-- ground-truth tip
     |                                          +-- markers/diagnostics
     +-- device state/event/transport

                         +-------------------+
 all truth/target/plan -->| RViz visualizer   |--> MarkerArray
                         +-------------------+
```

The controller and plant must load the same checkpoint and Jacobian artifacts,
but they must be distinct runtime instances with no shared mutable state. The
plant never consumes marker observations and never adapts its Jacobian. The
controller continues to estimate from synthetic observations exactly as it
does from cameras.

## Components

### 1. Pure model plant

Add `catheter_control/sim_plant.py` with a ROS-independent
`ModelInLoopPlant`:

- load `V171StreamingCatheterRuntime` using the same explicit artifact paths as
  the controller;
- initialize from configurable six-axis encoder counts, defaulting to physical
  home;
- hold the latest manager-approved logical velocity command with zero-order
  hold;
- use `HardwareContract.project_velocity()` to reproduce manager limits,
  firmware coupling, integer-RPM quantization, and deadbands;
- integrate the realized motor radians, quantize them to integer encoder
  counts, and integrate the realized six-axis logical joint position;
- advance the independent v171 runtime from those encoder counts at a fixed
  plant step;
- expose current four markers, tip, joint position, encoder counts, realized
  command, and simulation time;
- stop at hard position bounds and on malformed/non-finite input.

The initial implementation should be deterministic and ideal:

- fixed 10 ms plant step (100 Hz);
- no process noise, backlash beyond what is already in v171, delay, dropout,
  gain error, or measurement noise;
- `adaptation_enabled=False` in both plant and controller;
- fixed random seed for MPPI.

The most important unit test is one-step and multi-step equivalence between the
plant and `V171StreamingCatheterRuntime.predict_sequence()` for the same
projected motor commands. This catches unit, sign, coupling, count-rounding,
and timestep errors before ROS is involved.

### 2. Simulated device and perception ROS nodes

Use separate `catheter_sim_device` and `catheter_sim_perception` processes.
The split keeps the 100 Hz device/watchdog path independent from PyTorch model
latency and prevents learned-model work from starving manager feedback. They:

- subscribe to `/sim/manager/control` (`DeviceStream`);
- publish paired POS and ENC messages on `/sim/device/state` using the same
  nonzero header stamp for both;
- publish a marker `PointCloud` on `/sim/shape_tracking/markers` at 30 Hz with
  the four required channels:
  `marker_id`, `confidence`, `reprojection_error_px`, and
  `source_rig_count`;
- publish `automation/four_ring_markers: TRACKING` on
  `/sim/shape_tracking/marker_status`;
- publish transient-local `SERIAL_READY:SIMULATED_V171` on
  `/sim/device/transport_status` so the real manager lifecycle is exercised;
- implement `/sim/device/command` with the small firmware-compatible subset
  used by manager qualification: connection, stop, fault status, reset when
  appropriate, and six parseable driver diagnostics;
- publish `/sim/device/event` for relevant stop/fault events;
- publish ground-truth tip, joint state, realized command, and compact
  diagnostics under `/sim/catheter_sim/*`.

The service responses must satisfy the existing manager parsers rather than
adding simulation exceptions to the manager. STOP must immediately clear the
held command. Unsupported or unsafe predicates fail closed.

Use wall-clock-paced execution initially. The manager's watchdogs use steady
time, so `/clock` alone would not make the complete graph deterministic.
POS and ENC stamps come from one actuator tick and remain identical. Perception
consumes the latest ENC sample and stamps markers/truth with its model update.

The co-located simulation runs a second learned-model runtime that physical
hardware does not. Keep the production manager's 250 ms feedback watchdog
unchanged, but give the simulated controller a separately named, configurable
500 ms feedback/marker freshness window. This prevents host contention from
being mistaken for model/control failure; the hardware launch retains its
150 ms defaults. Record callback ages so this relaxation remains explicit.

### 3. Reuse the production manager

Launch `control_interface/manager.py` unchanged with explicit remaps for:

- `/teleop/control`, `/teleop/event`;
- `/manager/control`, `/manager/state`, `/manager/event`,
  `/manager/safety_status`, `/manager/qualify_driver_power`;
- `/device/state`, `/device/event`, `/device/transport_status`, and
  `/device/command`.

Use the real `imricor_test` limits file. Perform the normal simulated
qualification before arming. This tests the same readiness heartbeat,
arbitration, command clamp, stale-source stop, feedback watchdog, and fault
latching used on hardware.

Do not add a `simulation_mode` branch to the production manager. Contract
fidelity is stronger if the simulated device conforms to the manager instead.

### 4. Launch and target helper

Add `launch/simulation.launch.py` with arguments matching `control.launch.py`
for artifact paths, device, estimator, MPPI horizon/samples, and rates. It
starts:

1. simulated device and independent model/perception processes;
2. remapped production manager;
3. remapped production MPPI controller with
   `command_output_enabled:=true`;
4. visualization adapter;
5. optional RViz and rosbag recorder.

The launch must print a conspicuous `SIMULATION ONLY` banner and the complete
topic prefix. It must never include the serial bridge.
It requires a non-default `ROS_DOMAIN_ID` and takes an exclusive per-domain
file lock, so two simulation graphs cannot accidentally mix feedback.

The simulated truth process starts from an independently configurable
`truth_initial_interface_pose`. Its default is the first-posterior v171 pose,
which gives the nominal material frame a realistic non-identity placement in
`robot_base`. This pose is never passed to the controller: the controller must
recover material roll from marker observations and the loaded distal model.
Changing this truth-only parameter tests startup in a different configuration
without changing the controller prior.

Add `catheter_sim_target`, a small command-line helper that waits for the
current ground-truth tip and publishes an absolute target from a requested
offset. Examples for uncoupled experiments should look like:

```bash
ros2 run catheter_control catheter_sim_target --dx-mm 5
ros2 run catheter_control catheter_sim_target --dy-mm 5
ros2 run catheter_control catheter_sim_target --dz-mm 5
```

Exactly one offset should be nonzero unless an explicit combined target is
requested. The helper publishes only into `/sim` by default.

### 5. RViz visualization

Add `catheter_control/sim_visualizer.py` and
`config/mppi_sim.rviz`. Publish a `visualization_msgs/MarkerArray` containing:

- four ground-truth marker spheres and a line strip through them;
- a distinct sphere for the ground-truth/observed tip;
- a target sphere;
- an arrow from current tip to target, colored by error magnitude;
- the MPPI predicted-tip sequence as a separate line strip;
- optional text with error norm, active command, controller state, and model
  artifact hashes.

Use `robot_base` as the fixed frame and metres as visualization units. Preserve
the four measured points as the authoritative catheter representation for the
minimal version; a dense centerline can be added later through a public
`current_curve()` method in `cr_meta_lnn` if it materially improves inspection.

Also publish numeric topics suitable for `rqt_plot` or Foxglove:

- `/sim/catheter_sim/tip_error_mm` (`Vector3Stamped`);
- `/sim/catheter_sim/ground_truth_tip` (`PointStamped`);
- `/sim/catheter_sim/joint_states` (`JointState`).

## Implementation sequence

### Phase S0 — contract and isolation tests

- Centralize the `/sim` remapping table in the simulation launch file.
- Add a launch/static test proving no simulated publisher or subscriber uses
  the unprefixed `/manager/control`, `/teleop/control`, or `/device/*` graph.
- Verify the launch contains no serial executable or serial-port argument.

### Phase S1 — ideal plant core

- Implement pure command projection, motor/count integration, and independent
  v171 stepping.
- Test zero hold, signs, units, coupling, count quantization, hard limits,
  clone isolation, and equivalence to direct rollout.

### Phase S2 — ROS feedback and manager emulation boundary

- Publish synchronized ideal POS/ENC and perfect four-marker observations.
- Implement the simulated device service/transport contract.
- Run the unmodified manager through qualification and confirm continuous
  `MANAGER_READY` without the controller.

### Phase S3 — closed-loop MPPI

- Add the remapped controller and target helper.
- First run at home with a target equal to the initial tip.
- Arm, then test separate reachable `+x`, `-x`, `+y`, `-y`, `+z`, and `-z`
  offsets, starting at 5 mm and returning to the baseline between trials.
- Record the commanded direction, error norm, settling behavior, effective
  sample count, deadline statistics, and limit activity.

Before interpreting a failed Cartesian direction, compute the local tip
controllability from the model/Jacobian. A target outside the locally reachable
subspace is not an MPPI failure.

### Phase S4 — visualization and automated scenario runner

- Add RViz markers/configuration and numeric plot topics.
- Add a scenario runner for repeatable axis-separated targets and seeded MPPI.
- Save bags and JSON summaries below
  `/media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions`.

### Phase S5 — controlled robustness experiments

Only after the ideal baseline passes, independently enable:

1. marker noise;
2. marker latency and jitter;
3. frame dropout;
4. encoder quantization/noise;
5. actuation lag and gain error;
6. model/Jacobian mismatch;
7. bounded online adaptation.

The implementation details, calibrated sweep ranges, scenario metrics, and
regression gates are maintained in `SIMULATION_ROBUSTNESS_TEST_PLAN.md`.

Never enable more than one new perturbation in the first run. This gives a
direct attribution for each degradation or gate failure.

## Verification and acceptance criteria

### Unit and integration checks

- Plant rollout matches direct v171 prediction to numerical/count-quantization
  tolerance.
- POS and ENC share a timestamp and remain causally ahead of or equal to marker
  timestamps.
- Perfect markers initialize and remain accepted without estimator drift.
- Every manager-forwarded command is projected exactly once into the realized
  simulated motor command.
- STOP, source timeout, feedback timeout, malformed command, and hard-limit
  tests all produce zero motion and the expected fault/status transition.
- Fixed MPPI seed produces repeatable command and trajectory traces.
- Existing hardware launch and tests remain unchanged.

### Ideal closed-loop pass

- no manager, marker, freshness, estimator, or planner fault;
- no planning deadline miss after warmup;
- observed and ground-truth tip agree to numerical precision;
- the first-step predicted displacement agrees with the next simulated
  measurement within count/timestep tolerance;
- every locally reachable axis-separated target reduces tip-error norm by at
  least 80 percent and reaches a configurable terminal band (initially
  0.5 mm) without violating joint limits;
- zero target and unreachable-direction tests do not create meaningless
  sustained motion;
- stopping or killing the controller causes the manager and plant to reach
  zero command through the normal watchdog path.

## Interpretation of results

- **Ideal simulation fails:** debug controller cost, controllability, model
  state initialization, estimator, unit conversion, sampling, or ROS lifecycle
  before further hardware experiments.
- **Ideal simulation passes but perturbed simulation fails:** quantify the
  specific sensitivity and tune estimation/control against that perturbation.
- **Perturbed simulation passes but hardware fails:** focus on unmodeled
  mechanics, camera calibration, actuator response, encoder integrity, or
  timing differences measured in hardware bags.
- **Both pass with similar transients:** proceed to cautious hardware targets
  using the same axis-separated scenario and compare the recorded traces.

## Minimal useful delivery

The quickest useful version is S0 through S3 plus a basic RViz configuration:
one ideal model plant node, the existing remapped manager and controller, a
target-offset helper, four-marker/target/forecast visualization, unit tests,
and one launch file. Fault injection, dense catheter rendering, `/clock`, and
batch scenario reports can wait until the ideal closed loop is demonstrated.
