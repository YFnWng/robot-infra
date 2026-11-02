# Simulation

Driver-native streamed-position MPPI is an opt-in execution test: pass
`mppi_execution_mode:=position_stream` to the existing simulation launch. The
typed endpoint topic and guarded session service are explicitly remapped under
`/sim`; no serial bridge is launched. Build `control_interface` first: its native
`libcatheter_position_stream.so` runs the actual firmware executor/session owner,
not a duplicated Python servo. The plant idealizes native endpoint execution;
it does not model driver acceleration or ACK/stop timing. Default execution remains velocity. Stream mode
currently supports MPPI with no compensation or the shared explicit take-up
transactions; AdapJ, legacy feedforward-only compensation and synchronized
rotation are not admitted. See
`audits/ros-realtime/MPPI_DRIVER_NATIVE_POSITION_INTEGRATION.md` for verification and
hardware qualification boundaries. No hardware precision claim follows from
this exact/perturbed actuator simulation.

This ROS 2 package owns the isolated actuator, plant, perception, scenario,
target, and RViz simulation runtimes. Simulation endpoints remain under `/sim`;
the package depends one-way on the controller core for shared safety contracts.

Launch the complete isolated stack through `bringup`:

```bash
ros2 launch bringup simulation.launch.py
```

Plain two-axis AdapJ can use the same plant and RViz launch with
`stack_config:=simulation_adapj_plain_twoaxis` and a fresh
`adapj_initialization_dir`. Initialize explicitly before reaching; all manager,
device, task and initialization endpoints must remain under `/sim`. See
[AdapJ simulation commands](../../../repository_archives/cleanup_20261102/files/robot-infra/docs/research/ADAPJ_TIP_REACHING.md#existing-rviz-simulation).
For the matched compensated variant, select `simulation_adapj_takeup_twoaxis`
and use the `adapj_takeup_twoaxis_sim.yaml` task/initialization profile. It reuses
the existing take-up arbiter and freezes AdapJ parameter updates during its
transactions; it does not enable continuous AdapJ path tracking.

Individual simulation executables retain their established names:

```bash
ros2 run simulation catheter_sim_device
ros2 run simulation catheter_sim_perception
ros2 run simulation catheter_sim_visualizer
ros2 run simulation catheter_sim_target
ros2 run simulation catheter_sim_scenario
```
