# Simulation

This ROS 2 package owns the isolated actuator, plant, perception, scenario,
target, and RViz simulation runtimes. Simulation endpoints remain under `/sim`;
the package depends one-way on the controller core for shared safety contracts.

Launch the complete isolated stack through `bringup`:

```bash
ros2 launch bringup simulation.launch.py
```

Individual simulation executables retain their established names:

```bash
ros2 run simulation catheter_sim_device
ros2 run simulation catheter_sim_perception
ros2 run simulation catheter_sim_visualizer
ros2 run simulation catheter_sim_target
ros2 run simulation catheter_sim_scenario
```
