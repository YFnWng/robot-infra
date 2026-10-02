# Control tasks

This package owns operator-facing controller task clients and action servers.
It depends on the controller core but the controller core does not depend on it.

Executable names remain unchanged; invoke them through this ROS package:

```bash
ros2 run control_tasks catheter_target_offset --help
ros2 run control_tasks catheter_sparse_point_experiment --help
ros2 run control_tasks catheter_tip_path_file --help
```
