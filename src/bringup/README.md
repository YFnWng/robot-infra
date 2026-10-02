# Bringup

This package owns launch composition and deployment entry points. Runtime
implementation remains in the functional packages it launches.

Canonical launch commands use:

```bash
ros2 launch bringup control.launch.py
ros2 launch bringup simulation.launch.py
ros2 launch bringup <experiment-or-perception>.launch.py
```
