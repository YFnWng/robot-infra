# Experiments

This package owns deterministic excitation schedules and the guarded ROS data
collection node. It does not own controller policy, perception, session
qualification, or hardware safety authority.

The canonical collection command is:

```bash
ros2 run experiments collection
```

`session_recording` owns coordinated research-recorder lifecycle and the session
manifest; `bringup/research_session.launch.py` composes its camera, controller,
and bag commands. The recorder has no robot command publishers or device
services. `recording_session.py` owns filesystem identity and finalized
evidence, independently testable without ROS. Task execution and research
analysis/report generation remain separate gates.
