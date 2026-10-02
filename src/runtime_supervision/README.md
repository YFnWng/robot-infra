# Runtime supervision

This package owns runtime-identity capture, finalized-session validation, and
offline stationary-session qualification and recorded-input compute profiling.
It does not own experiment schedules,
controller policy, perception, or hardware safety authority.

Canonical commands:

```bash
ros2 run runtime_supervision causal_runtime_identity --help
ros2 run runtime_supervision causal_session_check --help
ros2 run runtime_supervision causal_stationary_analysis --help
ros2 run runtime_supervision compute_profile --help
```
