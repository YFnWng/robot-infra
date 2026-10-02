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
evidence, independently testable without ROS.

## Journaled research reaching

After coordinated recording is ready, the operator can explicitly run the
existing guarded sparse-point task against a reviewed frozen Cartesian target
YAML. This is actuating; recording alone never starts the task:

```bash
ros2 run experiments reaching_session \
  --session "$RESEARCH_SESSION" --targets "$REVIEWED_TARGET_YAML" --execute
```

Stop the recorder normally after the task, then generate offline results:

```bash
ros2 run experiments reaching_analysis --session "$RESEARCH_SESSION"
```

For source-time finalized-bag errors, planned/transmitted/encoder reversals,
observed motor travel, gap evidence and per-trial plots:

```bash
ros2 run experiments reaching_analysis --session "$RESEARCH_SESSION" --bag-metrics
```

Use a sourced ROS Humble terminal; analysis does not start a ROS node or device.
Missing streams and stale endpoints are reported, not filled with zeros.

See the [research experiment plan](../../docs/research/MODELING_AND_CONTROL_EXPERIMENT_PLAN.md)
for metric definitions and the explicit limits of this journal-based gate.
