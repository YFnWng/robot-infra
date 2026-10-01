# Python runtime package layout

Phase 3 separates runtime responsibilities without changing ROS names,
messages, topics, services, actions, parameters, safety behavior, or console
commands. New code must import the canonical modules below. The former module
paths remain compatibility shims until Phase 7.

## `catheter_control`

```text
catheter_control/
├── node.py                  # thin ROS composition root and callback ownership
├── orchestration/           # timing, causal scheduling, runtime loading,
│                            # marker validation, configuration, diagnostics
├── planning/                # MPPI, path references, trajectories, tip tracking
├── transmission/            # play/backlash, engagement/gain belief, reversals
├── safety/                  # hardware contract, lifecycle gates, validation
├── simulation/              # plant, device, perception, scenarios, RViz adapter
├── applications/            # action/task clients, files, overlays, recording
└── <legacy modules>.py      # import/`python -m` compatibility only
```

The composition root still owns ROS callback groups, immutable planner
snapshots, estimator and planner execution, command publication, and fault
latching. Pure marker-message validation, callback timing instrumentation, and
learned-runtime loading have moved behind `orchestration` interfaces. This is
the first seam for the Phase 4 C++ ROS shell; it does not change scheduling.

Allowed dependency direction is:

```text
applications ─┐
simulation  ──┼──> planning / transmission / safety
node ─────────┴──> orchestration + planning + transmission + safety
```

Canonical implementation modules must not import root compatibility shims.
The shims may import canonical implementations and expose private names only
to preserve existing tests and external scripts.

## `automation`

The historical package is retained, but its mixed responsibilities are now
separated internally:

```text
automation/
├── perception/              # marker tracking, UDP input, state estimator, EM
├── experiments/             # collection node and experiment schedules
├── supervision/             # runtime identity, session checks, analysis
├── marker_tracking/         # compatibility paths
├── estimation/              # compatibility paths
└── collection/              # compatibility paths
```

Installed console-command names are unchanged. Their entry points now target
the canonical responsibility packages. This internal split is the migration
boundary for future `catheter_perception` and `catheter_experiments` ROS
packages; no package split is required for Phase 3.

## Compatibility and retirement

Compatibility modules are intentionally small and contain no behavior. Add
features and fixes only to canonical modules. Compatibility paths are covered
by identity tests and may be removed only in Phase 7 after downstream callers
and recorded reproduction instructions have migrated.
