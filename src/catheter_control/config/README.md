# Catheter configuration

The active hardware configuration is composed from semantic layers. The fixed
merge order is:

1. controller (base, then exactly one mode);
2. platform;
3. performance.

Experiment and visualization files are references recorded with the stack, not
parameters passed to catheter_mppi.

## Layout

- controller/base.yaml: controller and estimator policy shared by modes.
- controller/modes/: grouped, plain-with-take-up, and plain policies.
- platform/: model artifacts, estimator backend, and hardware axis limits.
- performance/: compute device, sample count, rates, and deadlines.
- stacks/: reviewed semantic compositions.
- experiments/: canonical task definitions used by semantic stacks.
- rviz/: canonical visualization profiles.
- root-level versioned files: compatibility aliases pending the package split.

A stack is selected by semantic name:

~~~bash
ros2 launch catheter_control control.launch.py \
  stack_config:=hardware_grouped_no_rotation_farther_tendon_12
~~~

An explicit stack YAML path is also accepted. A semantic stack cannot be
combined with the legacy controller_config or performance_config arguments.

## Validation and provenance

The resolver rejects:

- unknown stack keys or layers;
- paths escaping the installed configuration directory;
- missing files;
- parameters outside the active controller schema;
- a parameter owned by more than one layer;
- command_output_enabled in any YAML layer.

Hardware output remains controlled only by the explicit launch argument and
still defaults to false. When recording is enabled, the session manifest
contains every resolved parameter, each parameter's source file, semantic
stack identity, compatibility aliases, and experiment/visualization
references.

The old versioned controller YAMLs remain supported replay profiles. Each
semantic stack declares its corresponding old filename in
compatibility_aliases; regression tests require the composed parameters to
match that legacy file exactly.
