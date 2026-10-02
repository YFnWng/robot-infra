# robot-infra implementation rules

The workspace-level `../AGENTS.md` remains authoritative for repository scope,
hardware safety, generated data, runtime environments, and verification. These
rules add repository-local maintainability constraints.

## Architecture and ownership

- Use the canonical package boundaries in
  `docs/architecture/PYTHON_RUNTIME_PACKAGE_LAYOUT.md`. New code must not
  depend on compatibility shims.
- Compatibility modules preserve imports and entry points only. Do not add
  behavior, configuration, state, or tests that target a shim as the owner.
- Keep ROS nodes and launch files as composition surfaces. Put deterministic
  calculations and state transitions in ROS-independent modules.
- Each behavior has one implementation owner. Do not copy equations, state
  transitions, safety checks, parameter declarations, or configuration merge
  logic into a second location.
- Keep learned mechanics, estimator mathematics, and model rollout in
  `cr_meta_lnn`. Keep ROS scheduling, message conversion, lifecycle, and safety
  integration here.

## Simplicity and readability

- Prefer extending the existing owner over adding another wrapper, manager,
  scheduler, adapter, or configuration layer.
- Give modules one named responsibility. Do not create generic dumping grounds
  such as `utils.py`, `common.py`, or `helpers.py`.
- Keep APIs narrow and explicit. Avoid wildcard imports and broad re-exports,
  except inside declared compatibility shims.
- Avoid hidden import fallbacks, runtime monkeypatching, and new `sys.path`
  manipulation. Runtime loading belongs behind the existing orchestration
  boundary.
- Use explicit typed records or immutable snapshots across estimator, planner,
  and command-publication boundaries instead of sharing mutable node state.
- Prefer clear control flow and descriptive names over clever abstraction.
  Large files or deeply nested callbacks are review signals: extract a
  cohesive pure component when that makes ownership clearer, not merely to
  reduce line count.
- Do not introduce development-version suffixes for ordinary source or config
  evolution. Use semantic configuration names; version artifacts only where
  compatibility or scientific reproduction requires it.

## Change discipline

- Before adding a module, search for its current owner and callers. Before
  deleting or moving code, inspect imports, entry points, launch files,
  configurations, tests, and recorded reproduction commands.
- Separate behavior-preserving movement from model, controller, scheduling,
  QoS, safety, and parameter-tuning changes. Use separate commits when both are
  required.
- Every new parameter needs one owner, a documented unit, validation, and a
  focused test. Do not add two parameters for the same uncertainty or policy.
- Preserve command names and ROS interfaces during internal refactors unless
  the user explicitly requests a coordinated interface migration.
- Update ownership and package-layout documentation when a responsibility or
  dependency direction changes.

## Verification

- Add characterization tests before moving behavior that lacks coverage.
- Run focused tests while editing, then relevant package tests, lint, and ROS
  builds. For launch or packaging changes, verify installed entry points after
  rebuilding and sourcing the overlay.
- Use an isolated, non-actuating simulation or replay smoke test when runtime
  composition changes. Scheduling changes additionally require the
  `ros-realtime-audit` workflow and representative full-stack measurements.
- Do not declare a cleanup complete while canonical code imports compatibility
  shims, generated copies were edited, or unrelated working-tree changes were
  absorbed.
