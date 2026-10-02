# Phase 4 C++ ROS Shell and Shadow Deployment Plan

Status: planned
Owning repository: `robot-infra`
Depends on: completed Phase 3 package cleanup (`dd8d048`)

## 1. Objective

Phase 4 moves time-critical ROS composition and safety-adjacent scheduling into
a bounded C++ shell while retaining the Python estimator and MPPI as the
reference implementation. The first C++ deployment is shadow-only: it consumes
the same inputs, records decisions and timing, and cannot publish actuator
commands.

This phase addresses executor jitter and Python callback starvation. It does
not change model equations, estimator behavior, MPPI costs, engagement or
take-up policy, ROS interfaces used by the manager, hardware limits, or
firmware behavior.

## 2. Non-negotiable invariants

- Python retains command authority until every shadow and timing gate passes
  and command promotion is approved as a separate change.
- Shadow mode has no publisher connected to the manager command input.
- The manager, serial bridge, and firmware remain the final safety authorities.
- The encoder zero remains a read-only calibration reference. Neither the C++
  shell nor its tests may expose or issue `SET_ZERO`.
- Command-output enablement remains an explicit startup interlock and defaults
  to disabled.
- Late, duplicate, future, stale, malformed, or out-of-order worker results are
  discarded. They are never awaited by heartbeat or safety callbacks.
- Existing topic names, source stamps, freshness rules, fault reasons, axis
  ordering, units, limits, and command projection remain unchanged until a
  separately reviewed interface migration.
- No phase combines language migration with controller tuning, QoS changes,
  learned-model replacement, or safety-policy changes.

## 3. Baseline execution model

The Python controller currently uses an eight-thread `MultiThreadedExecutor`
and separate mutually-exclusive callback groups for estimator ownership,
marker ingress, device ingress, planning, heartbeat, services, and general I/O.
The important nominal periods are:

| Work | Nominal period | Current risk |
| --- | ---: | --- |
| Command heartbeat | 10 ms | delayed by process scheduling and Python work |
| Estimator owner | 20 ms | correction and rewind/replay can exceed one period |
| Planner | about 66.7 ms | GPU solve and callback entry can exceed 60 ms budget |
| Marker correction | bounded near 50 ms | delayed observation and replay cost |

Executor thread count does not guarantee parallel progress because Python code
remains subject to the GIL and native PyTorch/NumPy scheduling. Phase 4 must
measure callback ready-to-start delay separately from callback computation.

## 4. Target architecture

```text
device / marker / manager / target inputs
                  |
                  v
        C++ control shell (authoritative ROS clocking)
        - input validation and latest-value caches
        - immutable event/snapshot sequencing
        - lifecycle and freshness gates
        - heartbeat and zero-command fail-safe
        - result age/sequence validation
        - diagnostics and timing
                  |
           bounded asynchronous contract
                  |
                  v
        Python estimator/planner worker
        - v171 runtime and delayed marker correction
        - transmission/gain beliefs during Phase 4
        - target/reference preparation
        - MPPI rollout and selection
                  |
                  v
        versioned command decision result
```

The worker is a separate process so Python/GPU work cannot occupy the shell's
executor threads. There are no synchronous ROS service calls, futures, or
condition-variable waits in the heartbeat path.

### C++ execution domains

1. **Heartbeat/safety executor**: one dedicated thread; publishes zero or the
   most recent fresh, authorized result. It performs no model calls, file I/O,
   JSON serialization, or blocking logging.
2. **Input executor**: device, marker, manager, and target callbacks; validates
   stamps and replaces bounded latest-value/event slots.
3. **Worker-result executor**: validates result schema, sequence, source-state
   watermark, age, and numeric limits before an atomic snapshot exchange.
4. **Service executor**: arm/disarm/reset/preview requests with no dependency
   on heartbeat progress.
5. **Diagnostics executor**: lower-rate reporting and timing aggregation,
   isolated from command publication.

The initial implementation uses normal OS scheduling. CPU affinity or
real-time policies require separate measurements and review; they are not
assumed by this plan.

## 5. Language-neutral contracts

Contracts belong in `control_interface`, with units and invariants documented
in the message definition or adjacent schema documentation.

### Input event

Every accepted device, marker, manager, and target update receives:

- schema version and event kind;
- monotonic shell sequence number;
- source/acquisition stamp and shell receive stamp;
- source identity and target revision where applicable;
- validated payload in canonical SI units;
- explicit validity and rejection reason.

Events are bounded and processed exactly once by sequence. A worker restart
requires an explicit resynchronization epoch; old-epoch results are invalid.

### Planning request watermark

A request identifies the complete input prefix that must be processed before
planning:

- shell epoch and request sequence;
- latest required event sequence for each source;
- estimator/source timestamp represented by the request;
- target/reference revision;
- controller mode and resolved configuration identity;
- artifact/configuration hashes.

The contract does not serialize PyTorch state. Learned state remains owned by
the Python worker in Phase 4.

### Command decision

A result contains:

- schema version, shell epoch, and request sequence;
- input watermarks and estimator state timestamp;
- computation start/end timestamps;
- validity, controller state, and stable reason code;
- six-axis logical velocity in canonical units;
- projection and limit metadata needed for independent validation;
- estimator, planner, and belief health summaries;
- deterministic trace fields required for Python/C++ comparison.

The shell rejects a result unless its epoch, request, target revision,
watermarks, age, dimensions, finiteness, limits, and mode all match current
state. A rejected or missing result produces zero; it never reuses an expired
command.

## 6. Deployment modes

| Mode | Command authority | Purpose |
| --- | --- | --- |
| `python_reference` | Python | frozen behavioral baseline |
| `cpp_shadow` | Python | C++ observes inputs and records gates/timing only |
| `cpp_shadow_with_worker` | Python | exercise event/request/result contract |
| `cpp_shell_python_planner` | C++ shell using fresh Python results | later promotion candidate |
| `cpp_production` | promoted C++ components | outside initial Phase 4 |

The first three modes must coexist in launch/configuration without changing
the Python reference defaults. `cpp_shell_python_planner` is not enabled merely
because shadow tests pass; it requires a separate reviewed authority change.

## 7. Implementation sequence

### P4.0 — Freeze fixtures and reason codes

- Inventory every input, timestamp, gate, lock, callback, output, and fault
  reason used by `catheter_control/node.py`.
- Capture representative non-actuating replay fixtures for initialization,
  normal tracking, delayed marker correction, take-up, timeout, and fault.
- Store compact fixtures in Git only when appropriate; keep bags and traces in
  the external session root.
- Define comparison tolerances before implementing C++ behavior.

Acceptance: fixtures reproduce stable Python decisions and discrete reasons
from a clean sourced overlay.

### P4.1 — Add the non-commanding C++ shell

- Create `control_cpp` as an `ament_cmake` package.
- Implement subscriptions, bounded caches, event sequencing, lifecycle state,
  diagnostics, and timing probes.
- Hard-disable command publication in code and configuration.
- Add unit tests for timestamp validation, epochs, latest-value replacement,
  freshness, and unconditional `SET_ZERO` absence/rejection.

Acceptance: shell runs beside Python without altering the ROS graph seen by
the manager and records complete shadow traces.

### P4.2 — Define and exercise the worker contract

- Add versioned input/request/result interfaces to `control_interface`.
- Add a thin Python worker adapter around the existing canonical controller;
  do not duplicate model, estimator, belief, or MPPI equations.
- Use bounded depth-one/latest-useful-data queues where history has no control
  meaning; preserve explicitly ordered event history where rewind/replay needs
  it.
- Reject duplicate, skipped, old-epoch, stale-target, and late results in
  focused tests.

Acceptance: recorded requests produce repeatable Python results and the shell
never blocks awaiting them.

### P4.3 — Gate and lifecycle conformance

- Mirror readiness, arm/disarm, target validity, feedback freshness, manager
  readiness, estimator health, planner deadline, command age, and fault latch
  transitions in the C++ shadow.
- Compare every transition and reason code against the Python reference.
- Resolve mismatches in isolated commits; do not silently normalize them in a
  comparison script.

Acceptance: exact discrete transition/reason agreement on all fixtures.

### P4.4 — Replay and simulation qualification

- Run Python reference and C++ shadow on identical recorded event sequences.
- Run grouped, plain-with-takeup, and plain controller modes in simulation.
- Compare accepted/rejected inputs, state transitions, command eligibility,
  result freshness, projected commands, and fault timing.
- Inject late, missing, reordered, duplicated, and non-finite worker results.

Acceptance: no unsafe divergence; numeric tolerances and all exceptions are
documented and enforced by tests.

### P4.5 — Full-stack timing qualification

- Measure under representative dual-camera, estimator, GPU planner, manager,
  serial bridge, and diagnostic load.
- Report count, P50, P95, P99, maximum, and deadline misses for input arrival
  to callback start, callback duration, snapshot/request publication, worker
  turnaround, result commit, heartbeat lateness, and sensor-to-command age.
- Compare with the frozen Python baseline using the same workload.
- Treat tracing overhead as an explicit measurement condition.

Acceptance: the shell heartbeat has no missed releases caused by Python worker
load; stale or late results consistently yield zero; no freshness/fault metric
regresses without review.

### P4.6 — Shadow hardware observation

- Requires explicit hardware authorization.
- Keep Python command authority and C++ command output structurally disabled.
- Record C++/Python gate decisions and timing during representative tasks.
- Investigate every disagreement before considering authority promotion.

Acceptance: repeated shadow sessions show stable conformance and timing with no
effect on hardware commands.

### P4.7 — Authority promotion decision

Prepare, but do not automatically execute, a separate change that enables
`cpp_shell_python_planner`. The review must include failure injection, rollback,
operator commands, resolved hashes, and a new production baseline. Python
remains available as a shadow oracle.

## 8. Conformance matrix

| Surface | Required comparison |
| --- | --- |
| Parameters/configuration | same resolved value, owner, unit, and validation |
| Input acceptance | exact accept/reject and reason code |
| Lifecycle/gates | exact state transition and latch behavior |
| Timestamp/freshness | same boundary behavior using recorded clock values |
| Hardware projection | exact discrete RPM/count result where applicable |
| Worker result validation | exact accept/reject and zero-command fallback |
| Planner output | configured numeric tolerance plus identical validity reason |
| Diagnostics | stable required keys and equivalent semantic values |
| Safety | manager/firmware authority and all fail-closed paths preserved |

Numerical tolerances must be justified per field and committed with fixtures;
there is no single blanket floating-point tolerance.

## 9. Timing evidence and release gates

Phase 4 is not complete from an isolated benchmark. Qualification must include:

- representative full-stack load;
- callback ready-to-start delay as well as execution duration;
- worker request/result transport and queue age;
- estimator correction and rewind/replay episodes;
- GPU planner deadline misses;
- command heartbeat lateness and planned-command age;
- manager receipt age and source deadman behavior.

Generated traces, bags, and timing tables belong under the external catheter
session root. Maintained audit summaries belong under `audits/ros-realtime/`.

## 10. Planned commit sequence

1. `test: freeze controller event and decision fixtures`
2. `feat: add non-commanding C++ control shell`
3. `feat: add versioned controller worker contracts`
4. `feat: add Python planner worker adapter`
5. `test: add Python C++ lifecycle conformance replay`
6. `test: qualify C++ shell scheduling under full-stack load`
7. Optional, separately approved: `feat: promote C++ shell command authority`

Each commit must build and test independently. Scheduling changes, QoS changes,
controller changes, and authority changes remain separate commits.

## 11. Explicitly deferred work

- translating learned dynamics, UKF, rewind/replay, or MPPI to C++;
- changing engagement, take-up, gain, or reversal policy;
- changing controller frequency, horizon, samples, or costs;
- applying CPU affinity or real-time OS scheduling;
- moving computation to Teensy;
- changing manager, serial, or firmware safety authority;
- deleting the Python reference path.
