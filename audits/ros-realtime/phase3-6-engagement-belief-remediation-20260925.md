# Phase 3--6 engagement-belief remediation

Date: 2026-09-25
Scope: static architecture and non-actuating tests

## Finding RTE-ENG-01

Classification: observed.

The planner was already skipped during TAKEUP_ACTIVE, but PROVISIONAL was
treated as ready and selected axes could bypass the transaction through the
raw-response mask. MPPI also retained an interface-play rollout path. This
allowed a post-engagement plan to be evaluated or executed before all
commanded axes were observation-confirmed.

## Remediation

- Added atomic TAKEUP_ACTIVE / CONFIRMATION_HOLD / REPLAN_REQUIRED arbitration.
- MPPI is not called during take-up or confirmation hold.
- All commanded physical shafts participate; no raw-response bypass remains.
- The first credible response produces zero, persistent accepted observations
  establish ENGAGED, and the pre-engagement plan is discarded.
- MPPI rollouts use only effective post-engagement dynamics.
- Reversals reserve the upper belief bound and pay delay, uncertainty, and
  low-confidence costs.
- Belief checkpoints are rewound to marker source time and replayed by the
  existing sole estimator owner.

## Execution topology impact

No process, executor, callback group, timer, or mutable-state owner was added.
The estimator timer remains the only owner of learned runtime and engagement
belief state. Planner exchange remains replace-only immutable snapshots.
Take-up and confirmation hold continue through the existing planner and
heartbeat callbacks; neither waits or blocks.

## Safety invariants

Observed preserved: no SET_ZERO path; manager/firmware final authority;
position/rate projection; freshness gates; watchdog; fault latch; zero barrier
before post-engagement replanning.

## Verification

- Python syntax checks: passed.
- Pure backlash/MPPI tests: 84 passed.
- ROS trace/launch integration tests: 22 passed.
- colcon build for control_interface and catheter_control with symlink install:
  passed.

Live timing and hardware behavior remain unmeasured. Representative full-stack
simulation is the next gate before any explicitly authorized hardware test.
