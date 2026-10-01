# F-044 sparse rerun deadline regression — 2026-09-16 09:54

## Scope

Passive audit of simulation session `20260916_095434_mppi_sim` after the first
exact-hold/per-axis-ablation implementation. No hardware was commanded.

## Result

Point 2 terminated when the controller latched
`planner:repeated_deadline_miss`. The bag contains 22 `ACTIVE` status samples,
four `planner_deadline_miss_zero` samples, and one terminal `FAULTED` sample.
For 23 completed planner measurements, elapsed time was 42.092 ms median,
68.474 ms P95, and 71.144 ms maximum against the configured 60 ms deadline.

The expensive phase was learned rollout: 30.807 ms median, 57.489 ms P95, and
60.767 ms maximum. Cost weighting also reached 32.463 ms. The first F-044
implementation performed the ordinary candidate rollout and then invoked the
learned model again for the exact combined hold and per-axis ablations. Even
though the second batch was small, fixed accelerator/runtime overhead made it
unsafe for the control deadline.

## Correction

Ordinary candidates, their combined physical-axis hold, and per-axis ablation
variants now share one fixed-size candidate bank. The configured sample budget
is partitioned among those branches, so the controller performs one hardware
projection and one learned-model invocation per planning cycle without
increasing total rollout population. Selection weights remain restricted to
ordinary candidates; the paired variants are used only for scheduler evidence
and pending execution.

A focused regression records backend batch sizes and requires exactly one call
with no more than the configured population. The 27 focused planner/scheduler
tests and all 217 `catheter_control` tests pass, and the ROS package rebuilds.

## Status

Source remediation is complete. F-044 remains open pending another sparse GPU
simulation demonstrating both functional target tracking and zero repeated
deadline faults.
