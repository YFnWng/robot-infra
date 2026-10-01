# Session 20260929_174517 MPPI full-stack timing audit

## Scope and evidence

Passive audit of `/media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions/20260929_174517_mppi_demo` after the point-4 `accepted_marker_stale` fault. No commands were published and no hardware state was changed.

Primary generated evidence:

- session `full_stack_timing_qualification.json` (178 MPPI timing rows);
- recorded `/catheter_mppi/control_cycle_timing`, tracker diagnostics, command path, and device feedback;
- the latched pre-teardown fault snapshot.

## Result

The fault was not caused by loss of visual tracking or marker rejection. It was caused by controller-process scheduling/compute overload under the full 1024-sample workload. A long estimator update overlapped a long MPPI solve; the next accepted marker was not committed inside the unchanged 150 ms freshness limit.

Confidence: **observed** for the measured delays and gate predicate; **inferred-high-confidence** for shared CPU/GPU resource contention as the mechanism.

## Measured distributions

| Component | Count | P50 | P95 | P99 | Max |
| --- | ---: | ---: | ---: | ---: | ---: |
| Tracker processing | 2960 | 3.60 ms | 4.93 ms | 5.68 ms | 10.62 ms |
| Marker DDS/header age at recorder | 2961 | 53.37 ms | 58.03 ms | 58.89 ms | 66.67 ms |
| Marker pending age sampled at plans | 178 | 18.64 ms | 31.12 ms | 32.58 ms | 33.33 ms |
| Marker rewind | 178 | 7.44 ms | 14.76 ms | 18.30 ms | 19.36 ms |
| Marker correction | 178 | 28.85 ms | 40.40 ms | 46.12 ms | 49.43 ms |
| Marker replay | 178 | 17.04 ms | 27.93 ms | 35.42 ms | 48.08 ms |
| Marker update total | 178 | 55.01 ms | 67.12 ms | 78.39 ms | 102.47 ms |
| MPPI total | 178 | 53.84 ms | 62.72 ms | 76.81 ms | 116.00 ms |
| MPPI rollout | 178 | 36.58 ms | 44.06 ms | 50.35 ms | 60.21 ms |
| MPPI sample/projection | 178 | 11.29 ms | 17.56 ms | 21.69 ms | 70.91 ms |
| Manager forwarding, matched command | 2846 | 0.43 ms | 0.83 ms | 1.20 ms | 83.11 ms |
| Manager to serial TX, matched command | 2853 | 0.50 ms | 0.86 ms | 1.40 ms | 4.01 ms |
| Serial TX to next feedback | 2853 | 5.45 ms | 10.33 ms | 12.41 ms | 25.62 ms |

The planner missed its 60 ms deadline in 21/178 solves (11.8%). Per target: point 1 = 5 misses, point 2 = 10, point 3 = 3, point 4 = 3. This was sustained overload, not a point-4-only geometry problem.

## Terminal event

At the final point-4 event:

- tracker diagnostic: `TRACKING`;
- rejection streak: 0; last marker update: accepted;
- accepted-marker commit age: 158.17 ms, over the 150 ms gate;
- marker source age: 268.96 ms;
- estimator callback: 150.86 ms with 140.51 ms timer lateness;
- model marker update: 53.72 ms;
- MPPI: 116.00 ms against a 60 ms deadline;
- MPPI sample/projection phase: 70.91 ms;
- raw ENC remained fresh (10.47 ms receive age), while processed ENC was 75.46 ms old;
- tracker processing remained healthy.

The 70.91 ms sample/projection spike is much larger than its 11.29 ms median even though this phase is mostly CPU-side candidate generation/projection. Together with the simultaneous estimator and planner overruns, this is consistent with executor/GIL/native-thread/GPU synchronization contention inside the single controller process. The exact OS scheduling contribution is not proven without scheduler tracing.

## Findings

### F-001: The 1024-sample hardware profile is not qualified under full-stack load

Severity: high. Confidence: observed.

The earlier offline preflight measured the planner without concurrent live marker correction/rewind/replay. In this session, MPPI P95 exceeded the 60 ms deadline and missed 11.8% of solves.

### F-002: Visual tracking was healthy; accepted correction commits were starved

Severity: high. Confidence: observed.

The tracker processed frames in 3.60 ms median, reported `TRACKING`, and the estimator had zero consecutive rejections. The stale gate correctly detected that no accepted correction had committed for 158 ms.

### F-003: Estimator and planner workloads contend in one process

Severity: high. Confidence: inferred-high-confidence.

Separate callback groups and eight executor threads do not isolate Python, PyTorch native threads, or a shared CUDA device. The terminal estimator and planner callbacks simultaneously expanded to 151 ms and 116 ms. The evidence proves overlap in their observed long-latency window, but kernel-level attribution requires a bounded trace.

### F-004: Manager/serial transport is not the dominant delay

Severity: informational. Confidence: observed.

P95 forwarding and serial-publication delays were below 1 ms, and next feedback arrived within 10.33 ms P95.

## Recommended correction gate

Do not increase the 150 ms marker timeout. Reduce planner load first and requalify under the complete camera + estimator + controller stack. The lowest-risk next profile is 512 samples with the same four-step 0.8 s point horizon, planning rate, costs, and safety gates. Acceptance criteria:

1. MPPI P99 below 60 ms and zero repeated deadline faults;
2. accepted-marker commit age P99 below 120 ms, leaving 30 ms reserve;
3. estimator callback P99 below 120 ms;
4. no `accepted_marker_stale`, rejection, or manager/device fault;
5. timing report generated from a recorded full hardware run.

If 512 remains marginal, test 256 before attempting CUDA stream/process isolation. Process isolation is a larger architectural change and should follow a trace proving where shared-resource contention occurs.
