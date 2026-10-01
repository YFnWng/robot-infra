# Four-step 0.8 s sparse simulation and hardware-readiness audit

Session: `20260928_192522_mppi_sim`

## Outcome

The point-target horizon correction worked behaviorally. All four model-
generated two-axis targets reached tolerance. Unlike the preceding short-
horizon hardware run, points 2 and 3 selected nonzero tendon motion. The bag
confirms four prediction steps spanning 0.8 s, no prediction tail, CUDA, and
the explicit coarse backend.

This run qualifies the horizon mechanism for a staged no-rotation hardware
test, but it is not a complete hardware qualification. Four isolated plans
exceeded the 60 ms deadline, and the simulation was launched with engaged-gain
belief/scenarios disabled. Hardware must therefore begin with the actual
no-rotation profile and a non-actuating runtime-identity check, followed by the
axial gate before the near two-axis target block.

## Tracking result

| Point | Result | Final error | Nonzero insertion plans | Nonzero tendon plans |
|---:|---|---:|---:|---:|
| 1 | reached | 0.941 mm | 13/15 | 0/15 |
| 2 | reached | 1.013 mm | 24/29 | 9/29 |
| 3 | reached | 0.987 mm | 13/17 | 11/17 |
| 4 | reached | 1.249 mm | 17/19 | 10/19 |

Marker diagnostics were `TRACKING` for all 1,791 samples. Estimator health was
`TRACKING` for 571/591 status samples; the sole `DEGRADED` status occurred
disarmed during a simulation reset (`observation_before_rewind_buffer`), not
during target control.

## Runtime identity

- `horizon_steps=4`
- `mppi_point_rollout_step_s=0.2`
- `mppi_point_rollout_coarse_steps=true`
- `mppi_point_prediction_tail_steps=0`
- reported prediction horizon: four steps / 0.8 s
- samples: 144 on `cuda:0`
- engaged-gain belief and scenarios: disabled in this simulation launch
- rotation-disabled controller limit: `[10,0,4.5,4,25,25]`
- model validity: true

## Timing

| Measurement | Count | P50 | P95 | P99 | Maximum |
|---|---:|---:|---:|---:|---:|
| Total planner | 80 | 43.863 ms | 59.954 ms | 68.319 ms | 83.776 ms |
| Model rollout | 80 | 33.739 ms | 46.618 ms | 57.119 ms | 67.131 ms |
| Sample projection | 80 | 4.330 ms | 8.158 ms | 10.884 ms | 11.385 ms |

Four plans exceeded 60 ms, never consecutively. Every miss correctly produced
the fail-closed zero command, and all four targets still completed. The misses
occurred while active at 60.324, 63.489, 64.210, and 83.776 ms.

## Hardware gate

Proceed only through the reviewed no-rotation sequence:

1. launch `v175_grouped_hardware_no_rotation.yaml` with output disabled and
   verify runtime identity, fresh POS/ENC, marker tracking, UKF tracking,
