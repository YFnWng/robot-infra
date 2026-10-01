# Plain-MPPI hardware rerun audit — 2026-09-29

## Scope

Passive analysis of:

- `20260929_194506_mppi_demo`: 512-sample plain MPPI with v175 take-up
  compensation;
- `20260929_194759_mppi_demo`: 512-sample plain MPPI without compensation;
- `20260929_104353_mppi_demo`: successful grouped-MPPI reference that supplied
  the fixed Cartesian targets.

The two reruns used the intended controller profiles according to their saved
runtime manifests. No hardware commands or runtime changes were made during
this audit.

## Outcome

| Session | Reached | Timed out |
| --- | ---: | ---: |
| Plain + take-up | point 3 | points 1, 2, 4 |
| Plain, uncompensated | points 2, 3, 4 | point 1 |
| Grouped reference | points 1–4 | none |

Both new sessions made substantial progress on point 1. They did not fault;
they timed out outside the 1.8 mm acceptance ball.

## Point-1 geometry

The point-1 target was the fixed position `[24.686, 20.164, 84.481]` mm
recorded in the earlier grouped session. It was not regenerated from each
rerun's current homed shape.

| Run | Target minus homed tip (mm) | Closest error (mm) | Closest observed tip (mm) |
| --- | --- | ---: | --- |
| Grouped reference | `[+0.995, -0.323, +8.057]` | 1.097 | `[24.228, 20.816, 83.727]` |
| Plain + take-up | `[-0.654, -1.865, +9.333]` | 3.021 | `[25.554, 21.903, 82.169]` |
| Plain, uncompensated | `[-0.784, -1.064, +16.566]` | 2.326 | `[26.104, 22.007, 84.521]` |

The uncompensated run reached the target's Z coordinate but retained about
`[+1.418, +1.843, +0.040]` mm error. This is direct evidence that the failure
was a lateral reachable-set mismatch, not insufficient insertion travel.
Rotation was locked, the tendon began at its lower encoder boundary, and
positive insertion moved the tip along the current catheter configuration's
insertion curve. That curve differed from the curve present when the fixed
target was recorded.

The compensated run reached insertion position 33.8 mm, comparable with the
33.5 mm used by the successful grouped reference, but its observed tip was
still displaced laterally and 2.3 mm below the target. Continued insertion
would trade axial improvement for increasing lateral error, so the plain
weighted controller increasingly preferred holding or small corrections.

## Controller behavior

### Plain MPPI without compensation

- Target-1 insertion moved from 20.0 to 36.9 mm.
- Tendon position stayed at zero.
- The target-1 command stream was zero for 89.2% of recorded heartbeat rows.
- The closest error was 2.326 mm at 24.6 s; final error was 2.589 mm.

The long hold was rational under the constrained two-axis task: after reaching
the closest point on the current insertion curve, further insertion increased
the lateral residual, rotation was unavailable, and tendon relaxation was
impossible at the lower boundary.

### Plain MPPI with take-up compensation

- Target-1 insertion moved from 20.0 to 33.8 mm.
- It spent 80 diagnostic samples in `TAKEUP_ACTIVE` and 213 in
  `READY_TO_PLAN`.
- Actual axis-0 commands included both signs (`-2.0` to `+7.41` mm/s), and a
  small positive tendon transaction moved axis 2 to 0.181 mm.
- The measured/predicted response-magnitude ratio had median 0.662 and median
  direction cosine was only 0.377.

The compensator did not solve the geometric mismatch. It also interrupted the
plain weighted plans with take-up transactions, while the ungrouped weighted
mean lacked the grouped controller's discrete mode selection. This explains
why the compensated baseline subsequently timed out on more targets, but it
is not the primary reason shared point 1 failed.

## Timing and perception exclusion

Target 1 had healthy visual estimation in both reruns:

- every estimator trace reported `TRACKING`;
- consecutive marker rejections remained zero;
- marker RMS-after-correction P95 was 0.392 mm with compensation and 0.458 mm
  without compensation;
- marker diagnostics remained `TRACKING` throughout;
- neither bag contains a controller-fault log.

Planning was marginal but did not trip the fault gate:

| Run | Whole-session deadline misses | Target-1 misses | Maximum consecutive target-1 misses |
| --- | ---: | ---: | ---: |
| Plain + take-up | 46 / 851 | 4 | 1 |
| Plain, uncompensated | 17 / 668 | 9 | 1 |

Therefore scheduling jitter may alter individual commands, but it did not
cause either point-1 timeout.

## Conclusion

**Observed:** the common point-1 failure is caused primarily by reusing an
absolute Cartesian target generated under a different hidden catheter state.
The homed encoder vector `[20,0,0]` did not reproduce the earlier homed distal
shape, and the fixed target fell outside the current rotation-locked
insertion/tendon reachable set within the 1.8 mm tolerance.

For planner-only A/B tests, targets should be generated from one shared live
home snapshot and all variants should run without changing physical history.
For repeatability tests across separately homed sessions, report distance to
the current model-generated reachable set in addition to distance to the old
absolute Cartesian target.
