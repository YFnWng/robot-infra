# Session 20260929_101041 farther two-axis pause audit

## Outcome

All four rotation-disabled hardware targets reached the 1.8 mm tolerance.
The repeated visible stops were not planner deadline failures, stale-feedback
pauses, or controller faults. They were dominated by intentional slow
insertion-axis take-up transactions after MPPI selected a new insertion
direction. During `TAKEUP_ACTIVE` the motor command was normally nonzero but
the catheter tip was expected to move little; each transaction then inserted
a short zero `CONFIRMATION_HOLD` and response-triggered replan.

Confidence: **observed** from the recorded controller diagnostics, commands,
trajectory feedback, and response traces.

## Per-target result

| Point | Active time | Error, initial to final | Take-up active | Confirmation/replan | Actual zero command | Planned insertion reversals |
|---:|---:|---:|---:|---:|---:|---:|
| 1 | 2.253 s | 8.060 to 1.311 mm | 0.000 s | 0.000 s | 0.350 s | 0 |
| 2 | 16.934 s | 16.651 to 1.242 mm | 11.896 s | 1.400 s | 1.488 s | 9 plan-sign changes; 5 take-up transactions |
| 3 | 13.277 s | 16.669 to 0.629 mm | 8.700 s | 1.300 s | 1.340 s | 5 plan-sign changes; 4 take-up transactions |
| 4 | 7.866 s | 19.617 to 1.298 mm | 4.201 s | 0.598 s | 0.780 s | 1 plan-sign change; 2 take-up transactions |

Point 1 moved continuously apart from startup and a 0.158 s capture hold after
entering tolerance. Points 2--4 repeatedly reversed insertion. The configured
slow take-up command is 2 mm/s; the approximately 2.2--2.5 s insertion
transactions are consistent with the previously measured roughly 4.3--4.5 mm
insertion reversal window.

## Model-response evidence

| Point | Response windows | Median predicted motion | Median measured motion | Median measured/predicted | Median direction cosine | Opposed responses |
|---:|---:|---:|---:|---:|---:|---:|
| 1 | 19 | 0.948 mm | 1.141 mm | 1.173 | 0.905 | 0/19 |
| 2 | 36 | 1.892 mm | 1.117 mm | 0.636 | 0.816 | 1/36 |
| 3 | 31 | 2.422 mm | 1.777 mm | 0.936 | 0.896 | 6/31 |
| 4 | 29 | 3.509 mm | 2.524 mm | 0.605 | 0.904 | 4/29 |

The farther coupled targets therefore produced materially smaller and
occasionally opposed responses relative to the frozen rollout. After each
confirmed response the controller replanned from the new observed tip; the
new optimum sometimes changed insertion direction. Since MPPI is inhibited
during take-up, the arbiter then spent the full slow transaction clearing the
new direction before another effective command could execute.

## Timing and health exclusions

- No `planner_deadline_miss_zero` state occurred.
- Recorded active-plan timing was 44.650/48.950/55.415/59.077 ms at
  P50/P95/P99/max, below the 60 ms deadline.
- No `estimator_catchup_zero`, stale-feedback fault, or marker-health fault
  occurred while tracking.
- Rotation remained disabled by the hardware isolation contract.

## Mechanism

```text
fresh observed tip
 -> MPPI chooses an effective insertion/tendon plan
 -> insertion direction differs from currently engaged direction
 -> MPPI execution is inhibited
 -> slow 2 mm/s insertion take-up (tip nearly stationary)
 -> zero confirmation hold
 -> visual response confirms engagement
 -> fresh MPPI solve
 -> sometimes a different insertion sign is now optimal
 -> another take-up transaction
```

The compensation is behaving as designed, but repeated effective-plan
direction changes make a successful target unnecessarily slow. The dominant
remaining issue is therefore direction commitment/hysteresis at the MPPI to
take-up boundary, amplified by imperfect response magnitude predictions. It
is not a computation or camera-recording failure.

## Evidence

- Bag: `/media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions/20260929_101041_mppi_demo`
- Manifest: `/media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions/20260929_101041_mppi_demo_manifest.json`
- Controller profile: `src/catheter_control/config/v175_grouped_hardware_no_rotation.yaml`
- Target profile: `src/catheter_control/config/farther_two_axis_points_v175_hardware_no_rotation.yaml`
