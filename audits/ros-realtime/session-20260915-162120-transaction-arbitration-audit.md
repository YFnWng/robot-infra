# Take-up transaction simulation audit (2026-09-15 16:21)

## Scope

Passive analysis of `20260915_162120_mppi_sim`, the first recorded continuous
path run after separating post-take-up MPPI planning from physical take-up
execution. No hardware command was issued.

## Outcome

The architectural correction improved boundedness and rotation direction
stability, but the run did not complete. It timed out after approximately
60.6 s at 15.477 of 76.918 mm path progress. The dominant failure was an
unfinishable final take-up transaction at the catheter-bend lower position
boundary. In addition, the scored-candidate execution guard was not enabled in
the runtime that produced this bag, so this session did not validate the full
F-037 correction.

## Tracking and scheduling evidence

- The path trace contains 1,813 samples from 4.277 to 64.854 s.
- Progress reached 15.477 mm (20.1% of the 76.918 mm path).
- Closest-path error was 0.653/3.046/3.046 mm at P50/P95/max.
- Reference-point error was 0.821/9.311/9.312 mm at P50/P95/max.
- The governor reported 462 `RUNNING` and 1,351 `TRANSMISSION_HOLD`
  samples; it never entered `SLOWED` or `PAUSED`.
- During the approximately 60.5 s armed interval, status sampling attributed
  41.50 s to `TAKEUP_ACTIVE`, 17.70 s to `READY_TO_PLAN`, and 1.30 s to
  `REPLAN_REQUIRED`.
- The controller created 44 take-up transaction generations. The last one did
  not complete before the path action aborted at about 60 s.
- Plan time was 36.877/47.631/53.593/68.117 ms at P50/P95/P99/max. There was
  one deadline-zero event and no repeated-deadline fault.

Compared with the earlier approximately 6 mm divergence near 2 mm progress,
this is meaningful improvement: the controller advanced much farther and kept
closest-path error near 3 mm. It is not yet a successful validation because
the transaction occupied most of the run and the action aborted.

## Rotation and learned-model evidence

Of 266 planned controls, rotation was nonzero in 101: 97 negative and only
four positive. The earlier rapid rotation sign chatter is therefore largely
suppressed at the planner output.

Only eight response forecasts were emitted because forecasts are intentionally
suppressed during transmission hold. The two nonzero rotation examples showed
good short-window model agreement:

- command `[0,-9.039,0]`: predicted `[+0.076,-0.156,+0.020]` mm and measured
  `[+0.100,-0.146,-0.018]` mm, direction cosine 0.966;
- command `[0,-7.191,0]`: predicted `[+0.095,-0.101,+0.017]` mm and measured
  `[+0.030,-0.049,-0.006]` mm, direction cosine 0.955.

This run does not implicate the local Jacobian as the primary cause of the
terminal stall.

## Terminal transaction mechanism

Transaction generation 44 requested positive physical shaft directions on
shafts 0 and 2. It began near 50.36 s. Shaft 2 consumed part of its positive
take-up estimate, reducing its remaining gap from about 32.24 to 24.94 rad,
but its logical bend position simultaneously approached the lower bound:

- at 50.35 s, bend position was 1.919 mm;
- by 50.77 s, it was 0.085 mm;
- it then remained at 0.085 mm.

The take-up command for positive shaft 2 maps through the catheter
insertion/bend coupling to approximately
`[-4.505, 0, -4.500]` in logical velocity coordinates. Near the exact 0 mm
bend lower limit, the position-horizon projection clips the negative bend
component below the configured 2 mm/s reliable-speed floor and converts it to
zero. Consequently the physical shaft-2 command disappears before the
estimated 24.94 rad gap can be consumed.

The coupled insertion component is not removed with it. It alternates with the
shaft-0 transaction command, so raw shaft 0 repeatedly changes direction. The
estimator correspondingly alternates shaft 0 between `ENGAGED` and `TAKEUP`
and reloads roughly 6.6--8.3 rad of remaining gap. Shaft 2 stays pending for
the rest of the action. This is a control-arbitration deadlock at a projected
position boundary, not MPPI replanning inside an active transaction.

The current source constructs a motor-coordinate pending-shaft command, maps
it to logical coordinates, and then applies per-logical-axis position
projection. The command heartbeat projects the result again. Because shaft 0
and shaft 2 are coupled, independently clipping the logical bend component no
longer preserves either the intended physical pending mask or its latched
directions.

## Runtime configuration discrepancy

Every active status sample reported:

- `plan_scored_candidate_guard_applied=False`;
- prediction kind `weighted_feasible_candidate_mean`; and
- selected candidate index `-1`.

The source profile `causal_v2_fixed_hardware.yaml` enables
`mppi_best_candidate_guard`, but the node default and launch default are
false. Therefore the launch used for this run did not actually enable the
scored-candidate execution path. The new transaction logic was active, but the
planner portion of the correction was not.

## Required correction before another validation

1. Make take-up arbitration operate on a feasible, quantized physical-shaft
   command after position projection. If any pending shaft is blocked by a
   position reserve or reliable-speed floor, the transaction must terminate
   explicitly as infeasible rather than continuing indefinitely.
2. Preserve the pending physical-shaft zero mask and latched directions across
   logical coupling and final manager projection. A clipped coupled component
   must not reverse another participating shaft.
3. Add a bounded transaction time/travel watchdog with a distinct diagnostic,
   then force zero and fresh planning or fail the path cleanly.
4. Enable and verify `mppi_best_candidate_guard=true` at runtime; require the
   status diagnostics to show a scored candidate index before treating F-037
   as tested.
5. Repeat the same deterministic simulation and require transaction completion,
   no infeasible-boundary transaction, bounded closest-path error, and a
   successful action result before hardware testing.

## Confidence and limitations

The terminal boundary deadlock and inactive guard are directly observed in
the bag and supported by the current conversion/projection source. The action
timeout is a high-confidence inference from its approximately 60 s execution
and terminal ROS action status `ABORTED`; the action result topic was not
recorded. The bag recorder exited with `-6` during stack shutdown, but the
database contains the complete action interval used above.
