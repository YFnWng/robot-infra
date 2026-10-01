# Hardware sparse-point audit: 20260916_180903_mppi_demo

## Outcome

Point 6 faulted on `backlash_takeup_unconfirmed:axis_2`, but the recorded
sequence does not show a response-free tendon transaction.  It shows a causal
response arriving just after the estimator's pre-confirmation travel guard had
already failed the shaft.

The fault is therefore a timing race between fast encoder integration and the
delayed camera/UKF response path.  Extending full-rate take-up is not the
appropriate remedy: the shaft should stop at the nominal gap boundary and
wait, with a bounded timeout, for already in-flight response evidence.

Separately, points 1--5 all timed out.  This run did not demonstrate successful
sparse-point tracking before the point-6 fault.

## Experiment outcome

The initial home tip used to freeze the circle was
`[27.913, 15.732, 75.213] mm`.  Points 1--5 each used a 10 s budget and ended
with errors of approximately `10.170`, `10.971`, `9.496`, `9.457`, and
`8.968 mm`; all five were reported as timed out.

Encoder-position homing returned close to `[20, 0, 0]` between trials, but the
marker-derived home tip was not invariant.  The implied home tips before
points 2--6 differed by several millimetres, as expected for the intentionally
history-preserving hardware experiment.  Encoder homing does not erase tendon
remanence or torsional state.

Point 6 started with target error about `28.47 mm`.  Its action faulted after
about 2.6 s, with final reported error about `22.96 mm`.

## Terminal axis-2 transaction

- Take-up generation 35 began around `1789596636.003` with only shaft 2
  pending in the negative physical direction.
- The estimator's negative shaft-2 width had adapted to about `3.455 rad`
  (prior `3.587 rad`).  Status reported `remaining_rad` falling from
  `3.437` to zero by approximately `1789596636.303`.
- The fault was published around `1789596636.392`, about 89 ms after the
  sampled nominal-gap exhaustion boundary.
- Across the sampled terminal transaction, marker tip displacement was about
  `[+2.281, -4.376, +0.381] mm` (`4.950 mm` norm), and distance to the target
  improved from about `27.10` to `22.37 mm` over the matched marker window.
- The transaction was tendon-dominant.  The matched estimator window recorded
  roughly `-4898` shaft-2 counts, while the small shaft-1 change belonged to
  the preceding transaction/window boundary.

This is strong evidence that the tendon had mechanically engaged and was
moving the catheter in a useful direction.

## Timing mechanism

Accepted estimator corrections near the fault arrived 91--139 ms after their
camera source stamps (median 109 ms, P95 130 ms).  At the fault callback, the
latest causally evaluated distal-bending increment was only about `0.027`,
below the configured `0.05` threshold.  The first clearly suprathreshold
post-exhaustion sample had source time `1789596636.319`, projected bending
increment `-0.089`, and full tendon evidence, but it was published around
`1789596636.447`: about 55 ms after the controller had already faulted.

Subsequent accepted samples reached projected increments `-0.151`, `-0.240`,
and larger magnitude.  They cannot recover the transaction because the
encoder-side maximum-travel check has already changed shaft 2 to `FAILED`;
`observe_response()` treats only `TAKEUP` and `PROVISIONAL` shafts as pending.

The configured three-observation requirement was not itself reached.  More
fundamentally, the implementation allows the fast encoder path to consume the
entire pre-confirmation travel budget before the slower causal observation can
arrive.  The full-rate command continues during this evidence latency.

## Insertion, material roll, and simultaneous take-up

The session contains fourteen take-up generations in which shaft 0
(insertion) was pending.  Nine had no simultaneous shaft-1 pending state and
five (`1`, `7`, `12`, `14`, and `28`) explicitly drove insertion and rotation
take-up together.

For the insertion transactions without simultaneous rotation take-up, the
UKF material frame changed about its local z axis as follows.  `Observed roll`
is the complete pose change; `off-model residual` subtracts the fixed local
Jacobian prediction for all recorded shaft motion in the same window.

| Generation | Shaft-1 travel (rad) | Observed roll (deg) | Off-model residual (deg) |
| ---: | ---: | ---: | ---: |
| 2 | 0.000 | +0.669 | -2.325 |
| 4 | +0.425 | +0.279 | +1.824 |
| 11 | -0.006 | +1.263 | -1.412 |
| 18 | -0.104 | +0.517 | -1.961 |
| 21 | 0.000 | -2.458 | +1.413 |
| 24 | 0.000 | -0.581 | -3.056 |
| 26 | 0.000 | -1.112 | +2.412 |
| 30 | -0.274 | -0.697 | -3.350 |
| 32 | 0.000 | -0.812 | +2.229 |

The cleanest zero-rotation-shaft cases therefore changed observed material
roll by `0.58--2.46 deg`; their residual relative to the nominal insertion
Jacobian was `1.41--3.06 deg` in magnitude.  The alternating residual sign is
history dependent and is consistent with insertion releasing stored twist in
whichever direction had accumulated.  This confirms insertion-correlated,
off-model roll.  Calling the complete residual elastic torsion release remains
inferred-high-confidence rather than directly observed because pose and shaft
encoders alone cannot separate geometric backlash, elastic twist, and local
Jacobian error.

The five simultaneous insertion/rotation transactions had approximately
`0.5--0.8 s` of overlap.  During four negative-rotation overlaps, only about
`0.5--8.2%` of the fixed-Jacobian material-roll prediction appeared in the
measured material frame.  In generation 12 the rotation shaft moved positive,
but material roll moved `-4.60 deg` during the overlap, opposite the requested
rotation direction.  Thus simultaneous insertion does not ensure that
rotation take-up is transmitted; it can coexist with substantial untransmitted
rotation and can release previously stored twist strongly enough to dominate
the newly requested direction.

The exact newly stored elastic windup angle is not observable in this bag.
Raw shaft travel includes nominal transmission ratio, geometric gap, and
elastic storage.  The defensible conclusion is that the combined transaction
left most commanded rotation unexpressed at the material frame, not that every
missing motor radian became elastic torsion.

## Recommended correction

Introduce an explicit `AWAITING_RESPONSE` phase at nominal gap exhaustion:

1. When `remaining_rad` reaches zero without a credible response, command
   zero for that physical shaft and preserve the transaction direction and
   response window.
2. Wait a bounded response-grace interval sized from measured camera-to-UKF
   latency plus at least one accepted observation period.  The present data
   supports testing approximately 200 ms, with the exact value exposed and
   recorded.
3. If a credible causal response arrives during the grace interval, mark the
   shaft provisional, retain the existing zero/replan barrier, and generate a
   fresh MPPI plan from the corrected state.
4. If no response arrives, either fault at the unchanged reviewed travel bound
   or perform a separately bounded low-speed probe.  Do not continue the
   current full-rate take-up while waiting for delayed evidence.
5. Keep three observations for persistent engagement/width learning if
   desired, but stop take-up on the first credible response.  Do not require
   three observations before stopping physical take-up.

This correction preserves fail-closed behavior and does not alter manager,
joint-limit, freshness, or firmware safety enforcement.

## Classification

- Point-6 physical tendon response: observed.
- Fault-before-response-arrival race: observed/source-confirmed.
- Points 1--5 not reached: observed.
- Home-tip variation after encoder homing: observed and consistent with
  preserved mechanical history.
- Whether all sparse targets are physically reachable under the inherited
  history: unknown from this run.
- Insertion-correlated off-model material-roll transients: observed.
- Torsional release as the dominant source of those residuals:
  inferred-high-confidence.
- Exact stored/released elastic twist: unobservable from the available
  encoder/pose measurements.
