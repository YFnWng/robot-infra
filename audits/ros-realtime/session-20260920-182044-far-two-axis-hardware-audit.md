# Far two-axis hardware audit: point-2 settling and axis-0 home

Date: 2026-09-20

## Scope

Passive analysis of:

`/media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions/20260920_182044_mppi_demo`

No command was issued and no runtime setting was changed during this audit.

## Executive result

The first farther target reached with 0.834 mm final error. The second,
model-generated tendon target was geometrically approached but did not settle:
minimum error was 1.005 mm, yet its longest continuous interval inside the
1.8 mm tolerance was only 0.15 s, below the required 0.25 s. It timed out at
2.681 mm.

This was a model/plant and settling failure, not a marker-estimator failure or
a rotation leak. At the joint configuration the forward preview associated
with the target, the observed hardware tip remained 3.275 mm from that target.
The closest observed tip instead occurred at a very different insertion/tendon
configuration. MPPI repeatedly reversed both axes while trying to reconcile
that mismatch.

The point-3 decoupled tendon prehome succeeded. It returned tendon to zero
while holding physical chassis axis 0 fixed. The following insertion-only home
then timed out independently: it moved only 0.2575 mm of a requested 3.6263 mm
before stopping. The firmware endpoint gate required at least 0.9066 mm (25%)
of progress before permitting a correction, so it terminated with 3.3682 mm
remaining. No tendon limit transition interrupted this final transaction.

## Runtime health

- Controller: CUDA, 1,024 samples, grouped MPPI, v175 interface transmission,
  v171 distal history, adaptation disabled, logical rotation limit exactly
  zero.
- Rotation commands and POS feedback remained zero.
- Estimator health was `TRACKING` throughout point 2; consecutive rejection
  count remained zero.
- Marker post-fit RMS during point 2 was 0.359 mm P50, 0.541 mm P95,
  0.745 mm P99, and 0.870 mm maximum.
- One isolated planner deadline miss occurred; it did not repeat or fault and
  does not explain the 20-second oscillation.

## Target results

### Point 1: farther insertion candidate

- target action duration: about 1.28 s;
- error: 6.421 mm start, 0.630 mm minimum, 0.834 mm final;
- logical insertion: 19.997 to 28.677 mm;
- measured tip displacement: `[+0.380, +0.151, +6.127]` mm;
- response endpoint error: 0.494 mm P50 and 0.666 mm P95;
- median response direction cosine: 0.935.

The farther insertion candidate therefore remained well behaved.

### Point 2: farther tendon candidate

The controller preview realized the requested logical displacement as:

```text
delta    = [0.0205, 0, 4.5085] mm
endpoint = [20.0170, 0, 4.5085] mm
target   = [26.650, 21.846, 68.049] mm
```

Observed action statistics:

| Metric | Result |
| --- | ---: |
| Initial tip error | 7.318 mm |
| Minimum tip error | 1.005 mm |
| Final tip error | 2.681 mm |
| Samples inside 1.8 mm | 21 / 401 |
| Longest continuous in-tolerance interval | 0.15 s |
| Insertion range | 10.252 to 24.644 mm |
| Tendon range | approximately 0 to 8.524 mm |
| Insertion total variation | 138.621 mm |
| Tendon total variation | 19.152 mm |
| Planned insertion sign reversals | 17 |
| Planned tendon sign reversals | 10 |

The hardware configuration nearest the preview endpoint was
`[19.975, 0, 4.594]` mm. Its observed tip was
`[25.240, 19.872, 70.249]` mm, still 3.275 mm from the generated target.
The closest observed target approach occurred instead near
`[16.490, 0, 8.471]` mm, with 1.006 mm tip error. Thus the hardware required
roughly twice the previewed tendon travel plus a substantial insertion change.
The preview target was transiently reachable, but not at the model-predicted
actuation state.

Across 121 response windows:

- endpoint prediction error was 1.113 mm P50, 4.046 mm P95, and 6.636 mm max;
- median direction cosine was 0.451;
- 33.9% of valid measured responses opposed the predicted direction;
- predicted displacement norm was 1.055 mm median versus 0.507 mm measured.

The mismatch was history-dependent rather than a single gain error. Early in
the maneuver, one logged response predicted -1.111 mm in z but measured
-2.228 mm, producing an overshoot. Later windows often predicted more motion
than the plant delivered while take-up was active. That alternating mismatch
explains the repeated corrective reversals and failure to dwell at the target.

## Point-3 home isolation

After point 2, the runner computed a tendon prehome target of
`[16.3743, 0, 0]` from the current logical state. During this transaction:

- logical insertion and tendon each changed by -2.5607 mm;
- their difference, physical chassis axis 0, stayed fixed;
- the tendon lower-bound transition occurred during this isolated leg;
- firmware reported `POSITION_COMPLETE mask=0x04`;
- the final state was `[16.3737, 0, -0.0006]`.

This validates the new staging calculation and proves the earlier global-stop
interaction was removed.

The subsequent final home was axis 0 only (`mask=0x01`):

```text
requested: 16.3737 -> 20.0000 mm
observed:  16.3737 -> 16.6312 mm
progress:  0.2575 mm (7.1%)
residual:  3.3688 mm
```

After 500 ms without further encoder progress, `PositionMoveTracker` compared
the progress with its 25% threshold. The required progress was approximately
0.9066 mm, so it did not schedule an endpoint correction and emitted
`POSITION_TIMED_OUT`.

The repeatable 3.368 mm residual is consistent with insertion-direction
take-up between the motor command and the downstream chassis response. The
current firmware position transaction assumes enough downstream response from
the first driver move to authorize a retry. That assumption is incompatible
with a reversal whose first move is mostly consumed by mechanical play.

## Findings

### F-182044-1: Far tendon preview does not identify the hardware endpoint

Severity: high
Confidence: observed

The point-2 target was generated from `[20.017, 0, 4.509]`, but the hardware
was still 3.275 mm away when it revisited that configuration. The closest
approach required approximately `[16.490, 0, 8.471]`. This invalidates the
farther tendon target as a clean controller-only isolation.

### F-182044-2: Closed-loop reversals are a consequence, not the first cause

Severity: high
Confidence: inferred-high-confidence

MPPI crossed close to the target repeatedly, but inconsistent short-horizon
response magnitude and direction made it reverse 17/10 times on insertion and
tendon. The target was not strictly unreachable; it was not stably predictable
under the current model and history state.

### F-182044-3: Axis-0 position completion is not take-up aware

Severity: high
Confidence: observed

With tendon already settled and no limit event, the axis-0-only home produced
only 7.1% downstream progress. The fixed 25% retry qualification rejected the
transaction before take-up could be completed. Weakening this safety gate
without adding bounded take-up semantics is not recommended.

## Recommended next steps

1. Do not repeat the full farther four-target block unchanged.
2. Preserve the decoupled tendon prehome; it worked and isolated the fault.
3. Characterize axis-0 reversal take-up with a bounded, low-speed downstream
   feedback experiment before changing homing. A position-home implementation
   should explicitly distinguish take-up from a jam and retain finite travel,
   time, hard-limit, encoder-integrity, and watchdog bounds.
4. Return tendon isolation to the qualified 3 mm candidate, then acquire a
   monotonic one-way 4.5 mm tendon response from `[20,0,0]` without MPPI
   reversals. Compare the measured endpoint to the preview before using the
   farther tendon target in closed loop.
5. Do not advance to continuous two-axis hardware tracking until the preview
   endpoint and the axis-0 home transaction both pass independently.
