# Memory-cell law grouping: observation-only measurement

2026-11-01. Offline recorded data; no estimator source, hardware or default
changed. **The declared decision rule FAILED on the chassis-motion segment.**
No estimator grouping was implemented.

## Question and declared rule

Hypothesis: the number K of distinct conditional input laws stays small through
chassis onset and reversal, so one completion could serve many memory cells.
Two cells share a group if displacements agree within eps mm and their complete
input mean, covariance and previous-image history agree within eps, normalized by
mixture standard deviations. Decision rule declared before running: median K<=8
and P95 K<=16 at eps<=1e-3.

A wrapper ran beside the unchanged scalar filter and compared each cell's actual
output with its group representative's. Mixture errors below are ONE-STEP
counterfactuals from the exact scalar state; sharing error was not fed back, so
accumulated drift is NOT measured.

## Results

Gain coordinate and threads are as run; K is taken over active cells per image.

| Segment | Images | eps | K median/P95/max | Rule | Max mixture tip error, mm | Max weight TV |
|---|---:|---:|---:|---|---:|---:|
| Physical 664-672, stationary chassis | 8 | 1e-3 | 1/1/1 | Pass | 7e-11 | 6e-10 |
| Log 552-576, tendon onset, 0.3 mm chassis burst | 24 | 1e-3 | 1/4/12 | Pass | 1.3e-6 | 5e-5 |
| Log 445-485, sustained chassis motion | 40 | 1e-3 | 18/42/58 | **FAIL** | 9e-6 | 6.5e-5 |
| Log 445-485, informational | 40 | 1e-2 | 17/27/40 | Would fail | 8.7e-5 | 3.7e-3 |

Observed boundaries in 445-485:

- About 23 of 255 cells have nonzero transmitted displacement per image.
- K settles near 18 and peaks at 58 (image 457) during images 454-460, while the
  wall probability flips.
- Inferred, not separately measured: the floor near 18 comes from distinct
  displacements. Chassis motion of about 0.4 mm per image crosses about 7 of the
  0.0555 mm grid intervals on each of the two recovered supports.

Cell-level likelihood errors can be large at loose tolerance (up to 8.8 nats at
eps=1e-2) even though mixture errors stay small.

Low-mass cells must not be pruned on mass alone. At image 457 ESS goes from 1.0 to
123 after the image, about 9 nats of likelihood reviving low-weight recovery
cells. At image 570 it goes from 129 to 1.

## Physical-gain boundary (separate finding)

The physical-gain scalar route rejects `Physical gain requires positive
interface-parameter contexts` on two of the three selected real segments:

- image 446 from a fresh 445 start, reproduced without the wrapper;
- image 569 from a fresh 552 start, wrapper run only.

Its completed real evidence is the stationary-chassis 664-672 run. The two moving
segments completed only in log gain, so their results are log-gain, while the
3.5-s timing fixture is physical-gain.

Root cause at image 446 (observed): this is not a loss of positivity. The only
failing row is a zero-mass wall cell (log weight -inf) holding the all-zero
placeholder that `_position_mixture_moments` returns for empty grid cells, while
every active cell has alpha=1.1331. The check at `memory_marker_runtime.py:146` covered
all rows, including inactive ones. Image 569 is the same kind of first small chassis
motion, and the repair below confirmed the same cause there.

A scratch process applying the gate to active rows only completed 446-461 with the tip
within 5e-5 mm of log gain at the sampled even images. The knob scale stayed at its
initial value there, so alpha was not exercised:
positivity under real knob motion remains UNTESTED. No source was changed.

## Controls

- Wrapped and unwrapped 664-672 runs at default threads match exactly, as do two
  repeated unwrapped runs. The wrapper is observation-only and the route repeatable.
- A 4-thread run differs by at most 8.9e-11.
- Current runs differ from the archived 664-672 report by at most 6.6e-9, and log
  552-576 differs from its archive by at most 3.0e-9. Source has changed since those
  archives (shared extractions) and this was not traced further. Roundoff-scale, but
  not exact, so K at eps=1e-8 is near this noise level.

## Limits

Three short segments only, selected for onset and sustained motion; no chassis
direction reversal is confirmed in them. Mixture errors are one-step and do not
include forecasts. Step timings were taken under parallel load with the wrapper and
are not latency evidence. Total measurement wall time was about 4 minutes.

Raw per-image JSON, final-image checkpoints and the wrapper script were left in the
session scratchpad because the session drive is at its 384 MiB reserve. Rerun with
the recorded settings if the scratchpad is gone.

## Follow-up: displacement interpolation shadow (FAILED)

Hypothesis: group cells on their prepared previous-image law, then evaluate a few
displacement nodes per group and interpolate likelihood, mean and covariance over
whole-image chassis displacement. Declared rule: evaluations median<=8 and P95<=16,
mixture tip error<=1e-3 mm and weight TV<=1e-3 per image. Primary configuration
eps=1e-3 with 3 nodes; other combinations informational. All are one-step, log gain,
from the exact scalar state. No configuration passed on the combined images.

| Images | Configuration | Evaluations median/P95/max | Max tip error, mm | Max TV |
|---|---|---:|---:|---:|
| Combined (59) | 1e-3, 3 nodes | 6/26/48 | 0.028 | 0.55 |
| Combined (59) | 1e-2, 4 nodes | 8/16/31 | 0.0064 | 0.21 |
| 446-465, wall probability flipping | 1e-3, 3 nodes | 12/36/48 | 0.028 | 0.55 |
| 486-500, wall settled, learning mostly admitted | 1e-3, 4 nodes | 8/8/8 | 2.4e-6 | 3.1e-5 |
| 553-576, tendon onset | 1e-3, 3 nodes | 1/4/4 | 1.3e-6 | 3.3e-6 |

Failure boundary: images 455-465, while memory is still uncertain between wall
contact and interior play. There, interpolated log likelihood on cells with real mass
is wrong by up to 6.2 nats with 3 nodes and 0.9 with 4, and group counts reach 44.
Mixture tip error stays far below 1 mm, but the memory weights are materially wrong,
and how that propagates to later engagement and forecasts is not measured. Likelihood
is the failing quantity; whether moments alone interpolate adequately is not separated.

The positive-semidefinite comparison and the mixture covariance error in this shadow
are unreliable: normalizing by floored near-zero variances inflates roundoff (the exact
scalar cells themselves report large negative values in these units). Do not read them
as evidence for or against interpolated covariance validity.

## Follow-up: exact likelihoods with interpolated moments (FAILED)

Only change: every cell keeps its exact likelihood, so weights are exact by
construction and TV is zero. Same rule, images 446-465, log gain, one-step.

| Configuration | Evaluations median/P95/max | Max tip error, mm | Worst correlation eigenvalue, interpolated cells |
|---|---:|---:|---:|
| 1e-3, 3 nodes (primary) | 12/36/48 | 0.0056 | -0.034 |
| 1e-3, 4 nodes | 14/38/50 | 4.4e-4 | -0.0049 |
| 1e-2, 4 nodes | 10/21/31 | 4.7e-4 | -0.0059 |

Exact likelihoods reduce tip error about 5x at the primary configuration, and 4 nodes
meets the tip limit. It still fails: evaluation counts exceed the rule, with up to 44
genuinely distinct laws at image 457. Interpolated covariances are also INDEFINITE, while
the exact cells on the same per-cell metric reach only -8.6e-15. That corrected metric
normalizes each cell by its own variances and skips coordinates with variance <=1e-10.
Those states would not be valid Gaussians, and existing checks exist to reject such
states. Entrywise moment interpolation is therefore rejected. Repairing it, for example by
interpolating factors, would be a further mechanism and is not proposed here.

## Physical-gain gate repair (implemented)

Authorized change: `memory_marker_runtime.py` applies the physical positivity check to
active rows only. Every active row still fails closed; zero-mass rows stay present and
unaltered. Regression test added in `test_physical_gain_integration.py`; with the array
state and profile-validation suites, 59 tests pass. The source fingerprint changes, so
existing checkpoints do not resume. No estimator equation changed.

Physical gain on fresh 552-576, knob moving on most images, completes all 24 images
including 569, where one zero-mass row appears. This confirms the inferred cause there.
Compared with the matched log-gain run:

- active alpha stays positive, minimum 0.3335 at image 575;
- mixture alpha is 1.8 standard deviations from zero at that minimum, so the margin is
  not large, and positivity remains an approximation of the first-order update;
- tip differs by at most 0.022 mm and lambda by at most 7e-15.

Not established: physical gain over sustained chassis motion past image 461, forecast
accuracy, or longer sessions. Log gain remains the baseline until those are compared.

Next isolated question: the per-evaluation cost floor of the conditional completion, which
decides whether exact grouping plus a compiled kernel can meet the budget. No grouping or
interpolation is implemented.
