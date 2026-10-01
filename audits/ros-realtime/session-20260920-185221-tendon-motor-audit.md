# Tendon-motor isolation audit: 20260920_185221_causal_proximal

## Verdict

The session is a valid, fault-free isolated shaft-2 experiment. It supports a
real-hardware tendon reversal delay substantially larger than the gross motor
play encoded in v171. The dominant observed distal-shape turnaround occurs
after approximately 5.3 mm of downstream knob-coordinate travel, whereas the
v171 scalar tendon chain stores a 0.939 mm one-sided motor-play width (1.877 mm
full reversal window).

This 5.3 mm value is an effective command-to-distal-turnaround distance. It
includes tendon transmission, continuing distal relaxation/dynamics, camera
sampling, and UKF filtering; it must not be relabeled as pure mechanical motor
backlash. It is nevertheless the quantity that the rollout must predict to
schedule a reversal correctly.

## Session integrity

- Bag duration: 206.05 s.
- Accepted estimator traces: 4,012; matched marker rows in the exported replay:
  4,010.
- Causal command traces: 18,787; causal response windows: 8,275.
- Safety events: 0; device events: 0; motion-watchdog advisories: 0.
- Encoder telemetry was finite.
- Static translation increment: median 0.0159 mm, p95 0.0493 mm.
- Static rotation increment: median 0.000462 rad, p95 0.001318 rad.
- Online model adaptation was disabled.

The generic Phase-0--2 evaluator reports REVIEW only because this tendon-only
schedule intentionally omits the other required excitation bases. That is not
a failure of this isolation experiment.

## Excitation isolation

The schedule contains three slow and three fast `shaft_2` repetitions between
static start/end blocks. In every moving repetition:

- predicted raw shaft-0 request peak: 0;
- measured raw shaft-0 encoder span: 0 counts;
- shaft-0/shaft-2 encoder-span ratio: 0;
- shaft-2 span: 71,774--72,434 counts.

Therefore chassis-axis motor motion did not contaminate the commanded
excitation. This does not by itself measure uninstrumented physical backdrive,
but it rules out commanded or encoder-visible shaft-0 coupling.

## Distal evidence and v171 comparison

The distal response was evaluated two ways:

1. UKF distal strain projected onto the frozen v171 learned tendon-bending
   mode.
2. A rigid-motion-invariant scalar obtained from the six pairwise distances
   among the four observed markers.

The two signals correlate at 0.9943, and the first marker-shape principal
component explains 99.68% of the marker-shape variance. Thus the delayed
turnaround is present in raw marker geometry and is not an interface-pose-only
UKF artifact.

For each of the 12 shaft-2 reversals, travel was measured from the motor
extremum to the subsequent distal-shape extremum. Nine of 12 marker-shape
events form a dominant 5.30--6.84 mm cluster; the overall median is 5.305 mm.
Three events turn earlier (1.05--1.35 mm), showing that the effective delay is
history dependent rather than a single deterministic deadband.

The deployed v171 checkpoint contains:

- one-sided scalar tendon `motor_width`: 0.93864 mm;
- full ideal play reversal window: 1.87728 mm.

Continuous replay of the same encoder history makes the v171 internal motor
play turn after roughly 1.46 mm under the same smoothed-extremum diagnostic,
and its predicted equilibrium-shift trend turns after a median 0.40 mm because
the learned relaxation/memory terms continue evolving. Both are much earlier
than the dominant measured distal-shape turnaround.

## Interpretation

The experiment supports the original concern: the deployed v171 preview is
over-optimistic immediately after a shaft-2 reversal. It predicts useful
distal response while the measured catheter usually continues its previous
shape trend for several additional millimetres. This can make MPPI believe a
relaxation command has already changed the distal state, then replan or stop
before the hardware has actually turned around.

The numerical proximity between the dominant distal turnaround (~5.3 mm) and
the separately fitted v175 interface-knob reversal window (5.470 mm) is
not proof that they are the same state. v175 interface play must remain
separate from v171 tendon history. The next fit should use this tendon-only
session to identify a tendon-side upstream transmission state while keeping
the frozen v171 distal mechanics and preserving history continuously across
all repetitions.

## Artifacts

- `session-20260920-185221-tendon-motor-evaluation.json`: generic causal
  experiment evaluation.
- `session-20260920-185221-tendon-replay.npz`: accepted, ROS-independent
  estimator/encoder/marker replay table.
