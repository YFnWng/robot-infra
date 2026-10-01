# Insertion/rotation coupling audit: 20260920_191451_causal_proximal

## Verdict

The session is usable and gives strong evidence that axial motion transmits
previously stored torsional windup to the distal catheter/interface even while
the rotation motor is held fixed.  This is the insertion-causes-torsion-release
effect sought by the experiment.

Across all 12 insertion probes, the posterior interface roll changed in the
same direction as the preceding signed rotation preload.  The median absolute
change over one `20 -> 26 -> 20 -> 14 -> 20 mm` cycle was 22.84 degrees
(range 13.64--28.73 degrees).  A bending-plane orientation computed directly
from the four observed marker positions changed by a median 17.94 degrees in
the same direction, and its per-probe changes correlate with the UKF interface
roll changes at 0.9987.  The effect is therefore not an interface-pose-only UKF
artifact.

"Release" here means that rotation stored upstream during the preload becomes
visible downstream: after a positive preload, insertion makes the observed
interface/bending plane rotate further in the positive direction; after a
negative preload, it moves further in the negative direction.  It is not
expected to rotate back toward zero while the stored twist is being
transmitted.

## Session integrity

- Session completeness: PASS.
- Causal command traces: 71,244.
- Accepted estimator traces: 14,934; matched marker samples: 14,933.
- Marker messages: 22,541; device-state messages: 136,164.
- Safety events and latched device faults: 0.
- Online model adaptation was disabled.
- The runtime identity recorded the expected v171 distal checkpoint, v174
  Jacobian initialization, v175 interface-transmission checkpoint, and UKF.

The generic causal evaluator reports REVIEW only because this custom coupling
schedule intentionally omits the full standard Phase-0--2 basis set.  That is
not an experiment failure.

Six motion-watchdog advisories formed three SUSPECTED/RETRYING pairs during
rotation setup/unwind transitions:

- positive-fast repetition 1, bias exit;
- positive-fast repetition 2, bias exit;
- negative-fast repetition 3, bias enter.

There was no confirmed or latched stall.  These transitions should be tagged
when fitting a quantitative preload-response model, especially the last one,
but none occurred inside an insertion probe.

## Isolation checks

During every insertion probe:

- rotation-encoder span: exactly 0 counts;
- tendon-encoder span: exactly 0 counts;
- only insertion was commanded;
- the measured insertion loop returned to its start within 0.141 mm;
- the measured insertion span was 11.65--11.79 mm, consistent with the
  commanded +6/-6 mm excursion.

Consequently, the persistent roll change cannot be explained by a rotation
motor command, an encoder-visible rotation, a tendon command, or failure to
return the insertion encoder to its starting coordinate.

## Coupling magnitude

Mean full-cycle signed changes (mean +/- sample standard deviation across
three repetitions) were:

| Preload | Speed | UKF interface roll | Raw-marker bending-plane angle |
|---|---:|---:|---:|
| negative | slow | -25.13 +/- 4.41 deg | -20.35 +/- 4.18 deg |
| negative | fast | -21.56 +/- 3.21 deg | -17.33 +/- 2.86 deg |
| positive | slow | +21.58 +/- 6.08 deg | +17.04 +/- 3.49 deg |
| positive | fast | +19.77 +/- 5.31 deg | +16.11 +/- 3.24 deg |

All 12 full-cycle UKF changes and all 12 raw-marker changes agreed with the
preload sign.  The first forward-insertion leg released the largest portion:
its mean settled change was 12.05 degrees in the UKF and 12.99 degrees in raw
marker geometry.  Mean absolute settled changes then fell to 4.83, 3.38, and
1.75 degrees for legs 2--4 in the UKF.  This decaying response is consistent
with depletion of a stored torsional state rather than a fixed cross-axis
Jacobian.

The effect is well above drift.  The initial 14.9-second static block changed
by only 0.056 degrees in the UKF roll measure and 0.206 degrees in the marker
measure.  Even the 56.7-second final static interval changed by only 2.02 and
0.54 degrees, respectively.

Slow probes released modestly more roll than fast probes.  This leaves axial
travel and elapsed-time/viscoelastic relaxation partially confounded; this
session establishes the coupling but does not uniquely identify a purely
distance-driven law.

## History finding

The same commanded +/-75-degree motor preload did not establish the same
incremental interface roll on every repetition.  Bias-entry increments ranged
from roughly 7.5 to 65.9 degrees and alternated between low- and high-response
histories.  Returning the rotation motor command to zero therefore did not
reset the hidden torsional state.  The experiment must be interpreted as one
continuous-history trajectory, not as 12 reset trials.

## Modeling consequence

The proximal transmission model should couple insertion to the hidden
torsional state.  A suitable next model is a torsional reservoir in which:

1. rotation motor motion loads/unloads stored twist;
2. axial motion increases transmission/relaxation of that stored twist;
3. released twist advances interface material roll with the sign of the
   stored state;
4. the available reservoir decays as release occurs;
5. history remains continuous across episode labels and motor-zero returns.

This term belongs upstream of the frozen v171 distal mechanics.  It should not
be folded into a constant insertion Jacobian, because the coupling changes
sign with torsional history and decays strongly over repeated axial legs.
Rotation-only excitation is still needed to identify the reservoir loading
law cleanly; the present session is sufficient to identify and validate the
axial-release component conditional on the observed preload state.

## Artifacts

- `session-20260920-191451-insertion-rotation-evaluation.json`: generic causal
  experiment evaluation.
- `session-20260920-191451-insertion-rotation-replay.npz`: accepted,
  ROS-independent encoder/pose/strain/marker replay table.
