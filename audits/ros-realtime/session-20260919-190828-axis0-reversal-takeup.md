# Phase-2 axis-0 reversal take-up

Source replay:
`20260919_190828_causal_proximal/phase2_transmission_replay.npz`.

This diagnostic uses the six insertion-only episodes. Shafts 1 and 2 are
constant in these blocks, providing 12 qualified shaft-0 reversals across the
slow/fast tiers and three repetitions. History is continuous; episode labels
do not reset the play state.

For each reversal, posterior interface SE(3) displacement is projected onto
the deployed v175 physical-Jacobian shaft-0 column using the checkpoint state
scales. First credible response is defined as 0.25 mm of direction-aligned
projected response sustained for three accepted camera samples.

## Result

| Direction after reversal | Events | Median travel | P05--P95 |
|---|---:|---:|---:|
| negative | 6 | 4.387 mm | 4.209--4.495 mm |
| positive | 6 | 4.308 mm | 4.279--4.506 mm |
| combined | 12 | 4.354 mm | 4.235--4.522 mm |

The 0.079 mm difference between directional medians is small relative to the
event spread and does not support a mechanically large directional asymmetry.
The earlier marker-objective asymmetric fit reported a 4.035 mm total window
(3.045 mm plus 0.990 mm); that split should not be interpreted as two physical
one-sided gaps.

The deployed symmetric v175 fit reports a 5.374 mm full reversal window. It
is 1.021 mm larger than the median first-response travel. These quantities
answer different questions: 5.374 mm minimizes multi-step pose prediction
loss, while 4.354 mm marks the first persistent 0.25 mm interface response.
Use the latter for an observation-based engagement threshold and retain the
former as a conservative prediction/limit reserve until a held-out run shows
that reducing it improves endpoint prediction.

Machine-readable events and the response curves are in:

- `session-20260919-190828-axis0-reversal-takeup.json`
- `session-20260919-190828-axis0-reversal-takeup.png`

The reproducible diagnostic is
`cr_meta_lnn/scripts/diagnose_axis0_reversal_takeup.py`.

## Next isolated tendon check

Use the existing `tendon_motor` causal schedule. It commands equal logical
insertion and bending, so the firmware relation `raw0 = lin - bend` holds raw
shaft 0 fixed and excites raw shaft 2 alone. Compare the finalized session to
the v171 preview using the recorded encoder trajectory, not merely the nominal
command. This preserves the continuous tendon history and prevents actuator
tracking error from being attributed to the distal model.
