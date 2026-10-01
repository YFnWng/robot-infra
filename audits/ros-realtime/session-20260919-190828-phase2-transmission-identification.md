# Phase 2 separated-transmission identification

Date: 2026-09-20

Session: `20260919_190828_causal_proximal`

Machine-readable result:
`session-20260919-190828-phase2-transmission-identification.json`

## Outcome

The offline Phase 2 identification step is implemented and repeatable. It
uses timestamp-paired raw encoder counts and accepted marker observations,
holds repetition 3 out of the fit, and never modifies a model artifact.

The main result is not a unique pair of backlash widths. It is a stable total
cascaded bend reversal gap of approximately **7.62--7.65 mm** when the
motor-to-knob gap is constrained to the mechanically plausible small range of
0--1 mm. Phase 2 strongly supports a stateful gap between raw knob-drive
motion and distal bending, but it cannot uniquely allocate that gap between
motor-to-knob play and handle-body clamp travel.

Given the direct mechanical observation that motor-to-knob play is small, the
appropriate model initialization is a small fixed/prior-regularized knob-drive
gap and a handle-clamp state carrying most of the remaining gap. This is a
modeling choice informed by both data and mechanism; it is not a uniquely
identified physical measurement.

## Method

The tool is:

```text
audits/model-validation/identify_phase2_transmission.py
```

It performs four analyses:

1. selects stateful asymmetric shaft-0 and shaft-2 play widths using the
   directly measured proximal marker position, not the current UKF pose;
2. after fixing each width, fits the corresponding six-dimensional UKF
   interface-pose column;
3. evaluates causal superposition of the isolated columns on Phase 2C;
4. fits an additional handle-clamp play against rigid-motion-invariant marker
   pairwise distances and, separately, UKF distal strain.

All fits use 0.25 s backward-looking windows that remain inside one labelled
episode. Repetitions 1 and 2 train the fit; repetition 3 is held out. Of 9,588
accepted estimator traces, 9,587 were matched to their source marker cloud
within 2 ms.

## Interface-drive diagnostics

| isolated basis | positive side | negative side | full reversal | held-out improvement over no play |
| --- | ---: | ---: | ---: | ---: |
| shaft 0 / chassis | 3.045 mm | 0.990 mm | 4.035 mm | 26.9% |
| shaft 2 / unconstrained allocation | 4.313 mm | 2.710 mm | 7.023 mm | 29.4% |

The shaft-0 result is useful evidence for the comparatively large
motor-to-chassis transmission gap.

The unconstrained shaft-2 value must not be interpreted as motor-to-knob play.
The distal data admit a trade between motor-to-knob play and handle-clamp
play. When the unconstrained first play consumes about 7.0 mm, the downstream
marker fit assigns only another 0.62 mm to the clamp. This is an arbitrary
factorization of nearly the same total gap.

After nominal gain conversion, the fitted chassis and knob interface
translation columns have direction cosine 0.834. They are related but not
identical, so retaining separate columns `[c,r,k]` is preferable to forcing
both translations into one column at this stage.

## Cascaded-gap sensitivity

Fixing a small symmetric motor-to-knob reversal gap and refitting the clamp
gives:

| assumed knob-drive reversal | marker-inferred clamp reversal | combined total | held-out marker improvement |
| ---: | ---: | ---: | ---: |
| 0.00 mm | at least 7.633 mm | at least 7.633 mm | 41.2% |
| 0.25 mm | 7.396 mm | 7.646 mm | 40.8% |
| 0.50 mm | 7.120 mm | 7.620 mm | 40.4% |
| 1.00 mm | 6.625 mm | 7.625 mm | 39.6% |

The zero-prior fit reaches its search boundary, so 7.633 mm is a lower bound
in that row. Across the interior fits the combined total is essentially
constant. UKF distal-strain fits show the same trade and improve held-out
prediction by 51--54%, but are model-dependent and therefore secondary
evidence.

The approximately 7.6 mm full reversal gap is compatible with the earlier
2.6--3.2 mm directional-onset estimates: a full reversal traverses both
directional side offsets, whereas the earlier hinge diagnostic reported one
loading direction at a time.

## Phase 2C superposition

The isolated-column model was evaluated without refitting on 2,138
compensated-bending windows:

- median normalized squared error: 0.0711;
- mean normalized squared error: 0.3796;
- only 856 windows had a nonzero play-transmitted prediction;
- median direction cosine on those active windows: 0.586;
- fifth-percentile direction cosine: -0.433;
- fast held-out repetition 3 had the largest group mean error, 0.509.

Thus, stateful drive play plus two fixed interface columns is not yet an
adequate coupled transient model. The result supports proceeding with the
explicit clamp state and refactored v171 history rather than replacing the
runtime with these fitted columns alone.

## Confidence and limits

- **Observed:** raw encoders, marker positions, labels, and held-out errors.
- **Inferred-high-confidence:** a substantial cascaded shaft-2 reversal gap is
  required; the total is much more stable than its factorization.
- **Inferred:** shaft-0 play is materially asymmetric in this session.
- **Mechanism-informed:** most shaft-2 gap belongs to handle-body compliance
  because independent physical observation says motor-to-knob play is small.
- **Unknown:** the exact knob/clamp split without a handle-body or knob
  fiducial.

Marker pairwise geometry removes rigid interface motion but is not a complete
distal shape coordinate. The UKF pose/strain are retained only as secondary
state diagnostics because they inherit the current model. This output is not
a deployable checkpoint or a safety parameter.

## Next implementation step

Implement the batch-safe `ProximalTransmissionState` in `cr_meta_lnn` and run
the planned A/B/C offline replay:

- explicit small knob-drive play plus clamp play, downstream gross play zero;
- the same plus shrinkage-regularized residual downstream play;
- unchanged v171 baseline.

Initialize the combined knob/clamp full-reversal prior near 7.6 mm, but sweep
the allocation and require episode-held-out marker improvement before choosing
a new artifact.
