# Phase 2 handle-clamp compliance audit

Session: `20260919_190828_causal_proximal`

## Conclusion

The Phase 2 data support a two-stage bending transmission mechanism: shaft-2
motion can move the catheter interface/root before producing comparable distal
bending, and a finite additional travel is required after a reversal before the
distal bending response becomes approximately linear. This is consistent with
the observed compliant handle-body clamp, where initial knob travel drags the
handle body and later travel becomes relative knob/body (tendon) actuation.

The attribution to handle-body motion is high-confidence given the direct
mechanical observation, but is not uniquely identifiable from the present
sensors. Sheath play, knob-clamp motion, and distributed friction can produce a
similar trace. A handle-body fiducial or relative knob/body encoder would make
the attribution direct.

## Evidence

Static-estimator noise thresholds from Phase 2 were used to distinguish motion
from background variation:

- interface translation step p99: 0.0625 mm;
- interface rotation step p99: 0.0962 degrees;
- data-derived distal bending-coordinate threshold: 0.0798.

A piecewise-linear hinge was fitted to accumulated shaft-2 encoder travel on
the reversal legs. The interface response began without a resolvable hinge,
while the distal bending response required substantial travel:

| excitation | direction | interface hinge | distal hinge | equivalent logical bending travel |
| --- | ---: | ---: | ---: | ---: |
| shaft-2 isolated | negative | 0 counts | 20,574 counts | 3.06 mm |
| shaft-2 isolated | positive | 0 counts | 21,283 counts | 3.17 mm |
| compensated bending | negative | 0 counts | 18,308 counts | 2.73 mm |
| compensated bending | positive | 0 counts | 17,608 counts | 2.62 mm |

Conversions use 8,000 counts/revolution and 1.190625 mm/revolution. Therefore,
the inferred interface-to-distal transition is approximately 2.6--3.2 mm of
logical bending travel. This agrees in scale with the earlier operational bend
deadband estimate of about 3.108 mm.

Other Phase 2 results agree with this interpretation:

- isolated shaft-2 response has the correct direction (median direction cosine
  0.978) but only 0.730 of the nominal model gain;
- compensated-bending residuals remain large (roughly 0.74--0.77 of measured
  response) and cluster near reversals and endpoints;
- endpoint remanence remains after returning the encoder command;
- compensated response is not fully reproduced by instantaneous linear
  superposition of the isolated shaft-0 and shaft-2 responses.

The hinge estimate is a pooled diagnostic rather than a calibrated physical
parameter. Camera cadence, learned distal relaxation, and the finite velocity
profile limit onset-time resolution.

## Mismatch with the deployed model

The deployed external `BacklashStateEstimator` constructs one effective motor
coordinate for the local interface Jacobian. While it is in take-up, that
coordinate is held, so the interface Jacobian is prevented from responding.
At the same time, the raw motor coordinate is sent into the v171 distal
transmission/history model.

That ordering does not represent the observed mechanism:

1. initial shaft-2 travel can move the knob, catheter root, and interface;
2. the compliant handle body follows, leaving little relative knob/body tendon
   actuation;
3. after the handle body stops, additional travel becomes effective tendon
   actuation and produces distal bending.

The learned v171 distal model already contains motor play and asymmetric
memory/relaxation, so it can absorb some delayed response. However, the current
external take-up state conflates interface transmission, clamp compliance, and
distal tendon play. Its configured shaft-2 widths (34.858 rad positive and
3.587 rad negative) are also unlike the roughly symmetric 14--17 rad
interface-to-distal transition inferred here; those operational widths likely
combine several mechanisms rather than measuring clamp travel alone.

## Recommended model structure

Introduce a separate handle-body/clamp displacement state instead of applying
one deadzone to both interface and tendon dynamics. Conceptually, for raw
shaft-2 coordinate `q2`:

```text
q2 -------------------------------> interface/root kinematics
 |
 +--> directional clamp/play state h --> q_tendon = q2 - h
                                      --> learned distal transmission/history
                                      --> distal strain and shape
```

During initial common motion, `delta h` is approximately `delta q2`, so
`delta q_tendon` is small. Once clamp travel is exhausted, `delta h` becomes
small and subsequent `delta q2` becomes effective tendon actuation. The state
must be directional and history-dependent and should retain the existing v171
distal relaxation downstream rather than replacing it.

Recommended identification sequence:

1. Fit interface/root shaft-2 response from the isolated shaft-2 episodes.
2. Fit the directional clamp/play state from the separation between interface
   onset and distal-bending onset, using 2.6--3.2 mm only as an initial prior.
3. Drive the existing distal transmission/history with the inferred relative
   tendon coordinate, not directly with raw shaft-2 motion.
4. Validate on held-out isolated repetitions, then on compensated-bending
   episodes without refitting.
5. Compare current and two-state models using marker RMSE, interface-pose RMSE,
   distal-bending onset error, reversal response, and endpoint remanence.

Do not deploy the pooled hinge estimates directly as safety or control
parameters. First perform an offline replay comparison and verify that the new
state improves held-out prediction rather than merely shifting error between
the interface and distal models.

The cross-repository implementation plan is maintained in
`PROXIMAL_TRANSMISSION_AND_DISTAL_HISTORY_MODEL_PLAN.md`.
