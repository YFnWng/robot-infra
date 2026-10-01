# 2026-09-19 v174 rotation-disabled axial trial

Session: `20260919_171058_mppi_demo`

## Scope and evidence

This was a two-target active-hardware isolation trial. The experiment homed to
encoder position `[20,0,0]` before each target, disabled shaft rotation, and
commanded tip offsets of `+5 mm` and `-5 mm` along base Z. The analysis used a
read-only snapshot of the still-open rosbag; stop the recording launch to
finalize the original bag metadata.

Runtime `/parameter_events` observed:

- Jacobian: `real_joint_local_distal_v174.json`
- Jacobian SHA-256: `6d3f8573d10511dcbea3c27c97141f132b95cc42c450d3aec3702db33430ad2c`
- `controller_velocity_max=[10,0,4.5,4,25,25]`
- `adaptation_enabled=false`
- `marker_estimator=ukf`
- `device=cuda`, `samples=1024`
- grouped-mode sampling, backlash compensation, and take-up transactions
  enabled
- `command_output_enabled=true`

The manifest's `controller_parameters` mapping contains the launch defaults
before the controller-profile overlay (for example CPU/32 samples and
Gauss--Newton). It is therefore not an authoritative effective-parameter
snapshot. The runtime values above came from recorded `/parameter_events`, and
the manifest separately records the correct profile file and hash. This is a
recording-provenance defect to correct before relying on future manifests
without parameter-event replay.

## Observed target results

| Target | Duration | Action result | Marker-tip displacement during action | Device POS change (first 3) |
| --- | ---: | ---: | --- | --- |
| base Z `+5 mm` | 1.774 s | reached, 0.723 mm final error | `[+0.360,+0.552,+5.730] mm` | `[+2.737,0,-0.0007]` |
| base Z `-5 mm` absolute target | 1.433 s | reached, 0.785 mm final error | `[-0.333,-0.365,-7.587] mm` | `[-7.242,0,+2.331]` |

The second home-tip observation was about 2 mm above the first frozen home-tip
observation, so the second absolute target required about `-7.0 mm`, not just
`-5.0 mm`, from its run-start tip. This is consistent with the experiment's
history-preserving design.

The recorded planned controls had zero rotation for both actions. The positive
Z action was dominated by positive insertion. The negative Z action combined
negative insertion and positive tendon actuation. Each action contained one
late insertion sign reversal while entering the tolerance region.

## Model-response evidence

An offline v171/UKF replay with the recorded v174 Jacobian used 46 excited
200-ms windows. Across those windows:

- measured/predicted tip-response direction cosine: P50 `0.842`, P95 `0.915`;
- signed measured/predicted response gain: P50 `0.639`;
- endpoint prediction error: P50 `0.556 mm`, P95 `1.526 mm`;
- fitted versus v174 **insertion linear column** direction cosine: `0.9997`;
- fitted versus v174 insertion linear-column gain: `0.715`;
- fitted versus v174 insertion angular-column direction cosine: `0.976`;
- fitted versus v174 insertion angular-column gain: `0.787`.

The fit is diagnostic rather than independent ground truth because its
interface pose comes from the UKF-corrected model state. Rotation was disabled,
so the experiment has rank two and cannot evaluate the rotation column.

## Conclusion

**Observed:** both axial directions converged rapidly with sub-millimetre
action-result error while rotation remained disabled.

**Inferred with high confidence:** the deployed v174 insertion column has the
correct linear direction. Its local gain is too large for this trial: observed
interface translation was approximately 71% of the nominal v174 prediction.
That gain mismatch is real enough to affect feed-forward prediction, but it did
not prevent feedback convergence.

Therefore, the earlier no-rotation failure should not be attributed to an
incorrect sign or grossly incorrect direction in the v174 insertion column.
The earlier run loaded the causal-v2 shadow Jacobian and also involved coupled
insertion/tendon dynamics, history, and command timing. A strict artifact A/B
requires repeating this exact two-target file with only the Jacobian profile
changed back to causal-v2.
