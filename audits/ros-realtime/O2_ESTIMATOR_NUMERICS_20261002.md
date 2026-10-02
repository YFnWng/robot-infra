# O2 follow-up: estimator numerical qualification

## Finding and decision

**Observed:** CPU/CUDA float32 UKF correction has backend-sensitive observable
rank and covariance. It is not a loss of state in the packed device-copy path.
Identical-prior correction probes reproduce rank differences; changing only the
runtime precision to float64 eliminates the replay differences within the
original `rtol=3e-5, atol=2e-6` state tolerances.

This completes the diagnostic slice, not O2 deployment qualification. No
estimator equations, safety thresholds, deployment defaults, artifact files or
installed model wheel were changed. No hardware operation was performed.
The ROS real-time audit skill required separating this intrusive numerical
comparison from latency measurements and preserving the manifest interlock.

## Implemented diagnostic

- `runtime_supervision.qualify_estimator_numerics` reads the recorded bag and
  reuses the O0 causal replay, thinning and continuous-history implementation.
- An optional replay observer records each propagation/correction, including
  the prefix. There is no extra work beyond the optional check when absent.
- `cr_meta_lnn.deployment.experimental.estimator_numerics` owns physical-state
  comparisons and identical-prior probes. Probes call the authoritative
  covariance reconstruction and `_correct_state`; no UKF equations are copied.
- Every tensor/metadata state field is checked using the original O2
  comparator. Physical errors separately report pose translation in mm,
  small-angle chordal rotation in degrees, marker distance in mm, strain,
  lambda, covariance, observability projection and J/RLS discrepancies.
- Probe fixtures use the retained at-or-before observation state, not the
  interpolated rewind state. They isolate one correction on identical inputs;
  they are not replacements for the full delayed replay. Live states are not
  mutated by probes.
- The selected manifest disables model adaptation. An initial attempt to
  request RLS was correctly rejected by the loader. The final CLI respects
  manifest selection rather than overriding qualification. RLS-enabled and
  ROS engagement/gain-belief qualification remain unperformed.

## Evidence

Same session as O2:
`attribution_twoaxis_points_grouped_n512_mount01_repeat01_20261002T211138237441Z`.
First 45 seconds of selected bag receipts, with no episode/history reset,
2,484 estimator operations including 442 marker corrections per device/precision.
Two intra-op and one inter-op Torch threads, RTX 4090, Torch 2.5.1+cu121.

Results live outside source repositories:
`/media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions/compute_optimization_o2_numerics_complete_20261002/`.
`report.json` summarizes the comparison and 12 common-prior probes; the two
JSONL files retain all event comparisons and source timestamps. The incomplete
first attempt under `compute_optimization_o2_numerics_20261002` is preserved,
not promoted as a complete result.

| Quantity | float32 | float64 |
|---|---:|---:|
| First strict state difference, receipt offset | 20.635 s | None |
| Accept/reject disagreements | 0 | 0 |
| Accepted vs accepted-noop reason disagreements | 2 | 0 |
| Retained-state rank disagreements across operations | 586 | 0 |
| Common-prior correction rank disagreements | 4 / 12 | 0 / 12 |
| Maximum reconstructed marker difference | 0.0894 mm | 3.30e-10 mm |
| Maximum interface translation difference | 0.0275 mm | 2.96e-11 mm |
| Maximum interface rotation discrepancy | 0.238 degrees | 4.08e-10 degrees |
| Maximum covariance element discrepancy | 1.7435 | 2.39e-8 |
| Maximum projection Frobenius discrepancy | 1.0000 | 3.80e-9 |
| Complete replay state conformance | Fail | Pass |

The 586 count includes propagation operations retaining a differing projection;
it is not 586 separate SVD calls or rejection events. All timestamps, health,
initialization flags and accepted/rejected counters agree. J and RLS covariance
are unchanged in both cases because adaptation is disabled.

At 27.661 s CPU returned `accepted`, CUDA `accepted_noop`; at 28.861 s the
roles reversed. Both observations were accepted. Do not describe these as
accept/reject disagreements. The strict state mismatch starts earlier than
these reason changes, at an ordinary marker update, not an encoder copy.

The first common-prior probe returns rank 9 on CPU and 10 on CUDA in float32,
with covariance difference 1.0313 despite marker difference only 0.0000552 mm.
Float64 probes have identical ranks and covariance discrepancy at most 4.34e-9.
This distinguishes single-correction arithmetic from accumulated-history drift.

## Mechanism: established boundary and unresolved details

The authoritative path in `deployment/v171_streaming_runtime.py` is:

```text
symmetrize/eigh/clamp/reconstruct prior covariance
 -> Cholesky sigma points -> nonlinear marker FK
 -> cross and innovation covariance -> solves
 -> statistical Jacobian = solve(prior, cross).T
 -> SVD rank cutoff at 1e-3 of maximum singular value
 -> observable projection -> gauge-fixed covariance
 -> following propagation and correction
```

Observed single-prior rank flips plus float64 agreement establish precision
sensitivity within this path. A rank change adds/removes an entire projected
direction, explaining projection discrepancies near one and much larger
covariance discrepancies than instantaneous Cartesian discrepancies.

**Inferred:** roundoff amplified by the small covariance eigenvalue floor and
the statistical-Jacobian solve can perturb weak singular directions across
the rank cutoff. We have not isolated which intermediate contributes most:
eigen reconstruction, FK sigma-point cancellation, solve or SVD. No physical
observability claim is justified solely by a float32 rank 10 result.

Float64 numerical agreement does not prove physical estimation accuracy or
better closed-loop control. These instrumented, sequential passes do not
measure callback timing or camera/recording scheduling.

## Next implementation slice

Update: the [precision-boundary implementation](O2_PRECISION_BOUNDARY_20261002.md)
now supports the full float64 CPU reference with float32 GPU planning. It avoids
a duplicate mixed-arithmetic UKF. The original implementation sequence below
records the alternatives considered; same-dtype-only snapshot support is no
longer a blocker. Full-stack qualification remains open.

1. Stabilize the UKF numerical path using the float64 replay as the reference.
   Test selective higher-precision covariance/statistical-Jacobian arithmetic
   and marker-observation calculations against the complete float64 path.
   Do not simply force rank 9, relax the rank cutoff, disable gates or enlarge
   the covariance floor to mask discrepancies.
2. Compare the stabilized CPU estimator with float32 GPU rollout. A full
   float64 CPU estimator is a useful fallback candidate for benchmarking, but
   using different precisions across the compute boundary requires explicit
   casting semantics and conformance tests; it is not yet supported by the
   current same-dtype runtime-pair contract.
3. Run paced evolving-snapshot replay with the actual three gain scenarios,
   then authorized full-stack non-actuating shadow qualification. Preserve
   deadline, freshness and heartbeat checks throughout.
4. Qualify adaptation separately using an explicitly reviewed artifact/feature
   selection. The current diagnostic does not certify J/RLS updates or online
   engagement/gain beliefs.

## Reproduction and verification

With the same ROS overlay/source import environment documented in the main
O2 report, use a new output directory:

```bash
/home/chen-lab/Yifan/cr-venv/bin/python -m runtime_supervision.qualify_estimator_numerics \
  /media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions/attribution_twoaxis_points_grouped_n512_mount01_repeat01_20261002T211138237441Z \
  --model-manifest /home/chen-lab/Yifan/cr_meta_lnn/artifacts/manifests/20260929_175554_grouped_no_rotation_v2.json \
  --duration-s 45 \
  --output /media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions/compute_optimization_o2_numerics_repeat
```

The CLI refuses detected local controllers, requires CUDA, creates no ROS node
or command publisher, and records only numerical—not timing—qualification.
Model tests verify metric units and same-prior probe ownership; replay tests
verify observer order and unchanged causal event handling.
Verification: 49 CPU/CUDA model tests and 35 runtime-supervision/bringup tests
pass. The numerical CLI help smoke test passes with canonical source imports.
The `runtime_supervision` ROS package rebuild passes.
