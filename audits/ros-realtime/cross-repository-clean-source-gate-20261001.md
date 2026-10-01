# Cross-repository clean-source gate — 2026-10-01

## Outcome

The pre-cleanup production closure passes the clean-source build, declared-artifact, deterministic-replay, seeded-controller, safety-default, and full-simulation gates. The shared baseline identifier is `catheter-stack-pre-cleanup-20261001`.

This is a non-actuating qualification. It does not qualify live hardware timing, enable command output, alter firmware, or change encoder zero.

## Source closure

| Repository | Qualified source commit |
| --- | --- |
| `robot-infra` | `494a5f2db49a57475fbd3069e3f161ded0bd8c46` |
| `cr_meta_lnn` | `ae8752b7e140ad913ca2e742b99f5c4da348e01b` |
| `catheter-shape-tracking` | `56980d0d6a382a29d97218dd3a5de6f0793e519f` |
| `control` | `af8b10d2b5bf885e4daf5365a0013e3ca30b51f1` |
| `cr-common` | `db5cdd464c850b07f0a159bc3e984212104ab435` |

The gate exported each commit with `git archive` into an isolated tree. Only the three artifacts declared by `cr_meta_lnn/artifacts/manifests/20260929_175554_grouped_no_rotation.json` were copied into it. Generated builds, logs, caches, sessions, and unrelated working-tree files were excluded.

## Evidence

- Clean ROS build: `control_interface`, `catheter_control`, and `automation`.
- `robot-infra`: 492 tests passed after excluding only the unavailable optional `ros2_igtl_bridge` integration test.
- Production `cr_meta_lnn` deployment/artifact tests: 50 passed.
- `control` adaptive-Jacobian tests: 13 passed.
- `catheter-shape-tracking`: 223 tests passed.
- Focused output-interlock and encoder-zero gate: 20 tests passed.
- All three declared artifact sizes and SHA-256 hashes matched.

The seeded ROS suite includes MPPI, backlash-belief, engaged-gain-belief, and reversal-scheduler tests. The safety gate confirms hardware output defaults disabled, profiles cannot override the launch interlock, and manager and serial bridge both reject `SET_ZERO`.

### Recorded non-actuating replay

The timing audit regenerated from `20260929_175554_mppi_demo` is byte-for-byte identical to the maintained report: 181 control-cycle rows and five recorded deadline-miss rows. Key distributions were:

- planner elapsed: median 48.774 ms, p95 58.070 ms, max 76.840 ms;
- estimator callback: median 81.856 ms, p95 110.799 ms, max 157.697 ms;
- marker rewind/correct/replay: median 51.026 ms, p95 71.188 ms;
- marker source age: median 176.891 ms, p95 227.576 ms;
- manager-to-serial forwarding: median 0.500 ms, p95 0.849 ms;
- serial transmit to next feedback: median 5.452 ms, p95 10.272 ms.

These reproduce the recorded scheduling limitation; they do not claim hard real-time performance.

### Full simulation smoke

The clean snapshot ran the grouped v175 interface-transmission stack on CPU with 32 samples, four steps, rotation disabled, no recording, and no serial process. `two_axis_points_v175_sim_no_rotation.yaml` reached all four targets with final errors 0.907, 0.372, 0.289, and 0.268 mm.

A first manual invocation named the v174 JSON incorrectly and failed closed before activation. Using the manifest-declared `real_joint_local_distal_v174.json` completed successfully.

## Limitation

`src/automation/test/test_estimation_node_runtime.py` could not be collected because optional package `ros2_igtl_bridge` is not installed. The active four-ring path, all other automation tests, and full simulation passed. Installations claiming the optional OpenIGTLink path must run this test; it is not silently treated as passing here.

## Baseline meaning

The shared identifier freezes source compatibility and non-actuating reproducibility across five repositories. It does not make research history production-supported, bundle learned binaries into Git, or authorize hardware actuation. Restructuring must preserve the declared artifacts, safety interlocks, replay equivalence, and simulation gate or explicitly document a behavior change.
