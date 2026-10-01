# Reproducible audit tooling

The scripts below reproduce maintained model-validation and ROS timing
evidence. They are development tools, not installed ROS nodes or production
runtime dependencies. Generated JSON, NPZ, and PNG output belongs under the
external catheter-session root and is ignored by Git.

## Safety boundary

All tools except `passive_monitor.py` operate only on recorded bags, arrays,
checkpoints, and configuration files. `passive_monitor.py` creates one ROS 2
node with subscriptions only; it creates no publisher or client and does not
claim a control mode. None of these tools authorizes motor power, controller
arming, firmware operations, or encoder-zero changes.

## Model-validation tools

| Tool | Maintained purpose |
| --- | --- |
| `evaluate_causal_proximal_experiment.py` | Summarize recorded causal proximal experiments and provide shared bag-analysis helpers. |
| `identify_phase2_transmission.py` | Fit and export the Phase-2 proximal transmission comparison. |
| `audit_phase2e_insertion_curvature.py` | Audit Phase-2e reconstruction integrity and insertion-conditioned response. |
| `plot_phase2e_static_spatial_curves.py` | Plot static distal curves across insertion and tendon levels. |
| `evaluate_mppi_model_response.py` | Replay a recorded MPPI session against the deployed distal model and local Jacobian. |
| `replay_causal_shadow_comparison.py` | Compare fixed and offline-adaptive Jacobian variants without opening ROS or hardware paths. |
| `test_phase2_transmission_identification.py` | Regression tests for transmission play and fitting helpers. |
| `test_phase3_timing_analysis.py` | Regression tests for Phase-3 timing segmentation. |

The model-response tools derive the sibling `cr_meta_lnn` and `cr-common`
locations from the repository workspace. Their checkpoint and Jacobian paths
remain explicit CLI overrides so a different artifact bundle can be selected.

## ROS timing and plotting tools

| Tool | Maintained purpose |
| --- | --- |
| `tools/passive_monitor.py` | Bounded subscription-only ROS rate, age, diagnostic, and process monitor. |
| `tools/qualify_control_cycle_timing.py` | Build a causal full-stack timing report from a recorded session. |
| `tools/plot_circle_tracking_comparison.py` | Compare recorded tip paths against a circle reference. |
| `tools/plot_sparse_point_final_shapes.py` | Plot target and reconstructed final shapes from a sparse-point bag. |

## Environments and verification

ROS-bag tools require ROS 2 Humble to be sourced. Model replay and plotting
also require the supported `cr-venv`; ZED or camera processes are not needed
for offline analysis. From the repository root, the source-only verification
is:

```bash
python3 -m compileall -q audits/model-validation audits/ros-realtime/tools
python3 -m pytest -q \
  audits/model-validation/test_phase2_transmission_identification.py \
  audits/model-validation/test_phase3_timing_analysis.py
```

For ROS-dependent `--help` and recorded-data execution, use the supported
environment ordering:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
source /home/chen-lab/Yifan/cr-venv/bin/activate
```

Do not treat a successful offline audit as hardware timing qualification.
Production timing claims require representative full-stack load and retained
session evidence.
