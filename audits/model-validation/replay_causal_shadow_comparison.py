#!/usr/bin/env python3
"""Compare fixed Jacobians and offline adaptive commits on a causal bag.

The first two runtimes keep adaptation disabled: ``rls_shadow_ready`` records
a would-be update while the Jacobian remains bitwise fixed. A third runtime
may commit those updates in memory to score the strictly column-selective
updater. No ROS publisher, service, or hardware path is opened by this tool.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import importlib.util
import json
from pathlib import Path
import sys


HERE = Path(__file__).resolve().parent
REPOSITORY_ROOT = HERE.parents[1]
WORKSPACE_ROOT = REPOSITORY_ROOT.parent
DEFAULT_META = WORKSPACE_ROOT / "cr_meta_lnn"


def _load_evaluator():
    path = HERE / "evaluate_mppi_model_response.py"
    spec = importlib.util.spec_from_file_location("model_response", path)
    module = importlib.util.module_from_spec(spec)
    if spec.loader is None:
        raise RuntimeError(f"cannot load {path}")
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _bag_path(session: Path) -> Path:
    if (session / "metadata.yaml").is_file():
        return session
    if (session / "robot_bag" / "metadata.yaml").is_file():
        return session / "robot_bag"
    raise FileNotFoundError(f"no rosbag metadata below {session}")


def _metric_delta(prior, candidate):
    old = float(prior)
    new = float(candidate)
    return {
        "prior": old,
        "candidate": new,
        "change": new-old,
        "relative_change": None if old == 0.0 else (new-old)/old,
    }


def _comparison(prior, candidate):
    result = {
        "full_tip_error_norm_mm": {},
        "by_dominant_motor_axis": {},
        "accepted_tracking_observations": _metric_delta(
            prior["replay"]["accepted_tracking_observations"],
            candidate["replay"]["accepted_tracking_observations"]),
    }
    for statistic in ("mean", "p50", "p95", "maximum"):
        result["full_tip_error_norm_mm"][statistic] = _metric_delta(
            prior["full_tip_response_mm"]["error_norm"][statistic],
            candidate["full_tip_response_mm"]["error_norm"][statistic])
    axes = prior["full_tip_response_by_dominant_motor_axis_mm"]
    for axis in axes:
        result["by_dominant_motor_axis"][axis] = {}
        for statistic in ("mean", "p50", "p95"):
            result["by_dominant_motor_axis"][axis][statistic] = (
                _metric_delta(
                    axes[axis]["error_norm"][statistic],
                    candidate[
                        "full_tip_response_by_dominant_motor_axis_mm"
                    ][axis]["error_norm"][statistic]))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--prior-jacobian", type=Path, default=(
        DEFAULT_META / "evaluation" / "real_joint_local_distal_v174.json"))
    parser.add_argument("--candidate-jacobian", type=Path, default=(
        DEFAULT_META / "evaluation"
        / "real_joint_local_distal_causal_v2.json"))
    parser.add_argument("--distal-checkpoint", type=Path, default=(
        DEFAULT_META / "checkpoints"
        / "real_distal_first_order_v171_multistep_map_em.pt"))
    parser.add_argument("--window-s", type=float, default=.25)
    parser.add_argument("--stride-s", type=float, default=.25)
    parser.add_argument("--marker-rate-hz", type=float, default=20.)
    parser.add_argument("--torch-threads", type=int, default=2)
    args = parser.parse_args()

    evaluator = _load_evaluator()
    bag = _bag_path(args.session.expanduser().resolve())

    def configuration(jacobian, *, adaptation_enabled=False):
        # This namespace intentionally mirrors evaluator CLI fields. All gate
        # values come from causal_v2_shadow.yaml.
        class Values:
            pass
        value = Values()
        value.bag = bag
        value.cr_meta_lnn_root = DEFAULT_META
        value.cr_common_root = WORKSPACE_ROOT / "cr-common"
        value.distal_checkpoint = args.distal_checkpoint.expanduser().resolve()
        value.jacobian_initialization_json = jacobian.expanduser().resolve()
        value.marker_estimator = "ukf"
        value.window_s = args.window_s
        value.stride_s = args.stride_s
        value.window_tolerance_s = .04
        value.minimum_motor_delta_rad = .004
        value.marker_rate_hz = args.marker_rate_hz
        value.torch_threads = args.torch_threads
        value.estimator_filter_initial_covariance = .25
        value.estimator_filter_process_std_sqrt_s = 1.
        value.adaptation_minimum_observations = 8
        value.adaptation_minimum_normalized_action = .10
        value.adaptation_minimum_rotation_deg = .30
        value.adaptation_minimum_translation_mm = .30
        value.adaptation_minimum_response_snr = 1.
        value.adaptation_maximum_window_s = 1.
        value.adaptation_directional_purity = .90
        value.adaptation_reversal_holdoff_by_axis = (4.5, 6.5, 4.0)
        value.adaptation_confirmation_windows = 2
        value.adaptation_consistency_cosine = .80
        value.adaptation_minimum_column_gain = .50
        value.adaptation_maximum_column_gain = 1.50
        value.adaptation_maximum_direction_deviation_deg = 30.
        value.adaptation_enabled = adaptation_enabled
        return value

    prior = evaluator.evaluate(configuration(args.prior_jacobian))
    candidate = evaluator.evaluate(configuration(args.candidate_jacobian))
    adapted_candidate = evaluator.evaluate(configuration(
        args.candidate_jacobian, adaptation_enabled=True))
    report = {
        "schema": "catheter-causal-shadow-comparison-v2",
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "session": str(args.session.expanduser().resolve()),
        "safety": {
            "hardware_output": False,
            "adaptation_commits": adapted_candidate[
                "shadow_adaptation"]["committed_updates"],
            "recorded_messages_only": True,
        },
        "prior": prior,
        "candidate": candidate,
        "adapted_candidate": adapted_candidate,
        "comparison": _comparison(prior, candidate),
        "adaptive_comparison": _comparison(candidate, adapted_candidate),
        "gates": {
            "candidate_preserves_acceptance": (
                candidate["replay"]["accepted_tracking_observations"]
                >= .99*prior["replay"]["accepted_tracking_observations"]),
            "candidate_reduces_full_tip_p50_error": (
                candidate["full_tip_response_mm"]["error_norm"]["p50"]
                < prior["full_tip_response_mm"]["error_norm"]["p50"]),
            "candidate_reduces_full_tip_p95_error": (
                candidate["full_tip_response_mm"]["error_norm"]["p95"]
                < prior["full_tip_response_mm"]["error_norm"]["p95"]),
            "no_adaptation_commits": (
                prior["shadow_adaptation"]["committed_updates"] == 0
                and candidate["shadow_adaptation"]["committed_updates"] == 0),
            "offline_adaptation_committed": (
                adapted_candidate["shadow_adaptation"][
                    "committed_updates"] > 0),
            "adaptation_preserves_acceptance": (
                adapted_candidate["replay"][
                    "accepted_tracking_observations"]
                >= .99*candidate["replay"][
                    "accepted_tracking_observations"]),
            "adaptation_preserves_insertion_p95_within_2pct": (
                adapted_candidate[
                    "full_tip_response_by_dominant_motor_axis_mm"
                ]["insertion"]["error_norm"]["p95"]
                <= 1.02*candidate[
                    "full_tip_response_by_dominant_motor_axis_mm"
                ]["insertion"]["error_norm"]["p95"]),
            "adaptation_preserves_rotation_p95_within_2pct": (
                adapted_candidate[
                    "full_tip_response_by_dominant_motor_axis_mm"
                ]["rotation"]["error_norm"]["p95"]
                <= 1.02*candidate[
                    "full_tip_response_by_dominant_motor_axis_mm"
                ]["rotation"]["error_norm"]["p95"]),
        },
        "interpretation": [
            "Predictions start from each model's causal UKF posterior and use "
            "only the recorded future encoder increment for the scored window.",
            "The raw marker tip is the endpoint reference; UKF interface pose "
            "is not treated as independent ground truth.",
            "The prior and fixed-candidate branches count would-be decisions "
            "without committing them.",
            "The adapted-candidate branch commits updates only in process, "
            "using messages already recorded in the bag; it cannot command "
            "the robot.",
        ],
    }
    report["result"] = "PASS" if all(report["gates"].values()) else "REVIEW"
    output = args.output or (
        args.session.expanduser().resolve()
        / "causal_shadow_comparison.json")
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2)+"\n", encoding="utf-8")
    print(json.dumps({
        "result": report["result"],
        "output": str(output),
        "gates": report["gates"],
        "comparison": report["comparison"],
        "prior_shadow": prior["shadow_adaptation"],
        "candidate_shadow": candidate["shadow_adaptation"],
        "adapted_candidate": adapted_candidate["shadow_adaptation"],
        "adaptive_comparison": report["adaptive_comparison"],
    }, indent=2))


if __name__ == "__main__":
    main()
