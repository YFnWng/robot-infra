"""Offline Phase-5 qualification for the catheter control stack.

This module deliberately has no ROS, serial, or hardware output.  A passing
report means that the software preflight gates passed; it does not qualify
powered motion.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
from datetime import datetime
import json
from pathlib import Path
import sys
from time import monotonic
from typing import Any

import numpy as np
import torch

from ..transmission.backlash import BacklashSnapshot
from ..transmission.engaged_gain import EngagedGainConfig, EngagedGainEstimator
from .hardware_contract import load_hardware_contract
from ..orchestration.compute_device import resolve_compute_device
from ..planning.mppi import CatheterMppi, MppiConfig


DEFAULT_SESSION_ROOT = Path(
    "/media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions")


@dataclass(frozen=True)
class GateResult:
    """One auditable qualification gate."""

    name: str
    passed: bool
    metrics: dict[str, Any]
    detail: str = ""


def percentile_metrics(values) -> dict[str, float]:
    """Return stable latency summary values in milliseconds."""
    samples = np.asarray(values, dtype=np.float64)
    if (samples.ndim != 1 or samples.size == 0
            or not np.isfinite(samples).all()):
        raise ValueError("values must be a nonempty finite vector")
    return {
        "mean_ms": float(samples.mean()),
        "p50_ms": float(np.percentile(samples, 50)),
        "p95_ms": float(np.percentile(samples, 95)),
        "p99_ms": float(np.percentile(samples, 99)),
        "max_ms": float(samples.max()),
    }


def response_metrics(actual_delta, predicted_delta) -> dict[str, float]:
    """Metrics used later by powered small-step response gates."""
    actual = np.asarray(actual_delta, dtype=np.float64)
    predicted = np.asarray(predicted_delta, dtype=np.float64)
    if actual.shape != predicted.shape or actual.ndim != 2:
        raise ValueError("response arrays must have the same (N,D) shape")
    if (actual.shape[0] == 0 or not np.isfinite(actual).all()
            or not np.isfinite(predicted).all()):
        raise ValueError("response arrays must be nonempty and finite")
    error = actual-predicted
    actual_flat = actual.reshape(-1)
    predicted_flat = predicted.reshape(-1)
    denominator = float(np.dot(predicted_flat, predicted_flat))
    norm_product = float(
        np.linalg.norm(actual_flat)*np.linalg.norm(predicted_flat))
    return {
        "rmse": float(np.sqrt(np.mean(np.square(error)))),
        "max_error": float(np.max(np.abs(error))),
        "direction_cosine": (
            float(np.dot(actual_flat, predicted_flat)/norm_product)
            if norm_product > 0.0 else 0.0),
        "signed_gain": (
            float(np.dot(actual_flat, predicted_flat)/denominator)
            if denominator > 0.0 else 0.0),
    }


def _maximum_state_difference(first, second) -> float:
    differences: list[float] = []

    def visit(left, right):
        if torch.is_tensor(left):
            differences.append(float(torch.max(torch.abs(left-right))))
        elif isinstance(left, np.ndarray):
            differences.append(float(np.max(np.abs(left-right))))
        elif hasattr(left, "__dataclass_fields__"):
            for name in left.__dataclass_fields__:
                visit(getattr(left, name), getattr(right, name))
        elif hasattr(left, "state_dict") and hasattr(right, "state_dict"):
            for key, value in left.state_dict().items():
                if isinstance(value, np.ndarray):
                    visit(value, right.state_dict()[key])

    visit(first, second)
    return max(differences, default=0.0)


def _load_runtime(meta_root: Path, common_root: Path):
    for path in (meta_root.parent, common_root):
        if str(path) not in sys.path:
            sys.path.insert(0, str(path))
    from cr_meta_lnn.deployment import V171StreamingCatheterRuntime
    return V171StreamingCatheterRuntime


class _NonfiniteBackend:
    def predict_sequence(self, _state, motor_velocity_sequence, _dt):
        shape = np.asarray(motor_velocity_sequence).shape
        leading, horizon = shape[:-2], shape[-2]

        class Prediction:
            tip_base_m = torch.full((*leading, horizon, 3), float("nan"))
            markers_base_m = torch.full(
                (*leading, horizon, 4, 3), float("nan"))

        return Prediction()


class _AdvancingClock:
    def __init__(self, increment_s: float):
        self.value = -increment_s
        self.increment_s = increment_s

    def __call__(self):
        self.value += self.increment_s
        return self.value


def _projection_gate(contract, rng: np.random.Generator) -> GateResult:
    count = 256
    requested = rng.uniform(-2.0, 2.0, (count, 6))*contract.velocity_max
    positions = rng.uniform(
        contract.position_lower-1.0, contract.position_upper+1.0, (count, 6))
    batched = contract.project_velocity_batch(requested, positions)
    fields = (
        "logical_velocity", "requested_motor_axis_velocity", "motor_rpm",
        "motor_radians_per_second", "realized_motor_axis_velocity",
        "realized_logical_velocity")
    maximum = 0.0
    for index in range(count):
        scalar = contract.project_velocity(requested[index], positions[index])
        for field in fields:
            difference = np.max(np.abs(
                np.asarray(getattr(scalar, field))
                - np.asarray(getattr(batched, field))[index]))
            maximum = max(maximum, float(difference))
    saturated = np.any(
        np.abs(batched.logical_velocity-requested) > 1e-12, axis=1)
    return GateResult(
        "projection_scalar_batch_parity", maximum <= 1e-12,
        {"cases": count, "max_abs_difference": maximum,
         "saturated_fraction": float(saturated.mean())})


def _runtime_gate(runtime, initial_counts, rng) -> GateResult:
    state = runtime.initialize(1_000_000_000, initial_counts)
    before = state.clone()
    velocity = torch.as_tensor(
        rng.uniform(-0.15, 0.15, (4, 6, 3)), dtype=runtime.dtype,
        device=runtime.device)
    dt = torch.tensor(
        [0.017, 0.031, 0.040, 0.026, 0.039, 0.022],
        dtype=runtime.dtype, device=runtime.device)
    batched = runtime.predict_sequence(state, velocity, dt)
    scalar = torch.stack([
        runtime.predict_sequence(state, item, dt).markers_base_m
        for item in velocity])
    parity = float(torch.max(torch.abs(batched.markers_base_m-scalar)))
    clone_difference = max(
        _maximum_state_difference(state, before),
        _maximum_state_difference(runtime.clone_state(), before))

    long_velocity = velocity[0, :1]
    large = runtime.predict_sequence(state, long_velocity, [0.10])
    split = runtime.predict_sequence(
        state, long_velocity.expand(4, 3), [0.025]*4)
    subdivision = float(torch.max(torch.abs(
        large.markers_base_m[-1]-split.markers_base_m[-1])))
    reversal = runtime.predict_sequence(
        state, torch.tensor([[0.10, 0.05, -0.08],
                             [-0.10, -0.05, 0.08]],
                            dtype=runtime.dtype, device=runtime.device),
        [0.02, 0.03])
    finite = bool(torch.isfinite(reversal.markers_base_m).all())
    passed = (parity <= 3e-6 and clone_difference == 0.0
              and subdivision <= 3e-6 and finite)
    return GateResult(
        "streaming_rollout_equivalence", passed,
        {"batched_scalar_max_m": parity,
         "clone_mutation_max": clone_difference,
         "variable_dt_subdivision_max_m": subdivision,
         "reversal_finite": finite})


def _estimator_gate(runtime, initial_counts) -> GateResult:
    runtime.initialize(2_000_000_000, initial_counts)
    predicted = runtime.current_markers()
    observed = predicted+predicted.new_tensor([0.001, -0.0005, 0.0003])
    quality = {
        "confidence": [0.9]*4,
        "reprojection_error_px": [0.5]*4,
        "source_rig_count": [2.0]*4,
    }
    attempts = (runtime.estimator_initialization_observations
                + runtime.estimator_initialization_consecutive_inliers)
    results = [runtime.observe_markers(
        2_000_000_000+index, observed, quality)
        for index in range(attempts)]
    before = results[0].rms_before_mm
    after = results[-1].rms_after_mm
    accepted = sum(result.accepted for result in results)
    nis = [result.normalized_innovation for result in results
           if result.normalized_innovation is not None]
    passed = (accepted == attempts and before is not None and after is not None
              and after < before and results[-1].health == "TRACKING")
    return GateResult(
        "causal_estimator_synthetic_innovation", passed,
        {"accepted": accepted,
         "rejected_fraction": 1.0-accepted/float(attempts),
         "first_rms_before_mm": before, "final_rms_after_mm": after,
         "maximum_nis": max(nis, default=None),
         "health": results[-1].health},
        "Synthetic causal innovation only; recorded-camera replay remains "
        "open.")


def _failure_gate(runtime, state, position, target, contract) -> GateResult:
    bad = CatheterMppi(
        _NonfiniteBackend(), contract,
        MppiConfig(samples=8, horizon_steps=2, planning_deadline_s=0.06))
    invalid = bad.plan(state, position, target)
    deadline = CatheterMppi(
        runtime, contract,
        MppiConfig(samples=8, horizon_steps=2, planning_deadline_s=0.06),
        clock=_AdvancingClock(0.10)).plan(state, position, target)
    invalid_zero = bool(np.all(invalid.command_logical_velocity == 0.0))
    deadline_zero = bool(np.all(deadline.command_logical_velocity == 0.0))
    passed = (not invalid.valid and invalid.reason == "invalid_rollout"
              and invalid_zero and not deadline.valid
              and deadline.reason == "deadline_missed" and deadline_zero)
    return GateResult(
        "planner_failure_to_zero", passed,
        {"nonfinite_reason": invalid.reason,
         "nonfinite_zero": invalid_zero,
         "deadline_reason": deadline.reason,
         "deadline_zero": deadline_zero})


def _planner_gate(runtime, state, position, target, contract,
                  trials: int, deadline_s: float, *, samples: int = 32,
                  horizon_steps: int = 4,
                  rollout_step_s: float = 0.04,
                  engaged_gain_scenarios: bool = False) -> GateResult:
    planner = CatheterMppi(
        runtime, contract,
        MppiConfig(
            samples=samples, horizon_steps=horizon_steps,
            step_s=rollout_step_s,
            engaged_gain_scenarios=engaged_gain_scenarios,
            engaged_gain_maximum_first_step_shift=(
                8.0 if engaged_gain_scenarios else 0.0),
            planning_deadline_s=deadline_s, seed=17))
    transmission_state = None
    if engaged_gain_scenarios:
        gain = EngagedGainEstimator(EngagedGainConfig(
            enabled=True, prior_log_std=0.70)).snapshot()
        transmission_state = BacklashSnapshot(
            width_rad=np.zeros(3), width_positive_rad=np.zeros(3),
            width_negative_rad=np.zeros(3), remaining_rad=np.zeros(3),
            motion_direction=np.ones(3, dtype=np.int8),
            engaged_direction=np.ones(3, dtype=np.int8),
            confidence=np.ones(3),
            confirmation_count=np.ones(3, dtype=np.int32),
            phase=("ENGAGED", "ENGAGED", "ENGAGED"),
            engaged_gain=gain)
    # Untimed warm-up avoids charging lazy framework setup to the loop budget.
    planner.plan(
        state, position, target, transmission_state=transmission_state)
    elapsed_ms, effective_samples, reasons = [], [], []
    phase_ms = {
        "sample_projection": [], "rollout": [],
        "cost_weighting": [], "update_projection": []}
    for _ in range(trials):
        started = monotonic()
        plan = planner.plan(
            state, position, target,
            transmission_state=transmission_state)
        elapsed_ms.append((monotonic()-started)*1000.0)
        reasons.append(plan.reason)
        phase_ms["sample_projection"].append(plan.sample_projection_ms)
        phase_ms["rollout"].append(plan.rollout_ms)
        phase_ms["cost_weighting"].append(plan.cost_weighting_ms)
        phase_ms["update_projection"].append(plan.update_projection_ms)
        if plan.valid:
            effective_samples.append(plan.effective_samples)
    timing = percentile_metrics(elapsed_ms)
    valid = sum(reason == "ok" for reason in reasons)
    passed = valid == trials and timing["p99_ms"] <= deadline_s*1000.0
    return GateResult(
        ("real_model_mppi_gain_scenario_deadline"
         if engaged_gain_scenarios else "real_model_mppi_deadline"), passed,
        {**timing, "deadline_ms": deadline_s*1000.0,
         "samples": samples, "horizon_steps": horizon_steps,
         "rollout_step_s": rollout_step_s,
         "candidate_steps": samples*horizon_steps,
         "gain_scenario_count": (3 if engaged_gain_scenarios else 1),
         "expanded_rollout_steps": (samples*horizon_steps
                                    * (3 if engaged_gain_scenarios else 1)),
         "valid_plans": valid, "trials": trials,
         "minimum_effective_samples": min(effective_samples, default=0.0),
         "mean_effective_samples": (
             float(np.mean(effective_samples)) if effective_samples else 0.0),
         "reasons": reasons,
         "phase_timing_ms": {
             name: percentile_metrics(values)
             for name, values in phase_ms.items()}})


def run_preflight(args) -> dict[str, Any]:
    """Run all non-actuating Phase-5 gates and return a JSON-ready report."""
    meta_root = Path(args.cr_meta_lnn_root).expanduser().resolve()
    common_root = Path(args.cr_common_root).expanduser().resolve()
    compute_device = resolve_compute_device(args.device)
    runtime_type = _load_runtime(meta_root, common_root)
    runtime = runtime_type(
        Path(args.v171_distal_checkpoint).expanduser().resolve(),
        Path(args.jacobian_initialization_json).expanduser().resolve(),
        distal_tendon_allocation_checkpoint=str(
            args.distal_tendon_allocation_checkpoint),
        device=str(compute_device.device), adaptation_enabled=False)
    contract = load_hardware_contract(args.limits_file, args.catheter)
    counts = np.asarray(args.encoder_counts, dtype=np.float64)
    position = np.asarray(args.joint_position, dtype=np.float64)
    if counts.shape != (6,) or position.shape != (6,):
        raise ValueError(
            "encoder-counts and joint-position require six values")
    rng = np.random.default_rng(args.seed)

    gates = [_projection_gate(contract, rng),
             _runtime_gate(runtime, counts, rng),
             _estimator_gate(runtime, counts)]
    state = runtime.initialize(3_000_000_000, counts)
    target = runtime.current_markers()[-1].detach().cpu().numpy()
    gates.append(_planner_gate(
        runtime, state, position, target, contract,
        args.timing_trials, args.deadline_s,
        samples=args.samples, horizon_steps=args.horizon_steps,
        rollout_step_s=args.rollout_step_s,
        engaged_gain_scenarios=args.engaged_gain_scenarios))
    gates.append(_failure_gate(runtime, state, position, target, contract))
    passed = all(gate.passed for gate in gates)
    return {
        "schema_version": 2,
        "generated_at": datetime.now().astimezone().isoformat(),
        "scope": "OFFLINE_NON_ACTUATING_PREFLIGHT",
        "offline_preflight_passed": passed,
        "hardware_qualified": False,
        "encoder_zero_policy": "READ_ONLY_NEVER_SET_ZERO",
        "configuration": {
            "device_requested": args.device,
            "device": str(compute_device.device),
            "device_name": compute_device.name,
            "samples": args.samples,
            "horizon_steps": args.horizon_steps,
            "rollout_step_s": args.rollout_step_s,
            "engaged_gain_scenarios": args.engaged_gain_scenarios,
            "catheter": args.catheter,
            "encoder_counts": counts.tolist(),
            "joint_position": position.tolist(),
            "v171_distal_checkpoint": str(
                Path(args.v171_distal_checkpoint).resolve()),
            "jacobian_initialization_json": str(
                Path(args.jacobian_initialization_json).resolve()),
            "distal_tendon_allocation_checkpoint": (
                str(Path(args.distal_tendon_allocation_checkpoint).resolve())
                if args.distal_tendon_allocation_checkpoint else ""),
            "jacobian_adaptation": "SHADOW_WEIGHT_ZERO",
        },
        "gates": [asdict(gate) for gate in gates],
        "not_run": [
            "recorded_causal_replay",
            "ROS_power_off_process_kill_watchdog",
            "powered_static_hold",
            "powered_single_axis_small_steps_and_reversals",
            "Cartesian_point_regulation_and_trajectories",
            "real_response_rmse_max_direction_cosine_signed_gain",
        ],
    }


def _parser():
    root = Path("/home/chen-lab/Yifan")
    meta = root/"cr_meta_lnn"
    parser = argparse.ArgumentParser(
        description="Run non-actuating minimal Phase-5 preflight")
    parser.add_argument("--cr-meta-lnn-root", default=str(meta))
    parser.add_argument("--cr-common-root", default=str(root/"cr-common"))
    parser.add_argument("--v171-distal-checkpoint", default=str(
        meta/"artifacts/deployed/20260929_175554_grouped_no_rotation"/
        "real_distal_first_order_v171_multistep_map_em.pt"))
    parser.add_argument("--jacobian-initialization-json", default=str(
        meta/"artifacts/deployed/20260929_175554_grouped_no_rotation"/
        "real_joint_local_distal_v174.json"))
    parser.add_argument("--distal-tendon-allocation-checkpoint", default="")
    parser.add_argument("--limits-file", default=str(
        root/"robot-infra/src/automation/config/catheter_limits.yaml"))
    parser.add_argument("--catheter", default="imricor_test")
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--samples", type=int, default=32)
    parser.add_argument("--horizon-steps", type=int, default=4)
    parser.add_argument("--rollout-step-s", type=float, default=0.04)
    parser.add_argument(
        "--engaged-gain-scenarios", action="store_true",
        help="exercise mean/lower/upper engaged-gain rollouts and safety cap")
    parser.add_argument("--encoder-counts", nargs=6, type=float,
                        default=[4883.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    parser.add_argument("--joint-position", nargs=6, type=float,
                        default=[20.0, 0.0, 7.5, 40.0, 0.0, 0.0])
    parser.add_argument("--timing-trials", type=int, default=10)
    parser.add_argument("--deadline-s", type=float, default=0.06)
    parser.add_argument("--seed", type=int, default=5)
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--session-root", type=Path,
                        default=DEFAULT_SESSION_ROOT)
    return parser


def main(argv=None):
    args = _parser().parse_args(argv)
    if (args.timing_trials < 1 or args.deadline_s <= 0.0
            or args.samples < 8 or args.horizon_steps < 1
            or args.rollout_step_s <= 0.0):
        raise ValueError(
            "timing-trials, samples, horizon, step, and deadline must be "
            "positive; samples must be at least eight")
    report = run_preflight(args)
    output = args.output
    if output is None:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output = args.session_root/f"{stamp}_phase5_preflight.json"
    output = output.expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_suffix(output.suffix+".tmp")
    temporary.write_text(json.dumps(report, indent=2)+"\n", encoding="utf-8")
    temporary.replace(output)
    print(json.dumps({
        "offline_preflight_passed": report["offline_preflight_passed"],
        "hardware_qualified": False,
        "report": str(output),
    }))
    return 0 if report["offline_preflight_passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
