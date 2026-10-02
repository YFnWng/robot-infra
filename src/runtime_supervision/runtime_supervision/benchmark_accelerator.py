"""O3 warmed four-step tensor execution comparison; no ROS or actuators.

Synthetic deterministic fixtures are NOT representative closed-loop timing.
Cold preparation is separate. Failed compilation/capture is recorded explicitly,
never replaced by eager execution labelled as acceleration.
"""
from __future__ import annotations

import argparse
from dataclasses import replace
from functools import partial
import hashlib
import json
from pathlib import Path
import time

import torch
import numpy as np

from cr_meta_lnn.deployment import load_runtime_bundle
from cr_meta_lnn.deployment.control_accelerator import (
    FourStepRollout, PreparedTensorKernel, rollout_inputs)
from catheter_control.planning.tensor_selection import weighted_selection
from catheter_control.planning.mppi import CatheterMppi, MppiConfig
from catheter_control.safety.hardware_contract import load_hardware_contract
from catheter_control.transmission.backlash import BacklashSnapshot
from catheter_control.transmission.engaged_gain import EngagedGainConfig, EngagedGainEstimator
from catheter_control.safety.validation import percentile_metrics
from .compute_profile import require_controller_idle


def synchronize(device):
    if device.type == "cuda":
        torch.cuda.synchronize(device)


def measure(function, repeats, device):
    samples = []
    for _ in range(repeats):
        synchronize(device)
        start = time.perf_counter()
        function()
        synchronize(device)
        samples.append((time.perf_counter()-start)*1000)
    return percentile_metrics(samples)


def compare_outputs(actual, expected):
    actual_leaves, actual_tree = torch.utils._pytree.tree_flatten(actual)
    expected_leaves, expected_tree = torch.utils._pytree.tree_flatten(expected)
    if actual_tree != expected_tree:
        raise AssertionError("accelerator output structure changed")
    for a, b in zip(actual_leaves, expected_leaves):
        torch.testing.assert_close(a, b, rtol=3e-5, atol=2e-6)


def planner_fixture(runtime, args, state):
    """Frozen proposals across all three variants; no take-up arbiter or commands."""
    contract = load_hardware_contract(args.limits_file, "imricor_test")
    upper = np.minimum(contract.velocity_max, [10., 0., 4.5, 4., 25., 25.])
    contract = replace(contract, velocity_max=upper,
                       velocity_min=np.minimum(contract.velocity_min, upper))
    gain = EngagedGainEstimator(EngagedGainConfig(enabled=True)).snapshot()
    direction = np.array([1, 0, 1], dtype=np.int8)
    belief = BacklashSnapshot(
        width_rad=np.zeros(3), width_positive_rad=np.zeros(3), width_negative_rad=np.zeros(3),
        remaining_rad=np.zeros(3), motion_direction=direction.copy(), engaged_direction=direction,
        confidence=np.ones(3), confirmation_count=np.ones(3, dtype=np.int32),
        phase=("ENGAGED", "UNKNOWN", "ENGAGED"), engaged_gain=gain)
    target = runtime.markers_for_state(state)[-1].cpu().numpy()+[0., 0., .005]
    config = MppiConfig(samples=args.samples, horizon_steps=4,
                        point_rollout_step_s=.2, point_rollout_coarse_steps=True,
                        engaged_gain_scenarios=args.scenarios == 3,
                        seed=17, planning_deadline_s=.06)
    def solve(variant):
        cfg = replace(config, grouped_mode_sampling=variant == "grouped",
                      takeup_risk_weight=4. if variant != "plain" else 0.,
                      best_candidate_guard=variant == "grouped")
        planner = CatheterMppi(runtime, contract, cfg)
        return planner.plan(state, [20., 0., 0., 0., 0., 0.], target,
                            transmission_state=belief,
                            direction_lease=direction if variant == "grouped" else np.zeros(3))
    solve.candidate_counts = {
        name: len(CatheterMppi(runtime, contract, replace(
            config, grouped_mode_sampling=name == "grouped"))._grouped_candidates(
                direction if name == "grouped" else np.zeros(3))[0])
        for name in ("grouped", "plain_compensation", "plain")}
    return solve


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model-manifest", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--limits-file", type=Path,
                        help="also compare frozen full planner decisions for all variants")
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--samples", type=int, default=512)
    parser.add_argument("--scenarios", type=int, choices=(1, 3), default=3)
    parser.add_argument("--repeats", type=int, default=100)
    parser.add_argument("--warmup", type=int, default=5)
    parser.add_argument("--modes", nargs="+", choices=("eager", "compiled", "cuda_graph"),
                        default=["eager", "compiled", "cuda_graph"])
    args = parser.parse_args(argv)
    if not 8 <= args.samples <= 1024 or not 10 <= args.repeats <= 500 or args.warmup < 1:
        parser.error("require samples 8..1024, repeats 10..500, warmup >=1")
    if args.output.exists():
        parser.error("output already exists")
    require_controller_idle()
    torch.set_num_threads(2)
    torch.set_num_interop_threads(1)
    runtime = load_runtime_bundle(args.model_manifest, device=args.device,
                                  options={"adaptation_enabled": False}).runtime
    runtime.initialize(1_000_000_000, [0, 0, 0])
    runtime.advance_encoder(1_200_000_000, [1000, 500, 6000])
    state = runtime.clone_state()
    n = args.samples*args.scenarios
    velocity = (torch.randn(n, 4, 3, generator=torch.Generator().manual_seed(17))*.2).to(runtime.device)
    velocity[0] = 0
    dt = velocity.new_full((4,), .2)
    gain = velocity.new_tensor([1.] if args.scenarios == 1 else [1., .6, 1.4]).repeat(args.samples)
    example = rollout_inputs(runtime, state, velocity, dt, gain=gain)
    changed = rollout_inputs(runtime, state, -velocity, dt, gain=gain*.9)
    kernel = FourStepRollout(runtime)
    baseline = runtime.predict_control_sequence_coarse(state, velocity, dt, distal_gain=gain)
    tensor_reference = kernel(*example)
    compare_outputs(tensor_reference[:3], (baseline.tip_base_m, baseline.motor_angles_rad, baseline.distal_lambda))
    tips = tensor_reference[0][::args.scenarios]
    target = tips[0]+velocity.new_tensor([0., 0., .005])
    costs = ((tips-target)*1000).square().sum((1, 2))
    eligible = torch.arange(args.samples, device=runtime.device) % 3 != 1
    logical = velocity[::args.scenarios]
    select_inputs = costs, eligible, tips, logical
    selection = partial(weighted_selection, temperature=1., fixed_shape=True)
    solve = None if args.limits_file is None else planner_fixture(runtime, args, state)
    variants = ("grouped", "plain_compensation", "plain")
    planner_reference = {} if solve is None else {name: solve(name) for name in variants}
    if any(not plan.valid for plan in planner_reference.values()):
        raise RuntimeError("reference fixture is not valid; no timing equivalence claim")
    report = {"scope": __doc__, "torch": torch.__version__,
              "device": str(runtime.device), "samples": args.samples,
              "scenarios": args.scenarios, "steps": 4, "step_s": .2,
              "repeats": args.repeats, "warmup": args.warmup,
              "manifest_sha256": hashlib.sha256(args.model_manifest.read_bytes()).hexdigest(),
              "rows": []}
    with torch.inference_mode():
        for _ in range(args.warmup):
            runtime.predict_control_sequence_coarse(state, velocity, dt, distal_gain=gain)
        report["reference_rollout_ms"] = measure(
            lambda: runtime.predict_control_sequence_coarse(state, velocity, dt, distal_gain=gain),
            args.repeats, runtime.device)
        if solve is not None:
            report["reference_planner"] = {}
            for name in variants:
                for _ in range(args.warmup):
                    solve(name)
                report["reference_planner"][name] = measure(
                    lambda: solve(name), args.repeats, runtime.device)
        for mode in args.modes:
            row = {"mode": mode}
            started = time.perf_counter()
            try:
                rollout = PreparedTensorKernel(kernel, example, mode=mode, warmup=args.warmup)
                select = PreparedTensorKernel(selection, select_inputs, mode=mode, warmup=args.warmup)
                row["preparation_ms"] = (time.perf_counter()-started)*1000
                compare_outputs(rollout(*example), tensor_reference)
                compare_outputs(rollout(*changed), kernel(*changed))
                compare_outputs(select(*select_inputs), selection(*select_inputs))
                changed_selection = costs+torch.arange(args.samples, device=runtime.device), ~eligible, tips, logical
                compare_outputs(select(*changed_selection), selection(*changed_selection))
                row["rollout_ms"] = measure(lambda: rollout(*example), args.repeats, runtime.device)
                row["selection_ms"] = measure(lambda: select(*select_inputs), args.repeats, runtime.device)
                # Include host validation, final-state reconstruction and owned outputs.
                runtime._control_accelerator = rollout
                compare_outputs((runtime.predict_control_sequence_coarse(state, velocity, dt, distal_gain=gain).tip_base_m,),
                                (baseline.tip_base_m,))
                row["public_rollout_ms"] = measure(
                    lambda: runtime.predict_control_sequence_coarse(state, velocity, dt, distal_gain=gain),
                    args.repeats, runtime.device)
                if solve is not None:
                    row["planner"] = {}
                    for name in variants:
                        # Grouped sampling APPENDS deterministic probes; its
                        # fixed tensor batch is not the stochastic sample count.
                        count = solve.candidate_counts[name]
                        rates = velocity.new_zeros(count*args.scenarios, 4, 3)
                        planner_inputs = rollout_inputs(runtime, state, rates, dt)
                        preparation_started = time.perf_counter()
                        runtime._control_accelerator = PreparedTensorKernel(
                            kernel, planner_inputs, mode=mode, warmup=args.warmup)
                        preparation_ms = (time.perf_counter()-preparation_started)*1000
                        actual, expected = solve(name), planner_reference[name]
                        if (actual.valid, actual.reason, actual.selected_candidate_index) != (
                                expected.valid, expected.reason, expected.selected_candidate_index):
                            raise AssertionError(
                                f"{name}: discrete planner decision changed; "
                                f"actual={(actual.valid, actual.reason, actual.selected_candidate_index)}, "
                                f"expected={(expected.valid, expected.reason, expected.selected_candidate_index)}")
                        np.testing.assert_allclose(actual.command_logical_velocity,
                                                   expected.command_logical_velocity,
                                                   rtol=3e-5, atol=2e-6)
                        np.testing.assert_allclose(actual.best_cost, expected.best_cost, rtol=3e-5, atol=2e-6)
                        plans = []
                        def measured_solve():
                            plans.append(solve(name))
                        timing = measure(measured_solve, args.repeats, runtime.device)
                        row["planner"][name] = {
                            "timing_ms": timing,
                            "deadline_misses": sum(plan.reason == "deadline_missed" for plan in plans),
                            "valid_count": sum(plan.valid for plan in plans),
                            "candidate_count": count,
                            "preparation_ms": preparation_ms,
                            "selected_candidate_index": actual.selected_candidate_index}
                row["conformance_passed"] = True
            except Exception as error:
                row.update(conformance_passed=False, error=f"{type(error).__name__}: {error}")
            finally:
                runtime._control_accelerator = None
            report["rows"].append(row)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x") as stream:
        json.dump(report, stream, indent=2)
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
