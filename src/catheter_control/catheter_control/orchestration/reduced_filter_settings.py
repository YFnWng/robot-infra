"""Settings shared by every reduced-filter controller (dry run and commanding backend).

One owner for the controller-only limits, the planner configuration, the switches the filter makes
illegal, the commit-triggered plan cadence and the stream/backend loader.
"""
from __future__ import annotations

from dataclasses import replace

import numpy as np

from ..planning.mppi import MppiConfig

PLAN_DUE_FRACTION = 0.9   # plans quantize to committed images; never faster than 1/(0.9*period) in image time
PLAN_OVERDUE_PERIODS = 2.0
# The filter alone owns play, take-up and engaged response; these October paths must stay off.
FILTER_OWNED_SWITCHES = ("backlash_compensation_enabled", "takeup_transaction_enabled", "engaged_gain_enabled",
                         "reversal_scheduler_enabled", "engagement_control_enabled", "adaptation_enabled")


def plan_due(last_plan_image_ns, image_ns, period_s):
    """Plan after this commit? First plan, or at least PLAN_DUE_FRACTION of a period of image time since the
    image of the last plan. Image stamps, not processing wall time, so commit-processing jitter cannot
    push a plan to a later commit."""
    return last_plan_image_ns is None or image_ns-last_plan_image_ns >= PLAN_DUE_FRACTION*period_s*1e9


def require_filter_owned_switches_off(value):
    """Refuse a configuration that turns on any October play/response path; value(name) reads a parameter."""
    for name in FILTER_OWNED_SWITCHES:
        if bool(value(name)):
            raise ValueError(f"{name} must be false: the filter alone owns play and response")
    if bool(value("planner_process_enabled")):
        raise ValueError("plan roots stay in this process; planner_process_enabled must be false")


def reduced_filter_contract(contract, controller_velocity_max, tendon_upper_mm):
    """Controller-only limits: configured velocity caps and the tendon upper limit."""
    velocity_max = np.where(np.asarray(controller_velocity_max) < 0.0, contract.velocity_max,
                            np.asarray(controller_velocity_max, dtype=np.float64))
    if np.any(velocity_max > contract.velocity_max):
        raise ValueError("controller_velocity_max cannot exceed hardware limits")
    if velocity_max[1] != 0.0:
        raise ValueError("the reduced filter models no rotation; rotation velocity must be 0")
    if not (contract.position_lower[2] < tendon_upper_mm <= contract.position_upper[2]):
        raise ValueError("reduced_filter_tendon_upper_mm must lie inside the hardware tendon range")
    upper = contract.position_upper.copy()
    upper[2] = float(tendon_upper_mm)
    return replace(contract, velocity_max=velocity_max, position_upper=upper,
                   velocity_min=np.minimum(contract.velocity_min, velocity_max))


def reduced_planner_config(node):
    def value(name):
        return node.get_parameter(name).value
    config = MppiConfig(
        horizon_steps=int(value("horizon_steps")), step_s=float(value("rollout_step_s")),
        point_rollout_step_s=float(value("mppi_point_rollout_step_s")),
        point_rollout_coarse_steps=bool(value("mppi_point_rollout_coarse_steps")),
        point_prediction_tail_steps=int(value("mppi_point_prediction_tail_steps")),
        samples=int(value("samples")), temperature=float(value("mppi_temperature")),
        noise_std=tuple(float(v) for v in value("mppi_noise_std")),
        noise_correlation=float(value("mppi_noise_correlation")),
        exploration_fraction=float(value("mppi_exploration_fraction")),
        engaged_gain_scenarios=bool(value("mppi_engaged_gain_scenarios")),
        engaged_gain_maximum_first_step_shift=float(value("mppi_engaged_gain_maximum_first_step_shift")),
        capture_radius_mm=float(value("mppi_capture_radius_mm")),
        capture_minimum_terminal_improvement_mm=float(value("mppi_capture_minimum_terminal_improvement_mm")),
        capture_hold_s=float(value("mppi_capture_hold_s")),
        grouped_mode_sampling=bool(value("mppi_grouped_mode_sampling")),
        best_candidate_guard=bool(value("mppi_best_candidate_guard")),
        seed=int(value("mppi_seed")), planning_deadline_s=float(value("planning_deadline_s")))
    if (config.horizon_steps != 4 or not config.point_rollout_coarse_steps or config.point_rollout_step_s <= 0
            or config.point_prediction_tail_steps != 0):
        raise ValueError("the reduced backend forecasts exactly four coarse steps with no tail")
    if config.engaged_gain_maximum_first_step_shift != 0.0:
        raise ValueError("Gate 3 lambda is not October lambda: the first-step lambda guard must be 0")
    return config


def load_reduced_filter(artifact_root, period_ms, startup, device):
    """(rollout backend, stream) on the real-time filter path; the research filter comes from the source tree."""
    import torch
    from cr_meta_lnn.training.hybrid.reduced_filter_stream import ReducedFilterRolloutBackend, load_stream
    from control_interface_py.encoder_coordinates import UNITS_PER_COUNT
    from ..safety.hardware_contract import ENCODER_RADIANS_PER_COUNT
    stream = load_stream(artifact_root, minimum_image_period_ns=int(period_ms*1e6), maximum_pending_images=3,
                         startup_pending_images=startup)
    backend = ReducedFilterRolloutBackend(
        stream.transmission, tendon_mm_per_shaft_rad=float(UNITS_PER_COUNT[2]/ENCODER_RADIANS_PER_COUNT),
        device=device, dtype=torch.float32, mode="cuda_graph" if device.startswith("cuda") else "eager")
    return backend, stream
