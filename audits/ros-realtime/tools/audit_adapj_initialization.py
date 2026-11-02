#!/usr/bin/env python3
"""Offline calibration audit using the existing AdapJ owner; never actuates.

Refitting writes only inside a temporary directory. Reported hold proposals
are diagnostic counterfactuals, not measured closed-loop outcomes or a claim
that zero error always requires zero actuation in a dynamical system.
"""
import argparse
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import tempfile

import numpy as np

from catheter_control.planning.adapj import TipAdapJ
from catheter_control.safety.hardware_contract import (
    HardwareContract, MOTOR_AXIS_UNITS_PER_ENCODER_COUNT, load_hardware_contract)


def audit(initialization, limits_file, step_s):
    path = Path(initialization).resolve()
    record = json.loads(path.read_text())
    stamps = np.asarray(record['timestamp_ns'], dtype=np.int64)
    tips = np.asarray(record['tip_m'], dtype=float)
    joints = np.asarray(record['achieved_logical_position'], dtype=float)
    contract = load_hardware_contract(limits_file, 'imricor_test')
    # Match the two-axis hardware profile's velocity envelope without changing it.
    maximum = np.minimum(contract.velocity_max, [10., 0., 4.5, 4., 25., 25.])
    contract = replace(contract, velocity_max=maximum,
                       velocity_min=np.minimum(contract.velocity_min, maximum))
    owner = TipAdapJ(contract, step_s)
    owner.begin()
    for stamp, tip, joint in zip(stamps, tips, joints):
        logical = np.zeros(6)
        logical[owner.axes] = joint
        counts = HardwareContract.logical_position_to_motor_axis_position(logical)/MOTOR_AXIS_UNITS_PER_ENCODER_COUNT
        owner.observe(int(stamp), tip, [(int(stamp), counts)], update_allowed=False)
    # Retain recorded admission facts; old artifacts have no such mask. Do not
    # invent engagement labels or re-admit takeup while refitting diagnostics.
    if 'initialization_fit_allowed' in record:
        allowed = np.asarray(record['initialization_fit_allowed'], dtype=bool)
        if allowed.shape != stamps.shape:
            raise ValueError('initialization admission mask must match samples')
        owner.initialization_fit_allowed.clear()
        owner.initialization_fit_allowed.extend(allowed.tolist())
        owner.exclude_takeup_from_initialization = bool(record['initialization_excludes_takeup'])
    with tempfile.TemporaryDirectory(prefix='adapj_calibration_audit_') as directory:
        fitted = json.loads(owner.finish(directory).read_text())
    starts = np.asarray(fitted['fit_interval_start_ns'], dtype=np.int64)
    ends = np.asarray(fitted['fit_interval_end_ns'], dtype=np.int64)
    index = np.searchsorted(stamps, starts)
    right = np.searchsorted(stamps, ends)
    fraction = ((ends-stamps[right-1])/(stamps[right]-stamps[right-1]))[:, None]
    states, actions = owner.coordinates(tips, joints)
    states_next = states[right-1]*(1-fraction)+states[right]*fraction
    actions_next = actions[right-1]*(1-fraction)+actions[right]*fraction
    phi = np.concatenate([states_next, states[index], actions[index]], axis=1)
    weights = np.concatenate([owner.controller.matrices[key] for key in ('A0','A1','B0')], axis=1)
    prediction_error = actions_next-phi@weights.T
    hold_error = actions_next-actions[index]
    singular = np.linalg.svd(phi, compute_uv=False)
    hold_delta = []
    projected_move = []
    for i in index:
        owner.controller.seed_history(states[i], actions[i])
        proposed = owner.q_origin+owner.q_scale*owner.controller.compute(states[i], states[i], stamps[i]*1e-9)
        delta = proposed-joints[i]
        hold_delta.append(delta)
        position = np.zeros(6)
        position[owner.axes] = joints[i]
        velocity = np.zeros(6)
        velocity[owner.axes] = delta/step_s
        velocity = np.clip(velocity, -contract.velocity_max, contract.velocity_max)
        command = contract.project_publishable_velocity(velocity, position)
        projected_move.append(command[owner.axes]*step_s)
    hold_delta = np.abs(hold_delta)
    projected_move = np.abs(projected_move)
    return dict(
        initialization=str(path), initialization_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        samples=len(stamps), refit_intervals=len(index), step_s=step_s,
        original_initialization_method=record.get('initialization_method', 'full_inverse_regression'),
        fitted_initialization_method=fitted.get('initialization_method', 'full_inverse_regression'),
        initialization_excludes_takeup=fitted['initialization_excludes_takeup'],
        eligible_initialization_samples=int(np.count_nonzero(fitted['initialization_fit_allowed'])),
        inverse_regressor_rank=int(np.linalg.matrix_rank(phi)),
        inverse_regressor_columns=phi.shape[1], inverse_regressor_singular_values=singular.tolist(),
        inverse_regressor_condition=float(singular[0]/singular[-1]),
        action_singular_values=fitted['action_singular_values'],
        normalized_fit_rmse=np.sqrt(np.mean(prediction_error**2, axis=0)).tolist(),
        normalized_hold_rmse=np.sqrt(np.mean(hold_error**2, axis=0)).tolist(),
        zero_error_proposal_abs_median_logical_mm=np.median(hold_delta, axis=0).tolist(),
        zero_error_proposal_abs_max_logical_mm=np.max(hold_delta, axis=0).tolist(),
        zero_error_projected_travel_abs_median_logical_mm=np.median(projected_move, axis=0).tolist(),
        zero_error_projected_travel_abs_max_logical_mm=np.max(projected_move, axis=0).tolist(),
        nonzero_projected_hold_fraction=float(np.mean(np.any(projected_move>0, axis=1))),
        fitted_algorithm=fitted['implementation'],
        qualification='diagnostic only; not a calibration acceptance rule or hardware result')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--initialization', type=Path, action='append', required=True)
    parser.add_argument('--limits-file', type=Path, required=True)
    parser.add_argument('--step-s', type=float, default=1/15.)
    args = parser.parse_args()
    for path in args.initialization:
        print(json.dumps(audit(path, args.limits_file, args.step_s), allow_nan=False))


if __name__ == '__main__':
    main()
