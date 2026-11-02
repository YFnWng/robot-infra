"""Read-only bag-root comparison of take-up scoring; no ROS node or commands.

Restricted to fixed-gain, always-engaged compensated v171 prediction. Recorded
pose/strain/J are sufficient for this route, not for history-dependent rollouts.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict, replace
import hashlib
import inspect
from itertools import product
import json
from pathlib import Path

import numpy as np
import torch

from catheter_control.planning.mppi import CatheterMppi
from catheter_control.planning.path_tracking import ReferenceHorizon
from catheter_control.safety.hardware_contract import load_hardware_contract
from catheter_control.safety.validation import percentile_metrics
from catheter_control.transmission.backlash import (
    BacklashSnapshot, BacklashConfig, BacklashFeedforwardCompensator)
from .qualify_paced_compute import recorded_config


TOPICS = {'/device/state', '/catheter_mppi/estimator_trace',
          '/catheter_mppi/status', '/catheter_mppi/reference_horizon'}
POLICIES = ('historical', 'bounded', 'zero_penalty')


def constant_candidates(contract, horizon):
    """Sustained physical-shaft probes; no within-horizon reversals.

    This is a diagnostic population, not the live planner's proposals. Map
    shaft translation plus tendon travel back to the existing logical axes;
    production projection still applies all minimum speeds and limits.
    """
    axes = []
    for axis in (0, 2):
        maximum = float(contract.velocity_max[axis])
        minimum = float(contract.velocity_min[axis])
        axes.append(sorted(set([0., -maximum, -.5*maximum, -minimum,
                                minimum, .5*maximum, maximum])))
    pairs = list(product(*axes))
    proposals = np.zeros((len(pairs)+1, horizon, 3))
    for index, (shaft, tendon) in enumerate(pairs, 1):
        proposals[index, :] = [shaft+tendon, 0., tendon]
    return proposals


def candidate_attribution(ledger):
    """Decompose hold versus moving candidates without relaxing eligibility."""
    moving = np.any(np.abs(ledger['motor_rates']) > 1e-9, axis=(1, 2))
    _, internal = CatheterMppi._first_physical_direction(ledger['requested'])
    rows = {}
    for label, mask in (
            ('hold', np.arange(len(moving)) == 0),
            ('best_eligible_motion', moving & ledger['eligible']),
            ('best_excluded_motion', moving & ~ledger['eligible']),
            ('best_excluded_sustained_motion', moving & ~ledger['eligible']
             & ~np.any(internal, axis=1))):
        indices = np.flatnonzero(mask)
        if not len(indices):
            rows[label] = None
            continue
        index = int(indices[np.argmin(ledger['total_cost'][indices])])
        rows[label] = dict(index=index, eligible=bool(ledger['eligible'][index]),
            internal_reversal=bool(np.any(internal[index])),
            costs={key: float(ledger[key][index]) for key in (
                'tracking_cost', 'slew_cost', 'boundary_cost', 'takeup_cost', 'total_cost')},
            first_logical_velocity=ledger['projected'][index, 0].tolist(),
            takeup_position_offset=ledger['takeup_position_offset'][index].tolist())
    return rows


def finite(values, shape, name):
    array = np.asarray(values, dtype=float)
    if array.shape != shape or not np.isfinite(array).all():
        raise ValueError(f'invalid {name}: require finite {shape}')
    return array


def status_array(status, key, shape=(3,), signs=False):
    array = finite(json.loads(status[key]), shape, key)
    if signs and (np.any(np.abs(array) > 1) or np.any(array != np.round(array))):
        raise ValueError(f'invalid signs: {key}')
    return array.astype(np.int8) if signs else array


def transmission_from_status(status):
    """Restore full planner-facing travel evidence, never an engaged fixture."""
    fields = {name: status_array(status, 'backlash_'+name) for name in (
        'width_rad', 'width_positive_rad', 'width_negative_rad', 'remaining_rad',
        'remaining_lower_rad', 'remaining_upper_rad', 'confidence')}
    if any(np.any(fields[name] < 0) for name in (
            'width_rad', 'width_positive_rad', 'width_negative_rad', 'remaining_rad')):
        raise ValueError('negative recorded travel')
    for direction in ('positive', 'negative'):
        lo, hi = status_array(status, f'backlash_width_{direction}_interval_rad', (2, 3))
        fields[f'width_{direction}_lower_rad'] = lo
        fields[f'width_{direction}_upper_rad'] = hi
        if np.any(lo < 0) or np.any(hi < lo):
            raise ValueError('invalid width interval')
    if (np.any(fields['remaining_lower_rad'] < 0)
            or np.any(fields['remaining_upper_rad'] < fields['remaining_lower_rad'])
            or np.any(fields['confidence'] < 0) or np.any(fields['confidence'] > 1)):
        raise ValueError('invalid remaining interval/confidence')
    for name in ('motion_direction', 'engaged_direction'):
        fields[name] = status_array(status, 'backlash_'+name, signs=True)
    counts = status_array(status, 'backlash_confirmation_count')
    if np.any(counts < 0) or np.any(counts != np.round(counts)):
        raise ValueError('invalid confirmation count')
    fields['confirmation_count'] = counts.astype(np.int32)
    phase = tuple(json.loads(status['backlash_phase']))
    if len(phase) != 3 or not all(value in (
            'UNKNOWN', 'ENGAGED', 'TAKEUP', 'PROVISIONAL', 'FAILED') for value in phase):
        raise ValueError('invalid transmission phase')
    return BacklashSnapshot(**fields, phase=phase)


class FixedCandidates(CatheterMppi):
    """Research-only subclass; production grouping/projection/scoring unchanged."""
    def __init__(self, runtime, contract, config, candidates, policy):
        super().__init__(runtime, contract, config)
        if policy not in POLICIES:
            raise ValueError('unknown scoring policy')
        self.candidates, self.policy = candidates.copy(), policy

    def _samples(self, rotation_lock_direction=0):
        if rotation_lock_direction:
            raise ValueError('startup comparison requires rotation disabled')
        return self.candidates.copy()

    def _takeup_risk_cost(self, risk_s, prediction_horizon_s):
        if self.policy == 'historical':
            return self.config.takeup_risk_weight*risk_s
        if self.policy == 'zero_penalty':
            return np.zeros_like(risk_s)
        return super()._takeup_risk_cost(risk_s, prediction_horizon_s)


def read_cases(session, offsets, maximum_skew_s):
    """Receipt-causal latest prior evidence at each requested path offset."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    start = int(json.loads((session/'path_result.json').read_text())['started_ros_ns'])
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(session/'robot_bag'), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    classes = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()
               if t.name in TOPICS}
    if TOPICS-set(classes):
        raise ValueError(f'missing topics: {TOPICS-set(classes)}')
    latest, cases = {}, []
    while reader.has_next() and len(cases) < len(offsets):
        topic, payload, receipt = reader.read_next()
        if topic not in classes:
            continue
        message = deserialize_message(payload, classes[topic])
        if topic == '/device/state':
            if message.predicate != ord('P'):
                continue
            key = 'position'
        else:
            key = topic.rsplit('/', 1)[-1]
        latest[key] = (receipt, message)
        # Status owns command/lease/block evidence. Never consume messages
        # received after this anchor, even if their source stamp is earlier.
        if key != 'status' or receipt < start+int(offsets[len(cases)]*1e9):
            continue
        required = ('position', 'estimator_trace', 'reference_horizon', 'status')
        if any(k not in latest for k in required):
            raise ValueError('prefix missing required evidence')
        ages = {k: (receipt-latest[k][0])*1e-9 for k in required}
        if any(not 0 <= age <= maximum_skew_s for age in ages.values()):
            raise ValueError(f'stale evidence at offset {offsets[len(cases)]}: {ages}')
        status = {v.key: v.value for item in message.status for v in item.values}
        cases.append(dict(requested_offset_s=offsets[len(cases)],
            actual_offset_s=(receipt-start)*1e-9, receipt_ns=receipt,
            evidence_age_s=ages, status=status,
            trace=latest['estimator_trace'][1],
            position=finite(latest['position'][1].data, (6,), 'position'),
            reference=latest['reference_horizon'][1],
            reference_receipt_ns=latest['reference_horizon'][0]))
    if len(cases) != len(offsets):
        raise ValueError('recording ended before requested cases')
    return cases


def restore_engaged_root(runtime, trace):
    """Reconstruct recorded observed state; hidden history is not claimed exact."""
    if trace.header.frame_id != 'robot_base' or trace.state_timestamp_ns <= 0:
        raise ValueError('invalid recorded estimator frame/timestamp')
    if trace.accepted_observations <= 0 or trace.estimator_health not in ('TRACKING', 'DEGRADED'):
        raise ValueError('recorded root lacks accepted visual evidence')
    pose = finite(trace.interface_pose, (16,), 'pose').reshape(4, 4)
    strain = finite(trace.distal_strain, (24,), 'strain')
    motor = finite(trace.motor_angle_rad, (3,), 'effective motor')
    runtime.initialize(trace.state_timestamp_ns,
                       finite(trace.raw_encoder_counts, (3,), 'raw counts'),
                       interface_pose=pose, interface_motor_angle_rad=motor)
    root = runtime.clone_state()
    root.strain = torch.as_tensor(strain, device=runtime.device, dtype=runtime.dtype).clone()
    j = finite(trace.interface_jacobian, (18,), 'Jacobian').reshape(6, 3)
    jacobian = root.adaptive_jacobian
    jacobian.J_normalized = j*jacobian.action_scale[None]/jacobian.state_scale[:, None]
    np.testing.assert_allclose(jacobian.jacobian, j, rtol=1e-12, atol=1e-12)
    return root


def reference_targets(case, root, config):
    message = case['reference']
    stamp = message.header.stamp.sec*10**9+message.header.stamp.nanosec
    if message.header.frame_id != 'robot_base':
        raise ValueError('invalid path reference frame')
    reference = ReferenceHorizon(
        path_id=message.path_id, sequence=message.sequence,
        source_timestamp_ns=stamp, received_at_s=0.,
        sample_period_s=message.sample_period_s,
        positions_m=[[p.x, p.y, p.z] for p in message.positions],
        tangents=[[p.x, p.y, p.z] for p in message.tangents],
        expiry_s=message.expiry_s,
        progress_m=message.progress_m, total_length_m=message.total_length_m,
        final_hold=message.final_hold, progress_paused=message.progress_paused)
    target, tangent = reference.sample(root.timestamp_ns, config.horizon_steps,
        config.step_s, (case['receipt_ns']-case['reference_receipt_ns'])*1e-9)
    return target, None if reference.final_hold else tangent


def compare_policies(runtime, contract, config, root, position, target, kwargs,
                     candidates, repeats):
    """Fresh equal proposals/roots; diagnostic fences separated from timing."""
    rows, ledgers = {}, {}
    for policy in POLICIES:
        captured = []
        diagnostic_config = replace(config, planning_deadline_s=60.)
        planner = FixedCandidates(runtime, contract, diagnostic_config, candidates, policy)
        result = planner.plan(root.clone(), position, target,
                              candidate_diagnostics=captured.append, **kwargs)
        if not result.valid or len(captured) != 1:
            raise RuntimeError(f'{policy} diagnostic solve failed: {result.reason}')
        ledgers[policy] = captured[0]
        times, misses, invalid = [], 0, 0
        for _ in range(repeats):
            timed = FixedCandidates(runtime, contract, config, candidates, policy)
            result_timed = timed.plan(root.clone(), position, target, **kwargs)
            times.append(result_timed.elapsed_s*1000)
            misses += result_timed.reason == 'deadline_missed'
            invalid += not result_timed.valid
            if result_timed.valid and result_timed.selected_candidate_index != result.selected_candidate_index:
                raise RuntimeError('timed solve changed the matched decision')
        rows[policy] = dict(selected_candidate=result.selected_candidate_index,
            command=result.command_logical_velocity.tolist(),
            total_cost=result.selected_total_cost, hold_cost=result.zero_total_cost,
            raw_delay_s=result.selected_takeup_risk_s, delay_cost=result.selected_takeup_risk_cost,
            blocked_count=result.blocked_candidate_count,
            timing_ms=percentile_metrics(times), repeats=repeats,
            deadline_misses=int(misses), invalid_timed_plans=int(invalid))
    first = ledgers['historical']
    invariant = ('requested', 'projected', 'motor_rates', 'takeup_position_offset',
                 'eligible', 'raw_takeup_risk_s', 'tracking_cost', 'slew_cost', 'boundary_cost')
    for ledger in ledgers.values():
        for key in invariant:
            np.testing.assert_array_equal(first[key], ledger[key], err_msg=f'unmatched {key}')
        np.testing.assert_allclose(ledger['total_cost'], sum(ledger[k] for k in (
            'tracking_cost', 'slew_cost', 'boundary_cost', 'takeup_cost')), rtol=1e-5, atol=1e-4)
    return rows, ledgers


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', type=Path)
    parser.add_argument('--model-manifest', required=True)
    parser.add_argument('--limits-file', required=True)
    parser.add_argument('--device', choices=('cpu', 'cuda'), default='cuda')
    parser.add_argument('--offsets-s', type=float, nargs='+', default=[.9, 4.9])
    parser.add_argument('--maximum-skew-s', type=float, default=.20)
    parser.add_argument('--repeats', type=int, default=10)
    parser.add_argument('--seed', type=int, default=17)
    parser.add_argument('--proposal-set', choices=('sampled', 'constant_grid'),
                        default='sampled', help='Offline proposal ablation only')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    if (not 1 <= len(args.offsets_s) <= 8 or args.offsets_s != sorted(set(args.offsets_s))
            or not all(np.isfinite(v) and 0 <= v <= 300 for v in args.offsets_s)
            or not 1 <= args.repeats <= 100 or not 0 < args.maximum_skew_s <= .25):
        parser.error('require 1..8 ascending unique offsets in [0,300], repeats 1..100, skew <=.25s')
    if args.device == 'cuda' and not torch.cuda.is_available():
        raise RuntimeError('CUDA requested but unavailable; no fallback')
    torch.set_num_threads(2)
    torch.set_num_interop_threads(1)
    record = json.loads((args.session/'controller_manifest.json').read_text())
    params = record['controller_parameters']
    if (not params['takeup_transaction_enabled'] or not params['backlash_compensation_enabled']
            or params.get('engagement_control_enabled', False) or params['engaged_gain_enabled']
            or params['mppi_engaged_gain_scenarios'] or not params['mppi_grouped_mode_sampling']):
        raise ValueError('requires baseline grouped, fixed-gain compensated session')
    if params.get('adaptation_enabled', False):
        raise ValueError('recorded Jacobian replay requires fixed adaptation configuration')
    config = replace(recorded_config(params), always_engaged_rollout=True, seed=args.seed)
    contract = load_hardware_contract(args.limits_file, 'imricor_test')
    maximum = finite(params['controller_velocity_max'], (6,), 'velocity max')
    if maximum[1] != 0 or np.any(maximum < 0) or np.any(maximum > contract.velocity_max):
        raise ValueError('requires rotation disabled and recorded velocities within limits')
    contract = replace(contract, velocity_max=maximum,
                       velocity_min=np.minimum(contract.velocity_min, maximum))
    takeup = BacklashFeedforwardCompensator(BacklashConfig(
        width_rad=tuple(params['backlash_width_rad']),
        takeup_velocity=tuple(params['backlash_takeup_velocity']))).motor_radians_per_second(contract)
    cases = read_cases(args.session, args.offsets_s, args.maximum_skew_s)
    from catheter_control.orchestration.runtime import load_runtime
    bundle = load_runtime(args.model_manifest, device=args.device,
                          options={'marker_estimator': 'ukf', 'adaptation_enabled': False})
    if bundle.identity.manifest_sha256 != record['model_manifest']['sha256']:
        raise ValueError('model manifest differs from recorded session')
    runtime = bundle.runtime
    args.output.mkdir(parents=True, exist_ok=False)
    rows = []
    for index, case in enumerate(cases):
        print(json.dumps({'stage': 'case_start', 'case': index,
                          'offset_s': case['actual_offset_s']}), flush=True)
        root = restore_engaged_root(runtime, case['trace'])
        tip_error_mm = float(1000*np.linalg.norm(
            runtime.markers_for_state(root)[-1].detach().cpu().numpy()
            - finite(case['trace'].estimated_tip_m, (3,), 'recorded estimated tip')))
        if tip_error_mm > .001:
            raise ValueError(f'restored root FK differs by {tip_error_mm} mm')
        target, tangent = reference_targets(case, root, config)
        status = case['status']
        kwargs = dict(target_tangent_base=tangent,
            observed_tip_base_m=finite(case['trace'].observed_tip_m, (3,), 'tip'),
            previous_logical_velocity=status_array(status, 'command', (6,)),
            transmission_state=transmission_from_status(status),
            takeup_motor_radians_per_second=takeup,
            blocked_motor_direction=status_array(status, 'planner_blocked_motor_direction', signs=True),
            direction_lease=status_array(status, 'reversal_lease_direction', signs=True),
            approved_reversal_direction=status_array(status, 'reversal_approved_direction', signs=True))
        candidates = (constant_candidates(contract, config.horizon_steps)
                      if args.proposal_set == 'constant_grid' else
                      CatheterMppi(runtime, contract, config)._samples())
        summaries, ledgers = compare_policies(runtime, contract, config, root,
            case['position'], target, kwargs, candidates, args.repeats)
        ledger_file = args.output/f'case_{index}_candidates.npz'
        np.savez_compressed(ledger_file, **{f'{policy}_{key}': value for policy, ledger in ledgers.items()
                                          for key, value in ledger.items()})
        inputs = dict(status=case['status'], reference=dict(
            source_ns=case['reference'].header.stamp.sec*10**9+case['reference'].header.stamp.nanosec,
            receipt_ns=case['reference_receipt_ns'], path_id=case['reference'].path_id,
            sequence=case['reference'].sequence, final_hold=case['reference'].final_hold,
            progress_paused=case['reference'].progress_paused),
            recorded_state={name: list(getattr(case['trace'], name)) for name in (
                'raw_encoder_counts', 'motor_angle_rad', 'interface_pose', 'distal_strain',
                'interface_jacobian', 'observed_tip_m', 'estimated_tip_m')})
        inputs_file = args.output/f'case_{index}_recorded_inputs.json'
        inputs_file.write_text(json.dumps(inputs, indent=2, allow_nan=False)+'\n')
        rows.append(dict(index=index, actual_offset_s=case['actual_offset_s'],
            root_ns=root.timestamp_ns, evidence_age_s=case['evidence_age_s'],
            root_source_age_s=(case['receipt_ns']-root.timestamp_ns)*1e-9,
            restored_tip_error_mm=tip_error_mm,
            position=case['position'].tolist(), target_m=target.tolist(),
            transmission={name: getattr(kwargs['transmission_state'], name) for name in (
                'width_positive_rad', 'width_negative_rad', 'remaining_rad',
                'width_positive_lower_rad', 'width_positive_upper_rad',
                'width_negative_lower_rad', 'width_negative_upper_rad',
                'remaining_lower_rad', 'remaining_upper_rad', 'phase',
                'motion_direction', 'engaged_direction', 'confidence')},
            previous_command=kwargs['previous_logical_velocity'].tolist(),
            policies=summaries, candidate_file=ledger_file.name,
            candidate_attribution={policy: candidate_attribution(ledger)
                                   for policy, ledger in ledgers.items()},
            candidate_sha256=hashlib.sha256(ledger_file.read_bytes()).hexdigest(),
            input_file=inputs_file.name,
            input_sha256=hashlib.sha256(inputs_file.read_bytes()).hexdigest(),
            matched_inputs=True))
        print(json.dumps({'stage': 'case_done', 'case': index, 'policies': summaries}), flush=True)
    def serial(value):
        if isinstance(value, np.ndarray):
            return value.tolist()
        raise TypeError(type(value).__name__)
    report = dict(scope='OFFLINE_RECORDED_VISUAL_ROOT_TAKEUP_SCORING', executed_commands=0,
        configuration=asdict(config), runtime_identity=asdict(bundle.identity), rows=rows,
        planner_source=inspect.getfile(CatheterMppi),
        planner_sha256=hashlib.sha256(Path(inspect.getfile(CatheterMppi)).read_bytes()).hexdigest(),
        runtime_source=inspect.getfile(type(runtime)),
        runtime_sha256=hashlib.sha256(Path(inspect.getfile(type(runtime))).read_bytes()).hexdigest(),
        source_sha256={name: hashlib.sha256((args.session/name).read_bytes()).hexdigest()
                       for name in ('controller_manifest.json', 'path_result.json', 'robot_bag/metadata.yaml')},
        session=str(args.session.resolve()), device=args.device,
        proposal_set=args.proposal_set,
        limitations=['No ROS nodes, publishers, services, hardware, arbiter execution or plant simulation',
            'Latest prior receipt evidence; source/receipt/root clocks differ; evidence ages reported',
            'Recorded pose, strain, effective motor and J; not exact original planner snapshot',
            'Hidden history initialized, not recorded; always-engaged fixed-gain route only',
            'Seeded fresh proposals, not original warm start or original sampled population',
            'Recorded preview expiry used; no reconstruction of node-level readiness/commit gates',
            'Input hashes cover extracted evidence and metadata, not the entire rosbag database',
            'Diagnostic candidate copies are intrusive and excluded from timed repeated solves',
            'Warmed isolated algorithm timings are not full-stack deadline qualification'])
    (args.output/'report.json').write_text(json.dumps(report, default=serial, indent=2, allow_nan=False)+'\n')
    print(json.dumps({'report': str(args.output/'report.json')}), flush=True)


if __name__ == '__main__':
    main()
