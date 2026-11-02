"""Offline task-space audit of recorded MPPI first-step forecasts.

Trusted local ROS-message cache only. No nodes, hardware, fit or tuning.
Target is the recorded terminal reference, NOT the full MPPI cost/horizon.
"""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import pickle

import numpy as np

from cr_meta_lnn.training.hybrid.control_response import task_response


def compare_candidates(args):
    """Conditional distal-prior comparison, NOT reconstructed closed-loop MPPI."""
    import inspect
    import torch
    from dataclasses import replace
    from itertools import product
    from cr_meta_lnn.deployment import load_runtime_bundle
    from cr_meta_lnn.training.hybrid.profile_diagnostic import StaticMapInputPrior, profile_shape_forecast
    from cr_meta_lnn.training.hybrid.control_response import candidate_position_cost, candidate_response_difference
    from catheter_control.planning.mppi import CatheterMppi, MppiConfig
    from catheter_control.safety.hardware_contract import load_hardware_contract

    torch.set_num_threads(2)
    saved = json.loads(Path(args.candidate_roots).read_text())
    if not saved['completed'] or saved['confirmation_policy'] != 'shared_baseline':
        raise ValueError('completed shared-confirmation roots required')
    for path in (args.cache,args.manifest):
        identity = dict(path=str(Path(path).resolve()),sha256=hashlib.sha256(Path(path).read_bytes()).hexdigest())
        if identity not in saved['identities']:
            raise ValueError('saved root input identity changed: '+path)
    datasets = pickle.loads(Path(args.cache).read_bytes())
    new = load_runtime_bundle(args.manifest, device='cpu', dtype='float32').runtime
    paths = [Path(p) for p in (args.cache, args.candidate_roots, args.manifest, args.limits, __file__,
        inspect.getfile(candidate_position_cost), inspect.getfile(profile_shape_forecast),
        inspect.getfile(CatheterMppi), inspect.getfile(type(new)))]
    report = dict(completed=False, scope='Offline conditional matched-root candidate sensitivity; no commands.',
        identities={str(p.resolve()):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths},
        limitations=[
            'Common saved experimental strain; common past-only baseline pose/J because experimental roots did not save pose. Material gauge compatibility is conditional, not qualified.',
            'This is a distal-prior ablation, not full baseline versus new-pipeline state/estimator parity.',
            'Baseline uses canonical always-engaged coarse rollout; profile variants use existing mean-state mechanics/FK, sharing its endpoint proximal pose.',
            'Terminal Cartesian position cost only: no moving corridor, action, reversal, take-up arbitration or commit gates. Not a replay of the complete MPPI optimizer.',
            'Candidate actions are hypothetical and projected by the canonical MPPI projection. No unexecuted outcome is observed.',
            'Zero-action response is modeled passive drift; subtracting it does not measure plant action response.',
            'All cost deltas are anchored at the raw root tip, removing model-root readout offset.',
            'The past terminal reference is at most 300ms old, held fixed; reference age and pose age are explicit.',
        ], sessions=[])
    for index in args.indices:
        data = datasets[index]
        source = next(s for s in saved['sessions'] if s['name'] == data['name'])
        base = load_runtime_bundle(data['params']['model_manifest'], device='cpu', dtype='float32',
            options={'marker_estimator':'ukf','adaptation_enabled':False}).runtime
        for path in (Path(data['params']['model_manifest']),Path(inspect.getfile(type(base)))):
            report['identities'][str(path.resolve())] = hashlib.sha256(path.read_bytes()).hexdigest()
        limits = load_hardware_contract(args.limits, 'imricor_test')
        maximum = np.asarray(data['params']['controller_velocity_max'], float)
        contract = replace(limits, velocity_max=maximum,
            velocity_min=np.where(maximum == 0, 0., limits.velocity_min))
        planner = CatheterMppi(base, contract, MppiConfig(always_engaged_rollout=True))
        law = StaticMapInputPrior(base.history_model, base.motor_transmission, new.tendon_mm_per_shaft_rad, new.profile.input_knots)
        weights = [1., 0., 1.] if isinstance(data['task'], dict) and data['task'].get('xz_only') else [1., 1., 1.]
        supported = []; rows = []; exclusions = Counter()
        for record in source['rows']:
            stamp = int(record['start_ns'])
            refs = [t for t in data['response'] if int(t.forecast_start_timestamp_ns) <= stamp and
                any(a <= int(t.forecast_start_timestamp_ns) <= stamp <= b for a,b in data['intervals'])]
            traces = [t for t in data['trace'] if int(t.state_timestamp_ns) <= stamp and int(t.observation_timestamp_ns) <= stamp]
            if not refs or not traces:
                exclusions['missing_past_reference_or_pose'] += 1; continue
            ref = max(refs,key=lambda t:t.forecast_start_timestamp_ns)
            trace = max(traces,key=lambda t:t.state_timestamp_ns)
            if stamp-int(ref.forecast_start_timestamp_ns)>300_000_000 or stamp-int(trace.state_timestamp_ns)>100_000_000:
                exclusions['stale_reference_or_pose'] += 1; continue
            marker = interpolate(data['markers'],stamp); counts = interpolate(data['encoder'],stamp)
            position = interpolate(data['position'],stamp)
            if any(v is None for v in (marker,counts,position)):
                exclusions['unsupported_root'] += 1; continue
            supported.append((record,ref,trace,marker,counts,position))
        chosen_indices = np.linspace(0,len(supported)-1,min(len(supported),args.root_limit),dtype=int)
        for root_index in chosen_indices:
            record,ref,trace,marker,counts,position = supported[root_index]
            stamp = int(record['start_ns'])
            pose = np.asarray(trace.interface_pose).reshape(4,4)
            state_mean = record['static_inference_root']
            base.initialize(stamp,counts,interface_pose=pose)
            root = base.clone_state()
            root.strain = torch.as_tensor(state_mean['strain']).to(root.strain).clone()
            root.adaptive_jacobian.J_normalized = (
                np.asarray(trace.interface_jacobian).reshape(6,3)
                * root.adaptive_jacobian.action_scale[None,:]
                / root.adaptive_jacobian.state_scale[:,None])
            equilibrium = float(base._stationary_equilibrium_shift(root.strain))
            # Include small motions: a coarse nine-command grid can make hold
            # win merely because every moving candidate overshoots a close target.
            intent = np.asarray([[a,0,b] for a,b in product((-4.,-1.,0.,1.,4.),(-2.,-.5,0.,.5,2.))])
            count = len(intent); hold = count//2
            for horizon in (.2,.8):
                steps = int(round(horizon/.2)); dt = np.full(steps,.2)
                requested = np.broadcast_to(intent[:,None,:],(count,steps,3)).copy()
                logical, realized, rates, _, _, blocked = planner._project(requested,position,dt_sequence=dt)
                prediction = base.predict_control_sequence_coarse(root,rates[:,:,:3],dt,always_engaged=True)
                initial_tip = new.kinematics(torch.as_tensor(pose).to(new.mechanics.mode),root.strain.to(new.mechanics.mode))[-1].detach().numpy()
                total = {'v171_equilibrium':1000*(prediction.tip_base_m[:,-1].detach().numpy()-initial_tip)}
                final_pose = prediction.final_state.interface_pose.detach()
                times = np.r_[0.,np.cumsum(dt)]
                for label,prior,anchor in (
                    ('static_equilibrium_conformance',law,equilibrium),
                    ('fitted_profile_equilibrium',new.profile,equilibrium),
                    ('static_posterior_coordinate',law,state_mean['coordinate'])):
                    deltas = []
                    for candidate in range(count):
                        tendon_mm = np.r_[0.,np.cumsum(rates[candidate,:,2]*new.tendon_mm_per_shaft_rad*dt)]
                        _,_,strain = profile_shape_forecast(new.mechanics,new.kinematics,prior,state_mean['strain'],anchor,times,tendon_mm,return_strain=True)
                        tip = new.kinematics(final_pose[candidate].to(new.mechanics.mode),torch.as_tensor(strain[-1]).to(new.mechanics.mode))[-1].detach().numpy()
                        deltas.append(1000*(tip-initial_tip))
                    total[label] = np.asarray(deltas)
                conformance_error_mm = float(np.max(np.linalg.norm(
                    total['static_equilibrium_conformance']-total['v171_equilibrium'],axis=1)))
                if conformance_error_mm > .01:
                    raise ValueError(f'common-mechanics/FK conformance failed: {conformance_error_mm} mm')
                error = 1000*(np.asarray(ref.target_tip_m)-marker[-1])
                metrics = {label:candidate_position_cost(error,delta,delta[hold],weights) for label,delta in total.items()}
                for metric in metrics.values():
                    metric['ordering'] = [i for i in metric['ordering'] if not blocked[i]]
                    if not metric['ordering']:
                        raise ValueError('no unblocked diagnostic candidate')
                rows.append(dict(start_ns=stamp,horizon_s=horizon,phase=record['phase'],
                    static_equilibrium_conformance_error_mm=conformance_error_mm,
                    reference_age_ms=(stamp-int(ref.forecast_start_timestamp_ns))*1e-6,
                    pose_age_ms=(stamp-int(trace.state_timestamp_ns))*1e-6,
                    equilibrium_coordinate=equilibrium,posterior_coordinate=state_mean['coordinate'],
                    error_mm=error.tolist(),intent_logical_velocity=intent.tolist(),
                    projected_logical=logical.tolist(),projected_motor_rates=rates.tolist(),blocked=blocked.tolist(),
                    total_delta_mm={k:v.tolist() for k,v in total.items()},metrics=metrics,
                    differences_from_v171={label:candidate_response_difference(total['v171_equilibrium'],delta,hold,weights)
                        for label,delta in total.items()}))
        summary = {}
        for horizon in (.2,.8):
            selected = [r for r in rows if r['horizon_s']==horizon]
            summary[str(horizon)] = {label:dict(n=len(selected),
                top_candidate_disagrees_with_v171=sum(r['metrics'][label]['ordering'][0]!=r['metrics']['v171_equilibrium']['ordering'][0] for r in selected),
                    hold_wins=sum(r['metrics'][label]['ordering'][0]==12 for r in selected),
                action_insensitive=sum(not r['metrics'][label]['action_sensitive'] for r in selected),
                median_passive_difference_mm=float(np.median([r['differences_from_v171'][label]['passive_difference_mm'] for r in selected])) if selected else None,
                median_candidate_input_difference_rms_mm=float(np.median([r['differences_from_v171'][label]['candidate_input_difference_rms_mm'] for r in selected])) if selected else None,
                median_max_input_response_mm=float(np.median([np.max(np.linalg.norm(np.asarray(r['metrics'][label]['input_delta_mm'])*np.sqrt(weights),axis=1)) for r in selected])) if selected else None)
                for label in ('v171_equilibrium','fitted_profile_equilibrium','static_posterior_coordinate')}
        report['sessions'].append(dict(name=data['name'],weights=weights,source_roots=len(source['rows']),
            supported_roots=len(supported),selected_roots=len(chosen_indices),excluded=dict(exclusions),summary=summary,rows=rows))
        print(data['name'],json.dumps(summary),flush=True)
    report['completed'] = True
    out = Path(args.output); out.mkdir(parents=True,exist_ok=False)
    (out/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print('Report:',out/'report.json')


def interpolate(rows, stamp):
    times = np.asarray([r[0] for r in rows], dtype=np.int64)
    j = int(np.searchsorted(times, stamp))
    if j < len(times) and times[j] == stamp:
        return np.asarray(rows[j][1]).copy()
    if j == 0 or j == len(times) or times[j]-times[j-1] > 100_000_000:
        return None
    a, b = rows[j-1], rows[j]
    return np.asarray(a[1])+(np.asarray(b[1])-a[1])*((stamp-a[0])/(b[0]-a[0]))


def summarize(rows, key):
    values = [r[key] for r in rows]
    resolved = [v for v in values if v['measured_response_mm'] >= .3]
    pairs = [(v['predicted_descent_projection_mm2'], v['measured_descent_projection_mm2'])
             for v in resolved if v['prediction_informative']]
    cosine = [v['response_cosine'] for v in resolved if v['response_cosine'] is not None]
    ratios = [v['response_scale_ratio'] for v in resolved if v['response_scale_ratio'] is not None]
    return dict(n=len(values), resolved_n=len(resolved),
        uninformative=sum(not v['prediction_informative'] for v in values),
        resolved_wrong_response_halfspace=sum(c < 0 for c in cosine),
        resolved_task_direction_disagreement=sum(p*m < 0 for p, m in pairs),
        resolved_expected_descent_observed_ascent=sum(p > 0 and m < 0 for p, m in pairs),
        resolved_observed_descent=sum(v['measured_descent_projection_mm2'] > 0 for v in resolved),
        resolved_predicted_overshoot=sum(v['predicted_overshoot'] for v in resolved),
        resolved_response_cosine_median=float(np.median(cosine)) if cosine else None,
        resolved_response_scale_ratio_median=float(np.median(ratios)) if ratios else None,
        expected_error_decrease_observed_increase=sum(
            v['predicted_error_reduction_mm2'] > 0 and v['measured_error_reduction_mm2'] < 0
            for v in resolved))


def summarize_cartesian(args):
    """Summarize own-pose replay outputs without rerunning or fitting models."""
    path = Path(args.cartesian_report)
    source = json.loads(path.read_text())
    if not source['completed']:
        raise ValueError('completed Cartesian replay required')
    report = dict(scope='Own-pose achieved-input Cartesian control response; no hardware or counterfactual plant outcomes.',
        source=str(path.resolve()),source_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        limitations=source['limitations'],sessions=[])
    for session in source['sessions']:
        rows = []
        for root in session['rows']:
            for value in root['cartesian']:
                measured = np.asarray(value['measured_delta_mm'])
                error = value['target_error_mm']
                row = dict(start_ns=root['start_ns'],end_ns=value['end_ns'],horizon_s=value['horizon_s'],
                    reference_supported=error is not None,phase=root['phase'],models={})
                for label,predicted in value['predictions'].items():
                    p = np.asarray(predicted); passive = np.asarray(value['passive'][label]); offset=np.asarray(value['root_offsets_mm'][label])
                    row['models'][label] = dict(
                        anchored=task_response([0,0,0] if error is None else error,p,measured,value['weights']),
                        absolute=task_response([0,0,0] if error is None else error,p+offset,measured,value['weights']),
                        input_response_mm=float(np.linalg.norm((p-passive)*np.sqrt(value['weights']))),
                        passive_response_mm=float(np.linalg.norm(passive*np.sqrt(value['weights']))),
                        root_offset_mm=float(np.linalg.norm(offset*np.sqrt(value['weights']))))
                rows.append(row)
        summary = {}
        for horizon in (.2,.8):
            selected = [r for r in rows if r['horizon_s']==horizon]
            summary[str(horizon)] = {}
            for label in sorted({k for r in selected for k in r['models']}):
                entry = {}; values=[r['models'][label] for r in selected]
                for mode in ('anchored','absolute'):
                    resolved = [r for r in selected if r['models'][label][mode]['measured_response_mm']>=.3]
                    task = [r for r in resolved if r['reference_supported']]
                    v = [r['models'][label][mode] for r in resolved]
                    cosines=[m['response_cosine'] for m in v if m['response_cosine'] is not None]
                    ratios=[m['response_scale_ratio'] for m in v if m['response_scale_ratio'] is not None]
                    entry[mode] = dict(n=len(selected),resolved_n=len(v),resolved_task_n=len(task),
                        wrong_response_halfspace=sum(c<0 for c in cosines),
                        median_response_cosine=float(np.median(cosines)) if cosines else None,
                        median_response_scale_ratio=float(np.median(ratios)) if ratios else None,
                        expected_descent_observed_ascent=sum(r['models'][label][mode]['predicted_descent_projection_mm2']>0
                            and r['models'][label][mode]['measured_descent_projection_mm2']<0 for r in task),
                        task_direction_disagreement=sum(r['models'][label][mode]['predicted_descent_projection_mm2']
                            *r['models'][label][mode]['measured_descent_projection_mm2']<0 for r in task))
                for key in ('input_response_mm','passive_response_mm','root_offset_mm'):
                    entry['median_'+key]=float(np.median([v[key] for v in values]))
                summary[str(horizon)][label]=entry
        report['sessions'].append(dict(name=session['name'],summary=summary,rows=rows))
        print(session['name'],json.dumps(summary),flush=True)
    out=Path(args.output);out.mkdir(parents=True,exist_ok=False)
    (out/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print('Report:',out/'report.json')


def run(args):
    cache = Path(args.cache)
    datasets = pickle.loads(cache.read_bytes())
    report = dict(scope='Recorded first-step task response, frozen terminal reference; not full MPPI cost or candidate ranking. No executed commands.',
        identities={str(p.resolve()): hashlib.sha256(p.read_bytes()).hexdigest()
                    for p in (cache, Path(__file__), Path(__import__(task_response.__module__, fromlist=['']).__file__))},
        limitations=[
            'Actual achieved action can change within a forecast; command is not assumed constant.',
            'Response labels interpolate raw marker tip offline, including bracketing future observations; no estimator uses these labels.',
            'Forecast due and observation endpoints differ; lateness is recorded.',
            'Absolute forecast is compared at due time; offset-free trace increment is an anchoring diagnostic, not a repaired model.',
            'Only executed actions are observed; no causal attribution of drift or verification of unexecuted candidates.',
            'Path terminal reference is frozen for this diagnostic; along-path trajectory cost is not reconstructed.',
        ], sessions=[])
    for index in args.indices:
        data = datasets[index]; rows = []; excluded = Counter()
        weights = [1., 0., 1.] if isinstance(data['task'], dict) and data['task'].get('xz_only') else [1., 1., 1.]
        for trace in data['response']:
            start = int(trace.forecast_start_timestamp_ns)
            due = int(trace.forecast_due_timestamp_ns)
            end = int(trace.observation_timestamp_ns)
            if not any(a <= start < due <= end <= b for a, b in data['intervals']):
                excluded['outside_active_task'] += 1; continue
            tip0 = interpolate(data['markers'], start)
            tip1 = interpolate(data['markers'], due)
            pos0 = interpolate(data['position'], start)
            pos1 = interpolate(data['position'], due)
            if any(x is None for x in (tip0, tip1, pos0, pos1)):
                excluded['missing_or_gapped_raw_label'] += 1; continue
            error = 1000*(np.asarray(trace.target_tip_m)-tip0[-1])
            measured = 1000*(tip1[-1]-tip0[-1])
            absolute_prediction = 1000*(np.asarray(trace.predicted_tip_m)-tip0[-1])
            delta = pos1-pos0
            row = dict(start_ns=start, due_ns=due, observation_ns=end,
                horizon_s=(due-start)*1e-9, observation_lateness_ms=(end-due)*1e-6,
                start_observation_age_ms=(start-int(trace.start_observation_timestamp_ns))*1e-6,
                command_logical_velocity=list(trace.command_logical_velocity),
                achieved_logical_delta=list(delta), achieved_chassis_delta_mm=float(delta[0]-delta[2]),
                target_error_mm=list(error), measured_tip_delta_mm=list(measured),
                absolute_predicted_delta_mm=list(absolute_prediction),
                trace_predicted_delta_mm=list(trace.predicted_delta_mm),
                anchor_offset_mm=list(1000*(np.asarray(trace.start_tip_m)-tip0[-1])),
                absolute_forecast=task_response(error, absolute_prediction, measured, weights),
                offset_free_increment=task_response(error, trace.predicted_delta_mm, measured, weights))
            rows.append(row)
        diagnostics = {}
        for name, values in (
                ('start_observation_age_ms', [r['start_observation_age_ms'] for r in rows]),
                ('observation_lateness_ms', [r['observation_lateness_ms'] for r in rows]),
                ('task_anchor_offset_mm', [float(np.sqrt(np.asarray(weights) @
                     np.square(r['anchor_offset_mm']))) for r in rows])):
            diagnostics[name] = dict(p50=float(np.median(values)),
                p95=float(np.percentile(values, 95)), max=float(max(values))) if values else None
        report['sessions'].append(dict(name=data['name'], task_weights=weights,
            anchoring_and_timing=diagnostics,
            excluded=dict(excluded), summary={key:summarize(rows,key) for key in
                ('absolute_forecast','offset_free_increment')}, rows=rows))
    out = Path(args.output); out.mkdir(parents=True, exist_ok=False)
    (out/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps({s['name']:s['summary'] for s in report['sessions']}, indent=2))
    print('Report:', out/'report.json')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cache')
    parser.add_argument('--indices', nargs='+', type=int, default=[0, 4, 6])
    parser.add_argument('--output', required=True)
    parser.add_argument('--candidate-roots', help='Saved consistent-static roots; conditional candidate ablation')
    parser.add_argument('--manifest')
    parser.add_argument('--limits')
    parser.add_argument('--root-limit', type=int, default=12)
    parser.add_argument('--cartesian-report', help='Completed own-pose Cartesian replay report; summarize direction and task relevance')
    args = parser.parse_args()
    if args.cartesian_report:
        if args.candidate_roots:
            parser.error('choose one comparison mode')
        summarize_cartesian(args)
    elif args.candidate_roots:
        if not args.cache or not args.manifest or not args.limits or args.root_limit < 1:
            parser.error('candidate comparison requires cache, manifest, limits and positive root-limit')
        compare_candidates(args)
    else:
        if not args.cache:
            parser.error('recorded response comparison requires cache')
        run(args)
