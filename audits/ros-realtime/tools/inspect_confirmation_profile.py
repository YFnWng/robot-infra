"""Offline confirmation-policy and causal profile-registration diagnostic.

No nodes, publishers, hardware APIs, or production parameter mutations.
The cache must be the trusted read-only-bag cache named in the maintained audit.
"""
import argparse
from collections import Counter, deque
from dataclasses import asdict, fields
import hashlib
import inspect
import json
from pathlib import Path
import pickle
import time
from types import SimpleNamespace

import numpy as np
import torch

from catheter_control.transmission.backlash import BacklashConfig, BacklashStateEstimator
from catheter_control.planning.engagement import confirm_at_source, tendon_confirmation
from cr_meta_lnn.deployment import load_runtime_bundle
from cr_meta_lnn.deployment.v171_streaming_runtime import ENCODER_RADIANS_PER_COUNT
from cr_meta_lnn.training.hybrid.causal_shape_prediction import shape_signature, make_window, score_windows
from cr_meta_lnn.deployment.experimental.joint_engagement_filter import JointEngagementFilter
from cr_meta_lnn.training.hybrid.profile_diagnostic import (
    profile_shape_forecast, fit_completed_profile_prefix, response_error_geometry, encoder_memory,
    completed_strain_input)
from cr_meta_lnn.training.hybrid.profile_diagnostic import static_map_shape_forecast, fit_completed_static_prefix
from cr_meta_lnn.training.hybrid.profile_diagnostic import static_prior_filter_from_initial_event
from cr_meta_lnn.training.hybrid.profile_diagnostic import StaticMapInputPrior, distal_forecast_components, input_response_attribution
from cr_meta_lnn.training.hybrid.profile_diagnostic import profile_cartesian_forecast
from cr_meta_lnn.training.hybrid.control_response import task_response
from cr_meta_lnn.networks.hybrid.v171_engaged_lambda import FrozenV171Mechanics


def identity(path):
    path = Path(path)
    return dict(path=str(path.resolve()), sha256=hashlib.sha256(path.read_bytes()).hexdigest())


def interpolate(rows, stamp):
    times = np.asarray([r[0] for r in rows], np.int64)
    j = int(np.searchsorted(times, stamp))
    if j < len(times) and times[j] == stamp:
        return np.asarray(rows[j][1]).copy()
    if j == 0 or j == len(times) or times[j]-times[j-1] > 100_000_000:
        return None
    a, b = rows[j-1], rows[j]
    return np.asarray(a[1])+(np.asarray(b[1])-a[1])*((stamp-a[0])/(b[0]-a[0]))


def scores(rows):
    output = {}
    for label in sorted({k for r in rows for k in r['predictions']}):
        chosen = [r for r in rows if label in r['predictions']]
        size = len(chosen[0]['predictions'][label])
        error = np.asarray([np.asarray(r['predictions'][label])-np.asarray(r['measured'])[:size] for r in chosen])
        norms = np.sqrt(np.mean(error[:, :6]**2, axis=1))*1000
        output[label] = dict(n=len(chosen), distance_rms_mm=float(np.sqrt(np.mean(error[:, :6]**2))*1000),
            bow_rms_mm=float(np.sqrt(np.mean(error[:, 6:]**2))*1000) if size>6 else None,
            distance_p50_mm=float(np.median(norms)), distance_p95_mm=float(np.percentile(norms, 95)),
            distance_max_mm=float(norms.max()))
    return output


def response_diagnostics(rows):
    """Exact label-only error decomposition; never update a model or forecast."""
    output = {}
    for label in sorted({k for r in rows for k in r['predictions']}):
        chosen = [r for r in rows if label in r['predictions']]
        geometry = [response_error_geometry(r['predictions'][label][:6], r['measured'][:6]) for r in chosen]
        groups = {'all': list(range(len(chosen))),
            'resolved_8descriptor_0.3mm': [i for i,r in enumerate(chosen)
                if np.sqrt(np.mean(np.square(r['measured'])))*1000 >= .3],
            'past_fit_eligible': [i for i,r in enumerate(chosen) if r['eligible_past_fit']]}
        for group, predicate in {
            'future_monotone_confirmed': lambda r: r['future_monotone_confirmed'],
            'resolved_future_monotone_confirmed': lambda r: r['future_monotone_confirmed'] and
                np.sqrt(np.mean(np.square(r['measured'])))*1000 >= .3,
            'future_encoder_reversal': lambda r: r['future_encoder_reversals'] > 0,
        }.items():
            groups[group] = [i for i,r in enumerate(chosen) if predicate(r)]
        for phase in ('UNKNOWN', 'TAKEUP', 'PROVISIONAL', 'ENGAGED'):
            groups['root_'+phase] = [i for i,r in enumerate(chosen) if r['phase']==phase]
        summary = {}
        for group, indices in groups.items():
            values = [geometry[i] for i in indices]
            if not values:
                continue
            item = dict(n=len(values), worse_than_hold=sum(r['worse_than_hold'] for r in values),
                wrong_halfspace=sum(r['wrong_halfspace'] is True for r in values),
                direction_defined=sum(r['cosine'] is not None for r in values),
                overshoots_parallel_twice=sum(r['overshoots_parallel_twice'] for r in values))
            for key in ('measured_rms_mm','predicted_rms_mm','error_rms_mm',
                        'parallel_error_rms_mm','orthogonal_error_rms_mm','oracle_scaled_error_rms_mm'):
                item[key] = float(np.sqrt(np.mean([r[key]**2 for r in values])))
            for key in ('angle_deg','magnitude_ratio','signed_parallel_ratio','oracle_nonnegative_scale'):
                samples = [r[key] for r in values if r[key] is not None]
                item[key] = (None if not samples else dict(p50=float(np.percentile(samples,50)),
                    p95=float(np.percentile(samples,95)),max=float(max(samples))))
            summary[group] = item
        output[label] = dict(summary=summary,
            rows=[dict(start_ns=r['start_ns'],end_ns=r['end_ns'],target_index=r['target_index'],
                phase=r['phase'],direction=r['direction'],**g) for r,g in zip(chosen,geometry)])
    return output


def compare_readouts(args):
    """Reuse saved causal roots, without refitting or replaying observations."""
    torch.set_num_threads(2)
    original = json.loads(Path(args.compare_readouts).read_text())
    if not original['completed'] or original['confirmation_policy'] != 'shared_baseline':
        raise ValueError('completed shared-confirmation roots required')
    for path in (args.cache, args.matched, args.manifest):
        current = identity(path)
        if current not in original['identities']:
            raise ValueError('saved root input identity changed: '+str(path))
    datasets = pickle.loads(Path(args.cache).read_bytes())
    matched = json.loads(Path(args.matched).read_text())
    new = load_runtime_bundle(args.manifest, device='cpu', dtype='float32').runtime
    out = Path(args.output); out.mkdir(exist_ok=False)
    report = dict(completed=False,
        scope='Offline paired readout comparison on saved causal strain roots; no commands or runtime changes.',
        limitations=['Achieved future encoder inputs are conditional, not causal command predictions.',
            'All readouts assume engagement, including reversal windows; no new confirmation gate is fitted.',
            'Errors are rigid-invariant distal shape increments, NOT Cartesian tip displacement.',
            'Equilibrium anchoring is a counterfactual reset, not transient state ground truth.',
            'Future confirmation and resolved-response strata only label evaluation.',
            'CPU research execution is not a production timing qualification.'],
        identities=[identity(p) for p in (__file__, args.compare_readouts, args.cache, args.matched,
            args.manifest, inspect.getfile(static_map_shape_forecast))], sessions=[])
    for source in original['sessions']:
        data = next(d for d in datasets if d['name']==source['name'])
        if identity(data['params']['model_manifest']) != source['baseline_manifest']:
            raise ValueError('baseline manifest identity changed')
        base = load_runtime_bundle(data['params']['model_manifest'], device='cpu', dtype='float32',
            options={'marker_estimator':'ukf','adaptation_enabled':False}).runtime
        mechanics = FrozenV171Mechanics.from_artifacts(SimpleNamespace(distal_model=base.distal_model, port=base.port))
        mechanical_differences = {}
        for name in ('natural_strain','stiffness','damping','mode'):
            a,b = getattr(mechanics,name), getattr(new.mechanics,name)
            # Reconstructing the fixed operators from the checkpoint versus
            # loading their saved float32 copies can differ by one ULP.
            torch.testing.assert_close(a,b,rtol=1e-6,atol=0)
            mechanical_differences[name] = float((a-b).abs().max())
        if mechanics.solve_floor != new.mechanics.solve_floor:
            raise ValueError('mechanical solve floor differs')
        reference = next(s['rows'] for s in matched['sessions'] if s['name']==source['name'])
        rows, held_error, profile_error, equilibrium_error = [], [], [], []
        for row in source['rows']:
            r = {k:v for k,v in row.items() if k not in ('fits','registration_windows','predictions')}
            r['predictions'] = {k:row['predictions'][k] for k in ('hold','new_registered','new_equilibrium')}
            trace = row['input_trace']; stamps = np.asarray(trace['timestamps_ns'], np.int64)
            if stamps[0] != row['start_ns'] or stamps[-1] != row['end_ns']:
                raise ValueError('saved source support does not match request')
            counts = [interpolate(data['encoder'], int(t)) for t in stamps]
            if any(c is None for c in counts):
                raise ValueError('missing calibrated encoder support')
            calibration = new.encoder_calibration
            x = np.asarray(counts)[:,2]*calibration['mm_per_count']+calibration['offset_mm']
            np.testing.assert_allclose(x, trace['encoder_mm'], rtol=0, atol=1e-10)
            downstream = base.motor_transmission(base._encoder_tensor(np.asarray(counts))).detach().cpu().numpy()[:,2]
            times = (stamps-stamps[0])*1e-9
            for label, anchor in [('registered',row['coordinate']), ('equilibrium',row['equilibrium_coordinate'])]:
                prediction, forcing, _ = static_map_shape_forecast(new.mechanics,new.kinematics,
                    base.history_model,row['root_strain'],anchor,times,downstream)
                r['predictions']['static_'+label] = (prediction[-1]-prediction[0]).tolist()
                r.setdefault('static_forcing',{})[label] = forcing.tolist()
                repeated, _ = profile_shape_forecast(new.mechanics,new.kinematics,new.profile,
                    row['root_strain'],anchor,times,x)
                profile_error.append(float(np.max(np.abs(repeated[-1]-repeated[0]-
                    np.asarray(row['predictions']['new_'+label])))))
            passive, _, _ = static_map_shape_forecast(new.mechanics,new.kinematics,base.history_model,
                row['root_strain'],row['coordinate'],times,np.full_like(downstream,downstream[0]))
            held_error.append(float(np.max(np.abs(passive[-1]-passive[0]-np.asarray(row['predictions']['zero_input'])))))
            equilibrium_error.append(abs(float(base._stationary_equilibrium_shift(
                torch.as_tensor(row['root_strain']).to(base.port.mode)))-row['equilibrium_coordinate']))
            label = next(v for v in reference if v['start_ns']==row['start_ns'] and v['end_ns']==row['end_ns'])
            np.testing.assert_allclose(row['measured'][:6],label['measured_shape_delta_m'],rtol=0,atol=1e-12)
            rows.append(r)
        checks = dict(profile_reproduction_max_m=max(profile_error), held_readout_difference_max_m=max(held_error),
            equilibrium_anchor_difference_max=max(equilibrium_error),mechanics_match_rtol=1e-6,
            mechanical_buffer_max_absolute_differences=mechanical_differences,
            measured_reference_match=True, calibrated_input_match=True)
        if max(profile_error)>1e-10 or max(held_error)>1e-6 or max(equilibrium_error)>1e-4:
            raise ValueError('matched readout reproduction failed: '+str(checks))
        session = dict(name=source['name'],baseline_manifest=source['baseline_manifest'],checks=checks,
            runtime_identities=[identity(inspect.getfile(type(rt))) for rt in (base,new)],
            rows=rows,scores=scores(rows),response_diagnostics=response_diagnostics(rows))
        report['sessions'].append(session)
        print('RESULT',source['name'],json.dumps(session['scores']),flush=True)
    report['completed'] = True
    (out/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print('REPORT',out/'report.json',flush=True)


def run(args):
    if args.cartesian and not args.consistent_static:
        raise ValueError('Cartesian comparison requires consistent-static replay')
    if sum(bool(v) for v in (args.compare_readouts,args.register_static,args.consistent_static,args.compare_horizons))>1:
        raise ValueError('choose one readout diagnostic mode')
    if args.compare_horizons:
        return compare_horizons(args)
    if args.compare_readouts:
        return compare_readouts(args)
    torch.set_num_threads(2)
    datasets = pickle.loads(Path(args.cache).read_bytes())
    matched = json.loads(Path(args.matched).read_text())
    out = Path(args.output); out.mkdir(exist_ok=False)
    report = dict(completed=False, scope='Offline common-input policy replay and past-only profile ablations; no commands.',
        confirmation_policy=args.confirmation_policy,
        limitations=['Recorded baseline phases are live observations; replay phases are not exact live reproduction.',
            'Control ownership in policy replay is approximated by recorded task intervals; no arbiter is run.',
            'Common source grid is 20ms encoders/50ms markers with exact requested roots and endpoints.',
            'Future achieved encoders are conditional known inputs, not assumed useful/transmitted travel.',
            'Posterior strain/coordinate and ENGAGED labels are not independent physical ground truth.',
            'Equilibrium-coordinate reset is an ablation, not valid transient registration or a deployed estimator.',
            'Direction metrics use six pair-distance increments, not Cartesian tip displacement.',
            'Resolved response >=0.3mm uses all eight descriptors for evaluation only, not a production gate.',
            'Oracle scaling uses the future endpoint label and is only an optimistic diagnostic lower bound.',
            'Zero-input holds future tendon input fixed but retains causal strain/coordinate transient.',
            'Input component subtracts zero-input from full prediction; it is not a standalone deployed forecast.',
            'Completed-strain inverse describes posterior consistency, not independently measured input; moving input averages are not root forcing.',
            'Past coherent-root ablations omit intermediate corrections and can mismatch observed root geometry; no deployment reset is proposed.',
            'Past interval-bias subtraction is a frozen diagnostic ablation, not an instantaneous input observation or a deployment correction.',
            'Future-monotone strata use 0.01mm accumulated encoder resolution and future confirmation labels only for scoring.'],
        identities=[identity(p) for p in [__file__, args.cache, args.matched, args.manifest,
            inspect.getfile(BacklashStateEstimator), inspect.getfile(profile_shape_forecast),
            inspect.getfile(shape_signature), inspect.getfile(JointEngagementFilter),
            inspect.getfile(confirm_at_source)]], sessions=[])
    if args.cartesian:
        report['limitations'].extend([
            'Cartesian forecasts use each source-time filter own pose and strain; no shared or future endpoint pose.',
            'Experimental mean forecasts assume engaged tendon input; proximal retains its existing transmission state. These are not replayed UKF corrections or unexecuted candidate outcomes.',
            'Future raw encoders are achieved inputs only. Raw marker endpoint is label-only. Frozen past terminal reference is not the full moving-path objective.',
            'Passive response holds all future shaft inputs, not mechanics. Its subtraction is model action sensitivity, not measured causal plant response.'])
        report['identities'].append(identity(inspect.getfile(task_response)))
    saved = None
    saved_path = args.register_static or args.consistent_static
    if saved_path:
        saved = json.loads(Path(saved_path).read_text())
        if not saved['completed'] or saved['confirmation_policy']!='shared_baseline' or args.confirmation_policy!='shared_baseline':
            raise ValueError('static registration requires completed shared-confirmation roots')
        for path in (args.cache,args.matched,args.manifest):
            if identity(path) not in saved['identities']:
                raise ValueError('registration source identity changed: '+str(path))
        report['identities'].append(identity(saved_path))
        if args.register_static:
            report['scope'] = 'Offline completed-past forcing registration with fixed v171 static prior; no commands.'
            report['limitations'].append('Static registration fits one forcing with a 0.5mm descriptor likelihood, earlier posterior prior and +/-3-sigma bounds; a labelled 10x prior-std ablation tests prior restriction, not a tuned deployment setting. No gains or map slopes fitted.')
        else:
            report['scope'] = 'Offline fixed v171 static prior inside existing joint UKF and forecast, with original confirmation tape; no commands.'
            report['limitations'].extend(['The original replay alone supplies external confirmation, including rewind-suffix decisions; the ablation does not redetect engagement.',
                'Static prior is injected at construction before the first filter event, with identical initial state/covariance; no equilibrium reset or gain adaptation.',
                'Original profile domain remains initialization/readiness metadata; static-map extrapolation is reported, not clamped or deployed.',
                'Inference/correction and proximal state code and noise parameters are unchanged; inferred state may differ.'])
    for index in args.indices:
        data = datasets[index]; print('SESSION', data['name'], flush=True)
        requests = next(s['rows'] for s in matched['sessions'] if s['name'] == data['name'])
        requests = [r for r in requests if r['horizon_s'] == .8]
        by_root = {r['start_ns']: r for r in requests}
        saved_roots = (None if saved is None else {r['start_ns']:r for s in saved['sessions']
            if s['name']==data['name'] for r in s['rows']})
        exact = {t for r in requests for t in [r['start_ns'], r['end_ns']]}
        base = load_runtime_bundle(data['params']['model_manifest'], device='cpu', dtype='float32',
            options={'marker_estimator': 'ukf', 'adaptation_enabled': False}).runtime
        new = load_runtime_bundle(args.manifest, device='cpu', dtype='float32').runtime
        shared = args.confirmation_policy == 'shared_baseline'
        if shared:
            new.use_external_tendon_confirmation()
        transmission = base.interface_transmission_artifacts.transmission
        width = tuple(base.interface_transmission_artifacts.symmetric_reversal_width_rad.cpu().double().tolist())
        configuration = {f.name: data['params']['backlash_'+f.name] for f in fields(BacklashConfig)
                         if 'backlash_'+f.name in data['params']}
        configuration.update(width_rad=width, width_positive_rad=None, width_negative_rad=None,
                             distal_confirmation_enabled=True)
        config = BacklashConfig(**configuration)
        baseline_policy = BacklashStateEstimator(config)
        shared_shape_policy = BacklashStateEstimator(config)
        policy_history = deque(maxlen=512)
        events = {}; last = -10**30
        for stamp, counts in data['encoder']:
            if stamp-last >= 20_000_000:
                events[int(stamp)] = [counts, None]; last = stamp
        last = -10**30
        for stamp, points in data['markers']:
            if stamp-last >= 50_000_000 or stamp in exact:
                counts = interpolate(data['encoder'], stamp)
                if counts is not None:
                    events[int(stamp)] = [counts, points]; last = stamp
        corrections, forecasts, history = [], [], []
        static_filter = None
        tx = None; next_log = time.monotonic()+20
        mode = base.port.mode.detach().cpu().numpy()
        for stamp, (counts, points) in sorted(events.items()):
            if stamp > max(exact): break
            active = any(a <= stamp <= b for a, b in data['intervals'])
            raw = np.asarray(counts[:3])*ENCODER_RADIANS_PER_COUNT
            for policy in (baseline_policy, shared_shape_policy):
                policy.advance_motor(raw, external_motion=not active)
            policy_history.append([stamp, raw.copy(), shared_shape_policy.clone_state(), active])
            tensor = torch.as_tensor(raw).to(transmission.axis_scale)
            if base.state is None:
                if points is None: continue
                tx = transmission.initial_state(tensor)
                base.initialize(stamp, counts, markers=points,
                    interface_motor_angle_rad=transmission.equivalent_shaft_angle(tx))
                new.initialize(stamp, counts, markers=points,
                    tendon_confirmation=tendon_confirmation(new, shared_shape_policy.snapshot()) if shared else None)
                if args.consistent_static:
                    static_filter = static_prior_filter_from_initial_event(new,base.history_model,
                        base.motor_transmission,stamp,counts,points,
                        tendon_confirmation(new,shared_shape_policy.snapshot()))
                continue
            tx = transmission.advance(tx, tensor)
            base.advance_encoder(stamp, counts, interface_motor_angle_rad=transmission.equivalent_shaft_angle(tx))
            new.advance_encoder(stamp, counts, return_snapshot=False,
                tendon_confirmation=tendon_confirmation(new, shared_shape_policy.snapshot()) if shared else None)
            if static_filter is not None:
                static_filter.advance(stamp*1e-9,new._encoder(counts),raw,
                    confirmation=tendon_confirmation(new,shared_shape_policy.snapshot()))
            if points is None: continue
            accepted = base.observe_markers(stamp, points, {})
            decisions_for_static = []
            def confirm(estimator):
                _, updated, decisions = confirm_at_source(new, shared_shape_policy,
                    policy_history, stamp, estimator, points, control_owned=active)
                policy_history.clear(); policy_history.extend(updated)
                decisions_for_static.extend(decisions)
                return decisions
            before_correction = new.estimator.mean.copy()
            response = new.observe_markers(stamp, points, {},
                confirmation_observer=confirm if shared else None)
            if static_filter is not None:
                static_before = static_filter.mean.copy()
                static_response = static_filter.correct(stamp*1e-9,points,
                    point_std_m=np.full(4,new.config.point_std_m),
                    confirmation_observer=lambda _:decisions_for_static)
                if (static_filter.phase,static_filter.direction,static_filter.confirmation_count) != (
                        new.estimator.phase,new.estimator.direction,new.estimator.confirmation_count):
                    raise ValueError('fixed confirmation tape mismatch')
            if not accepted.accepted or response is None: continue
            q_base = base.state.strain.detach().cpu().numpy().copy()
            q_new = new.estimator.mean[:24].copy()
            if active:
                observations = [(baseline_policy, q_base, base.state.interface_pose.cpu().numpy())]
                if not shared:
                    observations.append((shared_shape_policy, q_new, new.state.interface_pose.cpu().numpy()))
                for policy, q, pose in observations:
                    model = base.state.adaptive_jacobian
                    policy.observe_response(raw, pose, model.jacobian, model.state_scale,
                        distal_strain=q, distal_bending_mode=mode, timestamp_ns=stamp,
                        observed_interface_position_m=points[0])
            else:
                for policy in (baseline_policy, shared_shape_policy): policy.reset_observation_history()
            row = dict(timestamp_ns=stamp, active=active, baseline_policy=baseline_policy.phase[2],
                baseline_policy_new_shape=shared_shape_policy.phase[2], new_policy=new.estimator.phase,
                baseline_count=int(baseline_policy.confirmation_count[2]),
                new_shape_count=int(shared_shape_policy.confirmation_count[2]),
                new_count=int(new.estimator.confirmation_count),
                prior_coordinate=float(before_correction[30]), posterior_coordinate=float(new.estimator.mean[30]),
                coordinate_correction=float(new.estimator.mean[30]-before_correction[30]),
                prior_equilibrium_coordinate=float(new.mechanics.stationary_lambda(
                    torch.as_tensor(before_correction[:24]).to(new.mechanics.mode))),
                posterior_equilibrium_coordinate=float(new.mechanics.stationary_lambda(
                    torch.as_tensor(q_new).to(new.mechanics.mode))),
                baseline_coordinate=float(base.state.lambda_value),
                innovation_rms_mm=float(response['innovation_rms_m']*1000),
                baseline_bending_increment=float(baseline_policy.distal_bending_increment),
                new_shape_bending_increment=float(shared_shape_policy.distal_bending_increment),
                raw_shape_response_mm=(None if shared else float(response['observed_shape_response_m']*1000)),
                raw_shape_threshold_mm=(None if shared else float(response['shape_response_threshold_m']*1000)))
            corrections.append(row)
            if static_filter is not None:
                predicted = static_filter.points()[0]
                row['static_inference'] = dict(phase=static_filter.phase,direction=static_filter.direction,
                    prior_coordinate=float(static_before[30]),posterior_coordinate=float(static_filter.mean[30]),
                    coordinate_correction=float(static_filter.mean[30]-static_before[30]),
                    coordinate_std=float(np.sqrt(static_filter.covariance[30,30])),
                    equilibrium_coordinate=float(new.mechanics.stationary_lambda(torch.as_tensor(static_filter.mean[:24]).to(new.mechanics.mode))),
                    innovation_rms_mm=float(static_response['innovation_rms_m']*1000),
                    posterior_marker_rms_mm=float(np.sqrt(np.mean((predicted-points)**2))*1000),
                    posterior_distance_rms_mm=float(np.sqrt(np.mean((shape_signature(predicted)[:6]-shape_signature(points)[:6])**2))*1000),
                    original_posterior_distance_rms_mm=float(np.sqrt(np.mean((shape_signature(new.estimator.points()[0])[:6]-shape_signature(points)[:6])**2))*1000),
                    input_supported=bool(static_filter.profile.input_knots[0]<=static_filter.mean[30]<=static_filter.profile.input_knots[-1]),
                    prediction_ready=static_filter.prediction_ready)
            state = dict(stamp=stamp, points=points, q=q_new, q_base=q_base,
                coordinate=float(new.estimator.mean[30]), std=float(np.sqrt(new.estimator.covariance[30, 30])),
                encoder_mm=new._encoder(counts), phase=new.estimator.phase, direction=new.estimator.direction)
            history.append(state)
            if stamp in by_root:
                request = by_root[stamp]; end = request['end_ns']
                enc = [(int(t), new._encoder(c)) for t, c in data['encoder'] if stamp < t < end]
                end_counts = interpolate(data['encoder'], end)
                measured_points = dict(data['markers']).get(end)
                if end_counts is None or measured_points is None: continue
                times = np.array([stamp]+[t for t, _ in enc]+[end], np.int64)
                x = np.array([state['encoder_mm']]+[v for _, v in enc]+[new._encoder(end_counts)])
                future = (times-times[0])*1e-9
                record = dict(start_ns=stamp, end_ns=end, target_index=next(i+1 for i,(a,b) in enumerate(data['intervals']) if a<=stamp<=b),
                    phase=state['phase'], direction=state['direction'], coordinate=state['coordinate'],
                    coordinate_std=state['std'], equilibrium_coordinate=float(new.mechanics.stationary_lambda(torch.as_tensor(q_new).to(new.mechanics.mode))),
                    baseline_equilibrium_coordinate=float(new.mechanics.stationary_lambda(torch.as_tensor(q_base).to(new.mechanics.mode))),
                    measured=(shape_signature(measured_points)-shape_signature(points)).tolist(), predictions={}, fits={},
                    registration_windows={}, baseline_coordinate=float(base.state.lambda_value),
                    root_strain=q_new.tolist(), baseline_root_strain=q_base.tolist())
                variants = {'new_registered': (q_new, state['coordinate'], 1.),
                    'new_equilibrium': (q_new, record['equilibrium_coordinate'], 1.),
                    'baseline_shape_new_coordinate': (q_base, state['coordinate'], 1.),
                    'baseline_shape_equilibrium': (q_base, record['baseline_equilibrium_coordinate'], 1.)}
                past = [s for s in history if stamp-800_000_000 <= s['stamp'] <= stamp]
                eligible = (len(past) >= 8 and stamp-past[0]['stamp'] >= 700_000_000
                    and max(np.diff([s['stamp'] for s in past])) <= 100_000_000
                    and state['direction'] != 0 and all(s['phase']=='ENGAGED' and s['direction']==state['direction'] for s in past)
                    and abs(past[-1]['encoder_mm']-past[0]['encoder_mm']) >= .05)
                record['eligible_past_fit'] = bool(eligible)
                if args.consistent_static:
                    source = saved_roots[stamp]
                    for key in ('root_strain','coordinate','equilibrium_coordinate','measured'):
                        np.testing.assert_allclose(record[key],source[key],rtol=0,atol=1e-10)
                    q_static = static_filter.mean[:24].copy()
                    u_static = float(static_filter.mean[30])
                    record['static_inference_root'] = dict(strain=q_static.tolist(),coordinate=u_static,
                        coordinate_std=float(np.sqrt(static_filter.covariance[30,30])),
                        equilibrium_coordinate=float(new.mechanics.stationary_lambda(torch.as_tensor(q_static).to(new.mechanics.mode))),
                        interface_pose=static_filter.poses(static_filter.mean[None])[0].tolist(),
                        mean=static_filter.mean.tolist(),covariance=static_filter.covariance.tolist(),
                        raw_shaft_rad=raw.tolist(),proximal_jacobian=static_filter.proximal.jacobian.jacobian.tolist())
                    record['original_interface_pose'] = new.estimator.poses(new.estimator.mean[None])[0].tolist()
                    record['baseline_interface_pose'] = base.state.interface_pose.detach().cpu().numpy().tolist()
                    if args.cartesian:
                        # Evaluate real prefix horizons on the exact encoder grid.
                        cartesian = []
                        for horizon in (.2,.8):
                            endpoints = [int(t) for t,_ in data['markers'] if t>=stamp+int(horizon*1e9)]
                            if not endpoints: continue
                            endpoint = endpoints[0]
                            if endpoint>end: continue
                            grid = np.r_[times[times<endpoint],endpoint].astype(np.int64)
                            future_counts = [interpolate(data['encoder'],int(t)) for t in grid]
                            if any(c is None for c in future_counts): continue
                            raw_inputs = np.asarray(future_counts)[:,:3]*ENCODER_RADIANS_PER_COUNT
                            tendon_inputs = np.asarray([new._encoder(c) for c in future_counts])
                            seconds = (grid-stamp)*1e-9
                            truth = dict(data['markers'])[endpoint]
                            predictions = {}; passive = {}; root_offsets = {}
                            for label,owner,prior,anchor in (
                                ('original_profile',new.estimator,new.profile,state['coordinate']),
                                ('consistent_static',static_filter,static_filter.profile,u_static),
                                ('static_equilibrium',static_filter,static_filter.profile,record['static_inference_root']['equilibrium_coordinate'])):
                                pose = owner.poses(owner.mean[None])[0]
                                predicted,_ = profile_cartesian_forecast(new.mechanics,new.kinematics,prior,
                                    owner.mean[:24],anchor,pose,owner.proximal,seconds,tendon_inputs,raw_inputs)
                                quiet,_ = profile_cartesian_forecast(new.mechanics,new.kinematics,prior,
                                    owner.mean[:24],anchor,pose,owner.proximal,seconds,np.full_like(tendon_inputs,tendon_inputs[0]),
                                    np.broadcast_to(raw_inputs[0],raw_inputs.shape))
                                predictions[label] = (1000*(predicted[-1,-1]-predicted[0,-1])).tolist()
                                passive[label] = (1000*(quiet[-1,-1]-quiet[0,-1])).tolist()
                                root_offsets[label] = (1000*(predicted[0,-1]-points[-1])).tolist()
                            # Canonical baseline routes, same achieved raw inputs;
                            # the history branch retains actual transmission state.
                            tx_future = tx.clone(); previous_effective = base.state.motor_angle_rad
                            effective_rates = []; intervals = np.diff(seconds)
                            for angle,interval in zip(raw_inputs[1:],intervals):
                                tx_future = transmission.advance(tx_future,torch.as_tensor(angle).to(transmission.axis_scale))
                                effective = transmission.equivalent_shaft_angle(tx_future)
                                effective_rates.append((effective-previous_effective)/interval)
                                previous_effective = effective
                            raw_rates = torch.as_tensor(np.diff(raw_inputs,axis=0)/intervals[:,None]).to(base.state.strain)
                            root_tip = base._marker_points(base.state.interface_pose,base.state.strain)[-1].detach().cpu().numpy()
                            for label,engaged in (('v171_history',False),('v171_engaged',True)):
                                full = base.predict_sequence(base.clone_state(),raw_rates if engaged else torch.stack(effective_rates),intervals,
                                    raw_motor_velocity_sequence=raw_rates,always_engaged=engaged)
                                quiet = base.predict_sequence(base.clone_state(),torch.zeros_like(raw_rates),intervals,
                                    raw_motor_velocity_sequence=torch.zeros_like(raw_rates),always_engaged=engaged)
                                predictions[label] = (1000*(full.tip_base_m[-1].detach().cpu().numpy()-root_tip)).tolist()
                                passive[label] = (1000*(quiet.tip_base_m[-1].detach().cpu().numpy()-root_tip)).tolist()
                                root_offsets[label] = (1000*(root_tip-points[-1])).tolist()
                            refs = [r for r in data['response'] if r.forecast_start_timestamp_ns<=stamp and
                                any(a<=r.forecast_start_timestamp_ns<=stamp<=b for a,b in data['intervals'])]
                            ref = max(refs,key=lambda r:r.forecast_start_timestamp_ns) if refs else None
                            error = None if ref is None or stamp-ref.forecast_start_timestamp_ns>300_000_000 else 1000*(np.asarray(ref.target_tip_m)-points[-1])
                            measured = 1000*(truth[-1]-points[-1])
                            weights = [1.,0.,1.] if isinstance(data['task'],dict) and data['task'].get('xz_only') else [1.,1.,1.]
                            cartesian.append(dict(horizon_s=horizon,end_ns=endpoint,duration_s=float(seconds[-1]),
                                weights=weights,measured_delta_mm=measured.tolist(),predictions=predictions,passive=passive,
                                root_offsets_mm=root_offsets,input_component_mm={k:(np.asarray(v)-passive[k]).tolist() for k,v in predictions.items()},
                                reference_age_ms=None if error is None else (stamp-ref.forecast_start_timestamp_ns)*1e-6,
                                target_error_mm=None if error is None else error.tolist(),
                                task_metrics={} if error is None else {k:task_response(error,v,measured,weights) for k,v in predictions.items()}))
                        record['cartesian'] = cartesian
                    # Same input law in UKF and open-loop branch; profile branch
                    # on this new root isolates inference from readout changes.
                    for label,q,u,law,inputs in [
                        ('static_readout_original_root',q_new,state['coordinate'],static_filter.profile,x),
                        ('consistent_static',q_static,u_static,static_filter.profile,x),
                        ('profile_readout_static_root',q_static,u_static,new.profile,x),
                        ('consistent_static_equilibrium',q_static,record['static_inference_root']['equilibrium_coordinate'],static_filter.profile,x),
                        ('consistent_static_zero_input',q_static,u_static,static_filter.profile,np.full_like(x,x[0]))]:
                        predicted,_ = profile_shape_forecast(new.mechanics,new.kinematics,law,q,u,future,inputs)
                        record['predictions'][label] = (predicted[-1]-predicted[0]).tolist()
                if args.register_static:
                    source = saved_roots[stamp]
                    for key in ('root_strain','coordinate','equilibrium_coordinate','measured'):
                        np.testing.assert_allclose(record[key],source[key],rtol=0,atol=1e-10)
                    future_counts = [interpolate(data['encoder'],int(t)) for t in times]
                    if any(c is None for c in future_counts):
                        raise ValueError('missing future conditional input support')
                    downstream = base.motor_transmission(base._encoder_tensor(np.asarray(future_counts))).detach().cpu().numpy()[:,2]
                    static_variants = {'static_registered':(q_new,state['coordinate']),
                        'static_equilibrium':(q_new,record['equilibrium_coordinate'])}
                    if eligible:
                        past_times = np.asarray([s['stamp'] for s in past],np.int64)
                        past_counts = [interpolate(data['encoder'],int(t)) for t in past_times]
                        if any(c is None for c in past_counts):
                            raise ValueError('missing completed input support')
                        past_downstream = base.motor_transmission(base._encoder_tensor(np.asarray(past_counts))).detach().cpu().numpy()[:,2]
                        prior,_,prior_q = static_map_shape_forecast(new.mechanics,new.kinematics,base.history_model,
                            past[0]['q'],past[0]['coordinate'],(past_times-past_times[0])*1e-9,past_downstream)
                        prior_error = float(np.sqrt(np.mean(
                            (prior[1:,:6]-prior[0,:6]-np.asarray([shape_signature(s['points'])[:6] for s in past[1:]])+
                             shape_signature(past[0]['points'])[:6])**2))*1000)
                        record['static_registration_diagnostic'] = dict(
                            observed_transition=completed_strain_input(new.mechanics,past[0]['q'],q_new,past_times,forecast_root_ns=stamp),
                            static_prior_transition=completed_strain_input(new.mechanics,past[0]['q'],prior_q[-1],past_times,forecast_root_ns=stamp),
                            camera_coordinate_correction_sum=float(sum(c['coordinate_correction'] for c in corrections
                                if past_times[0] < c['timestamp_ns'] <= stamp)))
                        for label,factor in [('past_static_coordinate',1.),('past_static_weak_prior',10.)]:
                            fit = fit_completed_static_prefix(new.mechanics,new.kinematics,base.history_model,
                                past[0]['q'],past[0]['coordinate'],max(1.,past[0]['std'])*factor,past_times,past_downstream,
                                [s['points'] for s in past],forecast_root_ns=stamp)
                            fit['root_signature_distance_rms_mm'] = float(np.sqrt(np.mean(
                                (np.asarray(fit['root_shape_signature'])[:6]-shape_signature(points)[:6])**2))*1000)
                            fit['unfitted_past_distance_rms_mm'] = prior_error
                            record['fits'][label] = fit
                            if fit['success']:
                                static_variants[label] = (q_new,fit['root_coordinate'])
                                static_variants[label+'_coherent_root'] = (fit['root_strain'],fit['root_coordinate'])
                    for label,(q,u) in static_variants.items():
                        predicted,_,_ = static_map_shape_forecast(new.mechanics,new.kinematics,base.history_model,
                            q,u,future,downstream)
                        record['predictions'][label] = (predicted[-1]-predicted[0]).tolist()
                    # Fixed readout and held tendon distinguish passive forcing
                    # relaxation from encoder-driven motion, not a deployed sum.
                    for label,(q,u) in static_variants.items():
                        passive,_,_ = static_map_shape_forecast(new.mechanics,new.kinematics,base.history_model,
                            q,u,future,np.full_like(downstream,downstream[0]))
                        delta = passive[-1]-passive[0]
                        record['predictions'][label+'_zero_input'] = delta.tolist()
                        record['predictions'][label+'_input_component'] = (np.asarray(record['predictions'][label])-delta).tolist()
                for support_s in (.2, .4, .8):
                    if args.register_static or args.consistent_static:
                        break
                    support = [s for s in past if stamp-int(support_s*1e9) <= s['stamp'] <= stamp]
                    if len(support) < 4 or stamp-support[0]['stamp'] < support_s*.8*1e9:
                        continue
                    source_times = np.asarray([s['stamp'] for s in support], np.int64)
                    if max(np.diff(source_times)) > 100_000_000:
                        continue
                    t = (source_times-source_times[0])*1e-9
                    encoders = np.asarray([s['encoder_mm'] for s in support])
                    _, _, registered_q = profile_shape_forecast(new.mechanics, new.kinematics,
                        new.profile, support[0]['q'], support[0]['coordinate'], t, encoders, return_strain=True)
                    record['registration_windows'][str(support_s)] = dict(
                        observed_new_strain=completed_strain_input(new.mechanics, support[0]['q'], q_new,
                            source_times, forecast_root_ns=stamp),
                        observed_baseline_strain=completed_strain_input(new.mechanics, support[0]['q_base'], q_base,
                            source_times, forecast_root_ns=stamp),
                        registered_profile_equivalent=completed_strain_input(new.mechanics, support[0]['q'], registered_q[-1],
                            source_times, forecast_root_ns=stamp),
                        encoder_net_mm=float(encoders[-1]-encoders[0]),
                        encoder_range_mm=float(np.ptp(encoders)),
                        past_confirmed_monotone=bool(all(s['phase']=='ENGAGED' and s['direction']==state['direction'] for s in support)),
                        initial_coordinate=float(support[0]['coordinate']), root_coordinate=state['coordinate'])
                    window = record['registration_windows'][str(support_s)]
                    bias = (window['registered_profile_equivalent']['equivalent_input']-
                            window['observed_new_strain']['equivalent_input'])
                    candidate = state['coordinate']-bias
                    _, memory = encoder_memory(encoders, .01)
                    nonzero = memory[memory != 0]
                    window['interval_bias'] = bias
                    window['bias_subtracted_root_coordinate'] = candidate
                    window['bias_candidate_supported'] = bool(window['past_confirmed_monotone'] and
                        abs(window['encoder_net_mm']) >= .05 and len(nonzero) and
                        np.all(nonzero == state['direction']) and
                        new.profile.input_knots[0] <= candidate <= new.profile.input_knots[-1])
                    if window['bias_candidate_supported']:
                        variants['past_interval_bias_'+str(support_s)] = (q_new, candidate, 1.)
                if eligible and not (args.register_static or args.consistent_static):
                    for label, flags in [('past_coordinate', (True, False)), ('past_gain', (False, True)), ('past_joint', (True, True))]:
                        fit = fit_completed_profile_prefix(new.mechanics, new.kinematics, new.profile,
                            past[0]['q'], past[0]['coordinate'], past[0]['std'],
                            [s['stamp'] for s in past], [s['encoder_mm'] for s in past],
                            [s['points'] for s in past], forecast_root_ns=stamp,
                            fit_coordinate=flags[0], fit_gain=flags[1])
                        record['fits'][label] = fit
                        if fit['success']:
                            variants[label] = (q_new, fit['root_coordinate'], fit['gain_scale'])
                            variants[label+'_coherent_root'] = (fit['root_strain'], fit['root_coordinate'], fit['gain_scale'])
                            fit['root_signature_distance_rms_mm'] = float(np.sqrt(np.mean(
                                (np.asarray(fit['root_shape_signature'])[:6]-shape_signature(points)[:6])**2))*1000)
                            fit['root_signature_bow_rms_mm'] = float(np.sqrt(np.mean(
                                (np.asarray(fit['root_shape_signature'])[6:]-shape_signature(points)[6:])**2))*1000)
                for label, (q, u, gain) in variants.items():
                    predicted, _ = profile_shape_forecast(new.mechanics, new.kinematics, new.profile,
                        q, u, future, x, gain_scale=gain)
                    record['predictions'][label] = (predicted[-1]-predicted[0]).tolist()
                if args.consistent_static:
                    for label in variants:
                        np.testing.assert_allclose(record['predictions'][label],source['predictions'][label],rtol=0,atol=1e-10)
                passive, _ = profile_shape_forecast(new.mechanics, new.kinematics, new.profile,
                    q_new, state['coordinate'], future, np.full_like(x,x[0]))
                record['predictions']['zero_input'] = (passive[-1]-passive[0]).tolist()
                record['predictions']['input_component'] = (np.asarray(record['predictions']['new_registered'])-
                    np.asarray(record['predictions']['zero_input'])).tolist()
                record['input_trace'] = dict(timestamps_ns=times.tolist(),encoder_mm=x.tolist())
                record['predictions']['hold'] = [0.]*8
                forecasts.append(record)
            if time.monotonic() > next_log:
                print('PROGRESS', data['name'], 'forecasts', len(forecasts), flush=True); next_log=time.monotonic()+20
        for record in forecasts:
            # Endpoint labels and future phases only stratify evaluation; they
            # never enter forecast state, parameter fitting or deployment.
            x = np.asarray(record['input_trace']['encoder_mm'])
            _, directions = encoder_memory(x, .01)
            resolved = directions[directions != 0]
            record['future_encoder_reversals'] = int(np.count_nonzero(np.diff(resolved)))
            record['future_confirmation_phases'] = [r['new_policy'] for r in corrections
                if record['start_ns'] <= r['timestamp_ns'] <= record['end_ns']]
            record['future_monotone_confirmed'] = bool(record['phase']=='ENGAGED' and len(resolved) and
                np.all(resolved == record['direction']) and len(record['future_confirmation_phases']) >= 8 and
                all(p=='ENGAGED' for p in record['future_confirmation_phases']) and abs(x[-1]-x[0]) >= .05)
        task = [r for r in corrections if r['active']]
        actual = [m for m in data['trace'] if any(a<=m.observation_timestamp_ns<=b for a,b in data['intervals'])]
        fitted_rows = [r for r in forecasts if r['eligible_past_fit']]
        session = dict(name=data['name'], configuration=asdict(config),
            runtime_identities=[identity(inspect.getfile(type(runtime))) for runtime in [base,new]],
            baseline_manifest=identity(data['params']['model_manifest']),
            actual_baseline_compensation_enabled=data['params']['backlash_compensation_enabled'],
            actual_baseline_phases=dict(Counter(m.backlash_phase[2] for m in actual)),
            same_input_replay_phases={key:dict(Counter(r[key] for r in task)) for key in ['baseline_policy','baseline_policy_new_shape','new_policy']},
            policy_disagreement_with_shared_shape=sum(r['baseline_policy_new_shape']!=r['new_policy'] for r in task),
            corrections=corrections, rows=forecasts, scores=scores(forecasts),
            response_diagnostics=response_diagnostics(forecasts),
            eligible_past_fit_count=len(fitted_rows), eligible_scores=scores(fitted_rows),
            coordinate_diagnostics={k:dict(p50=float(np.median([r[k] for r in forecasts])),
                p95=float(np.percentile([r[k] for r in forecasts],95))) for k in ['coordinate','coordinate_std','equilibrium_coordinate','baseline_equilibrium_coordinate']})
        report['sessions'].append(session)
        if args.consistent_static:
            session['static_inference_summary'] = dict(initial_state_covariance_match=True,
                original_roots_forecasts_reproduced=len(forecasts),fixed_confirmation_disagreements=0,
                active_observations=len(task),unsupported_inputs=sum(not r['static_inference']['input_supported'] for r in task),
                prediction_not_ready=sum(not r['static_inference']['prediction_ready'] for r in task))
            for key in ('coordinate_correction','coordinate_std','innovation_rms_mm','posterior_marker_rms_mm',
                        'posterior_distance_rms_mm','original_posterior_distance_rms_mm'):
                values=np.asarray([r['static_inference'][key] for r in task])
                session['static_inference_summary'][key] = dict(p50=float(np.median(values)),
                    p95=float(np.percentile(values,95)),rms=float(np.sqrt(np.mean(values**2))))
        (out/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
        print('RESULT', data['name'], session['same_input_replay_phases'], 'past_fits', len(fitted_rows), flush=True)
    report['completed'] = True
    (out/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print('REPORT', out/'report.json', flush=True)


def compare_horizons(args):
    """Saved-root horizon comparison; no inference or confirmation is rerun."""
    torch.set_num_threads(2)
    saved = json.loads(Path(args.compare_horizons).read_text())
    if (not saved['completed'] or saved['confirmation_policy']!='shared_baseline'
            or any('static_inference_summary' not in s for s in saved['sessions'])):
        raise ValueError('completed consistent-static inference report required')
    for path in (args.cache,args.matched,args.manifest):
        if identity(path) not in saved['identities']:
            raise ValueError('saved horizon input identity changed: '+str(path))
    datasets = pickle.loads(Path(args.cache).read_bytes())
    matched = json.loads(Path(args.matched).read_text())
    new = load_runtime_bundle(args.manifest,device='cpu',dtype='float32').runtime
    out = Path(args.output);out.mkdir(exist_ok=False)
    report = dict(completed=False,scope='Offline saved-root 0.2/0.4/0.8s distal response attribution; no commands.',
        horizons_s=[.2,.4,.8],
        limitations=['Roots and confirmation tape come unchanged from the completed consistent-static replay.',
            'Horizon endpoint is the first real marker at or after the requested horizon; actual durations are reported.',
            'All three horizons require encoder/marker support without >0.1s gaps; exclusions and common roots are reported.',
            'Future achieved encoders are conditional inputs; forecasts assume engagement, including reversal windows.',
            'Passive prediction holds input, not dynamics. Input-component subtraction is not a standalone physical forecast.',
            'Scores are rigid-invariant distal shape increments, not Cartesian tip or interface prediction error.',
            'Archived v171 history/planner references are available only at exact saved matching endpoints; their earlier inference grid is not rerun.',
            'Baseline static reference uses saved baseline strain and equilibrium anchor, not the complete historical v171 history state.',
            'Adaptive references reuse existing zero-prior observed-shape RLS; they are not the deployed interface Jacobian or a new engagement detector.',
            'RLS labels update only after window end, using active task windows on a >=0.2s root grid; future resolved/confirmation labels do not select training.',
            'Overlapping windows are not independent trials. CPU execution is not live timing qualification.'],
        identities=[identity(p) for p in (__file__,args.compare_horizons,args.cache,args.matched,args.manifest,
            inspect.getfile(distal_forecast_components),inspect.getfile(score_windows))],sessions=[])
    for source in saved['sessions']:
        print('SESSION',source['name'],flush=True)
        data = next(d for d in datasets if d['name']==source['name'])
        if identity(data['params']['model_manifest'])!=source['baseline_manifest']:
            raise ValueError('baseline manifest identity changed')
        base = load_runtime_bundle(data['params']['model_manifest'],device='cpu',dtype='float32',
            options={'marker_estimator':'ukf','adaptation_enabled':False}).runtime
        law = StaticMapInputPrior(base.history_model,base.motor_transmission,
            new.tendon_mm_per_shaft_rad,new.profile.input_knots)
        marker_ns = np.asarray([t for t,_ in data['markers']],np.int64)
        points = np.asarray([p for _,p in data['markers']])
        encoder_ns = np.asarray([t for t,_ in data['encoder']],np.int64)
        shaft = np.asarray([v[:3] for _,v in data['encoder']])*ENCODER_RADIANS_PER_COUNT
        archived = {(r['start_ns'],r['end_ns']):r for s in matched['sessions']
            if s['name']==source['name'] for r in s['rows']}
        rows,excluded,windows,checks = [],[],[],[]
        for root in source['rows']:
            probes = []
            for h in report['horizons_s']:
                j = int(np.searchsorted(marker_ns,root['start_ns']+int(h*1e9)))
                end = None if j==len(marker_ns) else int(marker_ns[j])
                window = (None if end is None or end-root['start_ns']>int((h+.1)*1e9) else
                    make_window(marker_ns,points,encoder_ns,shaft,root['start_ns'],end,h,test=True))
                if window is None:
                    excluded.append(dict(start_ns=root['start_ns'],horizon_s=h,reason='unsupported endpoint or >0.1s sensor gap'))
                    break
                probes.append((h,end,window))
            if len(probes)!=3:continue
            for h,end,window in probes:
                trace = root['input_trace'];stamps = np.asarray(trace['timestamps_ns'],np.int64)
                if h==.8 and end!=root['end_ns']:
                    raise ValueError('long horizon endpoint changed')
                times = np.r_[stamps[stamps<end],end]
                counts = interpolate(data['encoder'],end)
                x = np.r_[np.asarray(trace['encoder_mm'])[stamps<end],new._encoder(counts)]
                future = (times-times[0])*1e-9
                state = root['static_inference_root']
                components = distal_forecast_components(new.mechanics,new.kinematics,law,
                    state['strain'],state['coordinate'],future,x)
                profile,_ = profile_shape_forecast(new.mechanics,new.kinematics,new.profile,
                    root['root_strain'],root['coordinate'],future,x)
                baseline,_ = profile_shape_forecast(new.mechanics,new.kinematics,law,
                    root['baseline_root_strain'],root['baseline_equilibrium_coordinate'],future,x)
                predictions = dict(hold=[0.]*8,original_profile=(profile[-1]-profile[0]).tolist(),
                    consistent_static=components['full'].tolist(),passive=components['passive'].tolist(),
                    baseline_static_equilibrium=(baseline[-1]-baseline[0]).tolist())
                old = archived.get((root['start_ns'],end))
                if old is not None:
                    np.testing.assert_allclose(window.delta_m[:6],old['measured_shape_delta_m'],rtol=0,atol=1e-12)
                    predictions['archived_v171_history'] = old['predicted_shape_delta_m']['v171']
                    predictions['archived_v171_planner'] = old['predicted_shape_delta_m']['v171_planner']
                if h==.8:
                    for label,old_label in [('consistent_static','consistent_static'),('passive','consistent_static_zero_input'),
                                            ('original_profile','new_registered')]:
                        checks.append(float(np.max(np.abs(np.asarray(predictions[label])-root['predictions'][old_label]))))
                        np.testing.assert_allclose(predictions[label],root['predictions'][old_label],rtol=0,atol=1e-10)
                    np.testing.assert_allclose(window.delta_m,root['measured'],rtol=0,atol=1e-12)
                _,directions = encoder_memory(x,.01);resolved = directions[directions!=0]
                phases = [r['new_policy'] for r in source['corrections'] if root['start_ns']<=r['timestamp_ns']<=end]
                minimum_observations = max(3,int(round(h/.8*8)))
                confirmed = bool(root['phase']=='ENGAGED' and len(resolved) and
                    np.all(resolved==root['direction']) and len(phases)>=minimum_observations and
                    all(p=='ENGAGED' for p in phases) and abs(x[-1]-x[0])>=.05)
                rows.append(dict(start_ns=root['start_ns'],end_ns=end,horizon_s=h,
                    duration_s=float(future[-1]),target_index=root['target_index'],phase=root['phase'],direction=root['direction'],
                    eligible_past_fit=root['eligible_past_fit'],future_encoder_reversals=int(np.count_nonzero(np.diff(resolved))),
                    future_monotone_confirmed=confirmed,minimum_confirmation_observations=minimum_observations,
                    measured=window.delta_m.tolist(),predictions=predictions,
                    input_component_m=components['input_component'].tolist(),
                    attribution=input_response_attribution(components['full'],components['passive'],window.delta_m)))
                windows.append(window)
        if not rows:raise ValueError('no common supported horizon roots')
        # Existing prequential RLS: every completed active window can train;
        # neither response resolution nor retrospective engagement selects it.
        last = None
        for start in marker_ns:
            start = int(start)
            if start>max(r['end_ns'] for r in rows):break
            if last is not None and start-last<200_000_000:continue
            last = start
            for h in report['horizons_s']:
                j = int(np.searchsorted(marker_ns,start+int(h*1e9)))
                if j==len(marker_ns):continue
                end = int(marker_ns[j])
                if end-start>int((h+.1)*1e9) or not any(a<=start<end<=b for a,b in data['intervals']):continue
                value = make_window(marker_ns,points,encoder_ns,shaft,start,end,h,train=True)
                if value is not None:windows.append(value)
        adaptive = {(r['start_ns'],r['end_ns'],r['horizon_s']):r for r in score_windows(windows)}
        for row in rows:
            reference = adaptive[(row['start_ns'],row['end_ns'],row['horizon_s'])]
            latest = reference['latest_training_end_ns']
            if latest is not None and latest>row['start_ns']:raise ValueError('pending label used for RLS')
            row['adaptive_training'] = dict(count=reference['matured_training_windows'],latest_end_ns=latest,
                features=reference['features'],feature_abs_max=reference['matured_training_feature_abs_max'])
            for label in ('causal_response','causal_tendon_response','causal_context_response','past_velocity'):
                row['predictions'][label] = reference['predictions_m'][label]
        summaries = {}
        for h in report['horizons_s']:
            chosen = [r for r in rows if r['horizon_s']==h]
            diagnostics = response_diagnostics(chosen)
            groups = {'all':chosen,'resolved':[r for r in chosen if np.sqrt(np.mean(np.square(r['measured'])))*1000>=.3],
                'resolved_confirmed':[r for r in chosen if r['future_monotone_confirmed'] and
                    np.sqrt(np.mean(np.square(r['measured'])))*1000>=.3],
                'reversing':[r for r in chosen if r['future_encoder_reversals']>0]}
            attribution = {}
            for label,values in groups.items():
                if not values:continue
                attribution[label] = dict(n=len(values),input_helps=sum(r['attribution']['input_excess_mse_mm2']<0 for r in values),
                    **{key:float(np.mean([r['attribution'][key] for r in values])) for key in chosen[0]['attribution']})
            summaries[str(h)] = dict(scores=scores(chosen),response_diagnostics=diagnostics,attribution=attribution,
                duration_s=dict(min=min(r['duration_s'] for r in chosen),max=max(r['duration_s'] for r in chosen)))
        session = dict(name=source['name'],source_roots=len(source['rows']),common_supported_roots=len(rows)//3,
            excluded=excluded,long_horizon_reproduction_max_m=max(checks),rows=rows,horizons=summaries,
            baseline_manifest=source['baseline_manifest'])
        report['sessions'].append(session)
        (out/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
        print('RESULT',source['name'],'common roots',session['common_supported_roots'],flush=True)
    report['completed'] = True
    (out/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print('REPORT',out/'report.json',flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--cache', required=True); parser.add_argument('--matched', required=True)
    parser.add_argument('--manifest', required=True); parser.add_argument('--output', required=True)
    parser.add_argument('--indices', nargs='+', type=int, default=[0, 4, 6])
    parser.add_argument('--compare-readouts', help='Completed saved-root report; compare v171 engaged static readout without replay/refitting')
    parser.add_argument('--register-static', help='Completed shared-confirmation root report; fit past-only forcing with fixed v171 static prior')
    parser.add_argument('--consistent-static', help='Completed shared-confirmation root report; replay existing UKF and forecast with fixed static prior and original decision tape')
    parser.add_argument('--compare-horizons', help='Completed consistent-static report; reuse roots for shorter horizons and passive/input attribution')
    parser.add_argument('--cartesian', action='store_true', help='Export complete consistent roots and own-pose achieved-input Cartesian forecasts')
    parser.add_argument('--confirmation-policy', choices=['standalone', 'shared_baseline'], default='shared_baseline',
        help='Shared production policy by default; standalone retains the historical detector ablation')
    run(parser.parse_args())
