"""Session prior identification for the reduced filter: export, fit and pass-2 validation. Offline only.

Formulation: audits/ros-realtime/PRIOR_IDENTIFICATION_SESSION_PLAN.md (offline fitting pipeline, approved
2026-11-02). Estimator mathematics stays in cr_meta_lnn (session_prior_identification, the readout and GP
owners); this module only reads sessions, labels images from the recorded causal trace, writes artifacts
and runs the declared checks. Nothing is installed: adopting a manifest is a separate approved change.

  export   one recorded session -> per-image arrays (lambda and tip readouts, inputs, episode labels)
  fit      block exports -> profiles, widths, axis, tip-lambda prior and a prior manifest (pass 1 only)
  validate block C pass 2 -> the declared checks against the October priors
"""
import argparse
from dataclasses import replace
import json
from pathlib import Path
import re

import numpy as np

from .prior_refit import BASE, image_inputs, refit_axis, select_images

OCTOBER_PROFILE = BASE/'gate3_compact_cycle_oracle_prior_20261025/initialization.npz'
OCTOBER_AXIS = BASE/'gate3_cartesian_gain_cli_445_485_20261027/cartesian_initialization.json'
OCTOBER_SUPPORT_MM = 10.766078
PASS = {1: re.compile(r'_p1_'), 2: re.compile(r'_p2_')}
SWEEP = re.compile(r'^(compensated_bend_insertion|insertion_sweep)_p[12]_')


def read_trace(session, prefix=''):
    """(stamp_ns, episode_name) of the recorded causal trace, in time order."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(Path(session)/'robot_bag'), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    topic = prefix+'/collection/causal_trace'
    kind = get_message('control_interface/msg/CausalExperimentTrace')
    rows = []
    while reader.has_next():
        name, payload, _ = reader.read_next()
        if name == topic:
            message = deserialize_message(payload, kind)
            rows.append((message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec, message.episode_name))
    rows.sort()
    return rows


def export(session, prefix, covariance=None):
    from cr_meta_lnn.training.hybrid.real_native_memory_bootstrap import load_native_artifacts
    from cr_meta_lnn.training.hybrid.session_prior_identification import lambda_readout
    from .compute_profile import read_events
    from .qualify_model_data import extract
    invalid = []
    sensors, _ = extract(read_events(Path(session), prefix, require_plan=False, sensors_only=True,
                                     invalid_samples=invalid), invalid)
    images = select_images(sensors)
    image_ns = sensors['marker_ns'][images]
    inside = (image_ns > sensors['encoder_ns'][0]) & (image_ns < sensors['encoder_ns'][-1])
    images, image_ns = images[inside], image_ns[inside]
    driver, tendon = image_inputs(sensors, image_ns)
    artifacts = load_native_artifacts(BASE)
    R = artifacts.marker_covariance_mm2
    if covariance is not None:
        with np.load(covariance, allow_pickle=False) as z:
            R = z['learned_full_covariance_mm2'].copy()
    points = sensors['points_m'][images]*1000
    lam, lam_var, tip, valid = (np.full(len(images), np.nan), np.full(len(images), np.nan),
                                np.full((len(images), 3), np.nan), np.zeros(len(images), bool))
    for k, (t, p) in enumerate(zip(image_ns, points)):
        readout = lambda_readout(artifacts.likelihood, t, p, R)
        if readout is not None:
            mean, cov = readout
            lam[k], lam_var[k], tip[k], valid[k] = mean[3], cov[3, 3], mean[:3], True
    trace = read_trace(session, prefix)
    stamps = np.array([t for t, _ in trace], np.int64)
    names = [n for _, n in trace]
    index = np.searchsorted(stamps, image_ns, side='right')-1
    episode = np.array([names[i] if i >= 0 else '' for i in index])
    arrays = dict(image_ns=image_ns, marker_index=images, points_mm=points, tendon_mm=tendon,
                  chassis_mm=driver[:, 0], rotation_mm=driver[:, 1], knob_mm=driver[:, 2], lam=lam, lam_var=lam_var,
                  tip_mm=tip, valid=valid, episode=episode, marker_receipt_ns=sensors['marker_receipt_ns'][images],
                  covariance_mm2=R, **{'sensors_'+k: v for k, v in sensors.items()})
    report = dict(session=str(session), prefix=prefix, images=int(len(images)), valid_readouts=int(valid.sum()),
                  episodes=sorted(set(episode)), covariance=str(covariance or 'october'))
    return arrays, report


def load(path):
    with np.load(path, allow_pickle=False) as z:
        return {k: z[k] for k in z.files}


def sensors_of(e):
    return {k[len('sensors_'):]: v for k, v in e.items() if k.startswith('sensors_')}


def label_mask(episode, pattern):
    return np.array([bool(pattern.search(str(name))) for name in episode])


def arm_table(e):
    """Arms of one export with onset and width; invalid reversal readouts give no onset."""
    from cr_meta_lnn.training.hybrid.session_prior_identification import engagement_onset, tendon_arms
    rows = []
    for start, end, direction in tendon_arms(e['tendon_mm']):
        if not e['valid'][start]:
            rows.append(dict(start=start, end=end, direction=direction, onset=None, width=None))
            continue
        onset, width = engagement_onset(e['tendon_mm'], np.where(e['valid'], e['lam'], np.nan),
                                        np.where(e['valid'], e['lam_var'], np.inf), start, end, direction)
        rows.append(dict(start=start, end=end, direction=direction, onset=onset, width=width))
    return rows


def still_intervals(e, mask):
    """Consecutive-image intervals inside mask with both readouts valid and the chassis still."""
    k = np.flatnonzero(mask[:-1] & mask[1:] & e['valid'][:-1] & e['valid'][1:]
                       & (np.abs(np.diff(e['chassis_mm'])) < .01))
    return (np.diff(e['tip_mm'], axis=0)[k], np.diff(e['knob_mm'])[k], np.diff(e['lam'])[k])


def fit(sweep, reversal, insertion, output, *, covariance=None, breakpoints=20):
    from threadpoolctl import threadpool_limits
    from cr_meta_lnn.training.hybrid.joint_initialization_bundle import (
        initialization_profile_settings, load_initialization, save_initialization)
    from cr_meta_lnn.training.hybrid.real_native_memory_bootstrap import OCTOBER_PRIORS, write_prior_manifest
    from cr_meta_lnn.training.hybrid.session_prior_identification import (
        fit_direction_profile, fit_tip_lambda_response, width_prior)
    c, d = load(sweep), load(reversal)
    pass1 = label_mask(c['episode'], PASS[1]) & label_mask(c['episode'], SWEEP)
    c_arms, d_arms = arm_table(c), arm_table(d)
    widths = {str(direction): [a['width'] for a in c_arms if a['direction'] == direction and pass1[a['start']]]
              + [a['width'] for a in d_arms if a['direction'] == direction] for direction in (-1, 1)}
    width_config = {key: {str(direction): width_prior(widths[str(direction)]) for direction in (-1, 1)}
                    for key in ('full_gap', 'initial_remaining')}
    profiles, fits = [], {}
    with threadpool_limits(limits=1):   # the verified conditioning is bit-exact single-threaded
        for settings in initialization_profile_settings(OCTOBER_PROFILE):
            direction = int(settings['direction'])
            index = np.array(sorted({k for a in c_arms if a['direction'] == direction and a['onset'] is not None
                                     for k in range(a['onset'], a['end']+1)}), int)
            index = index[pass1[index] & c['valid'][index] & (c['tendon_mm'][index] >= 0)
                          & (c['tendon_mm'][index] <= 15.5)]
            belief, info = fit_direction_profile(settings, c['tendon_mm'][index], c['lam'][index])
            profiles.append(belief)
            fits[str(direction)] = dict(info, training_images=int(len(index)))
    output.mkdir(parents=True)
    bundle = replace(load_initialization(OCTOBER_PROFILE), profiles=tuple(profiles))
    provenance = dict(origin='session prior identification, block C pass 1 engaged images', sweep=str(sweep),
                      reversal=str(reversal), insertion=str(insertion), observation_variance=1.0,
                      unchanged_from_october=['GP hyperparameters', 'J_mean', 'J_covariance', 'chassis gap fields'])
    save_initialization(output/'initialization.npz', bundle, provenance=provenance)
    source = 'engagement onset from distal deformation: block C pass-1 and block D reversals'
    (output/'widths.json').write_text(json.dumps(dict(width_config, samples=widths, provenance=source), indent=2)+'\n')
    axis_fit = refit_axis(sensors_of(load(insertion)))
    axis = dict(json.loads(OCTOBER_AXIS.read_text()), world_axial_direction=axis_fit['marker_mean']['direction'])
    (output/'axis.json').write_text(json.dumps(axis, indent=2)+'\n')
    tip_fit = fit_tip_lambda_response(*still_intervals(c, pass1))
    (output/'tip_lambda_prior.json').write_text(json.dumps(tip_fit, indent=2)+'\n')
    artifacts = {key: BASE/value for key, value in OCTOBER_PRIORS.items()}
    artifacts.update(profiles=output/'initialization.npz', widths=output/'widths.json', axis=output/'axis.json')
    if covariance is not None:
        artifacts['--marker-covariance'] = Path(covariance)
    write_prior_manifest(output/'prior_manifest.json', artifacts, profile_breakpoints=breakpoints,
                         tip_lambda_prior=output/'tip_lambda_prior.json', provenance=provenance)
    report = dict(profiles=fits, widths={k: v for k, v in width_config['full_gap'].items()},
                  width_samples={k: len([w for w in v if w is not None]) for k, v in widths.items()},
                  axis=dict(direction=axis_fit['marker_mean']['direction'],
                            angle_to_october_deg=axis_fit['angle_to_current_prior_deg']),
                  tip_lambda=tip_fit, profile_breakpoints=breakpoints, installed=False)
    (output/'fit_report.json').write_text(json.dumps(report, indent=2, default=float)+'\n')
    return report


def lambda_check(c, arms, pass2, new_profiles, old_profiles):
    from cr_meta_lnn.training.hybrid.session_prior_identification import profile_increment_prediction, rms
    errors = {'new_within': [], 'october_within': [], 'new_beyond': []}
    for a in arms:
        if a['onset'] is None or not pass2[a['start']]:
            continue
        k = np.arange(a['onset'], a['end']+1)
        k = k[c['valid'][k]]
        if len(k) < 3:
            continue
        observed = c['lam'][k]-c['lam'][k[0]]
        for name, profiles in (('new', new_profiles), ('october', old_profiles)):
            belief = next(p for p in profiles if p.model.direction == a['direction'])
            predicted = profile_increment_prediction(belief, c['tendon_mm'][k], 0)
            within = c['tendon_mm'][k] <= OCTOBER_SUPPORT_MM
            errors[name+'_within'] += list((predicted-observed)[within])
            if name == 'new':
                errors['new_beyond'] += list((predicted-observed)[~within])
    paired = [(n, o) for n, o in zip(errors['new_within'], errors['october_within']) if np.isfinite(n) and np.isfinite(o)]
    result = dict(new_rmse_within=rms([n for n, _ in paired]), october_rmse_within=rms([o for _, o in paired]),
                  paired_images=len(paired), new_rmse_beyond=rms(errors['new_beyond']),
                  images_beyond=int(np.isfinite(errors['new_beyond']).sum()))
    result['passed'] = bool(paired) and result['new_rmse_within'] <= result['october_rmse_within']
    return result


def replay(c, pass2, manifest, *, every=6, horizon_ns=800_000_000):
    """Cold start at the first pass-2 image; reduced filter through pass 2; frozen-root forecasts on achieved encoders."""
    import importlib.util
    import sys
    import torch
    from cr_meta_lnn.deployment.interface_transmission_checkpoint import load_interface_transmission_artifacts
    from cr_meta_lnn.training.hybrid.bounded_interface_readout import BoundedFitCurve, BoundedInterfaceReadoutFactory
    from cr_meta_lnn.training.hybrid.curve_marker_likelihood import CurveMarkerLikelihood
    from cr_meta_lnn.training.hybrid.real_native_memory_bootstrap import initialize_native_root, load_native_artifacts
    from cr_meta_lnn.training.hybrid.reduced_filter_stream import TRANSMISSION_ARTIFACT
    from cr_meta_lnn.training.hybrid.reduced_law_filter import (
        POSITION, ReducedEstimator, forecast_native_laws, reduce_scalar_memory)
    archived = Path('/home/chen-lab/Yifan/repository_archives/cleanup_20261102/files/cr_meta_lnn/training/hybrid/'
                    'real_joint_preparation.py')
    spec = importlib.util.spec_from_file_location('cr_meta_lnn.training.hybrid.real_joint_preparation', archived)
    prep = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = prep
    spec.loader.exec_module(prep)
    sensors = sensors_of(c)
    chosen = np.flatnonzero(pass2)
    transmission = load_interface_transmission_artifacts(TRANSMISSION_ARTIFACT, dtype=torch.float64).transmission
    packet, _ = prep.real_observation_packet(sensors, c['image_ns'][chosen], c['marker_index'][chosen], transmission)
    d = packet.arrays
    artifacts = load_native_artifacts(BASE, prior_manifest=manifest, learning_minimum_travel_mm=.01,
                                      profile_support_extension_mm=.5)
    ns, image_ns = d['native_ns'], d['image_ns']
    n0 = int(np.searchsorted(ns, image_ns[0]))
    root = initialize_native_root(artifacts, source_ns=int(image_ns[0]), tendon_mm=float(d['tendon_mm'][n0]),
                                  chassis_mm=float(d['driver_mm'][n0, 0]), knob_mm=float(d['driver_mm'][n0, 2]),
                                  points_mm=d['markers_mm'][0])
    base = artifacts.likelihood
    like = CurveMarkerLikelihood(BoundedFitCurve(base.curve), station_bounds_mm=base.bounds)
    root['runtime'].likelihood = like
    R = artifacts.marker_covariance_mm2
    readout = BoundedInterfaceReadoutFactory(like, image_ns, d['markers_mm'], R)
    est = ReducedEstimator(reduce_scalar_memory(root['state']), root['runtime'], root['partition'], root['admission'],
                           maximum_laws=2, knob_log_std_floor=0., knob_relative_std_floor=float(np.sqrt(np.expm1(.25))))
    tip = c['tip_mm'][chosen]
    rows, fans, health = {}, [], []
    for image in range(1, len(image_ns)):
        stamp = int(image_ns[image])
        i, j = int(np.searchsorted(ns, est.laws.source_ns)), int(np.searchsorted(ns, stamp))
        try:
            h, _ = est.step(native_ns=ns[i:j+1], tendon_mm=d['tendon_mm'][i:j+1], chassis_mm=d['driver_mm'][i:j+1, 0],
                            rotation_rad=d['driver_mm'][i:j+1, 1], knob_mm=d['driver_mm'][i:j+1, 2], stamp=stamp,
                            points_mm=d['markers_mm'][image], marker_covariance_mm2=R,
                            cumulative_q_mm2=np.zeros((j-i+1, 3, 3)), readout_factory=readout)
        except RuntimeError as error:
            health.append((image, 'fault', str(error)[:160]))
            break
        if h.status != 'tracking':
            health.append((image, h.status, (h.reason or '')[:160]))
            if h.status == 'fault':
                break
            continue
        rows[image] = est.laws.moments()[0][POSITION].copy()
        if image % every == 0:
            targets = [t for t in range(image+1, len(image_ns)) if image_ns[t] <= min(stamp+horizon_ns, ns[-1])]
            if len(targets) >= 6:
                predicted = []
                for t in targets:
                    b = int(np.searchsorted(ns, image_ns[t]))
                    predicted.append(forecast_native_laws(est.laws, est.runtime, ns[j:b+1], d['tendon_mm'][j:b+1],
                                                          d['driver_mm'][j:b+1, 0], d['driver_mm'][j:b+1, 2]))
                fans.append(dict(root=image, targets=targets, error=[float(np.linalg.norm(p-tip[t]))
                                                                     for p, t in zip(predicted, targets)],
                                 max_tendon=float(d['tendon_mm'][int(np.searchsorted(ns, image_ns[targets[-1]]))])))
    fit_error = [float(np.linalg.norm(p-tip[k])) for k, p in rows.items() if np.isfinite(tip[k]).all()]
    return dict(images=int(len(image_ns)), tracked=len(rows), health=health, fans=fans, fit_errors=fit_error)


def validate(sweep, fit_dir, output):
    from threadpoolctl import threadpool_limits
    from cr_meta_lnn.training.hybrid.joint_initialization_bundle import load_initialization
    from cr_meta_lnn.training.hybrid.session_prior_identification import rms
    c = load(sweep)
    pass2 = label_mask(c['episode'], PASS[2]) & label_mask(c['episode'], SWEEP)
    arms = arm_table(c)
    new = load_initialization(fit_dir/'initialization.npz').profiles
    old = load_initialization(OCTOBER_PROFILE).profiles
    report = dict(lambda_increments=lambda_check(c, arms, pass2, new, old))
    widths = json.loads((fit_dir/'widths.json').read_text())['full_gap']
    inside = []
    for a in arms:
        if pass2[a['start']] and a['width'] is not None:
            w = widths[str(a['direction'])]
            inside.append(abs(a['width']-w['location']) <= 1.645*w['scale'])
    report['onset'] = dict(reversals=len(inside), inside_central_90=float(np.mean(inside)) if inside else None,
                           passed=bool(inside) and float(np.mean(inside)) >= .8)
    tip = json.loads((fit_dir/'tip_lambda_prior.json').read_text())
    dp, dk, dl = still_intervals(c, pass2)
    a, j = np.asarray(tip['knob_response']), np.asarray(tip['tip_lambda_mean'])
    with_j = rms(np.linalg.norm(dp-np.outer(dk, a)-np.outer(dl, j), axis=1))
    without = rms(np.linalg.norm(dp-np.outer(dk, a), axis=1))
    report['tip_lambda'] = dict(intervals=int(len(dp)), rms_with_j=with_j, rms_zero_j=without, passed=with_j < without)
    with threadpool_limits(limits=1):
        runs = {'october': replay(c, pass2, None), 'session': replay(c, pass2, fit_dir/'prior_manifest.json')}
    summary = {}
    for name, run in runs.items():
        within = [e for f in run['fans'] if f['max_tendon'] <= OCTOBER_SUPPORT_MM for e in f['error']]
        summary[name] = dict(images=run['images'], tracked=run['tracked'], faults=[h for h in run['health'] if h[1] == 'fault'],
                             holds=len([h for h in run['health'] if h[1] == 'holding']),
                             forecast_rmse_within=rms(within), forecast_roots=len(run['fans']),
                             current_fit_rms=rms(run['fit_errors']))
    s, o = summary['session'], summary['october']
    summary['passed'] = (not s['faults'] and s['forecast_rmse_within'] <= o['forecast_rmse_within']
                         and s['current_fit_rms'] <= o['current_fit_rms']+.02)
    report['replay'] = summary
    report['passed'] = all(report[k]['passed'] for k in ('lambda_increments', 'onset', 'tip_lambda', 'replay'))
    output.write_text(json.dumps(dict(report, runs=runs), indent=2, default=float)+'\n')
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest='command', required=True)
    e = sub.add_parser('export')
    e.add_argument('--session', type=Path, required=True)
    e.add_argument('--prefix', default='', help="topic prefix, '/sim' for simulator sessions")
    e.add_argument('--covariance', type=Path, help='re-fitted R (prior_refit covariance output npz)')
    e.add_argument('--output', type=Path, required=True)
    f = sub.add_parser('fit')
    for name in ('sweep', 'reversal', 'insertion'):
        f.add_argument('--'+name, type=Path, required=True, help=f'export npz of the {name} block')
    f.add_argument('--covariance', type=Path)
    f.add_argument('--breakpoints', type=int, default=20)
    f.add_argument('--output', type=Path, required=True)
    v = sub.add_parser('validate')
    v.add_argument('--sweep', type=Path, required=True)
    v.add_argument('--fit', type=Path, required=True)
    v.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    if args.output.exists():
        parser.error('use a new output path')
    if args.command == 'export':
        arrays, report = export(args.session, args.prefix, args.covariance)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(args.output, **arrays)
    elif args.command == 'fit':
        report = fit(args.sweep, args.reversal, args.insertion, args.output, covariance=args.covariance,
                     breakpoints=args.breakpoints)
    else:
        report = validate(args.sweep, args.fit, args.output)
    print(json.dumps(report, indent=2, default=float))


if __name__ == '__main__':
    main()
