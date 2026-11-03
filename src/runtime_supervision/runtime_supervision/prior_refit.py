"""Re-fit the reduced filter's camera-frame priors from a new session's recording. Offline, read-only.

axis: engaged chassis-only response direction (cr_meta_lnn engaged_axial_direction), compared with the
      current world axial-direction prior.
covariance: quiet marker covariance with the restored 2026-10-20 owner (marker_covariance_validation),
      on the longest still window, scored on held-out frames against baseline and the current October R.

Nothing is installed: outputs go to a new directory for review, and adopting either prior is a
separate, explicitly approved change to the filter's artifacts.
"""
import argparse
import json
from pathlib import Path

import numpy as np

BASE = Path('/media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions')
AXIS_PRIOR = BASE/'gate3_cartesian_gain_cli_445_485_20261027/cartesian_initialization.json'
OCTOBER_R = BASE/'gate3_marker_covariance_validation_20261020/covariance_validation.npz'
PLAY_WIDTH_MM = 7.05024141413636   # calibrated chassis play used by the filter (real_native_memory_bootstrap)


def load_arrays(session=None, sensors=None):
    if sensors is not None:
        return dict(np.load(sensors, allow_pickle=False))
    from .compute_profile import read_events
    from .qualify_model_data import extract
    invalid = []
    arrays, _ = extract(read_events(Path(session), require_plan=False, sensors_only=True,
                                    invalid_samples=invalid), invalid)
    return arrays


def select_images(arrays, period_ns=90_000_000):
    """Valid markers taken by the aligner's rule: once period_ns has passed since the last taken."""
    taken, last = [], None
    for k in np.flatnonzero(arrays['marker_valid']):
        t = int(arrays['marker_ns'][k])
        if last is None or t-last >= period_ns:
            taken.append(k)
            last = t
    return np.asarray(taken)


def image_inputs(arrays, image_ns):
    """Chassis, rotation and knob (v175 driver) and logical tendon at image times, via the filter's owners."""
    import torch
    from cr_meta_lnn.deployment.interface_transmission_checkpoint import load_interface_transmission_artifacts
    from cr_meta_lnn.training.hybrid.joint_curve_replay import encoder_source_at
    from cr_meta_lnn.training.hybrid.native_pi_replay import input_at
    from cr_meta_lnn.training.hybrid.reduced_filter_stream import TRANSMISSION_ARTIFACT
    transmission = load_interface_transmission_artifacts(TRANSMISSION_ARTIFACT, dtype=torch.float64).transmission
    shaft = np.array([encoder_source_at(arrays['encoder_ns'], arrays['shaft_rad'], int(t), maximum_gap_s=.1)[0]
                      for t in image_ns])
    with torch.no_grad():
        driver = transmission.driver(torch.tensor(shaft, dtype=torch.float64)).numpy()*1000
    tendon = input_at(arrays['encoder_ns'], arrays['logical_position'][:, 2], np.asarray(image_ns, np.int64))
    return driver, np.asarray(tendon)


def refit_axis(arrays, *, width_mm=PLAY_WIDTH_MM):
    from cr_meta_lnn.training.hybrid.engaged_axial_direction import (
        angle_deg, engaged_chassis_intervals, fit_axial_response)
    images = select_images(arrays)
    image_ns = arrays['marker_ns'][images]
    inside = (image_ns >= arrays['encoder_ns'][0]) & (image_ns <= arrays['encoder_ns'][-1])
    images, image_ns = images[inside], image_ns[inside]
    driver, tendon = image_inputs(arrays, image_ns)
    markers = arrays['points_m'][images]*1000
    mask = engaged_chassis_intervals(driver[:, 0], tendon, driver[:, 2], width_mm=width_mm)
    if np.ptp(driver[:, 1]) > 1e-9:
        raise ValueError('rotation moved in this recording; the axis fit assumes no rotation')
    dc = np.diff(driver[:, 0])[mask]
    prior = np.asarray(json.loads(AXIS_PRIOR.read_text())['world_axial_direction'])
    mean_fit = fit_axial_response(np.diff(markers.mean(axis=1), axis=0)[mask], dc)
    per_marker = [fit_axial_response(np.diff(markers[:, i], axis=0)[mask], dc, bootstrap=200) for i in range(4)]
    return dict(images=int(len(images)), engaged_chassis_intervals=int(mask.sum()), play_width_mm=width_mm,
                marker_mean=mean_fit, per_marker_direction=[f['direction'] for f in per_marker],
                per_marker_gain=[f['gain'] for f in per_marker],
                current_prior=prior.tolist(), angle_to_current_prior_deg=angle_deg(mean_fit['direction'], prior),
                prior_std_reference=0.05,
                note='prior std 0.05 per component is about 2.9 degrees; a larger angle means the October prior '
                     'is confidently wrong for this mount/registration')


def still_windows(arrays, image_ns, *, tolerance_counts=2):
    """[(first, last)] image runs where the first three encoder channels stay within a count range."""
    index = np.clip(np.searchsorted(arrays['encoder_ns'], image_ns), 0, len(arrays['encoder_ns'])-1)
    counts = arrays['encoder_counts'][index, :3]
    runs, start, low, high = [], 0, counts[0].copy(), counts[0].copy()
    for i in range(1, len(counts)):
        new_low, new_high = np.minimum(low, counts[i]), np.maximum(high, counts[i])
        if np.any(new_high-new_low > tolerance_counts):
            runs.append((start, i-1))
            start, low, high = i, counts[i].copy(), counts[i].copy()
        else:
            low, high = new_low, new_high
    runs.append((start, len(counts)-1))
    return runs


def refit_covariance(arrays, *, train_s=10.0, heldout_s=10.0):
    from cr_meta_lnn.training.hybrid.marker_covariance_validation import (
        gaussian_contrast_score, trend_contrasts, validate_covariance)
    images = select_images(arrays)
    image_ns = arrays['marker_ns'][images]
    runs = still_windows(arrays, image_ns)
    longest = max(runs, key=lambda r: image_ns[r[1]]-image_ns[r[0]])
    duration = (image_ns[longest[1]]-image_ns[longest[0]])*1e-9
    if duration < train_s+heldout_s:
        raise ValueError(f'longest still window is {duration:.1f} s; {train_s+heldout_s:.0f} s required')
    chosen = images[longest[0]:longest[1]+1]
    data = dict(source_ns=arrays['marker_ns'][chosen], markers_mm=arrays['points_m'][chosen]*1000)
    summary, out = validate_covariance(data, training_s=(0., train_s), heldout_s=(train_s, train_s+heldout_s))
    t = (data['source_ns']-data['source_ns'][0])*1e-9
    held = (t >= train_s) & (t < train_s+heldout_s)
    contrasts = trend_contrasts(t[held], data['markers_mm'][held].reshape(int(held.sum()), 12))['contrasts']
    with np.load(OCTOBER_R, allow_pickle=False) as z:
        october = z['learned_full_covariance_mm2']
    score, _, _ = gaussian_contrast_score(contrasts, october)
    refit = summary['windows']['heldout']['scores']['learned_full']['mean_log_density']
    summary['october_R_on_this_heldout'] = score
    summary['refit_minus_october_log_density'] = refit-score['mean_log_density']
    summary['still_window'] = dict(start_ns=int(image_ns[longest[0]]), duration_s=duration, images=int(len(chosen)))
    return summary, out


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('kind', choices=('axis', 'covariance'))
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--session', type=Path, help='recorded session directory with robot_bag')
    source.add_argument('--sensors', type=Path, help='sensors.npz from the measured-input export')
    parser.add_argument('--output', type=Path, required=True, help='new directory for the re-fit and its report')
    args = parser.parse_args(argv)
    if args.output.exists():
        parser.error('use a new output directory')
    arrays = load_arrays(args.session, args.sensors)
    args.output.mkdir(parents=True)
    if args.kind == 'axis':
        report = refit_axis(arrays)
    else:
        from threadpoolctl import threadpool_limits
        with threadpool_limits(limits=1):
            report, out = refit_covariance(arrays)
        np.savez_compressed(args.output/'covariance_refit.npz', **out)
    report.update(source=str(args.session or args.sensors), installed=False)
    (args.output/f'{args.kind}_refit.json').write_text(json.dumps(report, indent=2, default=float)+'\n')
    print(json.dumps({k: v for k, v in report.items() if k not in ('models', 'windows')}, indent=2, default=float))


if __name__ == '__main__':
    main()
