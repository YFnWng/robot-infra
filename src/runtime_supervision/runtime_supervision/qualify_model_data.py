"""Gate-3 M1: read-only ROS evidence export and measured-input baselines.

Run as a module; constructs no ROS nodes or hardware clients. Mode is metadata,
not an inclusion criterion. The canonical bag/marker and calibration owners
remain authoritative.
"""
import argparse
from collections import Counter
import hashlib
import inspect
import json
from pathlib import Path
import shutil

import numpy as np

from catheter_control.safety.hardware_contract import (
    ENCODER_RADIANS_PER_COUNT, MAX_MOTOR_RPM)
from control_interface_py.encoder_coordinates import counts_to_logical
from cr_meta_lnn.training.hybrid.recorded_predictability import (
    build_windows, distribution, evaluate_windows)
from .compute_profile import read_events


class EvidenceError(ValueError):
    """Invalid source evidence, distinct from a predictor/scoring failure."""


def sha256(path):
    result = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(8*1024**2), b''):
            result.update(chunk)
    return result.hexdigest()


def extract(events, invalid_samples=()):
    """Reject bad evidence explicitly; retain barriers instead of bridging it."""
    markers, encoders, issues, barriers = [], [], list(invalid_samples), []
    previous = {}
    for issue in issues:
        barriers.append(issue['source_ns'] if issue['source_ns'] > 0 else issue['receipt_ns'])
    for event in events:
        if event.kind not in ('marker', 'encoder'):
            continue
        stamp, receipt = int(event.source_ns), int(event.receipt_ns)
        if stamp <= previous.get(event.kind, 0):
            issues.append(dict(reason='nonadvancing_'+event.kind, source_ns=stamp, receipt_ns=receipt))
            barriers.append(max(stamp, previous.get(event.kind, stamp)))
            continue
        previous[event.kind] = stamp
        if stamp > receipt:
            issues.append(dict(reason='source_after_receipt', source_ns=stamp, receipt_ns=receipt))
            barriers.append(stamp)
            continue
        if event.kind == 'marker':
            quality = event.quality
            good = (np.all(np.asarray(quality['confidence']) > 0)
                    and np.all(np.asarray(quality['source_rig_count']) >= 1)
                    and np.all(np.asarray(quality['reprojection_error_px']) >= 0))
            markers.append((stamp, receipt, np.asarray(event.values), bool(good), quality))
            if not good:
                issues.append(dict(reason='marker_quality', source_ns=stamp, receipt_ns=receipt))
        else:
            counts = np.asarray(event.values, float)
            if np.any(counts != np.rint(counts)) or np.any(np.abs(counts) > 2147483647):
                issues.append(dict(reason='invalid_encoder_counts', source_ns=stamp, receipt_ns=receipt))
                barriers.append(stamp); continue
            if encoders:
                prior = encoders[-1]
                # A break flag, not a corrected/unwrapped encoder coordinate.
                # 250 motor RPM is the existing firmware cap; allow endpoint
                # tolerance for timestamp/encoder quantization.
                allowance = MAX_MOTOR_RPM/60*(2*np.pi/ENCODER_RADIANS_PER_COUNT)*(stamp-prior[0])*1e-9+67
                if np.any(np.abs(counts-prior[2]) > allowance):
                    issues.append(dict(reason='encoder_rate_discontinuity', source_ns=stamp, receipt_ns=receipt))
                    barriers.append(stamp)
            encoders.append((stamp, receipt, counts.copy()))
    if len(markers) < 2 or len(encoders) < 2:
        raise ValueError('insufficient raw marker/encoder evidence')
    mt = np.array([r[0] for r in markers], np.int64)
    et = np.array([r[0] for r in encoders], np.int64)
    counts = np.array([r[2] for r in encoders])
    arrays = dict(marker_ns=mt, points_m=np.array([r[2] for r in markers]),
        marker_valid=np.array([r[3] for r in markers]), encoder_ns=et,
        encoder_counts=counts,
        logical_position=counts_to_logical(counts),
        shaft_rad=counts[:, :3]*ENCODER_RADIANS_PER_COUNT,
        marker_receipt_ns=np.array([r[1] for r in markers], np.int64),
        encoder_receipt_ns=np.array([r[1] for r in encoders], np.int64),
        interruptions_ns=np.array(sorted(set(barriers)), np.int64))
    for name in ('confidence', 'source_rig_count', 'reprojection_error_px'):
        arrays['marker_'+name] = np.array([r[4][name] for r in markers])
    return arrays, issues


def load_json(path):
    return json.loads(path.read_text()) if path.is_file() else {}


def run_session(session, output, *, stride_s=.2):
    invalid = []
    try:
        events = read_events(session, require_plan=False, sensors_only=True, invalid_samples=invalid)
        arrays, issues = extract(events, invalid)
    except ValueError as error:
        raise EvidenceError(str(error)) from error
    trials_file = session/'trials.jsonl'
    trials = ([json.loads(line) for line in trials_file.read_text().splitlines() if line]
              if trials_file.is_file() else [])
    starts = sorted(int(t['ros_time_ns']) for t in trials if t.get('event') == 'trial_started')
    # Whole later target attempts when available; a one-trial session can only
    # offer a weaker time-prefix diagnostic, explicitly labeled in the report.
    split = starts[len(starts)//2] if len(starts) >= 2 else None
    windows, audit = build_windows(arrays['marker_ns'], arrays['points_m'],
        arrays['encoder_ns'], arrays['shaft_rad'], marker_valid=arrays['marker_valid'],
        interruptions_ns=arrays['interruptions_ns'], stride_s=stride_s, split_ns=split)
    summary, predictions = evaluate_windows(windows)
    output.mkdir(parents=True, exist_ok=False)
    np.savez_compressed(output/'sensors.npz', **arrays)
    for name, rows in predictions.items():
        with (output/(name+'.jsonl')).open('w') as stream:
            for row in rows:
                stream.write(json.dumps(row, allow_nan=False)+'\n')
    files = [session/name for name in ('session_manifest.json', 'controller_manifest.json',
             'task_manifest.json', 'task_targets.yaml', 'trials.jsonl') if (session/name).is_file()]
    files += sorted((session/'robot_bag').glob('*.db3'))
    files += sorted(session.glob('*_video/registration.json'))
    files.append(session/'robot_bag/metadata.yaml')
    report = dict(session=str(session), dataset=str(output/'sensors.npz'), audit=audit,
        split_policy=('earlier_complete_windows_vs_later_target_attempts' if split is not None
                      else 'exploratory_60_percent_time_prefix'),
        independent_heldout_sessions=0, target_attempts=len(starts),
        issue_counts=dict(Counter(r['reason'] for r in issues)), issues=issues,
        marker_source_to_receipt_ms=distribution(
            (arrays['marker_receipt_ns']-arrays['marker_ns'])*1e-6),
        encoder_source_to_receipt_ms=distribution(
            (arrays['encoder_receipt_ns']-arrays['encoder_ns'])*1e-6),
        source_identity={str(p):dict(sha256=sha256(p), size_bytes=p.stat().st_size) for p in files},
        registration_and_mount_epoch='requires_review_before_cross_session_pooling',
        reference_discontinuity_policy='mark barriers; never re-zero or repair measurements',
        reference_discontinuity_interpretation='rate outliers are review flags, not proven physical reference resets',
        session_metadata={key:load_json(session/'session_manifest.json').get(key)
            for key in ('stack_config', 'registration_policy', 'registration_frames',
                        'model_manifest', 'command_output_enabled', 'state', 'stop_reason')},
        scores=summary)
    (output/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', type=Path, action='append', default=[])
    parser.add_argument('--session-root', type=Path)
    parser.add_argument('--dates', nargs='+', default=['20261008', '20261010', '20261011', '20261014'])
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--stride-s', type=float, default=.2)
    args = parser.parse_args(argv)
    if not np.isfinite(args.stride_s) or args.stride_s <= 0:
        parser.error('stride must be positive and finite')
    sessions = set(p.resolve() for p in args.session)
    if args.session_root:
        sessions.update(p.resolve() for p in args.session_root.iterdir()
            if p.is_dir() and p.name.startswith(('engagement_', 'adapj_'))
            and any(date+'T' in p.name for date in args.dates)
            and (p/'session_manifest.json').is_file())
    if not sessions:
        parser.error('no matching sessions')
    args.output.mkdir(parents=True, exist_ok=False)
    results = []
    for session in sorted(sessions):
        if shutil.disk_usage(args.output).free < 256*1024**2:
            raise RuntimeError('less than 256 MiB free; source evidence untouched')
        try:
            report = run_session(session, args.output/session.name, stride_s=args.stride_s)
            results.append(dict(session=str(session),
                status='scored' if report['audit']['test_windows'] else 'data_only',
                windows=report['audit']['supported_windows'], test_windows=report['audit']['test_windows'],
                report=str(args.output/session.name/'report.json')))
        except (EvidenceError, FileNotFoundError) as error:
            results.append(dict(session=str(session), status='excluded', reason=str(error)))
        print(json.dumps(results[-1]), flush=True)
    from cr_meta_lnn.training.hybrid.causal_shape_prediction import score_windows
    from control.adapj import AdaptiveForwardJacobian
    source_files = {Path(inspect.getfile(owner)).resolve() for owner in
                    (build_windows, score_windows, AdaptiveForwardJacobian, read_events, counts_to_logical)}
    source_files.add(Path(__file__).resolve())
    source_identity = {str(p):sha256(p) for p in sorted(source_files)}
    snapshots = args.output/'source_snapshot'
    snapshots.mkdir()
    for path, digest in source_identity.items():
        shutil.copyfile(path, snapshots/(digest[:12]+'_'+Path(path).name))
    defaults = inspect.signature(build_windows).parameters
    report = dict(schema='gate3_measured_input_m1', sessions=results,
        source_identity=source_identity,
        window_contract=dict(stride_s=args.stride_s, **{key:defaults[key].default for key in
            ('horizons_s','max_gap_s','endpoint_slack_s')}),
        qualification='exploratory conditional distal prediction; not deployment or gate-3 passage',
        timestamp_contract='marker acquisition and host-stamped encoder source time; unknown device sampling delay',
        units=dict(points='m', shaft='physical_rad', logical='mm_deg', timestamps='int64_ns'),
        controller_mode_policy='metadata only; measured motor/vision alignment determines support',
        future_information='achieved encoders only; no future marker correction or engagement labels',
        availability='source-time matured labels; not ROS arrival-order replay',
        learning='existing zero-prior local RLS; frozen prefix and completed-label prequential arms',
        limits=['no v171/adaptive-play comparison yet', 'no unexecuted action outcomes',
                'no heldout-session qualification', 'correlated windows are not independent trials'])
    (args.output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(dict(report=str(args.output/'report.json'), scored=sum(r['status']=='scored' for r in results))))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
