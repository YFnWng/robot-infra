#!/usr/bin/env python3
"""Read recorded shadow timing only; never initialize ROS or issue commands.

CSV rows are appended at stage completion. Reconstructed correction ages use
that ordering, not an instrumented per-snapshot correction timestamp. Treat
them as inferred, even when the count matches the reported stale gate count.
"""
import argparse
import csv
import json
import statistics
from pathlib import Path


def distribution(values):
    values = sorted(values)
    if not values:
        return None
    return dict(count=len(values), p50=statistics.median(values),
                p95=values[int(.95 * (len(values) - 1))], max=values[-1])


def diagnose(directory):
    report = json.loads((directory / 'report.json').read_text())
    with (directory / 'timing.csv').open() as stream:
        rows = list(csv.DictReader(stream))
    trace = {row['source_ns']: row for row in report['trace']}
    queue = {int(row['source_ns']): float(row['wall_ms']) for row in rows
             if row['stage'] == 'input_owner_queue_wait'}
    corrections = {int(row['source_ns']): row for row in rows
                   if row['stage'] == 'joint_correction_rewind_replay'}
    first_stamp = report.get('first_observed_source_ns') or min(trace)
    intervals = []
    for low, high in ((0, 75), (75, 90), (90, 110)):
        selected = [row for stamp, row in trace.items()
                    if low <= (stamp - first_stamp) * 1e-9 < high]
        intervals.append(dict(
            source_time_interval_s=[low, high],
            marker_receipt_age_ms=distribution([
                (row['receipt_ns'] - row['source_ns']) * 1e-6
                for row in selected]),
            correction_wall_ms=distribution([
                float(corrections[row['source_ns']]['wall_ms'])
                for row in selected if row['source_ns'] in corrections])))
    latest = published = None
    inferred_stale = []
    for row in rows:
        stage, stamp = row['stage'], int(row['source_ns'])
        if stage in ('spatial_initialization', 'joint_correction_rewind_replay'):
            latest = stamp
        if stage == 'estimator_callback':
            published = latest
        if stage == 'planner_snapshot_age' and published is not None:
            age = (stamp - published) * 1e-6
            if age > 150:
                inferred_stale.append(dict(check_ns=stamp, age_ms=age))
    misses = []
    for plan in report['plans']:
        if plan['valid']:
            continue
        start = plan['source_ns']
        nearby = []
        for stamp, correction in corrections.items():
            if stamp not in queue or stamp not in trace:
                continue
            # Avoid adding small durations to an epoch float (precision loss).
            relative_start = ((trace[stamp]['receipt_ns'] - start) * 1e-6
                              + queue[stamp])
            if abs(relative_start) <= 150:
                nearby.append(dict(
                    marker_ns=stamp, approximate_start_relative_ms=relative_start,
                    approximate_end_relative_ms=(relative_start
                                                 + float(correction['wall_ms'])),
                    wall_ms=float(correction['wall_ms']),
                    thread_cpu_ms=float(correction['thread_cpu_ms'])))
        misses.append(dict(plan=plan, nearby_corrections=nearby))
    worker_rows = report.get('worker_trace', [])
    sends = {row['previous_reply_send']['sequence']: row['previous_reply_send']
             for row in worker_rows if row.get('previous_reply_send')}
    gc_record = (report.get('live_context') or {}).get('gc_trace')
    gc_intervals = []
    active = {}
    for event in (gc_record or {}).get('events', []):
        key = (event['native_thread_id'], event['generation'])
        if event['phase'] == 'start':
            active[key] = event
        elif key in active:
            start = active.pop(key)
            gc_intervals.append(dict(start_s=start['monotonic_s'],
                end_s=event['monotonic_s'], generation=key[1], native_thread_id=key[0],
                wall_ms=1000*(event['monotonic_s']-start['monotonic_s']),
                thread_cpu_ms=1000*(event['thread_cpu_s']-start['thread_cpu_s'])))
    discarded = []
    budget = report['configuration']['planning_deadline_s']
    for row in worker_rows:
        if not row['discarded']:
            continue
        start = row['request_started_s']
        send = sends.get(row['sequence'])
        overlaps = [event for event in gc_intervals
                    if event['start_s'] < start+budget and event['end_s'] > start]
        discarded.append(dict(sequence=row['sequence'],
            worker_reply_ready_after_request_ms=1000*(row['worker_reply_ready_s']-start),
            send_completed_after_request_ms=None if send is None else
                1000*(send['send_completed_s']-start),
            send_timing=send, parent_first_byte_after_request_ms=None if
                row.get('parent_first_byte_s') is None else
                1000*(row['parent_first_byte_s']-start), gc_overlaps=overlaps))
    return dict(scope='Recorded evidence only; no worker-internal GPU or OS tracing.',
                report=str(directory / 'report.json'), intervals=intervals,
                gc_experiment=(report.get('live_context') or {}).get('gc_experiment'),
                diagnostic_retention=report.get('diagnostic_retention'),
                diagnostic_evidence_complete=report.get('diagnostic_evidence_complete'),
                worker_timing_summary=report.get('worker_timing_summary'),
                reported_correction_age_skips=report['counts'].get(
                    'plan_stale:correction_age', 0),
                inferred_stale_age_ms=distribution([
                    row['age_ms'] for row in inferred_stale]),
                inferred_stale_checks=inferred_stale, invalid_plans=misses,
                discarded_request_diagnostics=discarded,
                gc_trace_available=gc_record is not None,
                gc_dropped_events=None if gc_record is None else gc_record['dropped'],
                gc_wall_ms=distribution([row['wall_ms'] for row in gc_intervals]))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    args = parser.parse_args()
    print(json.dumps(diagnose(args.directory), indent=2))
