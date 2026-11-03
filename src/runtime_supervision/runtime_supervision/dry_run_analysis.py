"""Summarize one reduced-filter dry-run session: filter health, timing and plans. Offline, read-only.

Input is the controller's reduced_filter_dry_run.json and, optionally, the dry_run_targets interval
log. Distributions are count/P50/P95/P99/max with deadline misses, not averages alone.
"""
import argparse
from collections import Counter
import json
from pathlib import Path

import numpy as np


def distribution(values):
    values = np.asarray([v for v in values if v is not None], float)
    if not len(values):
        return dict(count=0)
    p50, p95, p99 = np.percentile(values, [50, 95, 99])
    return dict(count=int(len(values)), p50=float(p50), p95=float(p95), p99=float(p99), max=float(values.max()))


def target_intervals(lines):
    """[(index, repeat, target_m, start_ns, end_ns)] from the dry_run_targets log."""
    rows, open_rows = [], {}
    for line in lines:
        event = json.loads(line)
        key = (event.get('index'), event.get('repeat'))
        if event['event'] == 'target_started':
            open_rows[key] = event
        elif event['event'] == 'target_ended' and key in open_rows:
            start = open_rows.pop(key)
            rows.append((key[0], key[1], start['target_m'], start['ros_time_ns'], event['ros_time_ns']))
    return rows


def summarize(report, intervals=(), deadline_ms=50.0):
    filter_rows = report['filter_rows']
    plans = report['plan_rows']
    solved = [row for row in plans if 'tick_ms' in row]
    tracked = [row['step_ms'] for row in filter_rows if row['status'] == 'tracking']
    faults = [dict(source_ns=row['source_ns'], reason=row['reason']) for row in filter_rows if row['status'] == 'fault']
    holds = [dict(source_ns=row['source_ns'], reason=row['reason'], consecutive=row['consecutive_rejections'])
             for row in filter_rows if row['status'] == 'holding']
    summary = dict(
        estimator_health_at_stop=report.get('estimator_health'), ready_at_stop=report.get('ready'),
        images=dict(output=len(filter_rows), dropped=report.get('images_dropped'),
                    skipped=report.get('images_skipped'), statuses=dict(Counter(r['status'] for r in filter_rows))),
        filter_step_ms=distribution(tracked), filter_step_over_100ms=int(sum(v > 100 for v in tracked)),
        first_faults=faults[:5], first_holds=holds[:5],
        kernel_preparations_ms=report.get('kernel_preparations_ms'), warmup_plan_ms=report.get('warmup_plan_ms'),
        plans=dict(total=len(plans), valid=int(sum(r['valid'] for r in plans)),
                   reasons=dict(Counter(r['reason'] for r in plans)),
                   deadline_missed=int(sum(r.get('deadline_missed', False) for r in solved)),
                   deadline_ms=deadline_ms),
        plan_tick_ms=distribution(r['tick_ms'] for r in solved),
        planner_ms=distribution(r['planner_ms'] for r in solved),
        rollout_ms=distribution(r['rollout_ms'] for r in solved),
        root_build_ms=distribution(r['root_ms'] for r in plans),
        root_age_ms=distribution(r['root_age_ms'] for r in solved),
        counts=report.get('counts'))
    per_target = []
    for index, repeat, target, start, end in intervals:
        inside = [r for r in plans if start <= r['wall_ns'] <= end]
        valid = [r for r in inside if r['valid'] and r.get('best_tip_sequence_m')]
        terminal = [float(np.linalg.norm(np.asarray(r['best_tip_sequence_m'][-1])-np.asarray(target))*1e3)
                    for r in valid]
        per_target.append(dict(index=index, repeat=repeat, target_m=target, plans=len(inside), valid=len(valid),
                               predicted_terminal_error_mm=distribution(terminal)))
    summary['per_target'] = per_target
    summary['note'] = ('dry run: commands were never executed, so predicted errors describe plans from '
                       'unmoved states, not tracking performance')
    return summary


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', type=Path, required=True, help='reduced_filter_dry_run.json')
    parser.add_argument('--targets-log', type=Path, help='dry_run_targets JSONL log')
    parser.add_argument('--deadline-ms', type=float, default=50.0)
    parser.add_argument('--output', type=Path, help='new summary JSON path')
    args = parser.parse_args(argv)
    report = json.loads(args.report.read_text())
    intervals = target_intervals(args.targets_log.read_text().splitlines()) if args.targets_log else ()
    summary = summarize(report, intervals, args.deadline_ms)
    text = json.dumps(summary, indent=2)
    if args.output:
        if args.output.exists():
            parser.error('use a new output path')
        args.output.write_text(text+'\n')
    print(text)


if __name__ == '__main__':
    main()
