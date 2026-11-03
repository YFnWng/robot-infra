"""Step a list of tip targets for the reduced-filter dry run; publishes targets only.

Fail closed: targets are published only while /catheter_mppi/status comes from the non-commanding
dry-run controller (hardware_id reduced_filter_dry_run, command output disabled) and is fresh. Any
other controller, a stale status or a command-capable status stops publication. Each target interval
is logged for analysis. No command, mode, arm, manager or device interface is used.
"""
import argparse
import json
from pathlib import Path
import time

import numpy as np
import yaml

DRY_RUN_ID = 'reduced_filter_dry_run'
STATUS_TIMEOUT_S = 3.0


def load_targets(targets_file=None, targets=()):
    """Targets in metres, from an experiment file (generator.targets_m) and/or explicit triples."""
    rows = []
    if targets_file is not None:
        document = yaml.safe_load(Path(targets_file).read_text()) or {}
        rows += list((document.get('generator') or {}).get('targets_m') or [])
    rows += [list(t) for t in targets]
    values = np.asarray(rows, dtype=float)
    if values.ndim != 2 or values.shape[1] != 3 or not len(values) or not np.isfinite(values).all():
        raise ValueError('at least one finite xyz target in metres is required')
    if np.any(np.abs(values) > 1.0):
        raise ValueError('targets must be in metres (|value| <= 1 m)')
    return values


def dry_run_status(message):
    """(is the non-commanding dry run, reason) for one DiagnosticArray."""
    for status in message.status:
        if status.name != 'catheter_mppi':
            continue
        values = {item.key: item.value for item in status.values}
        if status.hardware_id != DRY_RUN_ID:
            return False, f'controller is {status.hardware_id or "unknown"}, not the dry run'
        if values.get('command_output_enabled') != 'False' or values.get('command_capable') != 'False':
            return False, 'controller reports command capability'
        return True, None
    return False, 'no catheter_mppi status'


def main(argv=None):
    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray
    from geometry_msgs.msg import PointStamped
    from rclpy.utilities import remove_ros_args

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--targets-file', type=Path, help='experiment YAML with generator.targets_m')
    parser.add_argument('--target', type=float, nargs=3, action='append', default=[], metavar=('X', 'Y', 'Z'))
    parser.add_argument('--dwell-s', type=float, default=10.0, help='seconds per target')
    parser.add_argument('--repeat', type=int, default=1)
    parser.add_argument('--rate-hz', type=float, default=2.0)
    parser.add_argument('--frame-id', default='robot_base')
    parser.add_argument('--log', type=Path, required=True, help='new JSONL file of target intervals')
    args = parser.parse_args(remove_ros_args(args=argv)[1:])
    targets = load_targets(args.targets_file, args.target)
    if not (0 < args.dwell_s <= 600 and 0 < args.rate_hz <= 20 and 1 <= args.repeat <= 100):
        parser.error('dwell in (0,600] s, rate in (0,20] Hz and repeat in [1,100] required')
    if args.log.exists():
        parser.error('use a new log file')
    rclpy.init(args=argv)
    node = rclpy.create_node('dry_run_targets')
    state = dict(ok=False, reason='no status yet', at=None)

    def on_status(message):
        ok, reason = dry_run_status(message)
        state.update(ok=ok, reason=reason, at=time.monotonic())

    node.create_subscription(DiagnosticArray, '/catheter_mppi/status', on_status, 10)
    publisher = node.create_publisher(PointStamped, '/catheter_mppi/target_tip', 10)

    def gate_open():
        return state['ok'] and state['at'] is not None and time.monotonic()-state['at'] < STATUS_TIMEOUT_S

    stop_reason = 'complete'
    args.log.parent.mkdir(parents=True, exist_ok=True)
    with args.log.open('x') as log:
        try:
            deadline = time.monotonic()+30.0
            while not gate_open():
                rclpy.spin_once(node, timeout_sec=0.1)
                if time.monotonic() > deadline:
                    raise RuntimeError(f'dry-run controller not confirmed: {state["reason"]}')
            for repeat in range(args.repeat):
                for index, target in enumerate(targets):
                    started = node.get_clock().now().nanoseconds
                    log.write(json.dumps(dict(event='target_started', index=index, repeat=repeat,
                                              target_m=target.tolist(), ros_time_ns=started))+'\n')
                    log.flush()
                    end = time.monotonic()+args.dwell_s
                    next_publish = 0.0
                    while time.monotonic() < end:
                        rclpy.spin_once(node, timeout_sec=0.02)
                        if not gate_open():
                            raise RuntimeError(f'dry-run gate closed: {state["reason"] or "status stale"}')
                        if time.monotonic() >= next_publish:
                            message = PointStamped()
                            message.header.frame_id = args.frame_id
                            message.header.stamp = node.get_clock().now().to_msg()
                            message.point.x, message.point.y, message.point.z = (float(v) for v in target)
                            publisher.publish(message)
                            next_publish = time.monotonic()+1.0/args.rate_hz
                    log.write(json.dumps(dict(event='target_ended', index=index, repeat=repeat,
                                              ros_time_ns=node.get_clock().now().nanoseconds))+'\n')
                    log.flush()
        except KeyboardInterrupt:
            stop_reason = 'interrupted'
        except RuntimeError as error:
            stop_reason = str(error)
        log.write(json.dumps(dict(event='stopped', reason=stop_reason,
                                  ros_time_ns=node.get_clock().now().nanoseconds))+'\n')
    print(json.dumps(dict(stage='dry_run_targets_done', stop_reason=stop_reason)), flush=True)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0 if stop_reason == 'complete' else 2


if __name__ == '__main__':
    raise SystemExit(main())
