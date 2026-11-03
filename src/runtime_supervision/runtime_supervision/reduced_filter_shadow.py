"""Reduced real-time filter observer. No control publisher, client, or arm service.

Subscriptions copy raw encoder counts and markers into one ordered bounded inbox. One owner timer holds all
estimator state and feeds it through the cr_meta_lnn stream wrapper. Only a private String status is published;
no output reaches a planner or controller.
"""
import argparse
from collections import Counter
import json
from pathlib import Path
import subprocess
import time

import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud
from std_msgs.msg import String

from catheter_control.orchestration.estimator_owner import encoder_history_qos, marker_measurement, stamp_ns
from catheter_control.orchestration.reduced_filter_owner import OrderedInbox as ShadowInbox, encoder_sample
from control_interface.msg import DeviceStream

DEFAULT_BASE = '/media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions'


def code_identity(module):
    """Source file and git state of the imported research code, for the report."""
    path = Path(module.__file__).resolve()

    def git(*args):
        try:
            return subprocess.run(['git', '-C', str(path.parent), *args], capture_output=True, text=True,
                                  timeout=5, check=True).stdout.strip()
        except (OSError, subprocess.SubprocessError):
            return None
    status = git('status', '--porcelain')
    return dict(module_file=str(path), commit=git('rev-parse', 'HEAD'),
                dirty=None if status is None else bool(status))


def marker_quality_summary(quality):
    """Per-image marker quality for the report; logged, not used to admit or weight markers."""
    return dict(confidence=[float(v) for v in quality['confidence']],
                reprojection_error_px=[float(v) for v in quality['reprojection_error_px']],
                source_rig_count=[float(v) for v in quality['source_rig_count']])


class ReducedFilterShadow(Node):
    def __init__(self, stream, *, prefix=''):
        super().__init__('reduced_filter_shadow', namespace='/experimental_reduced_filter')
        self.stream = stream
        self.inbox = ShadowInbox()
        self.counts = Counter()
        self.rows = []
        self.image_arrival = {}
        self.input_group = MutuallyExclusiveCallbackGroup()
        self.owner_group = MutuallyExclusiveCallbackGroup()
        self.status_group = MutuallyExclusiveCallbackGroup()
        self.create_subscription(DeviceStream, prefix+'/device/state', self.device_callback,
            encoder_history_qos(), callback_group=self.input_group)
        self.create_subscription(PointCloud, prefix+'/shape_tracking/markers', self.marker_callback,
            qos_profile_sensor_data, callback_group=self.input_group)
        # Observation only: quality and camera diagnostics are logged per image, never used to gate input.
        self.image_quality = {}
        self.marker_diagnostic = None
        self.create_subscription(DiagnosticArray, prefix+'/shape_tracking/marker_status', self.marker_status_callback,
            10, callback_group=self.input_group)
        self.status = self.create_publisher(String, 'status', 10)
        self.create_timer(.005, self.owner_callback, callback_group=self.owner_group)
        self.create_timer(1., self.status_callback, callback_group=self.status_group)

    def device_callback(self, message):
        if message.predicate != DeviceStream.ENC:
            return
        arrival = time.monotonic_ns()
        if not self.inbox.put(('encoder', stamp_ns(message.header), list(message.data), arrival)):
            self.counts['inbox_overflow_encoder'] += 1

    def marker_callback(self, message):
        arrival = time.monotonic_ns()
        try:
            stamp, points, quality = marker_measurement(message, 'robot_base')
        except ValueError as error:
            self.counts['invalid_marker:'+str(error)] += 1
            return
        if not self.inbox.put(('marker', stamp, (points*1000, marker_quality_summary(quality)), arrival)):
            self.counts['inbox_overflow_marker'] += 1

    def marker_status_callback(self, message):
        for status in message.status:
            level = int.from_bytes(status.level, 'little') if isinstance(status.level, bytes) else int(status.level)
            self.marker_diagnostic = dict(level=level, message=str(status.message))
            self.counts[f'marker_diagnostic_level_{level}'] += 1

    def owner_callback(self):
        started = time.monotonic_ns()
        events, overflowed = self.inbox.drain()
        aligner = self.stream.aligner
        if overflowed:
            aligner.history_lost = True
        for kind, stamp, values, arrival in events:
            try:
                if kind == 'encoder':
                    shaft, tendon = encoder_sample(values)
                    aligner.add_encoder(stamp, shaft, tendon)
                    self.counts['encoders_received'] += 1
                else:
                    points, quality = values
                    self.counts['images_received'] += 1
                    if aligner.add_image(stamp, points):
                        self.image_arrival[stamp] = arrival
                        self.image_quality[stamp] = dict(quality, marker_diagnostic=self.marker_diagnostic)
            except ValueError as error:
                self.counts[f'invalid_{kind}:{error}'] += 1
        result = self.stream.process()
        if result is None:
            return
        finished = time.monotonic_ns()
        arrival = self.image_arrival.pop(result.source_ns, None)
        quality = self.image_quality.pop(result.source_ns, None)
        # Images dropped from the aligner's bounded queue never produce a result.
        for stale in [stamp for stamp in self.image_quality if stamp < result.source_ns]:
            self.image_quality.pop(stale)
            self.image_arrival.pop(stale, None)
        self.counts['status_'+result.status] += 1
        self.rows.append(dict(source_ns=result.source_ns, status=result.status, reason=result.reason,
            step_ms=result.seconds*1e3, owner_tick_ms=(finished-started)*1e-6,
            arrival_to_output_ms=None if arrival is None else (finished-arrival)*1e-6,
            events_drained=len(events),
            position_mm=None if result.position_mm is None else result.position_mm.tolist(),
            lambda_value=result.lambda_value, probability_engaged=result.probability_engaged,
            wall_probability=result.wall_probability, consecutive_rejections=result.consecutive_rejections,
            marker_quality=quality))

    def summary(self):
        aligner = self.stream.aligner
        return dict(runtime_family='reduced_realtime_filter_research_shadow', command_capable=False,
            counts=dict(self.counts), images_dropped=aligner.images_dropped, images_skipped=aligner.images_skipped,
            images_pending=len(aligner.pending), history_lost=aligner.history_lost,
            history_samples=len(aligner.history), images_output=len(self.rows))

    def status_callback(self):
        message = String()
        message.data = json.dumps(self.summary())
        self.status.publish(message)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--base', default=DEFAULT_BASE)
    parser.add_argument('--prefix', default='')
    # 90 ms takes every third 30 fps frame (regular 100 ms); a strict 100 ms rule takes every 3rd or 4th
    # (user decision and check, 2026-11-02; GATE3 deployment gap audit, step 2).
    parser.add_argument('--minimum-image-period-ms', type=float, default=90.)
    parser.add_argument('--maximum-pending-images', type=int, default=3,
        help='Images queued while the owner is busy; beyond this the oldest is dropped and counted')
    parser.add_argument('--startup-pending-images', type=int, default=8,
        help='Larger queue until the first tracked image has caught up (initialization takes about 0.45 s); 0 disables')
    parser.add_argument('--duration-s', type=float, default=60.)
    parser.add_argument('--stop-after-images', type=int, default=0)
    parser.add_argument('--outdir', required=True, type=Path)
    args, ros_args = parser.parse_known_args(argv)
    if not 0 < args.duration_s <= 3600:
        raise ValueError('bounded duration must be in (0,3600] seconds')
    if args.outdir.exists():
        raise ValueError('use a new report directory')
    from cr_meta_lnn.training.hybrid import reduced_filter_stream
    from cr_meta_lnn.training.hybrid.reduced_filter_stream import load_stream
    code = code_identity(reduced_filter_stream)
    startup = max(args.startup_pending_images, args.maximum_pending_images) if args.startup_pending_images else None
    stream = load_stream(args.base, minimum_image_period_ns=int(args.minimum_image_period_ms*1e6),
        maximum_pending_images=args.maximum_pending_images, startup_pending_images=startup)
    rclpy.init(args=ros_args)
    node = ReducedFilterShadow(stream, prefix=args.prefix)
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    print(json.dumps(dict(stage='shadow_ready', command_capable=False)), flush=True)
    started = time.monotonic()
    stop_reason = 'duration_limit'
    try:
        while rclpy.ok() and time.monotonic()-started < args.duration_s:
            executor.spin_once(timeout_sec=.1)
            if args.stop_after_images and len(node.rows) >= args.stop_after_images:
                stop_reason = 'images_complete'
                break
    except (KeyboardInterrupt, ExternalShutdownException):
        stop_reason = 'interrupted'
    except RCLError:
        # SIGINT shuts the context down while a worker may be entering its wait set; that is a normal stop.
        if rclpy.ok():
            raise
        stop_reason = 'interrupted'
    finally:
        executor.shutdown()
        args.outdir.mkdir(parents=True)
        (args.outdir/'report.json').write_text(json.dumps(dict(node.summary(), stop_reason=stop_reason,
            minimum_image_period_ms=args.minimum_image_period_ms, maximum_pending_images=args.maximum_pending_images,
            startup_pending_images=startup, rows=node.rows,
            artifacts=stream.artifacts.resources, code=code))+'\n')
        print(json.dumps(dict(stage='shadow_done', stop_reason=stop_reason, images_output=len(node.rows))), flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
