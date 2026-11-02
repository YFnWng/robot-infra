"""Reduced real-time filter observer. No control publisher, client, or arm service.

Subscriptions copy raw encoder counts and markers into one ordered bounded inbox. One owner timer holds all
estimator state and feeds it through the cr_meta_lnn stream wrapper. Only a private String status is published;
no output reaches a planner or controller.
"""
import argparse
from collections import Counter, deque
import json
from pathlib import Path
import threading
import time

import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud
from std_msgs.msg import String

from catheter_control.orchestration.estimator_owner import marker_measurement, stamp_ns
from catheter_control.safety.hardware_contract import ENCODER_RADIANS_PER_COUNT
from control_interface.msg import DeviceStream
from control_interface_py.encoder_coordinates import counts_to_logical

DEFAULT_BASE = '/media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions'


def encoder_sample(counts):
    """Raw six-axis counts to measured shaft radians and logical tendon mm."""
    counts = np.asarray(counts, float)
    if counts.shape != (6,) or not np.isfinite(counts).all():
        raise ValueError('six finite encoder counts required')
    return counts[:3]*ENCODER_RADIANS_PER_COUNT, float(counts_to_logical(counts[None])[0, 2])


class ShadowInbox:
    """Ordered handoff from subscriptions to the owner; overflow latches and is never silent."""

    def __init__(self, maximum_events=2048):
        self.events = deque()
        self.maximum_events = maximum_events
        self.overflowed = False
        self.lock = threading.Lock()

    def put(self, event):
        with self.lock:
            if len(self.events) >= self.maximum_events:
                self.overflowed = True
                return False
            self.events.append(event)
            return True

    def drain(self):
        with self.lock:
            events = list(self.events)
            self.events.clear()
            return events, self.overflowed


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
            qos_profile_sensor_data, callback_group=self.input_group)
        self.create_subscription(PointCloud, prefix+'/shape_tracking/markers', self.marker_callback,
            qos_profile_sensor_data, callback_group=self.input_group)
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
            stamp, points, _ = marker_measurement(message, 'robot_base')
        except ValueError as error:
            self.counts['invalid_marker:'+str(error)] += 1
            return
        if not self.inbox.put(('marker', stamp, points*1000, arrival)):
            self.counts['inbox_overflow_marker'] += 1

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
                    self.counts['images_received'] += 1
                    if aligner.add_image(stamp, values):
                        self.image_arrival[stamp] = arrival
            except ValueError as error:
                self.counts[f'invalid_{kind}:{error}'] += 1
        result = self.stream.process()
        if result is None:
            return
        finished = time.monotonic_ns()
        arrival = self.image_arrival.pop(result.source_ns, None)
        self.counts['status_'+result.status] += 1
        self.rows.append(dict(source_ns=result.source_ns, status=result.status, reason=result.reason,
            step_ms=result.seconds*1e3, owner_tick_ms=(finished-started)*1e-6,
            arrival_to_output_ms=None if arrival is None else (finished-arrival)*1e-6,
            events_drained=len(events),
            position_mm=None if result.position_mm is None else result.position_mm.tolist(),
            lambda_value=result.lambda_value, probability_engaged=result.probability_engaged,
            wall_probability=result.wall_probability, consecutive_rejections=result.consecutive_rejections))

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
    parser.add_argument('--minimum-image-period-ms', type=float, default=100.)
    parser.add_argument('--maximum-pending-images', type=int, default=3,
        help='Images queued while the owner is busy; beyond this the oldest is dropped and counted')
    parser.add_argument('--duration-s', type=float, default=60.)
    parser.add_argument('--stop-after-images', type=int, default=0)
    parser.add_argument('--outdir', required=True, type=Path)
    args, ros_args = parser.parse_known_args(argv)
    if not 0 < args.duration_s <= 3600:
        raise ValueError('bounded duration must be in (0,3600] seconds')
    if args.outdir.exists():
        raise ValueError('use a new report directory')
    from cr_meta_lnn.training.hybrid.reduced_filter_stream import load_stream
    stream = load_stream(args.base, minimum_image_period_ns=int(args.minimum_image_period_ms*1e6),
        maximum_pending_images=args.maximum_pending_images)
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
    finally:
        executor.shutdown()
        args.outdir.mkdir(parents=True)
        (args.outdir/'report.json').write_text(json.dumps(dict(node.summary(), stop_reason=stop_reason,
            minimum_image_period_ms=args.minimum_image_period_ms, rows=node.rows,
            artifacts=stream.artifacts.resources))+'\n')
        print(json.dumps(dict(stage='shadow_done', stop_reason=stop_reason, images_output=len(node.rows))), flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
