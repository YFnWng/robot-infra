"""Publish one recorded session's encoder counts and markers at their recorded arrival times.

Test source only: isolated DDS domain and a required topic prefix, with no manager, device link or control topic.
"""
import argparse
import json
import os
from pathlib import Path
import time

from geometry_msgs.msg import Point32
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import ChannelFloat32, PointCloud

from control_interface.msg import DeviceStream

REPLAY_DOMAIN = '214'
REPLAY_PREFIX = '/reduced_filter_replay'
DEFAULT_BASE = '/media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions'
SENSORS = ('gate3_measured_input_m1_20261015T000000Z/engagement_directional_twoaxis_points_grouped_n512_knob_settling_'
           'original_targets_mount01_repeat01_20261008T213127347675Z/sensors.npz')
IMAGES = 'gate3_real_autonomous_inputs_20261023/observations_and_plan.npz'


def require_isolation(domain, prefix):
    if domain != REPLAY_DOMAIN:
        raise RuntimeError(f'recorded replay requires isolated domain {REPLAY_DOMAIN}')
    if prefix != REPLAY_PREFIX:
        raise RuntimeError(f'recorded replay requires topic prefix {REPLAY_PREFIX}')


def replay_events(sensors, image_ns, *, start, stop, encoder_lead_ns=100_000_000):
    """(arrival_ns, is_marker, sensor_index), arrival-ordered, from just before the start image."""
    selected = np.searchsorted(sensors['marker_ns'], image_ns)
    if not np.array_equal(sensors['marker_ns'][selected], image_ns):
        raise ValueError('selected images must be exact recorded markers')
    first = int(image_ns[start])-encoder_lead_ns
    last = int(sensors['marker_receipt_ns'][selected[stop]])+encoder_lead_ns
    events = [(int(sensors['marker_receipt_ns'][k]), 1, int(k)) for k in selected[start:stop+1]]
    events += [(int(t), 0, i) for i, t in enumerate(sensors['encoder_receipt_ns'])
               if first <= sensors['encoder_ns'][i] and t <= last]
    return sorted(events)


def stamp(header, source_ns):
    header.stamp.sec, header.stamp.nanosec = int(source_ns//1_000_000_000), int(source_ns % 1_000_000_000)


def encoder_message(sensors, index):
    message = DeviceStream()
    stamp(message.header, int(sensors['encoder_ns'][index]))
    message.predicate = DeviceStream.ENC
    message.data = [float(value) for value in sensors['encoder_counts'][index]]
    return message


def marker_message(sensors, index):
    message = PointCloud()
    stamp(message.header, int(sensors['marker_ns'][index]))
    message.header.frame_id = 'robot_base'
    message.points = [Point32(x=float(x), y=float(y), z=float(z)) for x, y, z in sensors['points_m'][index]]
    message.channels = [
        ChannelFloat32(name='marker_id', values=[0., 1., 2., 3.]),
        ChannelFloat32(name='confidence', values=[float(v) for v in sensors['marker_confidence'][index]]),
        ChannelFloat32(name='reprojection_error_px', values=[float(v) for v in sensors['marker_reprojection_error_px'][index]]),
        ChannelFloat32(name='source_rig_count', values=[float(v) for v in sensors['marker_source_rig_count'][index]])]
    return message


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--base', default=DEFAULT_BASE, type=Path)
    parser.add_argument('--prefix', default=REPLAY_PREFIX)
    parser.add_argument('--start', type=int, default=3)
    parser.add_argument('--stop', type=int, default=1224)
    parser.add_argument('--discovery-timeout-s', type=float, default=60.)
    parser.add_argument('--report', required=True, type=Path)
    args, ros_args = parser.parse_known_args(argv)
    require_isolation(os.environ.get('ROS_DOMAIN_ID'), args.prefix)
    if args.report.exists():
        raise ValueError('use a new report path')
    sensors = dict(np.load(args.base/SENSORS, allow_pickle=False))
    with np.load(args.base/IMAGES, allow_pickle=False) as images:
        image_ns = images['image_ns'].copy()
    events = replay_events(sensors, image_ns, start=args.start, stop=args.stop)
    rclpy.init(args=ros_args)
    node = rclpy.create_node('reduced_filter_replay', namespace='/experimental_reduced_filter')
    device = node.create_publisher(DeviceStream, args.prefix+'/device/state', qos_profile_sensor_data)
    markers = node.create_publisher(PointCloud, args.prefix+'/shape_tracking/markers', qos_profile_sensor_data)
    deadline = time.monotonic()+args.discovery_timeout_s
    while device.get_subscription_count() < 1 or markers.get_subscription_count() < 1:
        if time.monotonic() > deadline:
            raise RuntimeError('shadow subscriptions not discovered')
        time.sleep(.05)
    time.sleep(.5)
    origin = time.monotonic_ns()-events[0][0]
    lateness = []
    for arrival, is_marker, index in events:
        wait = (origin+arrival-time.monotonic_ns())*1e-9
        if wait > 0:
            time.sleep(wait)
        lateness.append((time.monotonic_ns()-origin-arrival)*1e-6)
        if is_marker:
            markers.publish(marker_message(sensors, index))
        else:
            device.publish(encoder_message(sensors, index))
    args.report.write_text(json.dumps(dict(encoders_sent=sum(1 for e in events if not e[1]),
        images_sent=sum(1 for e in events if e[1]), domain=REPLAY_DOMAIN, prefix=args.prefix,
        publish_lateness_ms=dict(zip(('p50', 'p99', 'max'), map(float, np.percentile(lateness, [50, 99, 100]))))))+'\n')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
