"""Read-only check that the encoders and markers agree at home after a firmware boot.

Each firmware boot restarts the encoder counters at the current pose, while the reduced filter's
profiles and tendon support are absolute in encoder coordinates. So the board must boot at home.
This tool only subscribes: it never commands, claims a mode, re-zeros or calls a service.

Checks (declared defaults): the first three encoder channels within 67 counts of zero (the firmware's
position-completion tolerance) and stationary over the window; optionally every marker within 0.5 mm
of a saved home reference (fresh camera registration alone moved markers 0.2-0.35 mm on 2026-11-02).
"""
import argparse
import json
from pathlib import Path
import time

import numpy as np

ENCODER_TOLERANCE_COUNTS = 67
STATIONARY_COUNTS = 3
MARKER_TOLERANCE_MM = 0.5


def evaluate(encoder_counts, markers_m, reference=None, *, encoder_tolerance=ENCODER_TOLERANCE_COUNTS,
             stationary=STATIONARY_COUNTS, marker_tolerance_mm=MARKER_TOLERANCE_MM):
    """Pure evaluation of one window: encoder counts [N,6] and markers [M,4,3] in metres."""
    counts = np.asarray(encoder_counts, float)
    markers = np.asarray(markers_m, float)
    if counts.ndim != 2 or counts.shape[1] != 6 or len(counts) < 2:
        raise ValueError('at least two six-axis encoder samples required')
    if markers.ndim != 3 or markers.shape[1:] != (4, 3) or len(markers) < 2:
        raise ValueError('at least two four-marker samples required')
    median = np.median(counts, axis=0)
    spread = np.ptp(counts, axis=0)
    result = dict(
        encoder_median_counts=median.tolist(), encoder_spread_counts=spread.tolist(),
        encoder_near_zero=bool(np.all(np.abs(median[:3]) <= encoder_tolerance)),
        encoder_stationary=bool(np.all(spread[:3] <= stationary)),
        marker_mean_m=markers.mean(axis=0).tolist(),
        marker_spread_mm=(np.ptp(markers, axis=0).max(axis=1)*1e3).tolist(),
        tolerances=dict(encoder_counts=encoder_tolerance, stationary_counts=stationary,
                        marker_mm=marker_tolerance_mm))
    result['encoder_pass'] = result['encoder_near_zero'] and result['encoder_stationary']
    if reference is not None:
        offset = (markers.mean(axis=0)-np.asarray(reference['marker_mean_m']))*1e3
        distances = np.linalg.norm(offset, axis=1)
        result.update(marker_offset_mm=offset.tolist(), marker_distance_mm=distances.tolist(),
                      marker_pass=bool(np.all(distances <= marker_tolerance_mm)),
                      reference_label=reference.get('label'))
    result['passed'] = result['encoder_pass'] and result.get('marker_pass', True)
    return result


def main(argv=None):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.utilities import remove_ros_args
    from sensor_msgs.msg import PointCloud

    from catheter_control.orchestration.estimator_owner import encoder_history_qos, marker_measurement
    from control_interface.msg import DeviceStream

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration-s', type=float, default=3.0)
    parser.add_argument('--reference', type=Path, help='saved home reference JSON')
    parser.add_argument('--save-reference', type=Path, help='new path; saved only when the encoder check passes')
    parser.add_argument('--label', default='', help='label stored with a saved reference')
    parser.add_argument('--output', type=Path, help='new report JSON path')
    args = parser.parse_args(remove_ros_args(args=argv)[1:])
    if not 0.5 <= args.duration_s <= 60:
        parser.error('duration must be in [0.5, 60] s')
    for path in (args.save_reference, args.output):
        if path is not None and path.exists():
            parser.error(f'{path} exists; use a new path')
    reference = json.loads(args.reference.read_text()) if args.reference else None
    rclpy.init(args=argv)
    node = rclpy.create_node('home_check')
    encoders, markers = [], []

    def on_device(message):
        if message.predicate == DeviceStream.ENC and len(message.data) == 6:
            encoders.append(list(message.data))

    def on_marker(message):
        try:
            markers.append(marker_measurement(message, 'robot_base')[1])
        except ValueError:
            pass

    node.create_subscription(DeviceStream, '/device/state', on_device, encoder_history_qos())
    node.create_subscription(PointCloud, '/shape_tracking/markers', on_marker, qos_profile_sensor_data)
    end = time.monotonic()+args.duration_s
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_node()
    rclpy.shutdown()
    result = evaluate(encoders, markers, reference)
    result.update(encoder_samples=len(encoders), marker_samples=len(markers), duration_s=args.duration_s)
    if args.save_reference:
        if not result['encoder_pass']:
            raise SystemExit('encoder check failed; reference not saved')
        args.save_reference.write_text(json.dumps(dict(
            label=args.label, saved_ns=time.time_ns(), marker_mean_m=result['marker_mean_m'],
            encoder_median_counts=result['encoder_median_counts']), indent=2)+'\n')
    text = json.dumps(result, indent=2)
    if args.output:
        args.output.write_text(text+'\n')
    print(text)
    return 0 if result['passed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
