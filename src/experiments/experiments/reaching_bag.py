"""Source-time reaching evidence from finalized ROS bags; no node startup."""
import math

import numpy as np


MAX_GAP_NS = 250_000_000


def reversals(samples, deadbands, velocities=False):
    """Count direction changes confirmed by two samples; reset across gaps."""
    counts = [0] * len(deadbands)
    if len(samples) < 2:
        return None
    for axis, deadband in enumerate(deadbands):
        direction = pending = persistence = 0
        anchor = samples[0][1][axis]
        previous_time = samples[0][0]
        for stamp, values in (samples if velocities else samples[1:]):
            value = values[axis]
            if stamp - previous_time > MAX_GAP_NS:
                direction = pending = persistence = 0
                anchor = value
            previous_time = stamp
            delta = value if velocities else value - anchor
            if abs(delta) <= deadband:
                continue
            sign = 1 if delta > 0 else -1
            if not velocities:
                anchor = value
            persistence = persistence + 1 if sign == pending else 1
            pending = sign
            if persistence >= 2 and sign != direction:
                counts[axis] += int(direction != 0)
                direction = sign
    return counts


def window_metrics(streams, start, end, target):
    if end < start:
        raise ValueError("ROS clock moved backwards within trial")
    windows = {name: sorted((stamp, value) for stamp, value in samples
                           if start <= stamp <= end)
               for name, samples in streams.items()}
    evidence = {}
    for name, samples in windows.items():
        times = [stamp for stamp, _ in samples]
        gaps = np.diff([start, *times, end]) if times else [end - start]
        evidence[name] = {"samples": len(times),
                          "maximum_gap_ms": float(max(gaps, default=0) / 1e6),
                          "coverage_ok": bool(times) and max(gaps) <= MAX_GAP_NS}
    markers = windows.get("markers", [])
    errors = [1000 * math.dist(value, target) for _, value in markers]
    positions = windows.get("positions", [])
    travel = None
    if len(positions) > 1:
        # Never integrate across missing spans.
        travel = [sum(abs(b[1][axis] - a[1][axis])
                      for a, b in zip(positions, positions[1:])
                      if b[0] - a[0] <= MAX_GAP_NS) for axis in range(3)]
    result = {
        "maximum_error_mm": max(errors) if errors else None,
        "rms_error_mm": math.sqrt(np.mean(np.square(errors))) if errors else None,
        "bag_final_error_mm": errors[-1] if markers and end - markers[-1][0] <= MAX_GAP_NS else None,
        "command_reversals": reversals(windows.get("commands", []), [.02, .2, .02], True),
        "planned_reversals": reversals(windows.get("planned", []), [.02, .2, .02], True),
        "encoder_reversals": reversals(windows.get("encoders", []), [2, 2, 2]),
        "motor_travel_mm_deg_mm": travel,
        "stream_evidence": evidence,
    }
    return result, windows


def read_streams(bag, config):
    # Use the ROS storage/deserialization API, not a second SQLite/CDR parser.
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from control_tasks.target_offset import measured_tip

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    marker = config.get("marker_topic", "/shape_tracking/markers")
    device = config.get("position_topic", "/device/state")
    expected = {marker: "sensor_msgs/msg/PointCloud",
                device: "control_interface/msg/DeviceStream",
                "/device/command_tx": "control_interface/msg/DeviceStream",
                config.get("planned_control_topic", "/catheter_mppi/planned_control"):
                    "control_interface/msg/ControlStream"}
    streams = {name: [] for name in ("markers", "positions", "encoders", "commands", "planned")}
    issues = []
    types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    classes = {}
    for topic, expected_type in expected.items():
        if types.get(topic) != expected_type:
            issues.append(f"missing/wrong topic type: {topic}; expected {expected_type}")
        else:
            classes[topic] = get_message(expected_type)
    while reader.has_next():
        topic, data, receipt_ns = reader.read_next()
        if topic not in classes:
            continue
        message = deserialize_message(data, classes[topic])
        stamp = int(message.header.stamp.sec) * 1_000_000_000 + message.header.stamp.nanosec
        if stamp <= 0:
            issues.append(f"invalid source stamp: {topic}")
            continue
        if stamp > receipt_ns:
            issues.append(f"source stamp later than bag receipt: {topic}")
        try:
            if topic == marker:
                values = measured_tip(message, config.get("frame_id", "robot_base"))
                kind = "markers"
            elif topic == device:
                kind = {ord("P"): "positions", ord("E"): "encoders"}.get(message.predicate)
                values = list(message.data)[:3]
            elif topic == "/device/command_tx":
                kind = "commands" if message.predicate == ord("V") else None
                values = list(message.data)[:3]
            else:
                kind = "planned"
                values = list(message.joint_vel)[:3]
            if kind is not None:
                if len(values) != 3 or not all(math.isfinite(value) for value in values):
                    raise ValueError("invalid XYZ/axis values")
                if streams[kind] and stamp < streams[kind][-1][0]:
                    issues.append(f"non-monotonic source timestamps: {topic}")
                streams[kind].append((stamp, values))
        except ValueError as exc:
            issues.append(f"{topic}: {exc}")
    return streams, sorted(set(issues))


def write_trial_plot(path, windows, target, start):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figure, axes = plt.subplots(3, 1, figsize=(9, 8), constrained_layout=True)
    for name in ("markers", "positions", "commands"):
        samples = windows.get(name, [])
        panel = axes[("markers", "positions", "commands").index(name)]
        if samples:
            times = [(stamp - start) / 1e9 for stamp, _ in samples]
            values = np.asarray([value for _, value in samples])
            # NaNs break plot lines at gaps rather than inventing trajectories.
            gaps = np.flatnonzero(np.diff(times) > MAX_GAP_NS / 1e9) + 1
            values = values.copy()
            values[gaps] = np.nan
            for axis in range(3):
                panel.plot(times, values[:, axis] * (1000 if name == "markers" else 1),
                           label=("x", "y", "z")[axis] if name == "markers" else f"axis {axis}")
        else:
            panel.text(.5, .5, "No recorded samples in trial window",
                       ha="center", va="center", transform=panel.transAxes)
        if name == "markers":
            for axis, value in enumerate(target):
                panel.axhline(value * 1000, linestyle="--", alpha=.5,
                              color=f"C{axis}")
        panel.set_ylabel({"markers": "Marker XYZ (mm)", "positions": "Motor position (mm/deg/mm)",
                         "commands": "Transmitted velocity (mm/s, deg/s)"}[name])
        if samples:
            panel.legend(loc="upper right")
        panel.grid(alpha=.2)
    axes[-1].set_xlabel("Time since trial start (s)")
    figure.savefig(path)
    plt.close(figure)
