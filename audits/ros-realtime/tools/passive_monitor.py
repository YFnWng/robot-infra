#!/usr/bin/env python3
"""Bounded, subscription-only runtime monitor for the catheter ROS stack."""

import argparse
from collections import Counter, defaultdict
import json
import math
import os
from pathlib import Path
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import PointCloud

from control_interface.msg import ControlStream, DeviceStream, ManagerEvent


def percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    position = fraction * (len(ordered) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def distribution(values):
    if not values:
        return {"count": 0}
    return {
        "count": len(values),
        "p50": percentile(values, 0.50),
        "p95": percentile(values, 0.95),
        "p99": percentile(values, 0.99),
        "maximum": max(values),
        "minimum": min(values),
        "mean": sum(values) / len(values),
    }


def stamp_ns(message):
    header = getattr(message, "header", None)
    if header is None:
        return None
    stamp = header.stamp
    value = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    return value if value else None


def read_process(pid):
    base = Path("/proc") / str(pid)
    try:
        raw = (base / "stat").read_text()
        tail = raw[raw.rfind(")") + 2:].split()
        status = {}
        for line in (base / "status").read_text().splitlines():
            if ":" in line:
                key, value = line.split(":", 1)
                status[key] = value.strip()
        return {
            "cpu_ticks": int(tail[11]) + int(tail[12]),
            "processor": int(tail[36]),
            "threads": int(status["Threads"]),
            "rss_kib": int(status.get("VmRSS", "0 kB").split()[0]),
            "voluntary_context_switches": int(
                status.get("voluntary_ctxt_switches", "0")),
            "involuntary_context_switches": int(
                status.get("nonvoluntary_ctxt_switches", "0")),
        }
    except (FileNotFoundError, ProcessLookupError, KeyError, ValueError):
        return None


def find_process(fragment):
    """Find one process whose NUL-separated command line contains fragment."""
    matches = []
    own_pid = os.getpid()
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit() or int(entry.name) == own_pid:
            continue
        try:
            command = (entry/"cmdline").read_bytes().replace(b"\0", b" ")
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
        if fragment.encode() in command:
            matches.append((int(entry.name), command.decode(errors="replace")))
    if len(matches) != 1:
        detail = "; ".join(f"{pid}:{command}" for pid, command in matches)
        raise ValueError(
            f"process match {fragment!r} found {len(matches)} processes: "
            f"{detail}")
    return matches[0][0]


class PassiveMonitor(Node):
    def __init__(self):
        super().__init__("catheter_passive_audit_monitor")
        self.arrivals = defaultdict(list)
        self.ages_ms = defaultdict(list)
        self.status_messages = defaultdict(Counter)
        self.status_values = defaultdict(lambda: defaultdict(list))
        self.status_text_values = defaultdict(lambda: defaultdict(Counter))
        self.latest_device_arrival = {}
        self.latest_device_stamp = {}
        self.device_values = defaultdict(list)
        self.device_pair_arrival_skew_ms = []
        self.device_pair_stamp_skew_ms = []

        reliable = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        safety = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            PointCloud, "/shape_tracking/markers",
            lambda msg: self.record("markers", msg), qos_profile_sensor_data)
        self.create_subscription(
            DiagnosticArray, "/shape_tracking/marker_status",
            lambda msg: self.record_diagnostic("marker_status", msg), reliable)
        self.create_subscription(
            DeviceStream, "/device/state", self.record_device,
            qos_profile_sensor_data)
        self.create_subscription(
            DiagnosticArray, "/catheter_mppi/status",
            lambda msg: self.record_diagnostic("mppi_status", msg), reliable)
        self.create_subscription(
            ManagerEvent, "/manager/safety_status",
            lambda msg: self.record("manager_safety", msg), safety)
        self.create_subscription(
            ControlStream, "/teleop/control",
            lambda msg: self.record("teleop_control", msg), reliable)
        self.create_subscription(
            DeviceStream, "/manager/control",
            lambda msg: self.record("manager_control", msg), reliable)

    def record(self, name, message):
        now_mono = time.monotonic_ns()
        self.arrivals[name].append(now_mono)
        source_stamp = stamp_ns(message)
        if source_stamp is not None:
            now_ros = self.get_clock().now().nanoseconds
            self.ages_ms[name].append((now_ros - source_stamp) / 1e6)

    def record_device(self, message):
        if message.predicate == DeviceStream.POS:
            name = "device_pos"
        elif message.predicate == DeviceStream.ENC:
            name = "device_enc"
        else:
            name = f"device_predicate_{message.predicate}"
        self.record(name, message)
        self.device_values[name].append(
            [float(value) for value in message.data])
        now_mono = self.arrivals[name][-1]
        source_stamp = stamp_ns(message)
        self.latest_device_arrival[name] = now_mono
        if source_stamp is not None:
            self.latest_device_stamp[name] = source_stamp
        if name == "device_enc" and "device_pos" in self.latest_device_arrival:
            self.device_pair_arrival_skew_ms.append(abs(
                now_mono - self.latest_device_arrival["device_pos"]) / 1e6)
            if ("device_enc" in self.latest_device_stamp
                    and "device_pos" in self.latest_device_stamp):
                self.device_pair_stamp_skew_ms.append(abs(
                    self.latest_device_stamp["device_enc"]
                    - self.latest_device_stamp["device_pos"]) / 1e6)

    def record_diagnostic(self, name, message):
        self.record(name, message)
        for status in message.status:
            self.status_messages[name][status.message] += 1
            values = {item.key: item.value for item in status.values}
            for key, value in values.items():
                try:
                    number = float(value)
                except (TypeError, ValueError):
                    if (key in {
                            "reason", "estimator_health",
                            "marker_diagnostic", "marker_update_reason"}
                            or key.endswith("_worker_error")):
                        self.status_text_values[name][key][value] += 1
                    continue
                if math.isfinite(number):
                    self.status_values[name][key].append(number)

    def result(self, duration_s):
        topics = {}
        for name in sorted(set(self.arrivals) | {
                "markers", "marker_status", "device_pos", "device_enc",
                "mppi_status", "manager_safety", "teleop_control",
                "manager_control"}):
            arrivals = self.arrivals[name]
            intervals_ms = [
                (right - left) / 1e6
                for left, right in zip(arrivals, arrivals[1:])]
            rate = ((len(arrivals) - 1) /
                    ((arrivals[-1] - arrivals[0]) / 1e9)
                    if len(arrivals) > 1 else 0.0)
            topics[name] = {
                "count": len(arrivals),
                "rate_hz": rate,
                "interarrival_ms": distribution(intervals_ms),
                "header_age_ms": distribution(self.ages_ms[name]),
            }
        diagnostics = {}
        for name in sorted(self.status_messages):
            diagnostics[name] = {
                "messages": dict(self.status_messages[name]),
                "numeric_values": {
                    key: distribution(values)
                    for key, values in sorted(self.status_values[name].items())
                },
                "text_values": {
                    key: dict(values)
                    for key, values in sorted(
                        self.status_text_values[name].items())
                },
            }
        return {
            "duration_s": duration_s,
            "observer": "one subscription-only rclpy node; depth 1",
            "topics": topics,
            "feedback_pair_arrival_skew_ms": distribution(
                self.device_pair_arrival_skew_ms),
            "feedback_pair_header_skew_ms": distribution(
                self.device_pair_stamp_skew_ms),
            "device_axis_values": {
                name: [distribution([sample[axis] for sample in samples])
                       for axis in range(len(samples[0]))]
                for name, samples in sorted(self.device_values.items())
                if samples
            },
            "diagnostics": diagnostics,
        }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--process", action="append", default=[],
                        help="label=pid to sample from /proc")
    parser.add_argument(
        "--process-match", action="append", default=[],
        help="label=unique command-line fragment to resolve from /proc")
    args = parser.parse_args()
    if args.duration <= 0.0:
        raise ValueError("duration must be positive")
    processes = {}
    for item in args.process:
        label, raw_pid = item.split("=", 1)
        processes[label] = int(raw_pid)
    for item in args.process_match:
        label, fragment = item.split("=", 1)
        if label in processes:
            raise ValueError(f"duplicate process label: {label}")
        processes[label] = find_process(fragment)

    rclpy.init()
    node = PassiveMonitor()
    started = time.monotonic()
    process_start = {name: read_process(pid)
                     for name, pid in processes.items()}
    process_samples = defaultdict(list)
    next_process_sample = started
    try:
        while rclpy.ok() and time.monotonic() - started < args.duration:
            rclpy.spin_once(node, timeout_sec=0.02)
            now = time.monotonic()
            if now >= next_process_sample:
                for name, pid in processes.items():
                    sample = read_process(pid)
                    if sample is not None:
                        process_samples[name].append(sample)
                next_process_sample = now + 0.1
    finally:
        elapsed = time.monotonic() - started
        process_end = {name: read_process(pid)
                       for name, pid in processes.items()}
        result = node.result(elapsed)
        ticks_per_second = int(__import__("os").sysconf("SC_CLK_TCK"))
        process_results = {}
        for name in processes:
            first, last = process_start[name], process_end[name]
            samples = process_samples[name]
            if first is None or last is None:
                process_results[name] = {"available": False}
                continue
            process_results[name] = {
                "available": True,
                "cpu_percent_one_core_100": 100.0 * (
                    last["cpu_ticks"] - first["cpu_ticks"]
                ) / ticks_per_second / elapsed,
                "threads": distribution([item["threads"] for item in samples]),
                "rss_kib": distribution([item["rss_kib"] for item in samples]),
                "processors_observed": sorted(set(
                    item["processor"] for item in samples)),
                "voluntary_context_switches_delta": (
                    last["voluntary_context_switches"]
                    - first["voluntary_context_switches"]),
                "involuntary_context_switches_delta": (
                    last["involuntary_context_switches"]
                    - first["involuntary_context_switches"]),
            }
        result["processes"] = process_results
        print(json.dumps(result, indent=2, sort_keys=True))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
