"""Snapshot effective ROS parameters before a causal hardware experiment."""
from __future__ import annotations

import argparse
import hashlib
from importlib import metadata
import json
import os
from pathlib import Path
import sys

import rclpy
from rcl_interfaces.srv import GetParameters, ListParameters
from rclpy.node import Node
from rclpy.parameter import parameter_value_to_python


ARTIFACT_PARAMETERS = frozenset({"model_manifest", "limits_file"})
RUNTIME_DISTRIBUTIONS = (
    "catheter-control", "cr-meta-lnn", "cr-common",
    "numpy", "torch",
)


def _package_identity():
    result = {}
    for distribution in RUNTIME_DISTRIBUTIONS:
        try:
            result[distribution] = metadata.version(distribution)
        except metadata.PackageNotFoundError:
            result[distribution] = None
    return result


def _manifest_identity(path_value):
    identity = _artifact(path_value)
    path = Path(identity["path"])
    if not path.is_file():
        return identity
    try:
        raw = json.loads(path.read_text(encoding="utf-8"))
        identity.update({
            "schema_version": raw.get("schema_version"),
            "bundle_name": raw.get("bundle_name"),
            "deployment_api_version": raw.get("deployment_api_version"),
            "runtime_family": raw.get("runtime_family"),
            "artifacts": [
                {key: entry.get(key) for key in ("id", "bytes", "sha256")}
                for entry in raw.get("artifacts", [])
                if isinstance(entry, dict)
            ],
        })
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        identity["metadata_error"] = f"{type(exc).__name__}: {exc}"
    return identity


def _artifact(path_value):
    path = Path(str(path_value)).expanduser().resolve()
    result = {"path": str(path), "exists": path.is_file()}
    if path.is_file():
        digest = hashlib.sha256()
        with path.open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
        result.update({
            "bytes": path.stat().st_size,
            "sha256": digest.hexdigest(),
        })
    return result


class RuntimeIdentityCollector(Node):
    def __init__(self):
        super().__init__("causal_runtime_identity")

    def snapshot(self, remote_node: str, timeout_s: float):
        base = "/" + remote_node.strip("/")
        list_client = self.create_client(
            ListParameters, base + "/list_parameters")
        get_client = self.create_client(
            GetParameters, base + "/get_parameters")
        if not (list_client.wait_for_service(timeout_sec=timeout_s)
                and get_client.wait_for_service(timeout_sec=timeout_s)):
            raise RuntimeError("parameter services unavailable")
        request = ListParameters.Request()
        request.prefixes = []
        request.depth = 100
        listed = list_client.call_async(request)
        rclpy.spin_until_future_complete(self, listed, timeout_sec=timeout_s)
        if not listed.done() or listed.result() is None:
            raise RuntimeError("parameter listing timed out")
        names = sorted(listed.result().result.names)
        get_request = GetParameters.Request()
        get_request.names = names
        received = get_client.call_async(get_request)
        rclpy.spin_until_future_complete(self, received, timeout_sec=timeout_s)
        if not received.done() or received.result() is None:
            raise RuntimeError("parameter retrieval timed out")
        values = received.result().values
        parameters = {
            name: parameter_value_to_python(value)
            for name, value in zip(names, values)
        }
        artifacts = {
            name: (_manifest_identity(parameters[name])
                   if name == "model_manifest"
                   else _artifact(parameters[name]))
            for name in ARTIFACT_PARAMETERS
            if parameters.get(name)
        }
        return {
            "parameters": parameters,
            "artifacts": artifacts,
            "installed_packages": _package_identity(),
        }


def _parser():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--node", action="append", default=[])
    parser.add_argument("--required-node", action="append", default=[])
    parser.add_argument("--timeout-s", type=float, default=5.0)
    return parser


def main(args=None):
    parsed = _parser().parse_args(args)
    requested = list(dict.fromkeys(parsed.required_node + parsed.node))
    if not requested:
        raise SystemExit("at least one --required-node or --node is required")
    required = set(parsed.required_node)
    rclpy.init()
    collector = RuntimeIdentityCollector()
    result = {
        "schema_version": 1,
        "captured_at_ns": int(collector.get_clock().now().nanoseconds),
        "nodes": {},
        "errors": {},
    }
    failed_required = False
    try:
        for remote_node in requested:
            try:
                result["nodes"][remote_node] = collector.snapshot(
                    remote_node, parsed.timeout_s)
            except Exception as exc:  # Fail required nodes; retain optional errors.
                result["errors"][remote_node] = f"{type(exc).__name__}: {exc}"
                failed_required |= remote_node in required
    finally:
        collector.destroy_node()
        rclpy.shutdown()

    output = Path(parsed.output).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_suffix(output.suffix + ".tmp")
    with temporary.open("w", encoding="utf-8") as stream:
        json.dump(result, stream, indent=2, sort_keys=True)
        stream.write("\n")
    os.replace(temporary, output)
    if failed_required:
        print(json.dumps(result["errors"], sort_keys=True), file=sys.stderr)
        raise SystemExit(2)


if __name__ == "__main__":
    main()
