"""Wire configuration and geometry helpers for live shape estimation."""
from __future__ import annotations

from dataclasses import dataclass
import json
import math
import re
from typing import Any

import numpy as np


class ShapeConfigError(ValueError):
    """Raised when a Slicer shape-estimation configuration is invalid."""


_DEVICE_ID_PATTERN = re.compile(r"^[A-Za-z][A-Za-z0-9_-]*$")
_FILTERED_RX_PATTERN = re.compile(r"^RX([1-9][0-9]*)_filtered$")


@dataclass(frozen=True)
class CoilConfig:
    transform_name: str
    arc_length_m: float


@dataclass(frozen=True)
class DeviceConfig:
    """Validated configuration for one catheter-like device."""

    device_id: str
    rigid: bool
    length_m: float
    n_sections: int
    coils: tuple[CoilConfig, ...]
    coil_node_indices: tuple[int, ...]

    @property
    def transform_names(self) -> tuple[str, ...]:
        return tuple(coil.transform_name for coil in self.coils)

    @property
    def coil_locations_m(self) -> tuple[float, ...]:
        return tuple(coil.arc_length_m for coil in self.coils)

    @property
    def proximal_node_idx(self) -> int:
        if not self.coil_node_indices:
            raise ShapeConfigError(
                f"{self.device_id} has no coils and no proximal estimator node"
            )
        return self.coil_node_indices[0]

    @property
    def local_coil_indices(self) -> tuple[int, ...]:
        prox = self.proximal_node_idx
        return tuple(index - prox for index in self.coil_node_indices)

    @property
    def effective_locations_mm(self) -> list[float]:
        if self.rigid:
            return [value * 1000.0 for value in self.coil_locations_m]
        ds = self.length_m / self.n_sections
        return [index * ds * 1000.0 for index in self.coil_node_indices]

    @property
    def output_device_name(self) -> str:
        # OpenIGTLink device names are limited to 20 bytes. Keep this explicit
        # name short so Slicer updates the node registered by the renderer
        # instead of creating a second node under a silently truncated name.
        return f"{self.device_id}_shape"

    def status_payload(self) -> dict[str, Any]:
        return {
            "device_id": self.device_id,
            "backend": (
                "inactive"
                if not self.coils
                else "rigid_line" if self.rigid else "continuum"
            ),
            "length_mm": self.length_m * 1000.0,
            "n_sections": self.n_sections,
            "coils": [
                {
                    "transform": coil.transform_name,
                    "arc_length_mm": coil.arc_length_m * 1000.0,
                }
                for coil in self.coils
            ],
            "effective_locations_mm": self.effective_locations_mm,
            "coil_node_indices": list(self.coil_node_indices),
            "proximal_node_idx": (
                self.proximal_node_idx if self.coils else None
            ),
            "output_device": self.output_device_name,
        }


@dataclass(frozen=True)
class ShapeConfig:
    devices: tuple[DeviceConfig, ...]
    enabled: bool = True

    def status_payload(self) -> dict[str, Any]:
        return {
            "schema_version": 2,
            "state": "configured" if self.enabled else "disabled",
            "message": "" if self.enabled else "Shape estimation disabled",
            "devices": [device.status_payload() for device in self.devices],
        }


def parse_shape_config(data: str) -> ShapeConfig:
    """Parse schema v2, accepting the former single-device schema v1."""
    try:
        raw = json.loads(data)
    except (TypeError, json.JSONDecodeError) as exc:
        raise ShapeConfigError(f"invalid JSON: {exc}") from exc
    if not isinstance(raw, dict):
        raise ShapeConfigError("configuration must be a JSON object")

    version = raw.get("schema_version")
    if version == 1:
        raw = _upgrade_v1(raw)
    elif version != 2:
        raise ShapeConfigError("schema_version must be 1 or 2")

    enabled = raw.get("enabled", True)
    if not isinstance(enabled, bool):
        raise ShapeConfigError("enabled must be boolean")
    devices_raw = raw.get("devices")
    if not isinstance(devices_raw, list) or not devices_raw:
        raise ShapeConfigError("devices must contain at least one device")

    devices = tuple(
        _parse_device(device_raw, index)
        for index, device_raw in enumerate(devices_raw)
    )
    device_ids = [device.device_id for device in devices]
    if len(set(device_ids)) != len(device_ids):
        raise ShapeConfigError("device ids must be unique")
    transform_names = [
        name for device in devices for name in device.transform_names
    ]
    if len(set(transform_names)) != len(transform_names):
        raise ShapeConfigError(
            "each coil transform may be assigned to only one device"
        )
    return ShapeConfig(devices=devices, enabled=enabled)


def _parse_device(raw: Any, device_index: int) -> DeviceConfig:
    field = f"devices[{device_index}]"
    if not isinstance(raw, dict):
        raise ShapeConfigError(f"{field} must be an object")
    device_id = raw.get("id")
    if not isinstance(device_id, str) or not _DEVICE_ID_PATTERN.fullmatch(
        device_id
    ):
        raise ShapeConfigError(
            f"{field}.id must start with a letter and contain only "
            "letters, digits, '_' or '-'"
        )
    if len(f"{device_id}_shape".encode("ascii")) > 20:
        raise ShapeConfigError(
            f"{field}.id is too long for an OpenIGTLink shape device name"
        )
    rigid = raw.get("rigid", False)
    if not isinstance(rigid, bool):
        raise ShapeConfigError(f"{field}.rigid must be boolean")
    length_mm = _finite_number(raw.get("length_mm"), f"{field}.length_mm")
    if length_mm <= 0.0:
        raise ShapeConfigError(f"{field}.length_mm must be positive")
    n_sections = raw.get("n_sections")
    if (
        isinstance(n_sections, bool)
        or not isinstance(n_sections, int)
        or n_sections < 1
    ):
        raise ShapeConfigError(
            f"{field}.n_sections must be a positive integer"
        )
    if rigid and n_sections != 1:
        raise ShapeConfigError(
            f"{field}.n_sections must equal 1 in rigid mode"
        )
    if not rigid and n_sections < 2:
        raise ShapeConfigError(
            f"{field}.n_sections must be an integer >= 2 in continuum mode"
        )

    coils_raw = raw.get("coils")
    if not isinstance(coils_raw, list):
        raise ShapeConfigError(f"{field}.coils must be a list")
    if len(coils_raw) == 1:
        raise ShapeConfigError(
            f"{field}.coils must be empty or contain at least two coils"
        )
    coils = []
    for coil_index, coil_raw in enumerate(coils_raw):
        coil_field = f"{field}.coils[{coil_index}]"
        if not isinstance(coil_raw, dict):
            raise ShapeConfigError(f"{coil_field} must be an object")
        transform = coil_raw.get("transform")
        if not isinstance(transform, str) or not transform.strip():
            raise ShapeConfigError(
                f"{coil_field}.transform must be a non-empty string"
            )
        if transform != transform.strip():
            raise ShapeConfigError(
                f"{coil_field}.transform must not have surrounding whitespace"
            )
        arc_mm = _finite_number(
            coil_raw.get("arc_length_mm"), f"{coil_field}.arc_length_mm"
        )
        coils.append(CoilConfig(transform, arc_mm / 1000.0))
    coils.sort(key=lambda coil: coil.arc_length_m)
    if not coils:
        return DeviceConfig(
            device_id=device_id,
            rigid=rigid,
            length_m=length_mm / 1000.0,
            n_sections=n_sections,
            coils=(),
            coil_node_indices=(),
        )
    locations_mm = [coil.arc_length_m * 1000.0 for coil in coils]
    if locations_mm[0] < 0.0 or locations_mm[-1] > length_mm:
        raise ShapeConfigError(
            f"{field} coil locations must lie within the device length"
        )
    if len(set(locations_mm)) != len(locations_mm):
        raise ShapeConfigError(f"{field} coil locations must be distinct")
    local_names = [coil.transform_name for coil in coils]
    if len(set(local_names)) != len(local_names):
        raise ShapeConfigError(f"{field} coil transform names must be unique")

    section_length_mm = length_mm / n_sections
    nodes = tuple(
        min(
            n_sections,
            max(
                0,
                int(math.floor(
                    coil.arc_length_m * 1000.0 / section_length_mm + 0.5
                )),
            ),
        )
        for coil in coils
    )
    if not rigid and len(set(nodes)) != len(nodes):
        raise ShapeConfigError(
            f"{field} has multiple coils on one continuum estimator node; "
            "increase n_sections or separate the coil locations"
        )
    return DeviceConfig(
        device_id=device_id,
        rigid=rigid,
        length_m=length_mm / 1000.0,
        n_sections=n_sections,
        coils=tuple(coils),
        coil_node_indices=nodes,
    )


def _upgrade_v1(raw: dict[str, Any]) -> dict[str, Any]:
    locations = raw.get("coil_locations_mm")
    count = len(locations) if isinstance(locations, list) else 0
    return {
        "schema_version": 2,
        "enabled": raw.get("enabled", True),
        "devices": [{
            "id": "catheter",
            "rigid": False,
            "length_mm": raw.get("rod_length_mm"),
            "n_sections": raw.get("n_sections"),
            "coils": [
                {
                    "transform": f"RX{index + 1}_filtered",
                    "arc_length_mm": location,
                }
                for index, location in enumerate(locations or [])
            ],
        }] if count else [],
    }


def fit_straight_segment_mm(
    points_mm,
    arc_lengths_mm,
    segment_length_mm: float,
    n_sections: int,
) -> np.ndarray:
    """Fit p(s)=origin+direction*s and sample the complete rigid segment."""
    points = np.asarray(points_mm, dtype=float)
    arclengths = np.asarray(arc_lengths_mm, dtype=float)
    if points.ndim != 2 or points.shape[1:] != (3,):
        raise ShapeConfigError("coil points must have shape (N, 3)")
    if points.shape[0] != arclengths.size or arclengths.size < 2:
        raise ShapeConfigError(
            "straight-line fitting requires matching data for at least "
            "two coils"
        )
    if not np.all(np.isfinite(points)) or not np.all(np.isfinite(arclengths)):
        raise ShapeConfigError("coil points and arc lengths must be finite")
    centered_s = arclengths - arclengths.mean()
    denominator = float(centered_s @ centered_s)
    if denominator <= 1.0e-12:
        raise ShapeConfigError("straight-line coil locations must be distinct")
    center = points.mean(axis=0)
    slope = np.sum(centered_s[:, None] * (points - center), axis=0)
    slope /= denominator
    norm = float(np.linalg.norm(slope))
    if norm <= 1.0e-12:
        raise ShapeConfigError("coil positions do not define a line direction")
    direction = slope / norm
    origin = center - direction * arclengths.mean()
    samples = np.linspace(0.0, float(segment_length_mm), int(n_sections) + 1)
    return origin[None, :] + samples[:, None] * direction[None, :]


def filtered_rx_index(device_name: str) -> int | None:
    """Return the one-based RX index for an exact filtered transform name."""
    match = _FILTERED_RX_PATTERN.fullmatch(str(device_name))
    if match is None:
        return None
    return int(match.group(1))


def coil_points_mm_to_positions(
    points, local_indices
) -> dict[int, np.ndarray]:
    """Convert an ordered OpenIGTLink point frame from mm to SI metres."""
    if len(points) != len(local_indices):
        raise ShapeConfigError(
            f"expected {len(local_indices)} coils, received {len(points)}"
        )
    return {
        int(node): np.asarray(point, dtype=float).reshape(3) / 1000.0
        for node, point in zip(local_indices, points)
    }


def stream_is_stale(last_frame_time, now, timeout_sec: float) -> bool:
    """Return true only after a previously received frame exceeds timeout."""
    return (
        last_frame_time is not None
        and float(now) - float(last_frame_time) > float(timeout_sec)
    )


def _finite_number(value: Any, field: str) -> float:
    if isinstance(value, bool):
        raise ShapeConfigError(f"{field} must be numeric")
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ShapeConfigError(f"{field} must be numeric") from exc
    if not math.isfinite(result):
        raise ShapeConfigError(f"{field} must be finite")
    return result
