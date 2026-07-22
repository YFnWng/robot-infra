"""Wire configuration and node mapping for live coil shape estimation."""
from __future__ import annotations

from dataclasses import dataclass
import json
import math
from typing import Any

import numpy as np


class ShapeConfigError(ValueError):
    """Raised when a Slicer shape-estimation configuration is invalid."""


@dataclass(frozen=True)
class ShapeConfig:
    """Validated configuration with coil indices on the full rod grid."""

    rod_length_m: float
    n_sections: int
    coil_locations_m: tuple[float, ...]
    coil_node_indices: tuple[int, ...]

    @property
    def proximal_node_idx(self) -> int:
        return self.coil_node_indices[0]

    @property
    def local_coil_indices(self) -> tuple[int, ...]:
        prox = self.proximal_node_idx
        return tuple(index - prox for index in self.coil_node_indices)

    @property
    def effective_locations_mm(self) -> list[float]:
        ds = self.rod_length_m / self.n_sections
        return [index * ds * 1000.0 for index in self.coil_node_indices]

    def status_payload(self) -> dict[str, Any]:
        return {
            "schema_version": 1,
            "state": "configured",
            "rod_length_mm": self.rod_length_m * 1000.0,
            "n_sections": self.n_sections,
            "coil_locations_mm": [value * 1000.0 for value in self.coil_locations_m],
            "effective_locations_mm": self.effective_locations_mm,
            "coil_node_indices": list(self.coil_node_indices),
            "proximal_node_idx": self.proximal_node_idx,
        }


def parse_shape_config(data: str) -> ShapeConfig:
    """Parse and validate the versioned JSON configuration from Slicer."""
    try:
        raw = json.loads(data)
    except (TypeError, json.JSONDecodeError) as exc:
        raise ShapeConfigError(f"invalid JSON: {exc}") from exc
    if not isinstance(raw, dict):
        raise ShapeConfigError("configuration must be a JSON object")
    if raw.get("schema_version") != 1:
        raise ShapeConfigError("schema_version must be 1")

    length_mm = _finite_number(raw.get("rod_length_mm"), "rod_length_mm")
    if length_mm <= 0.0:
        raise ShapeConfigError("rod_length_mm must be positive")

    n_sections = raw.get("n_sections")
    if isinstance(n_sections, bool) or not isinstance(n_sections, int) or n_sections < 2:
        raise ShapeConfigError("n_sections must be an integer >= 2")

    locations_raw = raw.get("coil_locations_mm")
    if not isinstance(locations_raw, list) or len(locations_raw) < 2:
        raise ShapeConfigError("coil_locations_mm must contain at least two values")
    locations_mm = sorted(
        _finite_number(value, f"coil_locations_mm[{index}]")
        for index, value in enumerate(locations_raw)
    )
    if locations_mm[0] < 0.0 or locations_mm[-1] > length_mm:
        raise ShapeConfigError("coil locations must lie within the catheter length")

    section_length_mm = length_mm / n_sections
    nodes = tuple(
        min(n_sections, max(0, int(math.floor(value / section_length_mm + 0.5))))
        for value in locations_mm
    )
    if len(set(nodes)) != len(nodes):
        raise ShapeConfigError(
            "multiple coils map to the same estimator node; increase n_sections "
            "or separate the coil locations"
        )

    return ShapeConfig(
        rod_length_m=length_mm / 1000.0,
        n_sections=n_sections,
        coil_locations_m=tuple(value / 1000.0 for value in locations_mm),
        coil_node_indices=nodes,
    )


def coil_points_mm_to_positions(points, local_indices) -> dict[int, np.ndarray]:
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
