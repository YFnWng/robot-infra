"""Manifest-selected learned-runtime loading for ROS consumers."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping


LEGACY_MODEL_PARAMETERS = (
    "cr_meta_lnn_root",
    "cr_common_root",
    "v171_distal_checkpoint",
    "jacobian_initialization_json",
    "interface_transmission_checkpoint",
    "distal_tendon_allocation_checkpoint",
)
_SELECTED_MANIFEST_NAME = "20260929_175554_grouped_no_rotation_v2.json"


@dataclass(frozen=True)
class ModelSelection:
    """Resolved manifest selection and whether a launch alias selected it."""

    manifest_path: str
    compatibility_alias: str | None = None


def resolve_model_selection(
        model_manifest: str, *, legacy: Mapping[str, Any] | None = None
) -> ModelSelection:
    """Resolve temporary old launch arguments without restoring path imports.

    Legacy repository-root selection maps only to the one qualified bundle.
    Independently selected checkpoint combinations are rejected: they cannot
    silently claim the qualification represented by the manifest.
    """
    manifest = str(model_manifest).strip()
    values = {name: str(value).strip() for name, value in (legacy or {}).items()
              if str(value).strip()}
    unknown = sorted(set(values) - set(LEGACY_MODEL_PARAMETERS))
    if unknown:
        raise ValueError(f"unknown legacy model selectors: {unknown}")
    if manifest and values:
        raise ValueError(
            "model_manifest cannot be combined with legacy model selectors")
    if manifest:
        return ModelSelection(str(Path(manifest).expanduser().resolve()))
    if not values:
        raise ValueError("model_manifest is required")

    unsupported = sorted(set(values) - {"cr_meta_lnn_root", "cr_common_root"})
    if unsupported:
        raise ValueError(
            "individual checkpoint selection is retired; migrate the reviewed "
            f"bundle to a manifest (legacy selectors: {unsupported})")
    root = values.get("cr_meta_lnn_root", "")
    if not root:
        raise ValueError(
            "legacy cr_common_root cannot select a model without "
            "cr_meta_lnn_root")
    candidate = (Path(root).expanduser().resolve() / "artifacts" / "manifests"
                 / _SELECTED_MANIFEST_NAME)
    return ModelSelection(
        str(candidate), compatibility_alias="cr_meta_lnn_root")


def load_runtime(
        model_manifest: str, *, device: str = "cpu", dtype: str = "float32",
        options: Mapping[str, Any] | None = None):
    """Load the verified deployment bundle through the installed API."""
    from cr_meta_lnn.deployment import load_runtime_bundle

    return load_runtime_bundle(
        model_manifest, device=device, dtype=dtype, options=options)


def load_runtime_pair(model_manifest: str, *, estimator_device: str,
                      planner_device: str, estimator_dtype="float32",
                      planner_dtype="float32", options=None):
    """Reuse identical compute settings; verify split artifacts and precision."""
    if (estimator_dtype not in ("float32", "float64")
            or planner_dtype not in ("float32", "float64")):
        raise ValueError("runtime dtype must be float32 or float64")
    estimator = load_runtime(model_manifest, device=estimator_device,
                             dtype=estimator_dtype, options=options)
    if estimator_device == planner_device and estimator_dtype == planner_dtype:
        return estimator, estimator
    from cr_meta_lnn.deployment import RuntimeState

    if not hasattr(RuntimeState, "clone_to"):
        raise ValueError("split compute requires an updated model package with RuntimeState.clone_to")
    if estimator_dtype != planner_dtype:
        from inspect import signature
        if "dtype" not in signature(RuntimeState.clone_to).parameters:
            raise ValueError("mixed precision requires an updated model package with dtype-aware RuntimeState.clone_to")
    planner = load_runtime(model_manifest, device=planner_device,
                           dtype=planner_dtype, options=options)
    a, b = estimator.identity, planner.identity
    if (a.manifest_sha256 != b.manifest_sha256 or a.artifacts != b.artifacts
            or a.dtype != estimator_dtype or b.dtype != planner_dtype
            or a.runtime_family != b.runtime_family):
        raise ValueError("estimator/planner bundle identity mismatch")
    return estimator, planner
