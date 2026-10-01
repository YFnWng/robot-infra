"""Deterministic, fail-closed composition of catheter runtime profiles."""
from dataclasses import dataclass
from pathlib import Path, PurePosixPath
from typing import Any, Mapping

import yaml


SCHEMA_VERSION = 1
PARAMETER_LAYERS = ("controller", "platform", "performance")
REFERENCE_LAYERS = ("experiment", "visualization")
ALLOWED_STACK_KEYS = {
    "schema_version", "name", "node_name", "layers",
    "compatibility_aliases", "description",
}
FORBIDDEN_PARAMETERS = {"command_output_enabled"}
ACTIVE_CONTROLLER_PARAMETERS = frozenset({
    "adaptation_enabled",
    "backlash_width_rad",
    "backlash_width_positive_rad",
    "backlash_width_negative_rad",
    "backlash_takeup_velocity",
    "backlash_engagement_confirmation_observations",
    "backlash_provisional_rejection_observations",
    "backlash_minimum_distal_bending_increment",
    "engaged_gain_minimum",
    "engaged_gain_maximum",
    "engaged_gain_prior_mean",
    "engaged_gain_prior_log_std",
    "engaged_gain_reversal_log_std",
    "engaged_gain_process_log_std_sqrt_s",
    "engaged_gain_observation_std",
    "engaged_gain_minimum_nominal_increment",
    "engaged_gain_huber_sigma",
    "engaged_gain_maximum_normalized_innovation",
    "engaged_gain_contradiction_log_std",
    "engaged_gain_confidence_log_width",
    "engaged_gain_minimum_updates",
    "engaged_gain_credible_sigma",
    "estimator_filter_initial_covariance",
    "estimator_filter_process_std_sqrt_s",
    "estimator_initial_roll_hypotheses",
    "estimator_history_reconciliation_enabled",
    "estimator_history_reconciliation_maximum_shift",
    "horizon_steps",
    "rollout_step_s",
    "mppi_point_rollout_step_s",
    "mppi_point_rollout_coarse_steps",
    "mppi_point_prediction_tail_steps",
    "mppi_point_prediction_tail_step_s",
    "mppi_engaged_gain_risk_beta",
    "mppi_engaged_gain_cvar_alpha",
    "mppi_engaged_gain_maximum_first_step_shift",
    "mppi_engaged_gain_learning_velocity_scale",
    "mppi_capture_radius_mm",
    "mppi_capture_minimum_terminal_improvement_mm",
    "mppi_capture_hold_s",
    "mppi_capture_response_minimum_prediction_mm",
    "mppi_capture_response_minimum_ratio",
    "mppi_takeup_confirmation_time_s",
    "mppi_takeup_limit_reserve_scale",
    "mppi_transmission_aware_rollout",
    "takeup_confirmation_hold_timeout_s",
    "reversal_scheduler_required_plans",
    "reversal_scheduler_minimum_absolute_cost_improvement",
    "reversal_scheduler_minimum_fractional_cost_improvement",
    "reversal_scheduler_minimum_terminal_error_improvement_mm",
    "reversal_scheduler_minimum_accepted_observations",
    "reversal_scheduler_cooldown_s",
    "backlash_compensation_enabled",
    "takeup_transaction_enabled",
    "engaged_gain_enabled",
    "mppi_grouped_mode_sampling",
    "mppi_engaged_gain_scenarios",
    "mppi_best_candidate_guard",
    "mppi_takeup_risk_weight",
    "reversal_scheduler_enabled",
    "marker_estimator",
    "jacobian_initialization_json",
    "interface_transmission_checkpoint",
    "controller_velocity_max",
    "device",
    "samples",
    "planning_deadline_s",
    "estimator_catchup_timeout_s",
    "plan_rate_hz",
    "command_rate_hz",
})


@dataclass(frozen=True)
class ConfigurationLayer:
    kind: str
    path: Path
    parameters: Mapping[str, Any]


@dataclass(frozen=True)
class ResolvedConfiguration:
    name: str
    stack_path: Path
    node_name: str
    layers: tuple[ConfigurationLayer, ...]
    references: Mapping[str, Path]
    parameters: Mapping[str, Any]
    parameter_sources: Mapping[str, str]
    compatibility_aliases: tuple[str, ...]

    def as_manifest(self) -> dict[str, Any]:
        return {
            "schema_version": SCHEMA_VERSION,
            "name": self.name,
            "stack_path": str(self.stack_path),
            "node_name": self.node_name,
            "layers": [
                {
                    "kind": layer.kind,
                    "path": str(layer.path),
                    "parameters": dict(layer.parameters),
                }
                for layer in self.layers
            ],
            "references": {
                key: str(value) for key, value in self.references.items()
            },
            "parameters": dict(self.parameters),
            "parameter_sources": dict(self.parameter_sources),
            "compatibility_aliases": list(self.compatibility_aliases),
        }


def _mapping(value: Any, label: str) -> Mapping[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f"{label} must be a mapping")
    return value


def _inside(base: Path, value: Any, label: str) -> Path:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{label} must be a nonempty relative path")
    relative = PurePosixPath(value)
    if relative.is_absolute() or ".." in relative.parts:
        raise ValueError(f"{label} must stay inside the config directory")
    base = base.resolve()
    # Validate the path lexically before following it. Colcon's
    # ``--symlink-install`` places trusted package-data symlinks below the
    # package share directory whose targets live in the source tree.
    candidate = base / Path(*relative.parts)
    if candidate != base and base not in candidate.parents:
        raise ValueError(f"{label} escapes the config directory")
    if not candidate.is_file():
        raise FileNotFoundError(f"{label} does not exist: {candidate}")
    return candidate.resolve()


def load_ros_parameters(path: str | Path, node_name: str) -> dict[str, Any]:
    """Resolve wildcard then exact ROS parameter selectors."""
    resolved = Path(path).expanduser().resolve()
    content = yaml.safe_load(resolved.read_text(encoding="utf-8")) or {}
    content = _mapping(content, f"ROS parameter profile {resolved}")
    parameters: dict[str, Any] = {}
    for selector in ("/**", node_name, f"/{node_name}"):
        entry = content.get(selector, {})
        if not isinstance(entry, dict):
            raise ValueError(
                f"selector {selector} in {resolved} must be a mapping")
        values = entry.get("ros__parameters", {})
        if not isinstance(values, dict):
            raise ValueError(
                f"ros__parameters for {selector} in {resolved} "
                "must be a mapping")
        parameters.update(values)
    return parameters


def reject_forbidden_parameters(
        parameters: Mapping[str, Any], *, label: str) -> None:
    forbidden = FORBIDDEN_PARAMETERS.intersection(parameters)
    if forbidden:
        names = ", ".join(sorted(forbidden))
        raise ValueError(
            f"{label} declares protected parameter(s) {names}; use the "
            "explicit launch interlock")


def locate_stack(value: str | Path, config_root: str | Path) -> Path:
    """Resolve an explicit stack path or a semantic name under config/stacks."""
    candidate = Path(value).expanduser()
    if candidate.is_file():
        return candidate.resolve()
    root = Path(config_root).expanduser().resolve()
    name = str(value)
    if "/" in name or "\\" in name:
        raise FileNotFoundError(f"stack profile does not exist: {candidate}")
    if not name.endswith(".yaml"):
        name += ".yaml"
    resolved = root / "stacks" / name
    if not resolved.is_file():
        raise FileNotFoundError(f"unknown semantic stack profile: {value}")
    return resolved.resolve()


def resolve_stack(
        path: str | Path, *, config_root: str | Path,
        allowed_parameters: set[str] | None = None,
        expected_node_name: str | None = None) -> ResolvedConfiguration:
    """Load, validate, and deterministically merge one semantic stack."""
    stack_path = Path(path).expanduser().resolve()
    content = yaml.safe_load(stack_path.read_text(encoding="utf-8")) or {}
    content = _mapping(content, f"stack profile {stack_path}")
    unknown = set(content).difference(ALLOWED_STACK_KEYS)
    if unknown:
        raise ValueError(
            f"unknown stack keys: {', '.join(sorted(unknown))}")
    if content.get("schema_version") != SCHEMA_VERSION:
        raise ValueError(
            f"unsupported stack schema_version: "
            f"{content.get('schema_version')!r}")
    name = content.get("name")
    if not isinstance(name, str) or not name.strip():
        raise ValueError("stack name must be a nonempty string")
    node_name = content.get("node_name", "catheter_mppi")
    if not isinstance(node_name, str) or not node_name.strip():
        raise ValueError("stack node_name must be a nonempty string")
    if expected_node_name is not None and node_name != expected_node_name:
        raise ValueError(
            f"stack targets node {node_name!r}, expected "
            f"{expected_node_name!r}")
    layer_values = _mapping(content.get("layers"), "stack layers")
    unknown_layers = set(layer_values).difference(
        PARAMETER_LAYERS + REFERENCE_LAYERS)
    if unknown_layers:
        raise ValueError(
            f"unknown stack layers: {', '.join(sorted(unknown_layers))}")

    root = Path(config_root).expanduser().resolve()
    layers: list[ConfigurationLayer] = []
    merged: dict[str, Any] = {}
    sources: dict[str, str] = {}
    for kind in PARAMETER_LAYERS:
        values = layer_values.get(kind, [])
        if isinstance(values, str):
            values = [values]
        if not isinstance(values, list) or any(
                not isinstance(item, str) for item in values):
            raise ValueError(f"stack layer {kind} must be a path or path list")
        for index, value in enumerate(values):
            layer_path = _inside(root, value, f"{kind}[{index}]")
            parameters = load_ros_parameters(layer_path, node_name)
            reject_forbidden_parameters(
                parameters, label=f"{kind} layer {layer_path}")
            if allowed_parameters is not None:
                unknown_parameters = set(parameters).difference(
                    allowed_parameters)
                if unknown_parameters:
                    raise ValueError(
                        f"{kind} layer {layer_path} contains unknown "
                        f"parameters: "
                        f"{', '.join(sorted(unknown_parameters))}")
            duplicates = set(parameters).intersection(merged)
            if duplicates:
                raise ValueError(
                    f"{kind} layer {layer_path} duplicates parameters: "
                    f"{', '.join(sorted(duplicates))}")
            layers.append(ConfigurationLayer(
                kind=kind, path=layer_path, parameters=parameters))
            merged.update(parameters)
            sources.update({
                key: str(layer_path) for key in parameters
            })

    references = {}
    for kind in REFERENCE_LAYERS:
        value = layer_values.get(kind)
        if value is not None:
            references[kind] = _inside(root, value, kind)

    aliases = content.get("compatibility_aliases", [])
    if not isinstance(aliases, list) or any(
            not isinstance(alias, str) or not alias.strip()
            for alias in aliases):
        raise ValueError(
            "compatibility_aliases must be an array of nonempty strings")
    return ResolvedConfiguration(
        name=name,
        stack_path=stack_path,
        node_name=node_name,
        layers=tuple(layers),
        references=references,
        parameters=merged,
        parameter_sources=sources,
        compatibility_aliases=tuple(aliases),
    )
