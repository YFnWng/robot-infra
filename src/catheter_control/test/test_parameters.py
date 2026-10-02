from catheter_control.orchestration.configuration import (
    ACTIVE_CONTROLLER_PARAMETERS,
)
from catheter_control.orchestration.parameters import declare_parameters


class RecordingNode:
    def __init__(self):
        self.declarations = []

    def declare_parameter(self, name, default):
        assert name not in {item[0] for item in self.declarations}
        self.declarations.append((name, default))


def test_controller_parameter_surface_is_complete_and_stable(monkeypatch):
    monkeypatch.setenv("CR_META_LNN_ROOT", "/tmp/model-root")
    monkeypatch.setenv("CR_COMMON_ROOT", "/tmp/common-root")
    node = RecordingNode()

    declare_parameters(node, source_name="catheter_mppi")

    names = [name for name, _ in node.declarations]
    values = dict(node.declarations)
    assert len(names) == 130
    assert names[:6] == [
        "source_name",
        "command_output_enabled",
        "simulation_state_reset_enabled",
        "frame_id",
        "cr_meta_lnn_root",
        "cr_common_root",
    ]
    assert names[-2:] == ["marker_topic", "marker_diagnostic_topic"]
    assert set(ACTIVE_CONTROLLER_PARAMETERS) <= set(names)
    assert values["source_name"] == "catheter_mppi"
    assert values["command_output_enabled"] is False
    assert values["cr_meta_lnn_root"] == "/tmp/model-root"
    assert values["cr_common_root"] == "/tmp/common-root"
    assert values["v171_distal_checkpoint"].startswith(
        "/tmp/model-root/artifacts/deployed/")
    assert values["marker_estimator"] == "gauss_newton"
    assert values["samples"] == 32
    assert values["horizon_steps"] == 4
    assert values["maximum_planner_deadline_misses"] == 3
    assert values["marker_topic"] == "/shape_tracking/markers"
    assert values["marker_diagnostic_topic"] == (
        "/shape_tracking/marker_status")
