import json

import pytest

from automation.estimation.protocol import (
    ShapeConfigError,
    coil_points_mm_to_positions,
    filtered_rx_index,
    parse_shape_config,
    stream_is_stale,
)


def _payload(**overrides):
    payload = {
        "schema_version": 1,
        "rod_length_mm": 160.0,
        "n_sections": 16,
        "coil_locations_mm": [30.0, 70.0, 120.0],
    }
    payload.update(overrides)
    return json.dumps(payload)


def test_mapping_sorts_and_uses_proximal_coil_as_local_zero():
    config = parse_shape_config(_payload(coil_locations_mm=[120, 30, 70]))
    assert config.coil_node_indices == (3, 7, 12)
    assert config.local_coil_indices == (0, 4, 9)
    assert config.proximal_node_idx == 3
    assert config.effective_locations_mm == pytest.approx([30, 70, 120])


@pytest.mark.parametrize(
    "overrides",
    [
        {"schema_version": 2},
        {"rod_length_mm": 0},
        {"n_sections": 1},
        {"coil_locations_mm": [10]},
        {"coil_locations_mm": [-1, 10]},
        {"coil_locations_mm": [10, 170]},
        {"coil_locations_mm": [10, 11]},
    ],
)
def test_invalid_configurations_are_rejected(overrides):
    with pytest.raises(ShapeConfigError):
        parse_shape_config(_payload(**overrides))


def test_status_reports_effective_mapping():
    status = parse_shape_config(_payload()).status_payload()
    assert status["state"] == "configured"
    assert status["proximal_node_idx"] == 3
    assert status["coil_node_indices"] == [3, 7, 12]


def test_estimation_enable_flag_defaults_true_and_accepts_false():
    assert parse_shape_config(_payload()).enabled
    disabled = parse_shape_config(_payload(enabled=False))
    assert not disabled.enabled
    assert disabled.status_payload()["state"] == "disabled"
    with pytest.raises(ShapeConfigError, match="enabled must be boolean"):
        parse_shape_config(_payload(enabled=1))


def test_filtered_rx_transform_names_are_strict_and_one_based():
    assert filtered_rx_index("RX1_filtered") == 1
    assert filtered_rx_index("RX12_filtered") == 12
    assert filtered_rx_index("RX1") is None
    assert filtered_rx_index("RX0_filtered") is None
    assert filtered_rx_index("RX1_filtered_1") is None


def test_point_frame_converts_mm_to_m_and_checks_count():
    positions = coil_points_mm_to_positions(
        [(10, 20, 30), (40, 50, 60)], (0, 4)
    )
    assert positions[0].tolist() == pytest.approx([0.01, 0.02, 0.03])
    assert positions[4].tolist() == pytest.approx([0.04, 0.05, 0.06])
    with pytest.raises(ShapeConfigError, match="expected 2 coils"):
        coil_points_mm_to_positions([(1, 2, 3)], (0, 4))


def test_stale_transition_and_reconfiguration():
    assert not stream_is_stale(None, 10.0, 0.5)
    assert not stream_is_stale(10.0, 10.5, 0.5)
    assert stream_is_stale(10.0, 10.5001, 0.5)
    first = parse_shape_config(_payload())
    second = parse_shape_config(_payload(n_sections=32))
    assert first.coil_node_indices != second.coil_node_indices
