import json

import numpy as np
import pytest

from automation.estimation.protocol import (
    ShapeConfigError,
    coil_points_mm_to_positions,
    filtered_rx_index,
    fit_straight_segment_mm,
    parse_shape_config,
    stream_is_stale,
)


def _device(device_id, rigid, names, locations, **overrides):
    payload = {
        "id": device_id,
        "rigid": rigid,
        "length_mm": 160.0,
        "n_sections": 1 if rigid else 16,
        "coils": [
            {"transform": name, "arc_length_mm": location}
            for name, location in zip(names, locations)
        ],
    }
    payload.update(overrides)
    return payload


def _payload(**overrides):
    payload = {
        "schema_version": 2,
        "enabled": True,
        "devices": [
            _device(
                "catheter", False,
                ["RX1_filtered", "RX2_filtered", "RX3_filtered"],
                [30.0, 70.0, 120.0],
            ),
            _device(
                "sheath", True,
                ["RX4_filtered", "RX5_filtered"],
                [20.0, 80.0],
                length_mm=100.0,
                n_sections=1,
            ),
        ],
    }
    payload.update(overrides)
    return json.dumps(payload)


def test_two_devices_keep_named_coils_sorted_by_arc_length():
    config = parse_shape_config(_payload())
    catheter, sheath = config.devices
    assert catheter.device_id == "catheter"
    assert catheter.transform_names == (
        "RX1_filtered", "RX2_filtered", "RX3_filtered"
    )
    assert catheter.coil_node_indices == (3, 7, 12)
    assert catheter.local_coil_indices == (0, 4, 9)
    assert not catheter.rigid
    assert sheath.rigid
    assert sheath.output_device_name == "sheath_shape"


def test_coil_names_follow_locations_when_input_is_unsorted():
    device = _device(
        "catheter", True,
        ["tip", "base", "middle"], [120.0, 20.0, 70.0],
    )
    config = parse_shape_config(_payload(devices=[device]))
    assert config.devices[0].transform_names == ("base", "middle", "tip")
    assert config.devices[0].coil_locations_m == pytest.approx(
        [0.02, 0.07, 0.12]
    )


@pytest.mark.parametrize(
    "devices",
    [
        [],
        [_device("bad id", True, ["a", "b"], [10, 20])],
        [_device("abcdefghijklmnop", True, ["a", "b"], [10, 20])],
        [_device("one", True, ["a"], [10])],
        [_device("one", True, ["a", "b"], [-1, 20])],
        [_device("one", True, ["a", "b"], [10, 170])],
        [_device("one", True, ["a", "b"], [10, 10])],
        [_device("one", True, ["a", "b"], [10, 20], n_sections=2)],
        [_device("one", False, ["a", "b"], [10, 20], n_sections=1)],
        [_device("one", False, ["a", "b"], [10, 11], n_sections=2)],
    ],
)
def test_invalid_device_configurations_are_rejected(devices):
    with pytest.raises(ShapeConfigError):
        parse_shape_config(_payload(devices=devices))


def test_duplicate_device_or_transform_assignments_are_rejected():
    first = _device("catheter", True, ["a", "b"], [10, 20])
    duplicate_id = _device("catheter", True, ["c", "d"], [10, 20])
    duplicate_transform = _device("sheath", True, ["b", "c"], [10, 20])
    with pytest.raises(ShapeConfigError, match="device ids"):
        parse_shape_config(_payload(devices=[first, duplicate_id]))
    with pytest.raises(ShapeConfigError, match="only one device"):
        parse_shape_config(_payload(devices=[first, duplicate_transform]))


def test_status_reports_both_backends_and_outputs():
    status = parse_shape_config(_payload()).status_payload()
    assert status["schema_version"] == 2
    assert status["state"] == "configured"
    assert [item["backend"] for item in status["devices"]] == [
        "continuum", "rigid_line"
    ]
    assert [item["output_device"] for item in status["devices"]] == [
        "catheter_shape", "sheath_shape"
    ]


def test_zero_coil_device_is_valid_and_reported_inactive():
    catheter = _device(
        "catheter", True,
        ["RX1_filtered", "RX2_filtered"], [20.0, 80.0],
    )
    sheath = _device("sheath", True, [], [], length_mm=120.0)
    config = parse_shape_config(_payload(devices=[catheter, sheath]))

    inactive = config.devices[1]
    assert inactive.transform_names == ()
    assert inactive.coil_node_indices == ()
    status = inactive.status_payload()
    assert status["backend"] == "inactive"
    assert status["proximal_node_idx"] is None


def test_enable_flag_and_legacy_schema_are_supported():
    disabled = parse_shape_config(_payload(enabled=False))
    assert not disabled.enabled
    assert disabled.status_payload()["state"] == "disabled"
    with pytest.raises(ShapeConfigError, match="enabled must be boolean"):
        parse_shape_config(_payload(enabled=1))

    legacy = parse_shape_config(json.dumps({
        "schema_version": 1,
        "rod_length_mm": 160.0,
        "n_sections": 16,
        "coil_locations_mm": [30.0, 70.0, 120.0],
    }))
    assert len(legacy.devices) == 1
    assert legacy.devices[0].transform_names == (
        "RX1_filtered", "RX2_filtered", "RX3_filtered"
    )


def test_rigid_line_fit_recovers_origin_direction_and_full_length():
    arclengths = [20.0, 50.0, 90.0]
    origin = np.array([10.0, -5.0, 2.0])
    direction = np.array([0.0, 0.6, 0.8])
    points = np.array([
        origin + direction * arc for arc in arclengths
    ])
    fitted = fit_straight_segment_mm(points, arclengths, 120.0, 12)
    assert fitted.shape == (13, 3)
    np.testing.assert_allclose(fitted[0], origin, atol=1.0e-10)
    np.testing.assert_allclose(
        fitted[-1], origin + direction * 120.0, atol=1.0e-10
    )


def test_rigid_line_fit_uses_known_arclength_direction():
    points = [(10, 0, 0), (0, 0, 0)]
    fitted = fit_straight_segment_mm(points, [0, 10], 20, 2)
    np.testing.assert_allclose(fitted[:, 0], [10, 0, -10])


def test_filtered_rx_transform_names_remain_strict_for_legacy_users():
    assert filtered_rx_index("RX1_filtered") == 1
    assert filtered_rx_index("RX12_filtered") == 12
    assert filtered_rx_index("RX1") is None
    assert filtered_rx_index("RX0_filtered") is None


def test_point_frame_converts_mm_to_m_and_checks_count():
    positions = coil_points_mm_to_positions(
        [(10, 20, 30), (40, 50, 60)], (0, 4)
    )
    assert positions[0].tolist() == pytest.approx([0.01, 0.02, 0.03])
    assert positions[4].tolist() == pytest.approx([0.04, 0.05, 0.06])
    with pytest.raises(ShapeConfigError, match="expected 2 coils"):
        coil_points_mm_to_positions([(1, 2, 3)], (0, 4))


def test_stale_transition():
    assert not stream_is_stale(None, 10.0, 0.5)
    assert not stream_is_stale(10.0, 10.5, 0.5)
    assert stream_is_stale(10.0, 10.5001, 0.5)
