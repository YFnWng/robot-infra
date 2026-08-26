import json

import pytest
import rclpy
from ros2_igtl_bridge import msg as bridge_messages

from automation.estimation.node import (
    CONFIG_DEVICE,
    StateEstimationNode,
)


class _CapturePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def _rigid_device(device_id, names, length):
    return {
        "id": device_id,
        "rigid": True,
        "length_mm": length,
        "n_sections": 1,
        "coils": [
            {"transform": names[0], "arc_length_mm": 20.0},
            {"transform": names[1], "arc_length_mm": 80.0},
        ],
    }


def _inactive_device(device_id, length):
    return {
        "id": device_id,
        "rigid": True,
        "length_mm": length,
        "n_sections": 1,
        "coils": [],
    }


def _transform(name, xyz):
    message = bridge_messages.Transform()
    message.name = name
    message.transform.translation.x = float(xyz[0])
    message.transform.translation.y = float(xyz[1])
    message.transform.translation.z = float(xyz[2])
    return message


def test_two_rigid_devices_publish_independent_named_shapes_without_gtsam():
    rclpy.init()
    node = StateEstimationNode()
    shapes = _CapturePublisher()
    statuses = _CapturePublisher()
    node._shape_pub = shapes
    node._status_pub = statuses
    try:
        config = bridge_messages.String()
        config.name = CONFIG_DEVICE
        config.data = json.dumps({
            "schema_version": 2,
            "enabled": True,
            "devices": [
                _rigid_device(
                    "catheter", ("CatheterA", "CatheterB"), 100.0
                ),
                _rigid_device(
                    "sheath", ("SheathA", "SheathB"), 120.0
                ),
            ],
        })
        node._config_cb(config)

        node._coil_transform_cb(_transform("CatheterA", (20, 0, 0)))
        node._coil_transform_cb(_transform("SheathA", (0, 20, 0)))
        assert not shapes.messages

        node._coil_transform_cb(_transform("CatheterB", (80, 0, 0)))
        node._coil_transform_cb(_transform("SheathB", (0, 80, 0)))

        assert [message.name for message in shapes.messages] == [
            "catheter_shape",
            "sheath_shape",
        ]
        assert all(len(message.pointdata) == 2 for message in shapes.messages)
        catheter = shapes.messages[0].pointdata
        assert (catheter[0].x, catheter[-1].x) == pytest.approx((0, 100))
        sheath = shapes.messages[1].pointdata
        assert (sheath[0].y, sheath[-1].y) == pytest.approx((0, 120))
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_zero_coil_sheath_does_not_gate_catheter_or_publish_shape():
    rclpy.init()
    node = StateEstimationNode()
    shapes = _CapturePublisher()
    statuses = _CapturePublisher()
    node._shape_pub = shapes
    node._status_pub = statuses
    try:
        config = bridge_messages.String()
        config.name = CONFIG_DEVICE
        config.data = json.dumps({
            "schema_version": 2,
            "enabled": True,
            "devices": [
                _rigid_device(
                    "catheter", ("RX1_filtered", "RX2_filtered"), 100.0
                ),
                _inactive_device("sheath", 120.0),
            ],
        })
        node._config_cb(config)

        assert node._runtimes["sheath"].estimator is None
        assert node._runtimes["sheath"].config.transform_names == ()
        node._coil_transform_cb(_transform("RX1_filtered", (20, 0, 0)))
        node._coil_transform_cb(_transform("RX2_filtered", (80, 0, 0)))

        assert [message.name for message in shapes.messages] == [
            "catheter_shape"
        ]
    finally:
        node.destroy_node()
        rclpy.shutdown()
