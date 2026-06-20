"""EM coil ROS2 sensor adapter (stub).

When the EM tracker driver is available it will publish coil poses on a
ROS2 topic.  This adapter subscribes to that topic and converts the
messages to ``SensorReadings`` that the estimator can consume.

Current status: stub.  Fill in topic name, message type, and conversion
logic once the hardware driver is integrated.
"""
from __future__ import annotations

# import rclpy
# from geometry_msgs.msg import PoseArray
# from state_estimation.sensors.base import SensorReadings


class EMCoilROSAdapter:
    """ROS2 adapter that subscribes to an EM tracker topic.

    Parameters
    ----------
    node : rclpy.node.Node
        Parent ROS2 node (for creating subscriptions).
    topic : str
        Topic name publishing coil poses (e.g. ``/em_tracker/poses``).
    frame_to_node : dict[int, int]
        Mapping from coil index (in the EM tracker frame) to estimation
        node index.
    """

    def __init__(self, node, topic: str, frame_to_node: dict) -> None:
        raise NotImplementedError(
            "EMCoilROSAdapter is not yet implemented. "
            "Define topic, message type, and conversion when hardware driver is ready."
        )
