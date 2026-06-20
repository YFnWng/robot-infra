"""MRI coil ROS2 sensor adapter (stub).

Will receive coil position estimates from the MRI reconstruction pipeline
and map them to estimation node indices.

Current status: stub.
"""
from __future__ import annotations


class MRICoilROSAdapter:
    """ROS2 adapter for MRI tracking coils.

    Parameters
    ----------
    node : rclpy.node.Node
        Parent ROS2 node.
    topic : str
        Topic name publishing coil positions from the MRI pipeline.
    frame_to_node : dict[int, int]
        Mapping from coil index to estimation node index.
    """

    def __init__(self, node, topic: str, frame_to_node: dict) -> None:
        raise NotImplementedError(
            "MRICoilROSAdapter is not yet implemented. "
            "Define topic, message type, and conversion when MRI pipeline is ready."
        )
