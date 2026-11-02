from rclpy.qos import DurabilityPolicy, ReliabilityPolicy

from control_interface_py.position_stream_probe import command_trace_subscription_qos


def test_trace_subscription_matches_best_effort_volatile_publisher():
    qos = command_trace_subscription_qos()
    assert qos.reliability == ReliabilityPolicy.BEST_EFFORT
    assert qos.durability == DurabilityPolicy.VOLATILE
    assert qos.depth == 100
