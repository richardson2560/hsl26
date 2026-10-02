"""ROS-free unit tests for the R1 graph verifier's policy layer."""

from hsl_bringup.graph_verifier import (
    Endpoint,
    TransformEdge,
    qos_compatible,
    validate_physical_authority,
    validate_static_tf_edges,
    validate_topic_qos,
)


def endpoint(node: str, reliability: str = "reliable", durability: str = "volatile") -> Endpoint:
    return Endpoint(node=node, topic_type="geometry_msgs/msg/Twist", reliability=reliability, durability=durability)


def test_requires_exactly_one_mux_on_the_physical_topic():
    mux = endpoint("/cmd_vel_mux")
    assert validate_physical_authority([mux], topic="/commands/velocity", sole_publisher="/cmd_vel_mux") == []
    assert validate_physical_authority([], topic="/commands/velocity", sole_publisher="/cmd_vel_mux")
    assert validate_physical_authority([mux, endpoint("/legacy_driver")], topic="/commands/velocity", sole_publisher="/cmd_vel_mux")
    assert validate_physical_authority([endpoint("/legacy_driver")], topic="/commands/velocity", sole_publisher="/cmd_vel_mux")


def test_reliable_request_rejects_best_effort_offer():
    assert not qos_compatible(endpoint("/sensor", "best_effort"), endpoint("/fusion", "reliable"))
    assert validate_topic_qos("/sensors/points", [endpoint("/sensor", "best_effort")], [endpoint("/fusion", "reliable")])


def test_transient_local_request_rejects_volatile_offer():
    assert not qos_compatible(endpoint("/stage", durability="volatile"), endpoint("/consumer", durability="transient_local"))


def test_compatible_pairs_are_accepted():
    assert qos_compatible(endpoint("/stage", durability="transient_local"), endpoint("/consumer", durability="volatile"))
    assert qos_compatible(endpoint("/sensor", "best_effort"), endpoint("/viewer", "best_effort"))


def test_static_tf_rejects_duplicate_and_multiple_parent_edges():
    expected = (TransformEdge("base_link", "lidar_link"),)
    assert validate_static_tf_edges(expected, expected) == []
    assert validate_static_tf_edges(expected + expected, expected)
    assert validate_static_tf_edges(
        (TransformEdge("base_link", "lidar_link"), TransformEdge("map", "lidar_link")),
        expected,
    )
