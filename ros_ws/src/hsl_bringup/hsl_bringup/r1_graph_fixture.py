"""Deliberately-invalid, zero-motion fixtures for R1 negative graph tests."""

from __future__ import annotations

import argparse
from typing import Sequence


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--scenario",
        required=True,
        choices=("unauthorized_command", "qos_incompatible", "tf_valid", "tf_duplicate"),
    )
    parser.add_argument("--duration-seconds", type=float, default=20.0)
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    if args.duration_seconds <= 0:
        raise ValueError("--duration-seconds must be positive")

    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.qos import QoSProfile, QoSReliabilityPolicy
    from std_msgs.msg import String
    from geometry_msgs.msg import TransformStamped
    from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

    rclpy.init(args=None)
    node = rclpy.create_node(f"r1_{args.scenario}_fixture")
    try:
        if args.scenario == "unauthorized_command":
            # This is intentionally a second writer, but it is zero-only and
            # never runs with a hardware driver or outside an isolated test.
            publisher = node.create_publisher(Twist, "/commands/velocity", 1)
            message = Twist()
        elif args.scenario == "qos_incompatible":
            publisher = node.create_publisher(
                String,
                "/r1/qos_incompatible",
                QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT),
            )
            node.create_subscription(
                String,
                "/r1/qos_incompatible",
                lambda _message: None,
                QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE),
            )
            message = String(data="r1-qos-negative-fixture")
            node.create_timer(0.1, lambda: publisher.publish(message))
        else:
            broadcaster = StaticTransformBroadcaster(node)

            def edge(parent: str, child: str) -> TransformStamped:
                transform = TransformStamped()
                transform.header.frame_id = parent
                transform.child_frame_id = child
                transform.transform.rotation.w = 1.0
                return transform

            transforms = [edge("base_link", "lidar_link"), edge("base_link", "imu_link")]
            if args.scenario == "tf_duplicate":
                transforms.append(edge("base_link", "lidar_link"))
            broadcaster.sendTransform(transforms)
        if args.scenario == "unauthorized_command":
            node.create_timer(0.1, lambda: publisher.publish(message))
        # A spin loop rather than a sleeping process keeps DDS discovery alive.
        end_ns = node.get_clock().now().nanoseconds + int(args.duration_seconds * 1e9)
        while node.get_clock().now().nanoseconds < end_ns:
            rclpy.spin_once(node, timeout_sec=0.1)
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
