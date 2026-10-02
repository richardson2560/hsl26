"""Test-only R2-B probe for cmd_vel_mux stop priority.

This executable is never included in a launch file.  It deliberately publishes
a nonzero command to the logical supervisor input and verifies that the mux
physical output remains zero while the watchdog stop is alive.  It must only
run in the isolated Docker test described in ``docker/r2/README.md``.
"""

import argparse
import os
import sys
import time


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration-seconds", type=float, default=3.0)
    parser.add_argument("--linear-x", type=float, default=0.2)
    parser.add_argument("--minimum-samples", type=int, default=5)
    parser.add_argument(
        "--allow-test-nonzero",
        action="store_true",
        help="required acknowledgement for the intentional logical input",
    )
    args = parser.parse_args(argv)
    if not args.allow_test_nonzero:
        parser.error("--allow-test-nonzero is required")
    if args.duration_seconds <= 0.0 or args.minimum_samples <= 0:
        parser.error("duration and minimum samples must be positive")
    if args.linear_x == 0.0:
        parser.error("linear-x must be nonzero to exercise mux priority")
    return args


def main(argv=None):
    """Publish a bounded logical command and fail if physical output is nonzero."""

    args = parse_args(argv)
    if os.environ.get("HSL26_TEST_ONLY") != "1":
        raise RuntimeError(
            "r2_stop_priority_probe is disabled; set HSL26_TEST_ONLY=1 only "
            "inside the isolated R2-B container"
        )
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.node import Node

    rclpy.init(args=None)
    node = Node("r2_stop_priority_probe")
    publisher = node.create_publisher(Twist, "/hsl/cmd_vel_final", 10)
    samples = []

    def observe(message):
        values = (
            message.linear.x,
            message.linear.y,
            message.linear.z,
            message.angular.x,
            message.angular.y,
            message.angular.z,
        )
        samples.append(values)

    node.create_subscription(Twist, "/commands/velocity", observe, 10)
    command = Twist()
    command.linear.x = args.linear_x
    deadline = time.monotonic() + args.duration_seconds
    try:
        while time.monotonic() < deadline:
            publisher.publish(command)
            rclpy.spin_once(node, timeout_sec=0.05)
        if len(samples) < args.minimum_samples:
            raise RuntimeError(
                f"received {len(samples)} physical samples; expected at least "
                f"{args.minimum_samples}"
            )
        nonzero = [sample for sample in samples if any(value != 0.0 for value in sample)]
        if nonzero:
            raise RuntimeError(
                f"FAIL: watchdog stop did not dominate mux output: {nonzero[0]}"
            )
        print(
            f"PASS: {len(samples)} physical samples remained zero while "
            f"logical input linear.x={args.linear_x}"
        )
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
