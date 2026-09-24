# ros_ws/src/hsl_safety/launch/safety.launch.py
"""Launch the supervisor and watchdog as separate ROS processes."""


def generate_launch_description():
    """Return the isolated supervisor/watchdog launch description."""

    try:
        from launch import LaunchDescription
        from launch_ros.actions import Node
    except ImportError as exc:
        raise RuntimeError("ROS 2 launch dependencies are unavailable") from exc

    return LaunchDescription(
        [
            Node(
                package="hsl_safety",
                executable="safety_supervisor",
                name="safety_supervisor",
                output="screen",
            ),
            Node(
                package="hsl_safety",
                executable="safety_watchdog",
                name="safety_watchdog",
                output="screen",
            ),
        ]
    )
