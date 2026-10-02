# ros_ws/src/hsl_safety/launch/safety.launch.py
"""Launch the supervisor and watchdog as separate ROS processes."""


def generate_launch_description():
    """Return the isolated supervisor/watchdog launch description."""

    try:
        from launch import LaunchDescription
        from launch.actions import DeclareLaunchArgument
        from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
        from launch_ros.actions import Node
        from launch_ros.substitutions import FindPackageShare
    except ImportError as exc:
        raise RuntimeError("ROS 2 launch dependencies are unavailable") from exc

    config_hash = LaunchConfiguration("config_hash")
    stage_id = LaunchConfiguration("stage_id")
    no_motion_config = PathJoinSubstitution(
        [FindPackageShare("hsl_safety"), "config", "r2_no_motion.yaml"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_hash",
                default_value="UNCONFIGURED_NO_MOTION",
                description="R2 sentinel; this launch never grants motion.",
            ),
            DeclareLaunchArgument("stage_id", default_value="r2-disarmed"),
            Node(
                package="hsl_safety",
                executable="safety_supervisor",
                name="safety_supervisor",
                output="screen",
                parameters=[no_motion_config, {"config_hash": config_hash, "stage_id": stage_id}],
            ),
            Node(
                package="hsl_safety",
                executable="safety_watchdog",
                name="safety_watchdog",
                output="screen",
                parameters=[no_motion_config, {"config_hash": config_hash, "stage_id": stage_id}],
            ),
        ]
    )
