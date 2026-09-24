# ros_ws/src/hsl_bringup/launch/hsl26.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    mode = LaunchConfiguration("mode")
    role = LaunchConfiguration("role")
    bag = LaunchConfiguration("bag")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mode",
                default_value="real",
                choices=["real", "sim", "replay"],
                description="Execution profile; node orchestration is added with each layer implementation.",
            ),
            DeclareLaunchArgument(
                "role",
                default_value="unset",
                choices=["unset", "explorer", "guardian"],
                description="Competition role.",
            ),
            DeclareLaunchArgument(
                "bag",
                default_value="",
                description="Replay bag path used by the replay profile.",
            ),
            LogInfo(msg=["HSL26 bringup scaffold: mode=", mode, ", role=", role, ", bag=", bag]),
        ]
    )
