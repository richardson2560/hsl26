from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bag = LaunchConfiguration("bag")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "bag",
                description="Path to a ROS 2 bag prepared for HSL26 replay.",
            ),
            LogInfo(msg=["HSL26 replay profile scaffold; bag=", bag]),
        ]
    )
