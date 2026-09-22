from launch import LaunchDescription
from launch.actions import LogInfo


def generate_launch_description():
    return LaunchDescription(
        [
            LogInfo(
                msg=(
                    "HSL26 real-hardware profile scaffold: "
                    "Kobuki and Livox launch wiring is pending hardware validation."
                )
            )
        ]
    )
