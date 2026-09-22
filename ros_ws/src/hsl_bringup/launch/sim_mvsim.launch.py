from launch import LaunchDescription
from launch.actions import LogInfo


def generate_launch_description():
    return LaunchDescription(
        [
            LogInfo(
                msg=(
                    "HSL26 simulation profile scaffold: "
                    "the SimulationAdapter must be connected before starting MVSim."
                )
            )
        ]
    )
