from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="sanad_voice_control",
                executable="commander",
                name="llm_commander",
                output="screen",
                emulate_tty=True,  # This is crucial for `input()` to work in a launch file
            )
        ]
    )
