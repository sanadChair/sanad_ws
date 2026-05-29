from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    start_room_nav = LaunchConfiguration("start_room_nav")

    room_manager_node = Node(
        package="sanad_room_manager",
        executable="room_manager_node",
        output="screen",
    )

    room_nav_node = Node(
        package="sanad_room_manager",
        executable="room_nav",
        output="screen",
        condition=IfCondition(start_room_nav),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "start_room_nav",
                default_value="false",
                description="Start the Nav2-backed room navigation service.",
            ),
            room_manager_node,
            room_nav_node,
        ]
    )
