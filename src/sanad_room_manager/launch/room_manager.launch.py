from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    room_manager_node = Node(
        package="sanad_room_manager",
        executable="room_manager_node",
        output="screen",
    )

    room_nav_node = Node(
        package="sanad_room_manager",
        executable="room_nav",
        output="screen",
    )

    smart_home_device_node = Node(
        package="sanad_room_manager",
        executable="smart_home_device_node",
        output="screen",
    )

    return LaunchDescription([room_manager_node, room_nav_node, smart_home_device_node])
