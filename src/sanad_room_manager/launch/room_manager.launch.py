from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="false")

    room_manager_node = Node(
        package="sanad_room_manager",
        executable="room_manager_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    room_nav_node = Node(
        package="sanad_room_manager",
        executable="room_nav",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription([use_sim_time_arg, room_manager_node, room_nav_node])
