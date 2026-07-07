from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    start_room_nav = LaunchConfiguration("start_room_nav")

    ir_serial_port_arg = DeclareLaunchArgument(
        "ir_serial_port",
        default_value="/dev/ttyUSB0",
        description="Serial port used by the IR receiver",
    )
    ir_baud_rate_arg = DeclareLaunchArgument(
        "ir_baud_rate",
        default_value="9600",
        description="Baud rate used by the IR receiver",
    )

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

    ir_manager_node = Node(
        package="sanad_room_manager",
        executable="ir_manager_node",
        output="screen",
        parameters=[
            {
                "serial_port": LaunchConfiguration("ir_serial_port"),
                "baud_rate": ParameterValue(
                    LaunchConfiguration("ir_baud_rate"),
                    value_type=int,
                ),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "start_room_nav",
                default_value="false",
                description="Start the Nav2-backed room navigation service.",
            ),
            ir_serial_port_arg,
            ir_baud_rate_arg,
            room_manager_node,
            room_nav_node,
            ir_manager_node,
        ]
    )
