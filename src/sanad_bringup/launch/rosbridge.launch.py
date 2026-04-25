from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    enable_rosapi = LaunchConfiguration("enable_rosapi")

    rosbridge_host_arg = DeclareLaunchArgument(
        "rosbridge_host",
        default_value="0.0.0.0",
        description="Address rosbridge binds to.",
    )
    rosbridge_port_arg = DeclareLaunchArgument(
        "rosbridge_port",
        default_value="9090",
        description="TCP port for rosbridge websocket.",
    )
    enable_rosapi_arg = DeclareLaunchArgument(
        "enable_rosapi",
        default_value="true",
        description="Start rosapi alongside rosbridge.",
    )

    rosbridge = Node(
        package="rosbridge_server",
        executable="rosbridge_websocket",
        output="screen",
        parameters=[
            {
                "address": LaunchConfiguration("rosbridge_host"),
                "port": ParameterValue(
                    LaunchConfiguration("rosbridge_port"), value_type=int
                ),
            }
        ],
    )

    rosapi = Node(
        package="rosapi",
        executable="rosapi_node",
        name="rosapi",
        output="screen",
        condition=IfCondition(enable_rosapi),
    )

    return LaunchDescription(
        [
            rosbridge_host_arg,
            rosbridge_port_arg,
            enable_rosapi_arg,
            rosbridge,
            rosapi,
        ]
    )
