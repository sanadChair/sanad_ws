import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_slam = LaunchConfiguration("use_slam")
    enable_rosbridge = LaunchConfiguration("enable_rosbridge")

    use_slam_arg = DeclareLaunchArgument("use_slam", default_value="false")
    world_arg = DeclareLaunchArgument(
        name="world",
        default_value="small_house",
        description="Name of world to load (without .world extension)",
    )
    enable_rosbridge_arg = DeclareLaunchArgument(
        "enable_rosbridge",
        default_value="true",
        description="Start rosbridge websocket for remote ROS access.",
    )
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

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("sanad_description"),
                "launch",
                "gazebo.launch.py",
            )
        ),
        launch_arguments={"world": LaunchConfiguration("world")}.items(),
    )

    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("sanad_controller"),
                "launch",
                "controller.launch.py",
            )
        )
    )
    # safety_stop = Node(
    #     package="bumperbot_utils",
    #     executable="safety_stop",
    #     output="screen",
    #     parameters=[{"use_sim_time": True}]
    # )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("sanad_localization"),
                "launch",
                "global_localization.launch.py",
            )
        ),
        condition=UnlessCondition(use_slam),
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("sanad_mapping"),
                "launch",
                "slam.launch.py",
            )
        ),
        condition=IfCondition(use_slam),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("sanad_navigation"),
                "launch",
                "navigation.launch.py",
            )
        )
    )

    room_manager = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("sanad_room_manager"),
                "launch",
                "room_manager.launch.py",
            )
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    rosbridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("sanad_bringup"),
                "launch",
                "rosbridge.launch.py",
            )
        ),
        launch_arguments={
            "rosbridge_host": LaunchConfiguration("rosbridge_host"),
            "rosbridge_port": LaunchConfiguration("rosbridge_port"),
            "enable_rosapi": LaunchConfiguration("enable_rosapi"),
        }.items(),
        condition=IfCondition(enable_rosbridge),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            os.path.join(
                get_package_share_directory("nav2_bringup"),
                "rviz",
                "nav2_default_view.rviz",
            ),
        ],
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription(
        [
            use_slam_arg,
            world_arg,
            enable_rosbridge_arg,
            rosbridge_host_arg,
            rosbridge_port_arg,
            enable_rosapi_arg,
            gazebo,
            controller,
            # safety_stop,
            localization,
            slam,
            navigation,
            room_manager,
            rosbridge,
            rviz,
        ]
    )
