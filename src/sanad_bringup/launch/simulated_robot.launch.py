import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    from launch.actions import DeclareLaunchArgument
    from launch.substitutions import LaunchConfiguration

    world_arg = DeclareLaunchArgument(
        name="world",
        default_value="small_house",
        description="Name of world to load (without .world extension)",
    )

    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("sanad_description"),
            "launch",
            "gazebo.launch.py",
        ),
        launch_arguments={"world": LaunchConfiguration("world")}.items(),
    )

    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("sanad_controller"),
            "launch",
            "controller.launch.py",
        )
    )
    rviz = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("sanad_description"),
            "launch",
            "display.launch.py",
        )
    )

    return LaunchDescription(
        [
            world_arg,
            gazebo,
            controller,
            rviz,
        ]
    )
