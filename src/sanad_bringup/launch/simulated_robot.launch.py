import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_slam = LaunchConfiguration("use_slam")

    use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value="false"
    )
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
    # safety_stop = Node(
    #     package="bumperbot_utils",
    #     executable="safety_stop",
    #     output="screen",
    #     parameters=[{"use_sim_time": True}]
    # )

    localization = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("sanad_localization"),
            "launch",
            "global_localization.launch.py"
        ),
        condition=UnlessCondition(use_slam)
    )

    slam = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("sanad_mapping"),
            "launch",
            "slam.launch.py"
        ),
        condition=IfCondition(use_slam)
    )

    rviz_localization = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(
                get_package_share_directory("sanad_localization"),
                "rviz",
                "global_localization.rviz"
            )
        ],
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=UnlessCondition(use_slam)
    )

    rviz_slam = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(
                get_package_share_directory("sanad_mapping"),
                "rviz",
                "slam.rviz"
            )
        ],
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(use_slam)
    )
    
    return LaunchDescription([
        use_slam_arg,
        world_arg,
        gazebo,
        controller,
        # safety_stop,
        localization,
        slam,
        rviz_localization,
        rviz_slam
    ])
