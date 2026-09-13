import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_slam = LaunchConfiguration("use_slam")
    use_human = LaunchConfiguration("use_human")
    world_config = LaunchConfiguration("world")

    use_slam_arg = DeclareLaunchArgument("use_slam", default_value="false")
    use_human_arg = DeclareLaunchArgument("use_human", default_value="false")

    world_arg = DeclareLaunchArgument(
        name="world",
        default_value="small_house",
        description="Name of world to load (without .world extension)",
    )
    chosen_world = PythonExpression([
        "'small_house_w_human' if '", use_human, "' == 'true' else '", world_config, "'"
    ])

    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("sanad_description"),
            "launch",
            "gazebo.launch.py",
        ),
        launch_arguments={"world": chosen_world}.items(),
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
            "global_localization.launch.py",
        ),
        launch_arguments={"map_name": world_config}.items(),
        condition=UnlessCondition(use_slam),
    )

    joystick = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("sanad_controller"),
            "launch",
            "joystick_teleop.launch.py",
        ),
        launch_arguments={"use_sim_time": "True"}.items(),
    )

    slam = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("sanad_mapping"), "launch", "slam.launch.py"
        ),
        condition=IfCondition(use_slam),
    )
    navigation = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("sanad_navigation"),
            "launch",
            "navigation.launch.py",
        ),
        condition=UnlessCondition(use_human),
    )

    navigation_follow_person = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("sanad_navigation"),
            "launch",
            "navigation_follow_person.launch.py",
        ),
        condition=IfCondition(use_human),
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
            use_human_arg,
            world_arg,
            gazebo,
            controller,
            joystick,
            # safety_stop,
            localization,
            slam,
            navigation,
            navigation_follow_person,
            room_manager,
            rviz,
        ]
    )
