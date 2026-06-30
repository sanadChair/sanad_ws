import os

from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    lifecycle_nodes = [
        "controller_server",
        "planner_server",
        "smoother_server",
        "bt_navigator",
        "behavior_server",
    ]
    sanad_navigation_pkg = get_package_share_directory("sanad_navigation")

    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="true")

    nav2_controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        output="screen",
        parameters=[
            os.path.join(sanad_navigation_pkg, "config", "controller_server.yaml"),
            {"use_sim_time": use_sim_time},
        ],
    )

    nav2_planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[
            os.path.join(sanad_navigation_pkg, "config", "planner_server.yaml"),
            {"use_sim_time": use_sim_time},
        ],
    )

    nav2_smoother_server = Node(
        package="nav2_smoother",
        executable="smoother_server",
        name="smoother_server",
        output="screen",
        parameters=[
            os.path.join(sanad_navigation_pkg, "config", "smoother_server.yaml"),
            {"use_sim_time": use_sim_time},
        ],
    )

    nav2_bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[
            os.path.join(sanad_navigation_pkg, "config", "bt_navigator.yaml"),
            {"use_sim_time": use_sim_time},
            {
                "default_nav_to_pose_bt_xml": os.path.join(
                    sanad_navigation_pkg,
                    "behavior_tree",
                    "simple_navigation_w_replanning_and_recoveries.xml",
                )
            },
            {
                "default_nav_through_poses_bt_xml": os.path.join(
                    sanad_navigation_pkg,
                    "behavior_tree",
                    "simple_navigation_w_replanning.xml",
                )
            },
        ],
    )

    nav2_behavior = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[
            os.path.join(sanad_navigation_pkg, "config", "behavior_server.yaml"),
            {"use_sim_time": use_sim_time},
        ],
    )

    nav2_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[
            {"node_names": lifecycle_nodes},
            {"use_sim_time": use_sim_time},
            {"autostart": True},
        ],
    )
    laser_left_filter = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        parameters=[
            PathJoinSubstitution(
                [
                    get_package_share_directory("sanad_navigation"),
                    "config",
                    "laser_left_filter.yaml",
                ]
            )
        ],
        remappings=[
            ("scan", "/scan_left"),
            ("scan_filtered", "/scan_left_filtered"),
        ],
    )

    laser_right_filter = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        parameters=[
            PathJoinSubstitution(
                [
                    get_package_share_directory("sanad_navigation"),
                    "config",
                    "laser_right_filter.yaml",
                ]
            )
        ],
        remappings=[
            ("scan", "/scan_right"),  # input
            ("scan_filtered", "/scan_right_filtered"),  # output
        ],
    )


    dual_laser_merger_node = Node(
        package="dual_laser_merger",
        executable="dual_laser_merger_node",  # may be "merger_node" depending on build
        name="dual_laser_merger",
        output="screen",
        parameters=[
            os.path.join(sanad_navigation_pkg, "config", "dual_laser_merger.yaml"),
            {"use_sim_time": use_sim_time},
        ],
    )


    return LaunchDescription(
        [
            use_sim_time_arg,
            nav2_controller_server,
            nav2_planner_server,
            nav2_smoother_server,
            nav2_bt_navigator,
            nav2_behavior,
            nav2_lifecycle_manager,
            laser_left_filter,
            laser_right_filter,
            dual_laser_merger_node,
        ]
    )
