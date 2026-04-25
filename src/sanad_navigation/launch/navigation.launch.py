import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
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
        ## ! could cause error
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

    # dual_laser_merger_node = Node(
    #     package="ros2_laser_scan_merger",
    #     executable="ros2_laser_scan_merger",
    #     name="laser_scan_merger",
    #     parameters=[
    #         {
    #             "laser1_topic": "/scan_left_filtered",
    #             "laser2_topic": "/scan_right_filtered",
    #             "destination_frame": "base_link",
    #             "merged_topic": "/scan",
    #             "range_min": 0.1,
    #             "range_max": 30.0,
    #         }
    #     ],
    # )
    lidar_merger = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("ros2_laser_scan_merger"),
            "launch",
            "laser_scan_merger.launch.py",
        ),
    )

    # dual_laser_merger_node = ComposableNodeContainer(
    #     name="demo_container",
    #     namespace="",
    #     package="rclcpp_components",
    #     executable="component_container",
    #     composable_node_descriptions=[
    #         ComposableNode(
    #             package="dual_laser_merger",
    #             plugin="merger_node::MergerNode",
    #             name="dual_laser_merger",
    #             parameters=[
    #                 {"laser_1_topic": "/scan_left_filtered"},
    #                 {"laser_2_topic": "/scan_right_filtered"},
    #                 {"merged_scan_topic": "/scan"},
    #                 {"target_frame": "base_link"},
    #                 {"laser_1_x_offset": 0.0},
    #                 {"laser_1_y_offset": 0.0},
    #                 {"laser_1_yaw_offset": 0.0},
    #                 {"laser_2_x_offset": 0.0},
    #                 {"laser_2_y_offset": 0.0},
    #                 {"laser_2_yaw_offset": 0.0},
    #                 {"tolerance": 0.01},
    #                 {"queue_size": 5},
    #                 {"angle_increment": 0.001},
    #                 {"scan_time": 0.067},
    #                 {"range_min": 0.01},
    #                 {"range_max": 25.0},
    #                 {"min_height": -1.0},
    #                 {"max_height": 1.0},
    #                 {"angle_min": -3.141592654},
    #                 {"angle_max": 3.141592654},
    #                 {"inf_epsilon": 1.0},
    #                 {"use_inf": True},
    #                 {"allowed_radius": 0.45},
    #                 {"enable_shadow_filter": False},
    #                 {"enable_average_filter": False},
    #             ],
    #         )
    #     ],
    #     output="screen",
    # )

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
            # lidar_merger,
            dual_laser_merger_node,
        ]
    )
