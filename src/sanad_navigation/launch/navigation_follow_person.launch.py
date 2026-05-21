"""
navigation_follow_person.launch.py
-----------------------------------
Drop-in companion to navigation.launch.py for the human-following mode.

Strategy
--------
Every node definition is identical to navigation.launch.py except that:

  1. Each affected Node gets one extra entry appended to its `parameters` list:
       follow_person_overrides.yaml
     ROS2 merges parameter lists in order — later entries win — so the overrides
     silently replace only the values listed in that file. Everything else keeps
     its original value from the standard config YAMLs.

  2. bt_navigator gets `default_nav_to_pose_bt_xml` pointed at follow_person_bt.xml
     instead of simple_navigation_w_replanning_and_recoveries.xml.

The original navigation.launch.py is NOT modified.

Usage
-----
  # normal navigation (unchanged):
  ros2 launch sanad_navigation navigation.launch.py

  # human-following mode:
  ros2 launch sanad_navigation navigation_follow_person.launch.py

  # real hardware (sim time off):
  ros2 launch sanad_navigation navigation_follow_person.launch.py use_sim_time:=false
"""

import os

from ament_index_python.packages import get_package_share_directory

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

    # ── shared path helpers ───────────────────────────────────────────────────
    def cfg(filename):
        return os.path.join(sanad_navigation_pkg, "config", filename)

    def bt(filename):
        return os.path.join(sanad_navigation_pkg, "behavior_tree", filename)

    overrides = cfg("follow_person_overrides.yaml")   # the only new file

    # ── launch args ───────────────────────────────────────────────────────────
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true"   # default false for real hardware
    )

    # ── nav2 nodes ────────────────────────────────────────────────────────────
    # Each node is identical to navigation.launch.py with two differences:
    #   • `overrides` appended to parameters list  (affected nodes only)
    #   • bt_navigator BT XML points to follow_person_bt.xml

    nav2_controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        output="screen",
        parameters=[
            cfg("controller_server.yaml"),
            {"use_sim_time": use_sim_time},
            overrides,                              # ← override: stateful, vel, angle
        ],
    )

    nav2_planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[
            cfg("planner_server.yaml"),
            {"use_sim_time": use_sim_time},
            overrides,                              # ← override: tolerance
        ],
    )

    nav2_smoother_server = Node(
        package="nav2_smoother",
        executable="smoother_server",
        name="smoother_server",
        output="screen",
        parameters=[
            cfg("smoother_server.yaml"),
            {"use_sim_time": use_sim_time},
            # no overrides needed for smoother
        ],
    )

    nav2_bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[
            cfg("bt_navigator.yaml"),
            {"use_sim_time": use_sim_time},
            # ← swap BT to the no-recovery follow person tree
            {"default_nav_to_pose_bt_xml": bt("follow_person_bt.xml")},
            # nav_through_poses keeps the original BT (not used during following)
            {"default_nav_through_poses_bt_xml": bt("simple_navigation_w_replanning.xml")},
            overrides,
        ],
    )

    nav2_behavior = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[
            cfg("behavior_server.yaml"),
            {"use_sim_time": use_sim_time},
            # no overrides — spin/backup/wait stay registered for manual use
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

    # ── sensor nodes — identical to navigation.launch.py ─────────────────────
    laser_left_filter = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        parameters=[
            PathJoinSubstitution([sanad_navigation_pkg, "config", "laser_left_filter.yaml"])
        ],
        remappings=[
            ("scan",          "/scan_left"),
            ("scan_filtered", "/scan_left_filtered"),
        ],
    )

    laser_right_filter = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        parameters=[
            PathJoinSubstitution([sanad_navigation_pkg, "config", "laser_right_filter.yaml"])
        ],
        remappings=[
            ("scan",          "/scan_right"),
            ("scan_filtered", "/scan_right_filtered"),
        ],
    )

    dual_laser_merger_node = Node(
        package="dual_laser_merger",
        executable="dual_laser_merger_node",
        name="dual_laser_merger",
        output="screen",
        parameters=[
            cfg("dual_laser_merger.yaml"),
            {"use_sim_time": use_sim_time},
        ],
    )

    return LaunchDescription([
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
    ])