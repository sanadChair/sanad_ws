#!/usr/bin/env python3
"""Launch file for sanad_eye_tracking: starts both the eye tracker and the
gaze controller with parameters loaded from the YAML config."""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory("sanad_eye_tracking")
    params_file = os.path.join(pkg_share, "config", "eye_tracking_params.yaml")

    eye_tracker_node = Node(
        package="sanad_eye_tracking",
        executable="eye_tracker",
        name="eye_tracker",
        parameters=[params_file],
        output="screen",
    )

    gaze_controller_node = Node(
        package="sanad_eye_tracking",
        executable="gaze_controller",
        name="gaze_controller",
        parameters=[params_file],
        output="screen",
    )

    return LaunchDescription([
        eye_tracker_node,
        gaze_controller_node,
    ])
