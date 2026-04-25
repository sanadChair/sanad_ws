# merge_scans.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            # Convert scan_front → PointCloud2
            Node(
                package="pointcloud_to_laserscan",
                executable="laserscan_to_pointcloud_node",
                name="scan_left_to_cloud",
                remappings=[
                    ("scan_in", "/scan_left_filtered"),
                    ("cloud", "/cloud_left"),
                ],
                parameters=[{"target_frame": "base_link"}],
            ),
            # Convert scan_rear → PointCloud2
            Node(
                package="pointcloud_to_laserscan",
                executable="laserscan_to_pointcloud_node",
                name="scan_right_to_cloud",
                remappings=[
                    ("scan_in", "/scan_right_filtered"),
                    ("cloud", "/cloud_rear"),
                ],
                parameters=[{"target_frame": "base_link"}],
            ),
            # Merge the two clouds
            Node(
                package="pcl_ros",
                executable="pointcloud_concatenate_data_synchronizer",
                name="cloud_merger",
                parameters=[
                    {
                        "input_topics": ["/cloud_front", "/cloud_rear"],
                        "output_frame": "base_link",
                    }
                ],
                remappings=[("output", "/cloud_merged")],
            ),
            # Convert merged cloud → LaserScan
            Node(
                package="pointcloud_to_laserscan",
                executable="pointcloud_to_laserscan_node",
                name="cloud_to_scan",
                remappings=[
                    ("cloud_in", "/cloud_merged"),
                    ("scan", "/scan"),
                ],
                parameters=[
                    {
                        "target_frame": "base_link",
                        "angle_min": -3.14159,
                        "angle_max": 3.14159,
                        "range_min": 0.1,
                        "range_max": 30.0,
                    }
                ],
            ),
        ]
    )
