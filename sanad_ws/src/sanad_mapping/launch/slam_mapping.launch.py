from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # --- 1. Simulation (Gazebo)
    sim_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('sanad_bringup'),
                'launch',
                'sanad_gazebo.launch.xml'
            )
        )
    )

    # --- 2. SLAM params
    slam_params = os.path.join(
        get_package_share_directory('sanad_mapping'),
        'config',
        'slam_params.yaml'
    )

    slam = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params]
    )

    # --- 3. RViz
    # rviz_config = os.path.join(
    #     get_package_share_directory('sanad_mapping'),
    #     'rviz',
    #     'mapping.rviz'
    # )

    # rviz = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     arguments=['-d', rviz_config],
    #     output='screen'
    # )

    return LaunchDescription([
        sim_launch,
        slam,
    ])
