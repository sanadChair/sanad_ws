import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_slam = LaunchConfiguration("use_slam")

    use_slam_arg = DeclareLaunchArgument("use_slam", default_value="false")

    hardware_interface = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("sanad_firmware"),
            "launch",
            "hardware_interface.launch.py",
        ),
    )

    laser_params_file = os.path.join(
        get_package_share_directory("sanad_bringup"),
        "config",
        "ldlidar.yaml",
    )

    # laser_driver = Node(
    #     package="ldlidar",
    #     executable="ldlidar",
    #     name="ldlidar",
    #     output="screen",
    #     parameters=[laser_params_file],
    # )

    # ldlidar_node = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(
    #             get_package_share_directory("ldlidar_ros2"),
    #             "launch",
    #             "ld06.launch.py",
    #         )
    #     )
    # )
    rlidar_node = Node(
      package='ldlidar_ros2',
      executable='ldlidar_ros2_node',
      name='rlidar_publisher',
      output='screen',
      parameters=[
        {'product_name': 'LDLiDAR_LD06'},
        {'laser_scan_topic_name': 'scan_right'},
        {'point_cloud_2d_topic_name': 'pointcloud2d'},
        {'frame_id': 'laser_right_link'},
        {'port_name': '/dev/rlidar'},
        {'serial_baudrate': 230400},
        {'laser_scan_dir': True},
        {'enable_angle_crop_func': False},
        {'angle_crop_min': 135.0},  
        {'angle_crop_max': 225.0},  
        {'range_min': 0.02},
        {'range_max': 12.0} 
            ]
        )
    llidar_node = Node(
      package='ldlidar_ros2',
      executable='ldlidar_ros2_node',
      name='llidar_publisher',
      output='screen',
      parameters=[
        {'product_name': 'LDLiDAR_LD06'},
        {'laser_scan_topic_name': 'scan_left'},
        {'point_cloud_2d_topic_name': 'pointcloud2d'},
        {'frame_id': 'laser_left_link'},
        {'port_name': '/dev/llidar'},
        {'serial_baudrate': 230400},
        {'laser_scan_dir': True},
        {'enable_angle_crop_func': False},
        {'angle_crop_min': 135.0},
        {'angle_crop_max': 225.0},
        {'range_min': 0.02},
        {'range_max': 12.0}
            ]
        )

    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("sanad_controller"),
            "launch",
            "controller.launch.py",
        ),
        launch_arguments={
            "use_simple_controller": "False",
            "use_python": "False",
        }.items(),
    )

    joystick = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("sanad_controller"),
            "launch",
            "joystick_teleop.launch.py",
        ),
        launch_arguments={"use_sim_time": "False"}.items(),
    )

    imu_driver_node = Node(package="sanad_firmware", executable="mpu6050_driver.py")

    localization = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("sanad_localization"),
            "launch",
            "global_localization.launch.py",
        ),
        launch_arguments={"use_sim_time": "False"}.items(),
        condition=UnlessCondition(use_slam),
    )

    slam = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("sanad_mapping"), "launch", "slam.launch.py"
        ),
        launch_arguments={"use_sim_time": "False"}.items(),
        condition=IfCondition(use_slam),
    )

    navigation = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("sanad_navigation"),
            "launch",
            "navigation.launch.py",
        ),
        launch_arguments={"use_sim_time": "False"}.items(),
    )

    rviz_slam = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            os.path.join(
                get_package_share_directory("sanad_mapping"), "rviz", "slam.rviz"
            ),
        ],
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(use_slam),
    )

    rviz_nav = Node(
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
        condition=UnlessCondition(use_slam),
    )

    return LaunchDescription(
        [
            use_slam_arg,
            hardware_interface,
            rlidar_node,
            # llidar_node,
            controller,
            joystick,
            # imu_driver_node,
            # localization,
            # slam,
            # navigation,
            # rviz_slam,
            rviz_nav,
        ]
    )
