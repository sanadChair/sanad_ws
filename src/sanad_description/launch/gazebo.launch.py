import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.substitutions import (
    Command,
    LaunchConfiguration,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_gazebo(context, *args, **kwargs):
    sanad_description = get_package_share_directory("sanad_description")
    world_name = LaunchConfiguration("world").perform(context)
    use_gz_gui = LaunchConfiguration("use_gz_gui").perform(context).lower() == "true"
    gz_headless_rendering = (
        LaunchConfiguration("gz_headless_rendering").perform(context).lower() == "true"
    )

    world_path = os.path.join(sanad_description, "worlds", f"{world_name}.world")
    gz_args = [world_path, "-r", "-v", "4"]

    if not use_gz_gui:
        gz_args.append("-s")
    if gz_headless_rendering:
        gz_args.append("--headless-rendering")

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    os.path.join(get_package_share_directory("ros_gz_sim"), "launch"),
                    "/gz_sim.launch.py",
                ]
            ),
            launch_arguments={"gz_args": " ".join(gz_args)}.items(),
        )
    ]


def generate_launch_description():
    sanad_description = get_package_share_directory("sanad_description")
    model_path = str(Path(sanad_description).parent.resolve())
    model_path += os.pathsep + os.path.join(sanad_description, "models")
    world_arg = DeclareLaunchArgument(
        name="world",
        default_value="small_house",
        description="Name of world to load (without .world extension)",
    )
    use_gz_gui_arg = DeclareLaunchArgument(
        name="use_gz_gui",
        default_value="true",
        description="Start Gazebo with its GUI client enabled.",
    )
    gz_headless_rendering_arg = DeclareLaunchArgument(
        name="gz_headless_rendering",
        default_value="false",
        description="Enable Gazebo headless rendering for rendering sensors.",
    )

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(sanad_description, "urdf", "sanad.urdf.xacro"),
        description="Absolute path to robot urdf file",
    )

    gazebo_resource_path = SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", model_path)

    ros_distro = os.environ["ROS_DISTRO"]
    is_ignition = "True" if ros_distro == "humble" else "False"

    robot_description = ParameterValue(
        Command(
            ["xacro ", LaunchConfiguration("model"), " is_ignition:=", is_ignition]
        ),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description", "-name", "sanad"],
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/scan_rear@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/realsense/image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/realsense/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/realsense/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/realsense/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
        ],
        remappings=[
            ("/imu", "/imu/out"),
        ],
    )

    return LaunchDescription(
        [
            world_arg,
            use_gz_gui_arg,
            gz_headless_rendering_arg,
            model_arg,
            gazebo_resource_path,
            robot_state_publisher_node,
            OpaqueFunction(function=launch_gazebo),
            gz_spawn_entity,
            gz_ros2_bridge,
        ]
    )
