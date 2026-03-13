import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PythonExpression,
    PathJoinSubstitution,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    sanad_description = get_package_share_directory("sanad_description")
    model_path = str(Path(sanad_description).parent.resolve())
    model_path += os.pathsep + os.path.join(sanad_description, "models")
    world_arg = DeclareLaunchArgument(
        name="world",
        default_value="small_house",
        description="Name of world to load (without .world extension)",
    )
    world_name = LaunchConfiguration("world")
    world_path = PathJoinSubstitution(
        [
            sanad_description,
            "worlds",
            PythonExpression(expression=["'", world_name, "'", " + '.world'"]),
        ]
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

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("ros_gz_sim"), "launch"),
                "/gz_sim.launch.py",
            ]
        ),
        launch_arguments=[("gz_args", [world_path, " -r -v 4"])],
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
        ],
        remappings=[
            ("/imu", "/imu/out"),
        ],
    )

    return LaunchDescription(
        [
            world_arg,
            model_arg,
            gazebo_resource_path,
            robot_state_publisher_node,
            gazebo,
            gz_spawn_entity,
            gz_ros2_bridge,
        ]
    )
