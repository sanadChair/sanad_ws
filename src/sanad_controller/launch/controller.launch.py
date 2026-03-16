from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import UnlessCondition, IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
import os
from ament_index_python.packages import get_package_share_directory
import numpy as np

def noisy_controller(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration("use_sim_time")
    wheel_radius = float(LaunchConfiguration("wheel_radius").perform(context))
    wheel_separation = float(LaunchConfiguration("wheel_separation").perform(context))
    wheel_radius_error = float(
        LaunchConfiguration("wheel_radius_error").perform(context)
    )
    wheel_separation_error = float(
        LaunchConfiguration("wheel_separation_error").perform(context)
    )

    noisy_controller_node = Node(
        package="sanad_controller",
        executable="noisy_controller",
        parameters=[
            {
                "wheel_radius": wheel_radius + wheel_radius_error,
                "wheel_separation": wheel_separation + wheel_separation_error,
                "use_sim_time": use_sim_time,
            }
        ],
    )
    return [noisy_controller_node]


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
    )
    wheel_radius_arg = DeclareLaunchArgument(
        "wheel_radius",
        default_value="0.033",
    )
    wheel_separation_arg = DeclareLaunchArgument(
        "wheel_separation",
        default_value="0.17",
    )
    wheel_radius_error_arg = DeclareLaunchArgument(
        "wheel_radius_error",
        default_value="0.005",
    )
    wheel_separation_error_arg = DeclareLaunchArgument(
        "wheel_separation_error",
        default_value="0.02",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_separation = LaunchConfiguration("wheel_separation")
    
    sanad_controller_pkg = get_package_share_directory('sanad_controller')

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("sanad_controller"),
            "config",
            "sanad_controllers.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
            ("/diff_drive_controller/cmd_vel", "/cmd_vel"),
        ],
    )
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    wheel_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "sanad_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    noisy_controller_launch = OpaqueFunction(function=noisy_controller)

    twist_mux_launch = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("twist_mux"),
            "launch",
            "twist_mux_launch.py"
        ),
        launch_arguments={
            "cmd_vel_out": "sanad_controller/cmd_vel_unstamped",
            "config_locks": os.path.join(sanad_controller_pkg, "config", "twist_mux_locks.yaml"),
            "config_topics": os.path.join(sanad_controller_pkg, "config", "twist_mux_topics.yaml"),
            "config_joy": os.path.join(sanad_controller_pkg, "config", "twist_mux_joy.yaml"),
            "use_sim_time": use_sim_time,
        }.items(),
    )

    twist_relay_node = Node(
        package="sanad_controller",
        executable="twist_relay",
        name="twist_relay",
        parameters=[{"use_sim_time": use_sim_time}]
    )

    return LaunchDescription(
        [
            control_node,
            use_sim_time_arg,
            wheel_radius_arg,
            wheel_separation_arg,
            wheel_radius_error_arg,
            wheel_separation_error_arg,
            joint_state_broadcaster_spawner,
            wheel_controller_spawner,
            noisy_controller_launch,
            twist_mux_launch,
            twist_relay_node,
        ]
    )
