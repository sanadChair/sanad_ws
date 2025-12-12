import os
from os import pathsep
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Get package directories
    sanad_description = get_package_share_directory("sanad_description")
    sanad_bringup = get_package_share_directory("sanad_bringup")

    # Declare arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            name="model",
            default_value=os.path.join(sanad_description, "urdf", "sanad.urdf.xacro"),
            description="Absolute path to robot urdf file",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            name="world_name",
            default_value="small_house",
            description="Name of world to load (without .world extension)",
        )
    )

    # Initialize Arguments
    gui = LaunchConfiguration("gui")
    model_arg = LaunchConfiguration("model")
    world_name = LaunchConfiguration("world_name")

    # Setup model path for Gazebo
    model_path = str(Path(sanad_bringup).parent.resolve())
    model_path += pathsep + os.path.join(sanad_bringup, "models")

    gazebo_resource_path = SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", model_path)

    # Build world path dynamically
    world_path = PathJoinSubstitution(
        [
            sanad_bringup,
            "worlds",
            PythonExpression(expression=["'", world_name, "'", " + '.world'"]),
        ]
    )

    # Get ROS distro to set is_ignition flag
    ros_distro = os.environ.get("ROS_DISTRO", "humble")
    is_ignition = "True" if ros_distro == "humble" else "False"

    # Get URDF via xacro
    robot_description_content = Command(
        [
            "xacro ",
            model_arg,
            " ros2_control_hardware:=gazebo is_ignition:=",
            is_ignition,
        ]
    )
    robot_description = ParameterValue(robot_description_content, value_type=str)

    # RViz config file
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("sanad_description"), "rviz", "config.rviz"]
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments=[
            ("gz_args", PythonExpression(["'", world_path, " -v 4 -r'"]))
        ],
        condition=IfCondition(gui),
    )

    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments=[
            (
                "gz_args",
                PythonExpression(
                    ["'--headless-rendering -s -r -v 4 ", world_path, "'"]
                ),
            )
        ],
        condition=UnlessCondition(gui),
    )

    # Gazebo bridge
    gazebo_config_file = PathJoinSubstitution(
        [FindPackageShare("sanad_bringup"), "config", "gazebo_bringup.yaml"]
    )
    gazebo_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"config_file": gazebo_config_file}],
        output="screen",
    )

    # Spawn entity
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic",
            "/robot_description",
            "-name",
            "sanad",
            "-allow_renaming",
            "true",
        ],
    )

    # Robot controllers
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("sanad_bringup"),
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

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_drive_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )

    nodes = [
        gazebo_resource_path,
        control_node,
        robot_state_publisher_node,
        gazebo,
        gazebo_headless,
        gazebo_bridge,
        gz_spawn_entity,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes)

# import os
# from ament_index_python.packages import get_package_share_directory

# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
# from launch_ros.actions import Node
# from launch_ros.substitutions import FindPackageShare


# def generate_launch_description():
#     # Declare variables (equivalent to <let>)
#     urdf_path = PathJoinSubstitution([
#         FindPackageShare('sanad_description'),
#         'urdf',
#         'sanad.urdf.xacro'
#     ])
    
#     rviz_config_path = PathJoinSubstitution([
#         FindPackageShare('sanad_description'),
#         'rviz',
#         'config.rviz'
#     ])
    
#     gazebo_config_path = PathJoinSubstitution([
#         FindPackageShare('sanad_bringup'),
#         'config',
#         'gazebo_bringup.yaml'
#     ])

#     default_world = os.path.join(
#         get_package_share_directory('sanad_bringup'),
#         'worlds',
#         'small_house.world'
#     )
 
#     world = LaunchConfiguration('world')

#     world_arg = DeclareLaunchArgument(
#         'world',
#         default_value=default_world,
#         description='world to load'
#     )
    
#     # Set environment variable
#     gz_sim_resource_path = SetEnvironmentVariable(
#         name='GZ_SIM_RESOURCE_PATH',
#         value=PathJoinSubstitution([
#             FindPackageShare('sanad_bringup'),
#             'models'
#         ])
#     )
    
#     # Robot State Publisher Node
#     robot_state_publisher_node = Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         parameters=[{
#             'robot_description': Command(['xacro ', urdf_path])
#         }]
#     )
    
#     # Joint State Publisher GUI Node
#     joint_state_publisher_gui_node = Node(
#         package='joint_state_publisher_gui',
#         executable='joint_state_publisher_gui'
#     )
    
#     # RViz2 Node
#     rviz2_node = Node(
#         package='rviz2',
#         executable='rviz2',
#         output='screen',
#         arguments=['-d', rviz_config_path]
#     )
    
#     # Include Gazebo Sim Launch File
#     gazebo_sim = IncludeLaunchDescription(
#                 PythonLaunchDescriptionSource([os.path.join(
#                     get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
#                     launch_arguments={'gz_args': ['-r -v4 ', world], 'on_exit_shutdown': 'true'}.items()
#              )
    
#     # ROS-Gazebo Sim Create Node (spawn entity)
#     ros_gz_create_node = Node(package='ros_gz_sim', executable='create',
#                         arguments=['-topic', 'robot_description',
#                                    '-name', 'sanad',
#                                    '-z', '0.1'],
#                         output='screen')
    
#     # ROS-Gazebo Bridge Node
#     ros_gz_bridge_node = Node(
#         package='ros_gz_bridge',
#         executable='parameter_bridge',
#         parameters=[{
#             'config_file': gazebo_config_path
#         }]
#     )

#     ros_gz_image_bridge = Node(
#         package="ros_gz_image",
#         executable="image_bridge",
#         arguments=["/camera/image_raw"]
#     )
    
    
    
#     return LaunchDescription([
#         world_arg,
#         gz_sim_resource_path,
#         robot_state_publisher_node,
#         # joint_state_publisher_gui_node,
#         rviz2_node,
#         gazebo_sim,
#         ros_gz_bridge_node,
#         ros_gz_create_node,
#         ros_gz_image_bridge
#     ])