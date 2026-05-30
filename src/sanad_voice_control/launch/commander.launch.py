from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_cloud_arg = DeclareLaunchArgument(
        'use_cloud',
        default_value='true',
        description='Whether to use cloud LLM service (true) or local Koboldcpp (false)'
    )

    return LaunchDescription(
        [
            use_cloud_arg,
            # Dedicated LLM Service Node
            Node(
                package="sanad_voice_control",
                executable="llm_node",
                name="llm_node",
                output="screen",
                parameters=[{
                    'use_cloud': LaunchConfiguration('use_cloud')
                }]
            ),
            # TTS Speech Synthesis Node
            Node(
                package="sanad_voice_control",
                executable="tts_node",
                name="tts_node",
                output="screen",
                parameters=[{
                    'use_cloud': LaunchConfiguration('use_cloud')
                }]
            ),
            # Central orchestrator/broker
            Node(
                package="sanad_voice_control",
                executable="commander",
                name="llm_commander",
                output="screen",
            ),
            # Telegram Polling and Sending Manager Node
            Node(
                package="sanad_voice_control",
                executable="telegram_node",
                name="telegram_node",
                output="screen",
            ),
            # Console Interactive Terminal Input Node
            Node(
                package="sanad_voice_control",
                executable="console_input_node",
                name="console_input_node",
                output="screen",
                emulate_tty=True,  # Crucial for reading stdin/input() in a launch file
            ),
        ]
    )
