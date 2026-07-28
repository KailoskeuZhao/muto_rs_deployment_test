import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_params_file = os.path.join(
        get_package_share_directory('muto_vlm_socket'),
        'config',
        'vlm_socket.yaml',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='VLM socket parameter file.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true.',
        ),
        DeclareLaunchArgument(
            'action_name',
            default_value='/vlm/generate',
            description='Public GenerateVlm action name.',
        ),
        DeclareLaunchArgument(
            'base_url',
            default_value='http://127.0.0.1:8000/v1',
            description='OpenAI-compatible API base URL.',
        ),
        DeclareLaunchArgument(
            'default_model',
            default_value='gpt-5.5',
            description='Model used when an action goal leaves model empty.',
        ),
        Node(
            package='muto_vlm_socket',
            executable='vlm_socket_node',
            name='vlm_socket',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {
                    'use_sim_time': ParameterValue(
                        LaunchConfiguration('use_sim_time'),
                        value_type=bool,
                    ),
                    'action_name': ParameterValue(
                        LaunchConfiguration('action_name'),
                        value_type=str,
                    ),
                    'base_url': ParameterValue(
                        LaunchConfiguration('base_url'),
                        value_type=str,
                    ),
                    'default_model': ParameterValue(
                        LaunchConfiguration('default_model'),
                        value_type=str,
                    ),
                },
            ],
        ),
    ])
