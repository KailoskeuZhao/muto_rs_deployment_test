import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params_file = os.path.join(
        get_package_share_directory('muto_command_layer'),
        'config',
        'command_layer.yaml',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Object command-layer parameter file.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true.',
        ),
        DeclareLaunchArgument(
            'action_name',
            default_value='/go_to_object',
            description='Public GoToObject action name.',
        ),
        DeclareLaunchArgument(
            'registry_service',
            default_value='/sam2/get_stored_objects',
            description='Exact-name object-registry query service.',
        ),
        DeclareLaunchArgument(
            'navigate_to_pose_action',
            default_value='/navigate_to_pose',
            description='Nav2 NavigateToPose action name.',
        ),
        DeclareLaunchArgument(
            'global_frame',
            default_value='map',
            description='Nav2 global planning frame.',
        ),
        DeclareLaunchArgument(
            'robot_base_frame',
            default_value='base_frame',
            description='Robot frame used to choose the approach side.',
        ),
        DeclareLaunchArgument(
            'approach_distance',
            default_value='0.75',
            description='Object-centroid standoff distance in metres.',
        ),
        DeclareLaunchArgument(
            'launch_object_search',
            default_value='true',
            description='Start the registry/VLM FindObject action server.',
        ),
        DeclareLaunchArgument(
            'find_object_action',
            default_value='/find_object',
            description='Public FindObject action name.',
        ),
        DeclareLaunchArgument(
            'vlm_action',
            default_value='/vlm/generate',
            description='GenerateVlm child action used for object search.',
        ),
        DeclareLaunchArgument(
            'object_match_topic',
            default_value='/object_search/matches',
            description='Topic receiving one message per final match.',
        ),
        DeclareLaunchArgument(
            'vlm_model',
            default_value='',
            description='Object-search VLM model; empty uses socket default.',
        ),
        Node(
            package='muto_command_layer',
            executable='command_layer_node',
            name='command_layer',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'action_name': LaunchConfiguration('action_name'),
                    'registry_service': LaunchConfiguration(
                        'registry_service'
                    ),
                    'navigate_to_pose_action': LaunchConfiguration(
                        'navigate_to_pose_action'
                    ),
                    'global_frame': LaunchConfiguration('global_frame'),
                    'robot_base_frame': LaunchConfiguration(
                        'robot_base_frame'
                    ),
                    'approach_distance': LaunchConfiguration(
                        'approach_distance'
                    ),
                },
            ],
        ),
        Node(
            package='muto_command_layer',
            executable='object_search_node',
            name='object_search',
            output='screen',
            condition=IfCondition(LaunchConfiguration('launch_object_search')),
            parameters=[
                LaunchConfiguration('params_file'),
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'action_name': LaunchConfiguration(
                        'find_object_action'
                    ),
                    'registry_service': LaunchConfiguration(
                        'registry_service'
                    ),
                    'vlm_action': LaunchConfiguration('vlm_action'),
                    'match_topic': LaunchConfiguration(
                        'object_match_topic'
                    ),
                    'vlm_model': LaunchConfiguration('vlm_model'),
                },
            ],
        ),
    ])
