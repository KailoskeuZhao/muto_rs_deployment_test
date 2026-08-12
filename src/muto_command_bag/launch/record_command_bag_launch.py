import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


DEFAULT_EXCLUDE_REGEX = (
    r'^(/camera/[^/]+/(image_raw|points)(/.*)?|'
    r'/sam2/(annotated_image|mask|instance_mask|instance_pointcloud)(/.*)?|'
    r'/lidar/PointCloud.*)$'
)


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('muto_command_bag'),
        'config',
        'command_bag.yaml',
    )

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'output_directory', default_value='/opt/muto_rs_ws/bags'),
        DeclareLaunchArgument('storage_id', default_value='mcap'),
        DeclareLaunchArgument('storage_preset', default_value='none'),
        DeclareLaunchArgument('topics_regex', default_value=''),
        DeclareLaunchArgument(
            'exclude_regex', default_value=DEFAULT_EXCLUDE_REGEX),
        DeclareLaunchArgument('max_cache_size', default_value='104857600'),
        DeclareLaunchArgument('max_bag_directories', default_value='20'),
        DeclareLaunchArgument('post_terminal_delay', default_value='0.5'),
        DeclareLaunchArgument(
            'lifecycle_event_topic',
            default_value='/model_commander/recording_event',
        ),
        DeclareLaunchArgument(
            'status_topic', default_value='/model_commander/bag_status'),
        DeclareLaunchArgument(
            'path_topic', default_value='/model_commander/last_bag_path'),
        DeclareLaunchArgument(
            'operator_event_topic',
            default_value='/model_commander/operator_event',
        ),
        DeclareLaunchArgument(
            'owner_heartbeat_topic',
            default_value='/model_commander/status',
        ),
        DeclareLaunchArgument('owner_heartbeat_timeout', default_value='10.0'),
        Node(
            package='muto_exploration_bag',
            executable='exploration_bag_recorder',
            name='command_bag_recorder',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {
                    'use_sim_time': ParameterValue(
                        LaunchConfiguration('use_sim_time'), value_type=bool),
                    'output_directory': LaunchConfiguration(
                        'output_directory'),
                    'storage_id': LaunchConfiguration('storage_id'),
                    'storage_preset': LaunchConfiguration('storage_preset'),
                    'topics_regex': LaunchConfiguration('topics_regex'),
                    'exclude_regex': LaunchConfiguration('exclude_regex'),
                    'max_cache_size': ParameterValue(
                        LaunchConfiguration('max_cache_size'), value_type=int),
                    'max_bag_directories': ParameterValue(
                        LaunchConfiguration('max_bag_directories'),
                        value_type=int,
                    ),
                    'post_terminal_delay': ParameterValue(
                        LaunchConfiguration('post_terminal_delay'),
                        value_type=float,
                    ),
                    'lifecycle_event_topic': LaunchConfiguration(
                        'lifecycle_event_topic'),
                    'status_topic': LaunchConfiguration('status_topic'),
                    'path_topic': LaunchConfiguration('path_topic'),
                    'operator_event_topic': LaunchConfiguration(
                        'operator_event_topic'),
                    'owner_heartbeat_topic': LaunchConfiguration(
                        'owner_heartbeat_topic'),
                    'owner_heartbeat_timeout': ParameterValue(
                        LaunchConfiguration('owner_heartbeat_timeout'),
                        value_type=float,
                    ),
                    'bag_prefix': 'muto_command',
                    'manifest_schema': 'command_mission_v1',
                    'status_schema': 'muto_command_bag_status_v1',
                    'recording_label': 'command_mission',
                },
            ],
        ),
    ])
