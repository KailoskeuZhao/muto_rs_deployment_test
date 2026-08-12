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
        get_package_share_directory('muto_exploration_bag'),
        'config',
        'exploration_bag.yaml',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Exploration-bag recorder parameter file.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time while discovering topics.',
        ),
        DeclareLaunchArgument(
            'output_directory',
            default_value='/opt/muto_rs_ws/bags',
            description='Parent directory for action-scoped bags.',
        ),
        DeclareLaunchArgument('storage_id', default_value='mcap'),
        DeclareLaunchArgument('storage_preset', default_value='none'),
        DeclareLaunchArgument(
            'topics_regex',
            default_value='',
            description=(
                'Optional inclusion regex; empty discovers all topics before '
                'applying the exclusion regex.'
            ),
        ),
        DeclareLaunchArgument(
            'exclude_regex',
            default_value=DEFAULT_EXCLUDE_REGEX,
            description=(
                'Topic exclusion regex; the default omits high-bandwidth '
                'images and point clouds. Pass an empty value to retain all.'
            ),
        ),
        DeclareLaunchArgument('max_cache_size', default_value='104857600'),
        DeclareLaunchArgument('max_bag_directories', default_value='20'),
        DeclareLaunchArgument('post_terminal_delay', default_value='0.25'),
        DeclareLaunchArgument(
            'lifecycle_event_topic',
            default_value='/explore_and_record/recording_event',
        ),
        DeclareLaunchArgument(
            'status_topic',
            default_value='/explore_and_record/bag_status',
        ),
        DeclareLaunchArgument(
            'path_topic',
            default_value='/explore_and_record/last_bag_path',
        ),
        DeclareLaunchArgument(
            'operator_event_topic',
            default_value='/explore_and_record/operator_event',
        ),
        Node(
            package='muto_exploration_bag',
            executable='exploration_bag_recorder',
            name='exploration_bag_recorder',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {
                    'use_sim_time': ParameterValue(
                        LaunchConfiguration('use_sim_time'),
                        value_type=bool,
                    ),
                    'output_directory': LaunchConfiguration(
                        'output_directory'
                    ),
                    'storage_id': LaunchConfiguration('storage_id'),
                    'storage_preset': LaunchConfiguration('storage_preset'),
                    'topics_regex': LaunchConfiguration('topics_regex'),
                    'exclude_regex': LaunchConfiguration('exclude_regex'),
                    'max_cache_size': ParameterValue(
                        LaunchConfiguration('max_cache_size'),
                        value_type=int,
                    ),
                    'max_bag_directories': ParameterValue(
                        LaunchConfiguration('max_bag_directories'),
                        value_type=int,
                    ),
                    'post_terminal_delay': ParameterValue(
                        LaunchConfiguration('post_terminal_delay'),
                        value_type=float,
                    ),
                    'lifecycle_event_topic': LaunchConfiguration(
                        'lifecycle_event_topic'
                    ),
                    'status_topic': LaunchConfiguration('status_topic'),
                    'path_topic': LaunchConfiguration('path_topic'),
                    'operator_event_topic': LaunchConfiguration(
                        'operator_event_topic'
                    ),
                },
            ],
        ),
    ])
