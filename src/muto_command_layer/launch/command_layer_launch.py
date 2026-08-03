import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_params_file = os.path.join(
        get_package_share_directory('muto_command_layer'),
        'config',
        'command_layer.yaml',
    )
    default_frontier_params = os.path.join(
        get_package_share_directory('muto_slam_mapping'),
        'config',
        'frontier_exploration_params.yaml',
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
            'global_costmap_service',
            default_value='/global_costmap/get_costmap',
            description='Nav2 master global-costmap query service.',
        ),
        DeclareLaunchArgument(
            'global_costmap_timeout',
            default_value='5.0',
            description='Maximum global-costmap service wait in seconds.',
        ),
        DeclareLaunchArgument(
            'global_frame',
            default_value='map',
            description='Nav2 global planning frame.',
        ),
        DeclareLaunchArgument(
            'robot_base_frame',
            default_value='base_frame',
            description='Robot frame used to seed costmap reachability.',
        ),
        DeclareLaunchArgument(
            'approach_distance',
            default_value='0.75',
            description='Minimum object-centroid approach radius in metres.',
        ),
        DeclareLaunchArgument(
            'approach_robot_radius',
            default_value='0.16',
            description='Robot-radius lower bound for object standoff.',
        ),
        DeclareLaunchArgument(
            'approach_start_snap_distance',
            default_value='0.5',
            description='Maximum costmap start-cell snap distance in metres.',
        ),
        DeclareLaunchArgument(
            'approach_maximum_cost',
            default_value='252',
            description='Largest traversable raw Nav2 cost; maximum is 252.',
        ),
        DeclareLaunchArgument(
            'explore_service',
            default_value='/explore',
            description='Public start/stop exploration service.',
        ),
        DeclareLaunchArgument(
            'save_map_service',
            default_value='/save_map',
            description='Public sanitized occupancy-map save service.',
        ),
        DeclareLaunchArgument(
            'slam_toolbox_save_map_service',
            default_value='/slam_toolbox/save_map',
            description='Underlying SLAM Toolbox occupancy-map save service.',
        ),
        DeclareLaunchArgument(
            'map_save_directory',
            default_value='',
            description='Absolute map output directory; empty uses ~/.ros/maps.',
        ),
        DeclareLaunchArgument(
            'default_map_name',
            default_value='muto_map',
            description='Map basename used when a save request has no name.',
        ),
        DeclareLaunchArgument(
            'save_map_timeout',
            default_value='10.0',
            description='Total SLAM Toolbox map-save timeout in seconds.',
        ),
        DeclareLaunchArgument(
            'save_map_result_timeout',
            default_value='15.0',
            description='Natural-language map-save result timeout in seconds.',
        ),
        DeclareLaunchArgument(
            'explore_and_record_action',
            default_value='/explore_and_record',
            description='Synthetic exploration and object-recording action.',
        ),
        DeclareLaunchArgument(
            'frontier_control_service',
            default_value='/control_exploration',
            description='Frontier explorer runtime-control service.',
        ),
        DeclareLaunchArgument(
            'spin_action',
            default_value='/spin',
            description='Nav2 Spin behavior action.',
        ),
        DeclareLaunchArgument(
            'registry_save_service',
            default_value='/sam2/save_stored_objects',
            description='Object-registry checkpoint service.',
        ),
        DeclareLaunchArgument(
            'exploration_completion_topic',
            default_value='/explore/exploration_complete',
            description='Frontier-exhaustion event topic.',
        ),
        DeclareLaunchArgument(
            'exploration_service_timeout',
            default_value='5.0',
            description='Maximum frontier-control wait in seconds.',
        ),
        DeclareLaunchArgument(
            'program_endpoint_timeout',
            default_value='5.0',
            description='Synthetic-program dependency timeout in seconds.',
        ),
        DeclareLaunchArgument(
            'exploration_cycle_duration',
            default_value='10.0',
            description='Exploration time between observation stops.',
        ),
        DeclareLaunchArgument(
            'observation_duration',
            default_value='3.0',
            description='Stationary object-observation dwell per scan step.',
        ),
        DeclareLaunchArgument(
            'scan_step_count',
            default_value='8',
            description='Equal spin-and-observe steps in one 360-degree scan.',
        ),
        DeclareLaunchArgument(
            'spin_time_allowance',
            default_value='15.0',
            description='Maximum Nav2 spin execution time in seconds.',
        ),
        DeclareLaunchArgument(
            'navigation_settle_time',
            default_value='1.0',
            description='Delay after canceling the frontier Nav2 goal.',
        ),
        DeclareLaunchArgument(
            'visibility_coverage_enabled',
            default_value='true',
            description=(
                'Run adaptive predicted-visibility coverage after frontier '
                'exhaustion.'
            ),
        ),
        DeclareLaunchArgument(
            'visibility_map_topic',
            default_value='/map',
            description=(
                'SLAM occupancy grid used for predicted visibility coverage.'
            ),
        ),
        DeclareLaunchArgument(
            'visibility_target_pose_topic',
            default_value='/explore/visibility_target_pose',
            description='Selected visibility viewpoint poses.',
        ),
        DeclareLaunchArgument(
            'visibility_completion_ratio',
            default_value='0.98',
            description=(
                'Required predicted observable free and boundary coverage.'
            ),
        ),
        DeclareLaunchArgument(
            'visibility_max_viewpoints',
            default_value='0',
            description='Coverage viewpoint limit; zero is unlimited.',
        ),
        DeclareLaunchArgument(
            'launch_frontier_explorer',
            default_value='true',
            description=(
                'Start the Muto frontier explorer in command-controlled '
                'cold idle.'
            ),
        ),
        DeclareLaunchArgument(
            'frontier_params_file',
            default_value=default_frontier_params,
            description='Muto frontier exploration parameter file.',
        ),
        DeclareLaunchArgument(
            'frontier_log_level',
            default_value='info',
            description='Frontier explorer ROS log level.',
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
            'launch_active_object_search',
            default_value='true',
            description='Start the composed FindSomething action server.',
        ),
        DeclareLaunchArgument(
            'find_something_action',
            default_value='/find_something',
            description='Public active object-search action name.',
        ),
        DeclareLaunchArgument(
            'registry_topic',
            default_value='/sam2/stored_objects',
            description='Transient snapshot of confirmed static objects.',
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
        DeclareLaunchArgument(
            'launch_natural_language_command',
            default_value='true',
            description='Start the validated natural-language command router.',
        ),
        DeclareLaunchArgument(
            'natural_language_command_action',
            default_value='/natural_language_command',
            description='Public natural-language command action name.',
        ),
        Node(
            package='frontier_exploration_ros2',
            executable='frontier_explorer',
            name='frontier_explorer',
            output='screen',
            condition=IfCondition(
                LaunchConfiguration('launch_frontier_explorer')
            ),
            parameters=[
                LaunchConfiguration('frontier_params_file'),
                {
                    'use_sim_time': ParameterValue(
                        LaunchConfiguration('use_sim_time'),
                        value_type=bool,
                    ),
                    'autostart': False,
                    'control_service_enabled': True,
                    'completion_event_enabled': True,
                    'completion_event_topic': LaunchConfiguration(
                        'exploration_completion_topic'
                    ),
                },
            ],
            remappings=[
                (
                    'control_exploration',
                    LaunchConfiguration('frontier_control_service'),
                ),
            ],
            arguments=[
                '--ros-args',
                '--log-level',
                LaunchConfiguration('frontier_log_level'),
            ],
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
                    'global_costmap_service': LaunchConfiguration(
                        'global_costmap_service'
                    ),
                    'global_costmap_timeout': LaunchConfiguration(
                        'global_costmap_timeout'
                    ),
                    'global_frame': LaunchConfiguration('global_frame'),
                    'robot_base_frame': LaunchConfiguration(
                        'robot_base_frame'
                    ),
                    'approach_distance': LaunchConfiguration(
                        'approach_distance'
                    ),
                    'approach_robot_radius': LaunchConfiguration(
                        'approach_robot_radius'
                    ),
                    'approach_start_snap_distance': LaunchConfiguration(
                        'approach_start_snap_distance'
                    ),
                    'approach_maximum_cost': ParameterValue(
                        LaunchConfiguration('approach_maximum_cost'),
                        value_type=int,
                    ),
                    'explore_service': LaunchConfiguration(
                        'explore_service'
                    ),
                    'save_map_service': LaunchConfiguration(
                        'save_map_service'
                    ),
                    'slam_toolbox_save_map_service': LaunchConfiguration(
                        'slam_toolbox_save_map_service'
                    ),
                    'map_save_directory': LaunchConfiguration(
                        'map_save_directory'
                    ),
                    'default_map_name': LaunchConfiguration(
                        'default_map_name'
                    ),
                    'save_map_timeout': LaunchConfiguration(
                        'save_map_timeout'
                    ),
                    'explore_and_record_action': LaunchConfiguration(
                        'explore_and_record_action'
                    ),
                    'frontier_control_service': LaunchConfiguration(
                        'frontier_control_service'
                    ),
                    'spin_action': LaunchConfiguration('spin_action'),
                    'registry_save_service': LaunchConfiguration(
                        'registry_save_service'
                    ),
                    'exploration_completion_topic': LaunchConfiguration(
                        'exploration_completion_topic'
                    ),
                    'exploration_service_timeout': LaunchConfiguration(
                        'exploration_service_timeout'
                    ),
                    'program_endpoint_timeout': LaunchConfiguration(
                        'program_endpoint_timeout'
                    ),
                    'exploration_cycle_duration': LaunchConfiguration(
                        'exploration_cycle_duration'
                    ),
                    'observation_duration': LaunchConfiguration(
                        'observation_duration'
                    ),
                    'scan_step_count': LaunchConfiguration(
                        'scan_step_count'
                    ),
                    'spin_time_allowance': LaunchConfiguration(
                        'spin_time_allowance'
                    ),
                    'navigation_settle_time': LaunchConfiguration(
                        'navigation_settle_time'
                    ),
                    'visibility_coverage_enabled': ParameterValue(
                        LaunchConfiguration('visibility_coverage_enabled'),
                        value_type=bool,
                    ),
                    'visibility_map_topic': LaunchConfiguration(
                        'visibility_map_topic'
                    ),
                    'visibility_target_pose_topic': LaunchConfiguration(
                        'visibility_target_pose_topic'
                    ),
                    'visibility_completion_ratio': LaunchConfiguration(
                        'visibility_completion_ratio'
                    ),
                    'visibility_max_viewpoints': LaunchConfiguration(
                        'visibility_max_viewpoints'
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
        Node(
            package='muto_command_layer',
            executable='natural_language_command_node',
            name='natural_language_command_router',
            output='screen',
            condition=IfCondition(
                LaunchConfiguration('launch_natural_language_command')
            ),
            parameters=[
                LaunchConfiguration('params_file'),
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'action_name': LaunchConfiguration(
                        'natural_language_command_action'
                    ),
                    'vlm_action': LaunchConfiguration('vlm_action'),
                    'find_object_action': LaunchConfiguration(
                        'find_object_action'
                    ),
                    'find_something_action': LaunchConfiguration(
                        'find_something_action'
                    ),
                    'go_to_object_action': LaunchConfiguration('action_name'),
                    'explore_service': LaunchConfiguration('explore_service'),
                    'save_map_service': LaunchConfiguration(
                        'save_map_service'
                    ),
                    'save_map_result_timeout': LaunchConfiguration(
                        'save_map_result_timeout'
                    ),
                    'explore_and_record_action': LaunchConfiguration(
                        'explore_and_record_action'
                    ),
                    'vlm_model': LaunchConfiguration('vlm_model'),
                },
            ],
        ),
        Node(
            package='muto_command_layer',
            executable='active_object_search_node',
            name='active_object_search',
            output='screen',
            condition=IfCondition(
                LaunchConfiguration('launch_active_object_search')
            ),
            parameters=[
                LaunchConfiguration('params_file'),
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'action_name': LaunchConfiguration(
                        'find_something_action'
                    ),
                    'find_object_action': LaunchConfiguration(
                        'find_object_action'
                    ),
                    'explore_and_record_action': LaunchConfiguration(
                        'explore_and_record_action'
                    ),
                    'registry_topic': LaunchConfiguration('registry_topic'),
                },
            ],
        ),
    ])
