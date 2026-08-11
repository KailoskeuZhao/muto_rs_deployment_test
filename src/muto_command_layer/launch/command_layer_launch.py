import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import AndSubstitution, LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue


DEFAULT_EXPLORATION_BAG_EXCLUDE_REGEX = (
    r'^(/camera/[^/]+/(image_raw|points)(/.*)?|'
    r'/sam2/(annotated_image|mask|instance_mask|instance_pointcloud)(/.*)?|'
    r'/lidar/PointCloud.*)$'
)


def include_launch(package, filename, arguments):
    """Include one launch file with isolated arguments and simulation time."""
    source = PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory(package),
        'launch',
        filename,
    ))
    return GroupAction(
        scoped=True,
        actions=[
            SetParameter(
                name='use_sim_time',
                value=ParameterValue(
                    LaunchConfiguration('use_sim_time'),
                    value_type=bool,
                ),
            ),
            IncludeLaunchDescription(
                source,
                launch_arguments=arguments.items(),
            ),
        ],
    )


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
    default_vlm_params = os.path.join(
        get_package_share_directory('muto_command_layer'),
        'config',
        'object_pipeline_vlm.yaml',
    )
    default_exploration_bag_params = os.path.join(
        get_package_share_directory('muto_exploration_bag'),
        'config',
        'exploration_bag.yaml',
    )
    default_command_bag_params = os.path.join(
        get_package_share_directory('muto_command_bag'),
        'config',
        'command_bag.yaml',
    )

    object_pipeline = include_launch(
        'muto_command_layer',
        'object_pipeline_launch.py',
        {
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'image_topic': LaunchConfiguration('image_topic'),
            'depth_topic': LaunchConfiguration('depth_topic'),
            'depth_camera_info_topic': LaunchConfiguration(
                'depth_camera_info_topic'),
            'color_camera_info_topic': LaunchConfiguration(
                'color_camera_info_topic'),
            'detections_topic': LaunchConfiguration('detections_topic'),
            'detection_heartbeat_topic': LaunchConfiguration(
                'detection_heartbeat_topic'),
            'instance_pointcloud_topic': LaunchConfiguration(
                'instance_pointcloud_topic'),
            'yolo_model': LaunchConfiguration('yolo_model'),
            'yolo_device': LaunchConfiguration('yolo_device'),
            'yolo_confidence': LaunchConfiguration('yolo_confidence'),
            'detection_crop_jpeg_quality': LaunchConfiguration(
                'detection_crop_jpeg_quality'),
            'max_publish_rate': LaunchConfiguration('max_publish_rate'),
            'registry_service': LaunchConfiguration('registry_service'),
            'registry_objects_topic': LaunchConfiguration('registry_topic'),
            'registry_save_service': LaunchConfiguration(
                'registry_save_service'),
            'registry_output_yaml': LaunchConfiguration(
                'registry_output_yaml'),
            'registry_image_directory': LaunchConfiguration(
                'registry_image_directory'),
            'registry_store_images': LaunchConfiguration(
                'registry_store_images'),
            'registry_load_existing': LaunchConfiguration(
                'load_existing_map'),
            'target_frame': LaunchConfiguration('global_frame'),
            'registry_tf_retry_window': LaunchConfiguration(
                'registry_tf_retry_window'),
            'registry_tf_retry_rate': LaunchConfiguration(
                'registry_tf_retry_rate'),
            'vlm_params_file': LaunchConfiguration('vlm_params_file'),
            'vlm_action': LaunchConfiguration('vlm_action'),
            'vlm_base_url': LaunchConfiguration('vlm_base_url'),
            'vlm_wire_api': LaunchConfiguration('vlm_wire_api'),
            'vlm_model': LaunchConfiguration('vlm_model'),
        },
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
            'image_topic',
            default_value='/camera/color/image_raw',
            description='RGB image consumed by YOLO and SAM 2.',
        ),
        DeclareLaunchArgument(
            'depth_topic',
            default_value='/camera/depth/image_raw',
            description='Raw depth image used for instance point clouds.',
        ),
        DeclareLaunchArgument(
            'depth_camera_info_topic',
            default_value='/camera/depth/camera_info',
        ),
        DeclareLaunchArgument(
            'color_camera_info_topic',
            default_value='/camera/color/camera_info',
        ),
        DeclareLaunchArgument(
            'detections_topic',
            default_value='/sam2/detections',
        ),
        DeclareLaunchArgument(
            'detection_heartbeat_topic',
            default_value='/sam2/detection_heartbeat',
            description='Crop-free completion heartbeat for detector frames.',
        ),
        DeclareLaunchArgument(
            'instance_pointcloud_topic',
            default_value='/sam2/instance_pointcloud',
        ),
        DeclareLaunchArgument('yolo_model', default_value='yolo26m.pt'),
        DeclareLaunchArgument('yolo_device', default_value='0'),
        DeclareLaunchArgument('yolo_confidence', default_value='0.4'),
        DeclareLaunchArgument(
            'detection_crop_jpeg_quality',
            default_value='90',
        ),
        DeclareLaunchArgument(
            'max_publish_rate',
            default_value='7.0',
            description='Maximum YOLO/SAM processing start rate in Hz.',
        ),
        DeclareLaunchArgument('registry_output_yaml', default_value=''),
        DeclareLaunchArgument('registry_image_directory', default_value=''),
        DeclareLaunchArgument(
            'registry_store_images',
            default_value='true',
        ),
        DeclareLaunchArgument(
            'load_existing_map',
            default_value='false',
            description=(
                'Set true when Nav2 restores a saved map so its persisted '
                'object registry is loaded. False starts with clean objects.'
            ),
        ),
        DeclareLaunchArgument(
            'registry_tf_retry_window',
            default_value='1.0',
        ),
        DeclareLaunchArgument(
            'registry_tf_retry_rate',
            default_value='20.0',
        ),
        DeclareLaunchArgument(
            'vlm_params_file',
            default_value=default_vlm_params,
        ),
        DeclareLaunchArgument(
            'vlm_base_url',
            default_value='http://43.165.176.234:8080/v1',
        ),
        DeclareLaunchArgument(
            'vlm_wire_api',
            default_value='chat_completions',
            description='VLM protocol: responses or chat_completions.',
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
            description='Legacy composite exploration and recording action.',
        ),
        DeclareLaunchArgument(
            'explore_frontier_action',
            default_value='/command_primitives/explore_frontier',
            description='Internal bounded frontier-exploration primitive.',
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
            'rotate_cmd_vel_topic',
            default_value='/cmd_vel',
            description=(
                'Velocity topic used by the model commander direct rotate '
                'primitive.'
            ),
        ),
        DeclareLaunchArgument(
            'rotate_executable_yaw_velocity',
            default_value='0.19',
            description=(
                'Constant yaw rate for direct rotate, above the Muto minimum '
                'executable gait level.'
            ),
        ),
        DeclareLaunchArgument(
            'rotate_goal_tolerance',
            default_value='0.08',
            description='Odometry yaw tolerance for direct rotate completion.',
        ),
        DeclareLaunchArgument(
            'rotate_control_period',
            default_value='0.05',
            description='Direct rotate cmd_vel publish period in seconds.',
        ),
        DeclareLaunchArgument(
            'rotate_stop_publish_count',
            default_value='3',
            description='Number of zero Twist messages sent after direct rotate.',
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
            description=(
                'Minimum exploration time before scanning; active frontier '
                'travel may extend it.'
            ),
        ),
        DeclareLaunchArgument(
            'observation_duration',
            default_value='3.0',
            description='Maximum stationary observation time per scan step.',
        ),
        DeclareLaunchArgument(
            'observation_min_detection_frames',
            default_value='3',
            description=(
                'Fresh detector messages needed to finish observation early; '
                'zero retains a fixed dwell.'
            ),
        ),
        DeclareLaunchArgument(
            'scan_step_count',
            default_value='6',
            description='Equal spin-and-observe steps in one 360-degree scan.',
        ),
        DeclareLaunchArgument(
            'spin_time_allowance',
            default_value='15.0',
            description='Maximum Nav2 spin execution time in seconds.',
        ),
        DeclareLaunchArgument(
            'navigation_settle_time',
            default_value='0.25',
            description='Short stabilization delay before scanning.',
        ),
        DeclareLaunchArgument(
            'finish_active_frontier_goal_before_scan',
            default_value='true',
            description=(
                'Let the current frontier Nav2 goal finish after the minimum '
                'exploration interval.'
            ),
        ),
        DeclareLaunchArgument(
            'exploration_bag_enabled',
            default_value='true',
            description=(
                'Expect a standalone recorder and wait for its ready status.'
            ),
        ),
        DeclareLaunchArgument(
            'launch_exploration_bag_recorder',
            default_value='true',
            description=(
                'Launch the recorder here; false permits a separately '
                'started recorder.'
            ),
        ),
        DeclareLaunchArgument(
            'exploration_bag_required',
            default_value='false',
            description='Abort the mission if its rosbag cannot be opened.',
        ),
        DeclareLaunchArgument(
            'exploration_bag_start_timeout',
            default_value='2.0',
            description='Recorder-ready handshake timeout in seconds.',
        ),
        DeclareLaunchArgument(
            'exploration_bag_params_file',
            default_value=default_exploration_bag_params,
            description='Standalone exploration recorder parameter file.',
        ),
        DeclareLaunchArgument(
            'exploration_bag_output_directory',
            default_value='/opt/muto_rs_ws/bags',
            description='Parent directory for mission bags.',
        ),
        DeclareLaunchArgument(
            'exploration_bag_storage_id',
            default_value='mcap',
            description='rosbag2 storage plugin used for mission bags.',
        ),
        DeclareLaunchArgument(
            'exploration_bag_storage_preset',
            default_value='none',
            description='Storage preset such as none or zstd_fast.',
        ),
        DeclareLaunchArgument(
            'exploration_bag_topics_regex',
            default_value='',
            description=(
                'Optional inclusion regex; empty discovers all topics before '
                'applying the exclusion regex.'
            ),
        ),
        DeclareLaunchArgument(
            'exploration_bag_exclude_regex',
            default_value=DEFAULT_EXPLORATION_BAG_EXCLUDE_REGEX,
            description=(
                'Topic exclusion regex; the default omits high-bandwidth '
                'images and point clouds. Pass an empty value to retain all.'
            ),
        ),
        DeclareLaunchArgument(
            'exploration_bag_max_cache_size',
            default_value='104857600',
            description='Recorder write cache in bytes; zero disables it.',
        ),
        DeclareLaunchArgument(
            'exploration_bag_post_result_delay',
            default_value='0.25',
            description='Time to capture terminal events before bag closure.',
        ),
        DeclareLaunchArgument(
            'exploration_bag_event_topic',
            default_value='/explore_and_record/recording_event',
            description='Mission recording lifecycle event topic.',
        ),
        DeclareLaunchArgument(
            'exploration_bag_path_topic',
            default_value='/explore_and_record/last_bag_path',
            description='Transient topic containing the latest bag path.',
        ),
        DeclareLaunchArgument(
            'exploration_bag_status_topic',
            default_value='/explore_and_record/bag_status',
            description='Standalone recorder readiness and status topic.',
        ),
        DeclareLaunchArgument(
            'exploration_operator_event_topic',
            default_value='/explore_and_record/operator_event',
            description='Plain-text manual observation and milestone topic.',
        ),
        DeclareLaunchArgument(
            'command_bag_enabled',
            default_value='true',
            description=(
                'Record one parent bag for each complete model-commander '
                'mission.'
            ),
        ),
        DeclareLaunchArgument(
            'launch_command_bag_recorder',
            default_value='true',
            description=(
                'Launch the command recorder here; false permits a '
                'separately started recorder.'
            ),
        ),
        DeclareLaunchArgument(
            'command_bag_required',
            default_value='false',
            description='Abort the command mission if its bag cannot open.',
        ),
        DeclareLaunchArgument(
            'command_bag_start_timeout',
            default_value='2.0',
            description='Command-recorder ready timeout in seconds.',
        ),
        DeclareLaunchArgument(
            'command_bag_params_file',
            default_value=default_command_bag_params,
            description='Command mission recorder parameter file.',
        ),
        DeclareLaunchArgument(
            'command_bag_output_directory',
            default_value='/opt/muto_rs_ws/bags',
        ),
        DeclareLaunchArgument('command_bag_storage_id', default_value='mcap'),
        DeclareLaunchArgument(
            'command_bag_storage_preset', default_value='none'),
        DeclareLaunchArgument('command_bag_topics_regex', default_value=''),
        DeclareLaunchArgument(
            'command_bag_exclude_regex',
            default_value=DEFAULT_EXPLORATION_BAG_EXCLUDE_REGEX,
        ),
        DeclareLaunchArgument(
            'command_bag_max_cache_size', default_value='104857600'),
        DeclareLaunchArgument(
            'command_bag_post_result_delay', default_value='0.5'),
        DeclareLaunchArgument(
            'command_bag_event_topic',
            default_value='/model_commander/recording_event',
        ),
        DeclareLaunchArgument(
            'command_bag_path_topic',
            default_value='/model_commander/last_bag_path',
        ),
        DeclareLaunchArgument(
            'command_bag_status_topic',
            default_value='/model_commander/bag_status',
        ),
        DeclareLaunchArgument(
            'command_operator_event_topic',
            default_value='/model_commander/operator_event',
        ),
        DeclareLaunchArgument(
            'model_commander_decision_event_topic',
            default_value='/model_commander/decision_event',
        ),
        DeclareLaunchArgument(
            'model_commander_inspected_image_topic',
            default_value='/model_commander/inspected_image',
        ),
        DeclareLaunchArgument(
            'natural_language_decision_event_topic',
            default_value='/natural_language_command/decision_event',
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
            default_value='false',
            description='Start the deprecated FindSomething compatibility server.',
        ),
        DeclareLaunchArgument(
            'find_something_action',
            default_value='/find_something',
            description='Public active object-search action name.',
        ),
        DeclareLaunchArgument(
            'launch_model_commander',
            default_value='true',
            description=(
                'Start the persistent model-supervised object commander.'
            ),
        ),
        DeclareLaunchArgument(
            'look_for_object_action',
            default_value='/look_for_object',
            description='Public model-supervised object-search action name.',
        ),
        DeclareLaunchArgument(
            'model_commander_status_topic',
            default_value='/model_commander/status',
            description='Transient model-commander heartbeat and state.',
        ),
        DeclareLaunchArgument(
            'registry_topic',
            default_value='/sam2/stored_objects',
            description='Transient snapshot of confirmed static objects.',
        ),
        DeclareLaunchArgument(
            'robot_pose_topic',
            default_value='/odometry/filtered',
            description=(
                'Odometry topic sampled into model-commander decision memory.'
            ),
        ),
        DeclareLaunchArgument(
            'vlm_action',
            default_value='/vlm/generate',
            description=(
                'GenerateVlm action used for search, routing, and planning.'
            ),
        ),
        DeclareLaunchArgument(
            'object_match_topic',
            default_value='/object_search/matches',
            description='Topic receiving one message per final match.',
        ),
        DeclareLaunchArgument(
            'vlm_model',
            default_value='gpt-5.6-sol',
            description='Default model used by the VLM socket.',
        ),
        DeclareLaunchArgument(
            'object_search_vlm_model',
            default_value='gpt-5.6-sol',
            description='Model used by registry object matching.',
        ),
        DeclareLaunchArgument(
            'model_commander_vlm_model',
            default_value='gpt-5.6-luna',
            description='Fast model used by persistent command planning.',
        ),
        DeclareLaunchArgument(
            'model_commander_openblas_preload',
            default_value='/usr/lib/aarch64-linux-gnu/libopenblas.so.0',
            description=(
                'OpenBLAS library preloaded for model_commander camera '
                'conversion on Jetson/Humble.'
            ),
        ),
        DeclareLaunchArgument(
            'natural_language_vlm_model',
            default_value='gpt-5.3-codex-spark',
            description='Fast model used by natural-language command routing.',
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
        object_pipeline,
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
            package='muto_exploration_bag',
            executable='exploration_bag_recorder',
            name='exploration_bag_recorder',
            output='screen',
            condition=IfCondition(
                AndSubstitution(
                    LaunchConfiguration('exploration_bag_enabled'),
                    LaunchConfiguration('launch_exploration_bag_recorder'),
                )
            ),
            parameters=[
                LaunchConfiguration('exploration_bag_params_file'),
                {
                    'use_sim_time': ParameterValue(
                        LaunchConfiguration('use_sim_time'),
                        value_type=bool,
                    ),
                    'output_directory': LaunchConfiguration(
                        'exploration_bag_output_directory'
                    ),
                    'storage_id': LaunchConfiguration(
                        'exploration_bag_storage_id'
                    ),
                    'storage_preset': LaunchConfiguration(
                        'exploration_bag_storage_preset'
                    ),
                    'topics_regex': LaunchConfiguration(
                        'exploration_bag_topics_regex'
                    ),
                    'exclude_regex': LaunchConfiguration(
                        'exploration_bag_exclude_regex'
                    ),
                    'max_cache_size': ParameterValue(
                        LaunchConfiguration('exploration_bag_max_cache_size'),
                        value_type=int,
                    ),
                    'post_terminal_delay': ParameterValue(
                        LaunchConfiguration(
                            'exploration_bag_post_result_delay'
                        ),
                        value_type=float,
                    ),
                    'lifecycle_event_topic': LaunchConfiguration(
                        'exploration_bag_event_topic'
                    ),
                    'status_topic': LaunchConfiguration(
                        'exploration_bag_status_topic'
                    ),
                    'path_topic': LaunchConfiguration(
                        'exploration_bag_path_topic'
                    ),
                    'operator_event_topic': LaunchConfiguration(
                        'exploration_operator_event_topic'
                    ),
                },
            ],
        ),
        Node(
            package='muto_exploration_bag',
            executable='exploration_bag_recorder',
            name='command_bag_recorder',
            output='screen',
            condition=IfCondition(
                AndSubstitution(
                    LaunchConfiguration('command_bag_enabled'),
                    LaunchConfiguration('launch_command_bag_recorder'),
                )
            ),
            parameters=[
                LaunchConfiguration('command_bag_params_file'),
                {
                    'use_sim_time': ParameterValue(
                        LaunchConfiguration('use_sim_time'),
                        value_type=bool,
                    ),
                    'output_directory': LaunchConfiguration(
                        'command_bag_output_directory'
                    ),
                    'storage_id': LaunchConfiguration(
                        'command_bag_storage_id'
                    ),
                    'storage_preset': LaunchConfiguration(
                        'command_bag_storage_preset'
                    ),
                    'topics_regex': LaunchConfiguration(
                        'command_bag_topics_regex'
                    ),
                    'exclude_regex': LaunchConfiguration(
                        'command_bag_exclude_regex'
                    ),
                    'max_cache_size': ParameterValue(
                        LaunchConfiguration('command_bag_max_cache_size'),
                        value_type=int,
                    ),
                    'post_terminal_delay': ParameterValue(
                        LaunchConfiguration('command_bag_post_result_delay'),
                        value_type=float,
                    ),
                    'lifecycle_event_topic': LaunchConfiguration(
                        'command_bag_event_topic'
                    ),
                    'status_topic': LaunchConfiguration(
                        'command_bag_status_topic'
                    ),
                    'path_topic': LaunchConfiguration(
                        'command_bag_path_topic'
                    ),
                    'operator_event_topic': LaunchConfiguration(
                        'command_operator_event_topic'
                    ),
                    'bag_prefix': 'muto_command',
                    'manifest_schema': 'command_mission_v1',
                    'status_schema': 'muto_command_bag_status_v1',
                    'recording_label': 'command_mission',
                },
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
                    'explore_frontier_action': LaunchConfiguration(
                        'explore_frontier_action'
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
                    'detection_heartbeat_topic': LaunchConfiguration(
                        'detection_heartbeat_topic'
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
                    'observation_min_detection_frames': LaunchConfiguration(
                        'observation_min_detection_frames'
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
                    'finish_active_frontier_goal_before_scan': ParameterValue(
                        LaunchConfiguration(
                            'finish_active_frontier_goal_before_scan'
                        ),
                        value_type=bool,
                    ),
                    'exploration_bag_enabled': ParameterValue(
                        LaunchConfiguration('exploration_bag_enabled'),
                        value_type=bool,
                    ),
                    'exploration_bag_required': ParameterValue(
                        LaunchConfiguration('exploration_bag_required'),
                        value_type=bool,
                    ),
                    'exploration_bag_start_timeout': ParameterValue(
                        LaunchConfiguration('exploration_bag_start_timeout'),
                        value_type=float,
                    ),
                    'exploration_bag_event_topic': LaunchConfiguration(
                        'exploration_bag_event_topic'
                    ),
                    'exploration_bag_status_topic': LaunchConfiguration(
                        'exploration_bag_status_topic'
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
                    'vlm_model': LaunchConfiguration('object_search_vlm_model'),
                },
            ],
        ),
        Node(
            package='muto_command_layer',
            executable='model_commander_node',
            name='model_commander',
            output='screen',
            condition=IfCondition(
                LaunchConfiguration('launch_model_commander')
            ),
            additional_env={
                'LD_PRELOAD': LaunchConfiguration(
                    'model_commander_openblas_preload'
                ),
            },
            parameters=[
                LaunchConfiguration('params_file'),
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'action_name': LaunchConfiguration(
                        'look_for_object_action'
                    ),
                    'vlm_action': LaunchConfiguration('vlm_action'),
                    'find_object_action': LaunchConfiguration(
                        'find_object_action'
                    ),
                    'go_to_object_action': LaunchConfiguration('action_name'),
                    'explore_frontier_action': LaunchConfiguration(
                        'explore_frontier_action'
                    ),
                    'spin_action': LaunchConfiguration('spin_action'),
                    'rotate_cmd_vel_topic': LaunchConfiguration(
                        'rotate_cmd_vel_topic'
                    ),
                    'rotate_executable_yaw_velocity': ParameterValue(
                        LaunchConfiguration('rotate_executable_yaw_velocity'),
                        value_type=float,
                    ),
                    'rotate_goal_tolerance': ParameterValue(
                        LaunchConfiguration('rotate_goal_tolerance'),
                        value_type=float,
                    ),
                    'rotate_control_period': ParameterValue(
                        LaunchConfiguration('rotate_control_period'),
                        value_type=float,
                    ),
                    'rotate_stop_publish_count': ParameterValue(
                        LaunchConfiguration('rotate_stop_publish_count'),
                        value_type=int,
                    ),
                    'registry_save_service': LaunchConfiguration(
                        'registry_save_service'
                    ),
                    'detection_heartbeat_topic': LaunchConfiguration(
                        'detection_heartbeat_topic'
                    ),
                    'registry_topic': LaunchConfiguration('registry_topic'),
                    'robot_pose_topic': LaunchConfiguration(
                        'robot_pose_topic'
                    ),
                    'visual_observation_topic': LaunchConfiguration(
                        'image_topic'
                    ),
                    'status_topic': LaunchConfiguration(
                        'model_commander_status_topic'
                    ),
                    'command_bag_enabled': ParameterValue(
                        LaunchConfiguration('command_bag_enabled'),
                        value_type=bool,
                    ),
                    'command_bag_required': ParameterValue(
                        LaunchConfiguration('command_bag_required'),
                        value_type=bool,
                    ),
                    'command_bag_start_timeout': ParameterValue(
                        LaunchConfiguration('command_bag_start_timeout'),
                        value_type=float,
                    ),
                    'command_bag_event_topic': LaunchConfiguration(
                        'command_bag_event_topic'
                    ),
                    'command_bag_status_topic': LaunchConfiguration(
                        'command_bag_status_topic'
                    ),
                    'decision_event_topic': LaunchConfiguration(
                        'model_commander_decision_event_topic'
                    ),
                    'inspected_image_topic': LaunchConfiguration(
                        'model_commander_inspected_image_topic'
                    ),
                    'vlm_model': LaunchConfiguration('model_commander_vlm_model'),
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
                    'look_for_object_action': LaunchConfiguration(
                        'look_for_object_action'
                    ),
                    'go_to_object_action': LaunchConfiguration('action_name'),
                    'explore_service': LaunchConfiguration('explore_service'),
                    'save_map_service': LaunchConfiguration(
                        'save_map_service'
                    ),
                    'decision_event_topic': LaunchConfiguration(
                        'natural_language_decision_event_topic'
                    ),
                    'save_map_result_timeout': LaunchConfiguration(
                        'save_map_result_timeout'
                    ),
                    'vlm_model': LaunchConfiguration('natural_language_vlm_model'),
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
