import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter
from launch_ros.parameter_descriptions import ParameterValue


def include_launch(package, filename, arguments):
    """Include one package launch in an isolated argument/parameter scope."""
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
    default_vlm_params = os.path.join(
        get_package_share_directory('muto_command_layer'),
        'config',
        'object_pipeline_vlm.yaml',
    )
    default_command_params = os.path.join(
        get_package_share_directory('muto_command_layer'),
        'config',
        'command_layer.yaml',
    )
    default_frontier_params = os.path.join(
        get_package_share_directory('muto_slam_mapping'),
        'config',
        'frontier_exploration_params.yaml',
    )

    arguments = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time across the complete pipeline.',
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
            description='Typed detections and representative JPEG crops.',
        ),
        DeclareLaunchArgument(
            'instance_pointcloud_topic',
            default_value='/sam2/instance_pointcloud',
            description='Per-instance point cloud consumed by the registry.',
        ),
        DeclareLaunchArgument(
            'yolo_model',
            default_value='yolo26m.pt',
        ),
        DeclareLaunchArgument(
            'yolo_device',
            default_value='0',
        ),
        DeclareLaunchArgument(
            'yolo_confidence',
            default_value='0.4',
        ),
        DeclareLaunchArgument(
            'detection_crop_jpeg_quality',
            default_value='90',
        ),
        DeclareLaunchArgument(
            'max_publish_rate',
            default_value='7.0',
            description='Maximum YOLO/SAM processing start rate in Hz.',
        ),
        DeclareLaunchArgument(
            'registry_service',
            default_value='/sam2/get_stored_objects',
        ),
        DeclareLaunchArgument(
            'registry_objects_topic',
            default_value='/sam2/stored_objects',
        ),
        DeclareLaunchArgument(
            'registry_output_yaml',
            default_value='',
            description='Empty stores sam2_objects.yaml in the workspace root.',
        ),
        DeclareLaunchArgument(
            'registry_image_directory',
            default_value='',
            description='Empty stores images beside the registry YAML.',
        ),
        DeclareLaunchArgument(
            'registry_store_images',
            default_value='true',
        ),
        DeclareLaunchArgument(
            'target_frame',
            default_value='map',
            description='Registry storage and Nav2 global frame.',
        ),
        DeclareLaunchArgument(
            'registry_tf_retry_window',
            default_value='1.0',
            description=(
                'Maximum seconds to retain a paired observation while its '
                'exact timestamped TF becomes available.'
            ),
        ),
        DeclareLaunchArgument(
            'registry_tf_retry_rate',
            default_value='20.0',
            description='Rate in Hz for exact timestamped registry TF retries.',
        ),
        DeclareLaunchArgument(
            'vlm_params_file',
            default_value=default_vlm_params,
            description='Deployment-level VLM socket parameter file.',
        ),
        DeclareLaunchArgument(
            'vlm_action',
            default_value='/vlm/generate',
        ),
        DeclareLaunchArgument(
            'vlm_base_url',
            default_value='http://43.165.176.234:8080/',
        ),
        DeclareLaunchArgument(
            'vlm_wire_api',
            default_value='responses',
            description='VLM protocol: responses or chat_completions.',
        ),
        DeclareLaunchArgument(
            'vlm_model',
            default_value='gpt-5.6-sol',
        ),
        DeclareLaunchArgument(
            'command_params_file',
            default_value=default_command_params,
        ),
        DeclareLaunchArgument(
            'go_to_object_action',
            default_value='/go_to_object',
        ),
        DeclareLaunchArgument(
            'find_object_action',
            default_value='/find_object',
        ),
        DeclareLaunchArgument(
            'find_something_action',
            default_value='/find_something',
        ),
        DeclareLaunchArgument(
            'launch_natural_language_command',
            default_value='true',
            description='Start the validated VLM command router.',
        ),
        DeclareLaunchArgument(
            'natural_language_command_action',
            default_value='/natural_language_command',
        ),
        DeclareLaunchArgument(
            'object_match_topic',
            default_value='/object_search/matches',
        ),
        DeclareLaunchArgument(
            'navigate_to_pose_action',
            default_value='/navigate_to_pose',
        ),
        DeclareLaunchArgument(
            'global_costmap_service',
            default_value='/global_costmap/get_costmap',
        ),
        DeclareLaunchArgument(
            'global_costmap_timeout',
            default_value='5.0',
        ),
        DeclareLaunchArgument(
            'robot_base_frame',
            default_value='base_frame',
        ),
        DeclareLaunchArgument(
            'approach_distance',
            default_value='0.75',
        ),
        DeclareLaunchArgument(
            'approach_robot_radius',
            default_value='0.16',
        ),
        DeclareLaunchArgument(
            'approach_start_snap_distance',
            default_value='0.5',
        ),
        DeclareLaunchArgument(
            'approach_maximum_cost',
            default_value='252',
        ),
        DeclareLaunchArgument(
            'explore_service',
            default_value='/explore',
        ),
        DeclareLaunchArgument(
            'save_map_service',
            default_value='/save_map',
        ),
        DeclareLaunchArgument(
            'slam_toolbox_save_map_service',
            default_value='/slam_toolbox/save_map',
        ),
        DeclareLaunchArgument(
            'map_save_directory',
            default_value='',
        ),
        DeclareLaunchArgument(
            'default_map_name',
            default_value='muto_map',
        ),
        DeclareLaunchArgument(
            'save_map_timeout',
            default_value='10.0',
        ),
        DeclareLaunchArgument(
            'save_map_result_timeout',
            default_value='15.0',
        ),
        DeclareLaunchArgument(
            'explore_and_record_action',
            default_value='/explore_and_record',
        ),
        DeclareLaunchArgument(
            'frontier_control_service',
            default_value='/control_exploration',
        ),
        DeclareLaunchArgument(
            'spin_action',
            default_value='/spin',
        ),
        DeclareLaunchArgument(
            'registry_save_service',
            default_value='/sam2/save_stored_objects',
        ),
        DeclareLaunchArgument(
            'exploration_completion_topic',
            default_value='/explore/exploration_complete',
        ),
        DeclareLaunchArgument(
            'exploration_service_timeout',
            default_value='5.0',
        ),
        DeclareLaunchArgument(
            'program_endpoint_timeout',
            default_value='5.0',
        ),
        DeclareLaunchArgument(
            'exploration_cycle_duration',
            default_value='10.0',
        ),
        DeclareLaunchArgument(
            'observation_duration',
            default_value='3.0',
        ),
        DeclareLaunchArgument(
            'scan_step_count',
            default_value='8',
        ),
        DeclareLaunchArgument(
            'spin_time_allowance',
            default_value='15.0',
        ),
        DeclareLaunchArgument(
            'navigation_settle_time',
            default_value='1.0',
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
        ),
        DeclareLaunchArgument(
            'frontier_log_level',
            default_value='info',
        ),
    ]

    annotator = include_launch(
        'sam2_image_annotator',
        'sam2_image_annotator_launch.py',
        {
            'image_topic': LaunchConfiguration('image_topic'),
            'depth_topic': LaunchConfiguration('depth_topic'),
            'depth_camera_info_topic': LaunchConfiguration(
                'depth_camera_info_topic'),
            'color_camera_info_topic': LaunchConfiguration(
                'color_camera_info_topic'),
            'detections_topic': LaunchConfiguration('detections_topic'),
            'instance_pointcloud_topic': LaunchConfiguration(
                'instance_pointcloud_topic'),
            'yolo_model': LaunchConfiguration('yolo_model'),
            'yolo_device': LaunchConfiguration('yolo_device'),
            'yolo_confidence': LaunchConfiguration('yolo_confidence'),
            'detection_crop_jpeg_quality': LaunchConfiguration(
                'detection_crop_jpeg_quality'),
            'max_publish_rate': LaunchConfiguration('max_publish_rate'),
        },
    )
    registry = include_launch(
        'sam2_object_registry',
        'object_registry_launch.py',
        {
            'pointcloud_topic': LaunchConfiguration(
                'instance_pointcloud_topic'),
            'detections_topic': LaunchConfiguration('detections_topic'),
            'query_service': LaunchConfiguration('registry_service'),
            'objects_topic': LaunchConfiguration('registry_objects_topic'),
            'save_service': LaunchConfiguration('registry_save_service'),
            'output_yaml': LaunchConfiguration('registry_output_yaml'),
            'image_directory': LaunchConfiguration(
                'registry_image_directory'),
            'store_images': LaunchConfiguration('registry_store_images'),
            'target_frame': LaunchConfiguration('target_frame'),
            'yolo_confidence': LaunchConfiguration('yolo_confidence'),
            'tf_retry_window': LaunchConfiguration(
                'registry_tf_retry_window'),
            'tf_retry_rate': LaunchConfiguration('registry_tf_retry_rate'),
        },
    )
    vlm_socket = include_launch(
        'muto_vlm_socket',
        'vlm_socket_launch.py',
        {
            'params_file': LaunchConfiguration('vlm_params_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'action_name': LaunchConfiguration('vlm_action'),
            'base_url': LaunchConfiguration('vlm_base_url'),
            'wire_api': LaunchConfiguration('vlm_wire_api'),
            'default_model': LaunchConfiguration('vlm_model'),
        },
    )
    command_layer = include_launch(
        'muto_command_layer',
        'command_layer_launch.py',
        {
            'params_file': LaunchConfiguration('command_params_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'action_name': LaunchConfiguration('go_to_object_action'),
            'registry_service': LaunchConfiguration('registry_service'),
            'navigate_to_pose_action': LaunchConfiguration(
                'navigate_to_pose_action'),
            'global_costmap_service': LaunchConfiguration(
                'global_costmap_service'),
            'global_costmap_timeout': LaunchConfiguration(
                'global_costmap_timeout'),
            'global_frame': LaunchConfiguration('target_frame'),
            'robot_base_frame': LaunchConfiguration('robot_base_frame'),
            'approach_distance': LaunchConfiguration('approach_distance'),
            'approach_robot_radius': LaunchConfiguration(
                'approach_robot_radius'),
            'approach_start_snap_distance': LaunchConfiguration(
                'approach_start_snap_distance'),
            'approach_maximum_cost': LaunchConfiguration(
                'approach_maximum_cost'),
            'explore_service': LaunchConfiguration('explore_service'),
            'save_map_service': LaunchConfiguration('save_map_service'),
            'slam_toolbox_save_map_service': LaunchConfiguration(
                'slam_toolbox_save_map_service'),
            'map_save_directory': LaunchConfiguration('map_save_directory'),
            'default_map_name': LaunchConfiguration('default_map_name'),
            'save_map_timeout': LaunchConfiguration('save_map_timeout'),
            'save_map_result_timeout': LaunchConfiguration(
                'save_map_result_timeout'),
            'explore_and_record_action': LaunchConfiguration(
                'explore_and_record_action'),
            'frontier_control_service': LaunchConfiguration(
                'frontier_control_service'),
            'spin_action': LaunchConfiguration('spin_action'),
            'registry_save_service': LaunchConfiguration(
                'registry_save_service'),
            'exploration_completion_topic': LaunchConfiguration(
                'exploration_completion_topic'),
            'exploration_service_timeout': LaunchConfiguration(
                'exploration_service_timeout'),
            'program_endpoint_timeout': LaunchConfiguration(
                'program_endpoint_timeout'),
            'exploration_cycle_duration': LaunchConfiguration(
                'exploration_cycle_duration'),
            'observation_duration': LaunchConfiguration(
                'observation_duration'),
            'scan_step_count': LaunchConfiguration('scan_step_count'),
            'spin_time_allowance': LaunchConfiguration(
                'spin_time_allowance'),
            'navigation_settle_time': LaunchConfiguration(
                'navigation_settle_time'),
            'launch_frontier_explorer': LaunchConfiguration(
                'launch_frontier_explorer'),
            'frontier_params_file': LaunchConfiguration(
                'frontier_params_file'),
            'frontier_log_level': LaunchConfiguration(
                'frontier_log_level'),
            'launch_object_search': 'true',
            'find_object_action': LaunchConfiguration('find_object_action'),
            'launch_active_object_search': 'true',
            'find_something_action': LaunchConfiguration(
                'find_something_action'),
            'registry_topic': LaunchConfiguration('registry_objects_topic'),
            'launch_natural_language_command': LaunchConfiguration(
                'launch_natural_language_command'),
            'natural_language_command_action': LaunchConfiguration(
                'natural_language_command_action'),
            'vlm_action': LaunchConfiguration('vlm_action'),
            'object_match_topic': LaunchConfiguration('object_match_topic'),
            'vlm_model': LaunchConfiguration('vlm_model'),
        },
    )

    return LaunchDescription(
        arguments + [annotator, registry, vlm_socket, command_layer])
