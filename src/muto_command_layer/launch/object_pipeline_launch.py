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

    arguments = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description=(
                'Use simulation time across the object-identification '
                'pipeline.'
            ),
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
            'detection_heartbeat_topic',
            default_value='/sam2/detection_heartbeat',
            description='Crop-free completion heartbeat for detector frames.',
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
            'registry_save_service',
            default_value='/sam2/save_stored_objects',
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
            'registry_load_existing',
            default_value='false',
            description=(
                'Preserve persisted objects only when their saved map is '
                'being restored.'
            ),
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
            default_value='http://43.165.176.234:8080/v1',
        ),
        DeclareLaunchArgument(
            'vlm_wire_api',
            default_value='chat_completions',
            description='VLM protocol: responses or chat_completions.',
        ),
        DeclareLaunchArgument(
            'vlm_model',
            default_value='gpt-5.6-sol',
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
            'load_existing': LaunchConfiguration(
                'registry_load_existing'),
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
    return LaunchDescription(arguments + [annotator, registry, vlm_socket])
