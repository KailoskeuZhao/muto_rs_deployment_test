from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument(
            'pointcloud_topic', default_value='/sam2/instance_pointcloud'),
        DeclareLaunchArgument(
            'detections_topic', default_value='/sam2/detections'),
        DeclareLaunchArgument(
            'objects_topic', default_value='/sam2/stored_objects'),
        DeclareLaunchArgument(
            'marker_topic', default_value='/sam2/stored_object_markers'),
        DeclareLaunchArgument(
            'query_service', default_value='/sam2/get_stored_objects'),
        DeclareLaunchArgument(
            'save_service', default_value='/sam2/save_stored_objects'),
        DeclareLaunchArgument(
            'clear_service', default_value='/sam2/clear_stored_objects'),
        DeclareLaunchArgument(
            'output_yaml',
            default_value='',
            description=(
                'Registry YAML path. Empty resolves to sam2_objects.yaml in '
                'the active colcon workspace root.'
            )),
        DeclareLaunchArgument('target_frame', default_value='map'),
        DeclareLaunchArgument(
            'duplicate_distance_threshold', default_value='0.25'),
        DeclareLaunchArgument(
            'metadata_sync_tolerance', default_value='0.2'),
        DeclareLaunchArgument('sync_queue_size', default_value='10'),
        DeclareLaunchArgument('min_points', default_value='20'),
        DeclareLaunchArgument('yolo_confidence', default_value='0.4'),
        DeclareLaunchArgument(
            'confirmation_min_observations', default_value='3'),
        DeclareLaunchArgument(
            'confirmation_min_average_confidence', default_value='0.6'),
        DeclareLaunchArgument('confirmation_window', default_value='3.0'),
        DeclareLaunchArgument('confirmation_max_gap', default_value='1.5'),
        DeclareLaunchArgument('tf_timeout', default_value='0.1'),
        DeclareLaunchArgument('tf_cache_time', default_value='30.0'),
        DeclareLaunchArgument('snapshot_publish_rate', default_value='2.0'),
        DeclareLaunchArgument('marker_scale', default_value='0.12'),
        DeclareLaunchArgument('marker_text_height', default_value='0.12'),
        DeclareLaunchArgument('marker_text_offset', default_value='0.15'),
        DeclareLaunchArgument('save_on_shutdown', default_value='true'),
    ]

    node = Node(
        package='sam2_object_registry',
        executable='object_registry_node',
        name='object_registry',
        output='screen',
        parameters=[{
            'pointcloud_topic': LaunchConfiguration('pointcloud_topic'),
            'detections_topic': LaunchConfiguration('detections_topic'),
            'objects_topic': LaunchConfiguration('objects_topic'),
            'marker_topic': LaunchConfiguration('marker_topic'),
            'query_service': LaunchConfiguration('query_service'),
            'save_service': LaunchConfiguration('save_service'),
            'clear_service': LaunchConfiguration('clear_service'),
            'output_yaml': LaunchConfiguration('output_yaml'),
            'target_frame': LaunchConfiguration('target_frame'),
            'duplicate_distance_threshold': ParameterValue(
                LaunchConfiguration('duplicate_distance_threshold'),
                value_type=float),
            'metadata_sync_tolerance': ParameterValue(
                LaunchConfiguration('metadata_sync_tolerance'),
                value_type=float),
            'sync_queue_size': ParameterValue(
                LaunchConfiguration('sync_queue_size'), value_type=int),
            'min_points': ParameterValue(
                LaunchConfiguration('min_points'), value_type=int),
            'yolo_confidence': ParameterValue(
                LaunchConfiguration('yolo_confidence'), value_type=float),
            'confirmation_min_observations': ParameterValue(
                LaunchConfiguration('confirmation_min_observations'),
                value_type=int),
            'confirmation_min_average_confidence': ParameterValue(
                LaunchConfiguration('confirmation_min_average_confidence'),
                value_type=float),
            'confirmation_window': ParameterValue(
                LaunchConfiguration('confirmation_window'), value_type=float),
            'confirmation_max_gap': ParameterValue(
                LaunchConfiguration('confirmation_max_gap'), value_type=float),
            'tf_timeout': ParameterValue(
                LaunchConfiguration('tf_timeout'), value_type=float),
            'tf_cache_time': ParameterValue(
                LaunchConfiguration('tf_cache_time'), value_type=float),
            'snapshot_publish_rate': ParameterValue(
                LaunchConfiguration('snapshot_publish_rate'), value_type=float),
            'marker_scale': ParameterValue(
                LaunchConfiguration('marker_scale'), value_type=float),
            'marker_text_height': ParameterValue(
                LaunchConfiguration('marker_text_height'), value_type=float),
            'marker_text_offset': ParameterValue(
                LaunchConfiguration('marker_text_offset'), value_type=float),
            'save_on_shutdown': ParameterValue(
                LaunchConfiguration('save_on_shutdown'), value_type=bool),
        }],
    )

    return LaunchDescription(arguments + [node])
