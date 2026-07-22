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
            'query_service', default_value='/sam2/get_stored_objects'),
        DeclareLaunchArgument(
            'save_service', default_value='/sam2/save_stored_objects'),
        DeclareLaunchArgument(
            'output_yaml', default_value='~/.ros/sam2_objects.yaml'),
        DeclareLaunchArgument('target_frame', default_value='map'),
        DeclareLaunchArgument(
            'duplicate_distance_threshold', default_value='0.25'),
        DeclareLaunchArgument(
            'metadata_sync_tolerance', default_value='0.2'),
        DeclareLaunchArgument('sync_queue_size', default_value='10'),
        DeclareLaunchArgument('min_points', default_value='20'),
        DeclareLaunchArgument('yolo_confidence', default_value='0.4'),
        DeclareLaunchArgument('tf_timeout', default_value='0.1'),
        DeclareLaunchArgument('tf_cache_time', default_value='30.0'),
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
            'query_service': LaunchConfiguration('query_service'),
            'save_service': LaunchConfiguration('save_service'),
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
            'tf_timeout': ParameterValue(
                LaunchConfiguration('tf_timeout'), value_type=float),
            'tf_cache_time': ParameterValue(
                LaunchConfiguration('tf_cache_time'), value_type=float),
            'save_on_shutdown': ParameterValue(
                LaunchConfiguration('save_on_shutdown'), value_type=bool),
        }],
    )

    return LaunchDescription(arguments + [node])
