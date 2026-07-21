from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument(
            "pointcloud_topic",
            default_value="/sam2/instance_pointcloud",
            description="Instance-marked PointCloud2 input topic.",
        ),
        DeclareLaunchArgument(
            "segments_topic",
            default_value="/sam2/segments",
            description="YOLO/SAM JSON metadata input topic.",
        ),
        DeclareLaunchArgument(
            "save_service",
            default_value="/sam2/save_object_centroids",
            description="Trigger service that atomically writes the YAML database.",
        ),
        DeclareLaunchArgument(
            "output_yaml",
            default_value="~/.ros/sam2_objects.yaml",
            description="Object-centroid YAML output path.",
        ),
        DeclareLaunchArgument(
            "target_frame",
            default_value="map",
            description="TF frame in which object centroids are stored.",
        ),
        DeclareLaunchArgument(
            "duplicate_distance_threshold",
            default_value="0.25",
            description="Same-label centroid merge distance in metres.",
        ),
        DeclareLaunchArgument(
            "metadata_sync_tolerance",
            default_value="0.15",
            description="Maximum cloud/metadata timestamp offset in seconds.",
        ),
        DeclareLaunchArgument(
            "sync_queue_size",
            default_value="10",
            description="Number of unmatched clouds and metadata messages retained.",
        ),
        DeclareLaunchArgument(
            "min_points",
            default_value="20",
            description="Minimum instance points required for a centroid.",
        ),
        DeclareLaunchArgument(
            "tf_timeout",
            default_value="0.1",
            description="Timestamped target-frame TF lookup timeout in seconds.",
        ),
    ]

    recorder = Node(
        package="sam2_image_annotator",
        executable="object_centroid_recorder_node",
        name="object_centroid_recorder",
        output="screen",
        parameters=[{
            "pointcloud_topic": LaunchConfiguration("pointcloud_topic"),
            "segments_topic": LaunchConfiguration("segments_topic"),
            "save_service": LaunchConfiguration("save_service"),
            "output_yaml": LaunchConfiguration("output_yaml"),
            "target_frame": LaunchConfiguration("target_frame"),
            "duplicate_distance_threshold": ParameterValue(
                LaunchConfiguration("duplicate_distance_threshold"),
                value_type=float,
            ),
            "metadata_sync_tolerance": ParameterValue(
                LaunchConfiguration("metadata_sync_tolerance"),
                value_type=float,
            ),
            "sync_queue_size": ParameterValue(
                LaunchConfiguration("sync_queue_size"),
                value_type=int,
            ),
            "min_points": ParameterValue(
                LaunchConfiguration("min_points"),
                value_type=int,
            ),
            "tf_timeout": ParameterValue(
                LaunchConfiguration("tf_timeout"),
                value_type=float,
            ),
        }],
    )

    return LaunchDescription(arguments + [recorder])
