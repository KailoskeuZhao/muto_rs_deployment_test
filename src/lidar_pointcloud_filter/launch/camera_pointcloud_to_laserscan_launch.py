from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    input_topic_arg = DeclareLaunchArgument(
        "input_topic",
        default_value="/camera/depth/points",
        description="Input camera depth PointCloud2 topic.",
    )
    output_topic_arg = DeclareLaunchArgument(
        "output_topic",
        default_value="/camera/depth/scan",
        description="Output LaserScan topic.",
    )
    lidar_topic_arg = DeclareLaunchArgument(
        "lidar_topic",
        default_value="lidar/PointCloud",
        description="Optional LiDAR PointCloud2 topic to merge into the scan.",
    )
    use_lidar_arg = DeclareLaunchArgument(
        "use_lidar",
        default_value="true",
        description="Whether to merge the latest LiDAR cloud into each camera scan.",
    )
    min_z_arg = DeclareLaunchArgument(
        "min_z",
        default_value="-0.4",
        description="Minimum original-frame z value to keep.",
    )
    max_z_arg = DeclareLaunchArgument(
        "max_z",
        default_value="0.2",
        description="Maximum original-frame z value to keep.",
    )
    range_max_arg = DeclareLaunchArgument(
        "range_max",
        default_value="3.0",
        description="Maximum scan range in meters.",
    )
    angle_min_arg = DeclareLaunchArgument(
        "angle_min",
        default_value="-0.5096361108",
        description="Minimum scan angle in radians. Default is -29.2 degrees.",
    )
    angle_max_arg = DeclareLaunchArgument(
        "angle_max",
        default_value="0.5096361108",
        description="Maximum scan angle in radians. Default is 29.2 degrees.",
    )
    queue_size_arg = DeclareLaunchArgument(
        "queue_size",
        default_value="5",
        description="Point cloud and scan queue size.",
    )
    max_lidar_age_arg = DeclareLaunchArgument(
        "max_lidar_age",
        default_value="0.5",
        description="Maximum timestamp difference between camera and LiDAR clouds in seconds.",
    )
    transform_timeout_arg = DeclareLaunchArgument(
        "transform_timeout",
        default_value="0.05",
        description="TF lookup timeout in seconds.",
    )

    return LaunchDescription([
        input_topic_arg,
        output_topic_arg,
        lidar_topic_arg,
        use_lidar_arg,
        min_z_arg,
        max_z_arg,
        range_max_arg,
        angle_min_arg,
        angle_max_arg,
        queue_size_arg,
        max_lidar_age_arg,
        transform_timeout_arg,
        Node(
            package="lidar_pointcloud_filter",
            executable="camera_pointcloud_to_laserscan_node",
            name="camera_pointcloud_to_laserscan_node",
            output="screen",
            parameters=[{
                "input_topic": LaunchConfiguration("input_topic"),
                "lidar_topic": LaunchConfiguration("lidar_topic"),
                "output_topic": LaunchConfiguration("output_topic"),
                "use_lidar": ParameterValue(LaunchConfiguration("use_lidar"), value_type=bool),
                "min_z": ParameterValue(LaunchConfiguration("min_z"), value_type=float),
                "max_z": ParameterValue(LaunchConfiguration("max_z"), value_type=float),
                "range_max": ParameterValue(LaunchConfiguration("range_max"), value_type=float),
                "angle_min": ParameterValue(LaunchConfiguration("angle_min"), value_type=float),
                "angle_max": ParameterValue(LaunchConfiguration("angle_max"), value_type=float),
                "queue_size": ParameterValue(LaunchConfiguration("queue_size"), value_type=int),
                "max_lidar_age": ParameterValue(
                    LaunchConfiguration("max_lidar_age"),
                    value_type=float,
                ),
                "transform_timeout": ParameterValue(
                    LaunchConfiguration("transform_timeout"),
                    value_type=float,
                ),
            }],
        ),
    ])
