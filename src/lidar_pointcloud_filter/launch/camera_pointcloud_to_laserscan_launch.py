from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
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
        default_value="/fused/laserscan",
        description="Output LaserScan topic.",
    )
    processing_frame_arg = DeclareLaunchArgument(
        "processing_frame",
        default_value="camera_link",
        description="Frame used for camera/LiDAR projection, z filtering, and scan output.",
    )
    raw_lidar_topic_arg = DeclareLaunchArgument(
        "raw_lidar_topic",
        default_value="/lidar/PointCloud",
        description="Raw LiDAR PointCloud2 topic consumed by the LiDAR filter.",
    )
    filtered_lidar_topic_arg = DeclareLaunchArgument(
        "filtered_lidar_topic",
        default_value="/lidar/PointCloudFiltered",
        description="Downsampled filtered LiDAR PointCloud2 topic.",
    )
    filtered_lidar_no_downsample_topic_arg = DeclareLaunchArgument(
        "filtered_lidar_no_downsample_topic",
        default_value="/lidar/PointCloudFilteredNoDownsample",
        description="Filtered LiDAR PointCloud2 topic before voxel downsampling.",
    )
    lidar_topic_arg = DeclareLaunchArgument(
        "lidar_topic",
        default_value=LaunchConfiguration("filtered_lidar_no_downsample_topic"),
        description="LiDAR PointCloud2 topic to merge into the scan.",
    )
    launch_lidar_filter_arg = DeclareLaunchArgument(
        "launch_lidar_filter",
        default_value="true",
        description="Whether to launch the LiDAR PointCloud2 filter before scan synthesis.",
    )
    lidar_filter_target_frame_arg = DeclareLaunchArgument(
        "lidar_filter_target_frame",
        default_value="base_frame",
        description="Frame that filtered LiDAR point clouds are transformed into.",
    )
    voxel_leaf_size_arg = DeclareLaunchArgument(
        "voxel_leaf_size",
        default_value="0.02",
        description="LiDAR filter voxel leaf size in meters. 0.0 disables downsampling.",
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
        description="Maximum depth camera scan range in meters.",
    )
    lidar_range_max_arg = DeclareLaunchArgument(
        "lidar_range_max",
        default_value="15.0",
        description="Maximum LiDAR scan range in meters.",
    )
    angle_min_arg = DeclareLaunchArgument(
        "angle_min",
        default_value="-3.141592653589793",
        description="Minimum scan angle in radians. Default is full-circle -pi.",
    )
    angle_max_arg = DeclareLaunchArgument(
        "angle_max",
        default_value="3.141592653589793",
        description="Maximum scan angle in radians. Default is full-circle pi.",
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
        processing_frame_arg,
        raw_lidar_topic_arg,
        filtered_lidar_topic_arg,
        filtered_lidar_no_downsample_topic_arg,
        lidar_topic_arg,
        launch_lidar_filter_arg,
        lidar_filter_target_frame_arg,
        voxel_leaf_size_arg,
        use_lidar_arg,
        min_z_arg,
        max_z_arg,
        range_max_arg,
        lidar_range_max_arg,
        angle_min_arg,
        angle_max_arg,
        queue_size_arg,
        max_lidar_age_arg,
        transform_timeout_arg,
        Node(
            package="lidar_pointcloud_filter",
            executable="lidar_pointcloud_filter_node",
            name="lidar_pointcloud_filter_node",
            output="screen",
            condition=IfCondition(LaunchConfiguration("launch_lidar_filter")),
            parameters=[{
                "input_topic": LaunchConfiguration("raw_lidar_topic"),
                "output_topic": LaunchConfiguration("filtered_lidar_topic"),
                "no_downsample_output_topic": LaunchConfiguration(
                    "filtered_lidar_no_downsample_topic"
                ),
                "target_frame": LaunchConfiguration("lidar_filter_target_frame"),
                "voxel_leaf_size": ParameterValue(
                    LaunchConfiguration("voxel_leaf_size"),
                    value_type=float,
                ),
                "queue_size": ParameterValue(LaunchConfiguration("queue_size"), value_type=int),
                "transform_timeout": ParameterValue(
                    LaunchConfiguration("transform_timeout"),
                    value_type=float,
                ),
            }],
        ),
        Node(
            package="lidar_pointcloud_filter",
            executable="camera_pointcloud_to_laserscan_node",
            name="camera_pointcloud_to_laserscan_node",
            output="screen",
            parameters=[{
                "input_topic": LaunchConfiguration("input_topic"),
                "lidar_topic": LaunchConfiguration("lidar_topic"),
                "output_topic": LaunchConfiguration("output_topic"),
                "processing_frame": LaunchConfiguration("processing_frame"),
                "use_lidar": ParameterValue(LaunchConfiguration("use_lidar"), value_type=bool),
                "min_z": ParameterValue(LaunchConfiguration("min_z"), value_type=float),
                "max_z": ParameterValue(LaunchConfiguration("max_z"), value_type=float),
                "range_max": ParameterValue(LaunchConfiguration("range_max"), value_type=float),
                "lidar_range_max": ParameterValue(
                    LaunchConfiguration("lidar_range_max"),
                    value_type=float,
                ),
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
