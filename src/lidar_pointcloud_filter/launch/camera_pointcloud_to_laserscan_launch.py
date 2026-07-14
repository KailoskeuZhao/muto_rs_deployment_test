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
        description="Output fused LaserScan topic.",
    )
    camera_scan_topic_arg = DeclareLaunchArgument(
        "camera_scan_topic",
        default_value="/camera/filtered_laserscan",
        description="Intermediate downsampled camera LaserScan topic.",
    )
    lidar_scan_topic_arg = DeclareLaunchArgument(
        "lidar_scan_topic",
        default_value="/lidar/filtered_laserscan_no_downsample",
        description="Filtered no-downsample LiDAR LaserScan topic merged into the fused scan.",
    )
    publish_camera_scan_arg = DeclareLaunchArgument(
        "publish_camera_scan",
        default_value="true",
        description="Whether to convert the camera PointCloud2 into the intermediate LaserScan.",
    )
    publish_fused_scan_arg = DeclareLaunchArgument(
        "publish_fused_scan",
        default_value="true",
        description="Whether to publish the fused camera/LiDAR LaserScan.",
    )
    filtered_lidar_scan_topic_arg = DeclareLaunchArgument(
        "filtered_lidar_scan_topic",
        default_value="/lidar/filtered_laserscan",
        description="Output LaserScan topic generated from the filtered LiDAR PointCloud2 only.",
    )
    processing_frame_arg = DeclareLaunchArgument(
        "processing_frame",
        default_value="base_frame",
        description="Frame used for camera projection, z filtering, and intermediate scan output.",
    )
    fused_scan_frame_arg = DeclareLaunchArgument(
        "fused_scan_frame",
        default_value="base_frame",
        description="Frame used for the final fused LaserScan output.",
    )
    filtered_lidar_scan_frame_arg = DeclareLaunchArgument(
        "filtered_lidar_scan_frame",
        default_value="base_frame",
        description="Frame used for the filtered LiDAR PointCloud2-only LaserScan output.",
    )
    lidar_topic_arg = DeclareLaunchArgument(
        "lidar_topic",
        default_value="/lidar/PointCloudFilteredNoDownsample",
        description="Existing LiDAR PointCloud2 topic to merge into the scan.",
    )
    publish_filtered_lidar_scan_arg = DeclareLaunchArgument(
        "publish_filtered_lidar_scan",
        default_value="false",
        description=(
            "Legacy path: also publish a LaserScan converted from the filtered LiDAR "
            "PointCloud2 only."
        ),
    )
    require_lidar_scan_arg = DeclareLaunchArgument(
        "require_lidar_scan",
        default_value="true",
        description="Whether fused scan output must wait for a timestamp-matched LiDAR scan.",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock if true.",
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
    camera_angle_increment_arg = DeclareLaunchArgument(
        "camera_angle_increment",
        default_value="0.017453292519943295",
        description="Intermediate camera LaserScan angular resolution in radians.",
    )
    fused_angle_increment_arg = DeclareLaunchArgument(
        "fused_angle_increment",
        default_value="0.004363323129985824",
        description="Final fused LaserScan angular resolution in radians.",
    )
    queue_size_arg = DeclareLaunchArgument(
        "queue_size",
        default_value="1",
        description="Point cloud and scan queue size.",
    )
    max_publish_rate_arg = DeclareLaunchArgument(
        "max_publish_rate",
        default_value="0.0",
        description="Maximum generated LaserScan rate in Hz. Set 0.0 to process every input cloud.",
    )
    input_point_stride_arg = DeclareLaunchArgument(
        "input_point_stride",
        default_value="1",
        description="Process every Nth point from the primary input cloud.",
    )
    lidar_point_stride_arg = DeclareLaunchArgument(
        "lidar_point_stride",
        default_value="1",
        description="Legacy path: process every Nth point from the merged LiDAR cloud.",
    )
    max_lidar_age_arg = DeclareLaunchArgument(
        "max_lidar_age",
        default_value="0.5",
        description="Maximum timestamp difference between camera and LiDAR scans in seconds.",
    )
    restamp_output_arg = DeclareLaunchArgument(
        "restamp_output",
        default_value="false",
        description=(
            "Use current ROS time for generated LaserScan stamps. Leave false unless input "
            "driver stamps are known bad while the data is fresh."
        ),
    )
    input_stamp_warning_age_arg = DeclareLaunchArgument(
        "input_stamp_warning_age",
        default_value="1.0",
        description="Warn when an input cloud/scan stamp differs from this node clock by more seconds.",
    )
    max_input_age_arg = DeclareLaunchArgument(
        "max_input_age",
        default_value="2.0",
        description=(
            "Drop input clouds/scans whose stamp differs from this node clock by more seconds. "
            "Set 0.0 only when using intentionally non-live stamps."
        ),
    )
    processing_time_warning_arg = DeclareLaunchArgument(
        "processing_time_warning",
        default_value="0.05",
        description="Warn when one LaserScan conversion takes longer than this many seconds.",
    )
    transform_timeout_arg = DeclareLaunchArgument(
        "transform_timeout",
        default_value="0.05",
        description="TF lookup timeout in seconds.",
    )

    return LaunchDescription([
        input_topic_arg,
        output_topic_arg,
        camera_scan_topic_arg,
        lidar_scan_topic_arg,
        publish_camera_scan_arg,
        publish_fused_scan_arg,
        filtered_lidar_scan_topic_arg,
        processing_frame_arg,
        fused_scan_frame_arg,
        filtered_lidar_scan_frame_arg,
        lidar_topic_arg,
        publish_filtered_lidar_scan_arg,
        require_lidar_scan_arg,
        use_sim_time_arg,
        min_z_arg,
        max_z_arg,
        range_max_arg,
        lidar_range_max_arg,
        angle_min_arg,
        angle_max_arg,
        camera_angle_increment_arg,
        fused_angle_increment_arg,
        queue_size_arg,
        max_publish_rate_arg,
        input_point_stride_arg,
        lidar_point_stride_arg,
        max_lidar_age_arg,
        restamp_output_arg,
        input_stamp_warning_age_arg,
        max_input_age_arg,
        processing_time_warning_arg,
        transform_timeout_arg,
        Node(
            package="lidar_pointcloud_filter",
            executable="camera_pointcloud_to_laserscan_node",
            name="camera_pointcloud_to_laserscan_node",
            output="screen",
            condition=IfCondition(LaunchConfiguration("publish_camera_scan")),
            parameters=[{
                "input_topic": LaunchConfiguration("input_topic"),
                "lidar_topic": LaunchConfiguration("lidar_topic"),
                "output_topic": LaunchConfiguration("camera_scan_topic"),
                "processing_frame": LaunchConfiguration("processing_frame"),
                "use_lidar": False,
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"),
                    value_type=bool,
                ),
                "min_z": ParameterValue(LaunchConfiguration("min_z"), value_type=float),
                "max_z": ParameterValue(LaunchConfiguration("max_z"), value_type=float),
                "range_max": ParameterValue(LaunchConfiguration("range_max"), value_type=float),
                "lidar_range_max": ParameterValue(
                    LaunchConfiguration("lidar_range_max"),
                    value_type=float,
                ),
                "angle_min": ParameterValue(LaunchConfiguration("angle_min"), value_type=float),
                "angle_max": ParameterValue(LaunchConfiguration("angle_max"), value_type=float),
                "angle_increment": ParameterValue(
                    LaunchConfiguration("camera_angle_increment"),
                    value_type=float,
                ),
                "queue_size": ParameterValue(LaunchConfiguration("queue_size"), value_type=int),
                "max_publish_rate": ParameterValue(
                    LaunchConfiguration("max_publish_rate"),
                    value_type=float,
                ),
                "input_point_stride": ParameterValue(
                    LaunchConfiguration("input_point_stride"),
                    value_type=int,
                ),
                "lidar_point_stride": ParameterValue(
                    LaunchConfiguration("lidar_point_stride"),
                    value_type=int,
                ),
                "max_lidar_age": ParameterValue(
                    LaunchConfiguration("max_lidar_age"),
                    value_type=float,
                ),
                "restamp_output": ParameterValue(
                    LaunchConfiguration("restamp_output"),
                    value_type=bool,
                ),
                "input_stamp_warning_age": ParameterValue(
                    LaunchConfiguration("input_stamp_warning_age"),
                    value_type=float,
                ),
                "max_input_age": ParameterValue(
                    LaunchConfiguration("max_input_age"),
                    value_type=float,
                ),
                "processing_time_warning": ParameterValue(
                    LaunchConfiguration("processing_time_warning"),
                    value_type=float,
                ),
                "transform_timeout": ParameterValue(
                    LaunchConfiguration("transform_timeout"),
                    value_type=float,
                ),
            }],
        ),
        Node(
            package="lidar_pointcloud_filter",
            executable="laserscan_fusion_node",
            name="laserscan_fusion_node",
            output="screen",
            condition=IfCondition(LaunchConfiguration("publish_fused_scan")),
            parameters=[{
                "camera_scan_topic": LaunchConfiguration("camera_scan_topic"),
                "lidar_scan_topic": LaunchConfiguration("lidar_scan_topic"),
                "output_topic": LaunchConfiguration("output_topic"),
                "output_frame": LaunchConfiguration("fused_scan_frame"),
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"),
                    value_type=bool,
                ),
                "range_max": ParameterValue(
                    LaunchConfiguration("lidar_range_max"),
                    value_type=float,
                ),
                "angle_min": ParameterValue(LaunchConfiguration("angle_min"), value_type=float),
                "angle_max": ParameterValue(LaunchConfiguration("angle_max"), value_type=float),
                "angle_increment": ParameterValue(
                    LaunchConfiguration("fused_angle_increment"),
                    value_type=float,
                ),
                "queue_size": ParameterValue(LaunchConfiguration("queue_size"), value_type=int),
                "max_lidar_age": ParameterValue(
                    LaunchConfiguration("max_lidar_age"),
                    value_type=float,
                ),
                "require_lidar": ParameterValue(
                    LaunchConfiguration("require_lidar_scan"),
                    value_type=bool,
                ),
                "restamp_output": ParameterValue(
                    LaunchConfiguration("restamp_output"),
                    value_type=bool,
                ),
                "input_stamp_warning_age": ParameterValue(
                    LaunchConfiguration("input_stamp_warning_age"),
                    value_type=float,
                ),
                "max_input_age": ParameterValue(
                    LaunchConfiguration("max_input_age"),
                    value_type=float,
                ),
                "processing_time_warning": ParameterValue(
                    LaunchConfiguration("processing_time_warning"),
                    value_type=float,
                ),
                "transform_timeout": ParameterValue(
                    LaunchConfiguration("transform_timeout"),
                    value_type=float,
                ),
            }],
        ),
        Node(
            package="lidar_pointcloud_filter",
            executable="camera_pointcloud_to_laserscan_node",
            name="filtered_lidar_pointcloud_to_laserscan_node",
            output="screen",
            condition=IfCondition(LaunchConfiguration("publish_filtered_lidar_scan")),
            parameters=[{
                "input_topic": LaunchConfiguration("lidar_topic"),
                "output_topic": LaunchConfiguration("filtered_lidar_scan_topic"),
                "processing_frame": LaunchConfiguration("filtered_lidar_scan_frame"),
                "use_lidar": False,
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"),
                    value_type=bool,
                ),
                "min_z": ParameterValue(LaunchConfiguration("min_z"), value_type=float),
                "max_z": ParameterValue(LaunchConfiguration("max_z"), value_type=float),
                "range_max": ParameterValue(
                    LaunchConfiguration("lidar_range_max"),
                    value_type=float,
                ),
                "angle_min": ParameterValue(LaunchConfiguration("angle_min"), value_type=float),
                "angle_max": ParameterValue(LaunchConfiguration("angle_max"), value_type=float),
                "angle_increment": ParameterValue(
                    LaunchConfiguration("fused_angle_increment"),
                    value_type=float,
                ),
                "queue_size": ParameterValue(LaunchConfiguration("queue_size"), value_type=int),
                "max_publish_rate": ParameterValue(
                    LaunchConfiguration("max_publish_rate"),
                    value_type=float,
                ),
                "input_point_stride": ParameterValue(
                    LaunchConfiguration("input_point_stride"),
                    value_type=int,
                ),
                "lidar_point_stride": ParameterValue(
                    LaunchConfiguration("lidar_point_stride"),
                    value_type=int,
                ),
                "restamp_output": ParameterValue(
                    LaunchConfiguration("restamp_output"),
                    value_type=bool,
                ),
                "input_stamp_warning_age": ParameterValue(
                    LaunchConfiguration("input_stamp_warning_age"),
                    value_type=float,
                ),
                "max_input_age": ParameterValue(
                    LaunchConfiguration("max_input_age"),
                    value_type=float,
                ),
                "processing_time_warning": ParameterValue(
                    LaunchConfiguration("processing_time_warning"),
                    value_type=float,
                ),
                "transform_timeout": ParameterValue(
                    LaunchConfiguration("transform_timeout"),
                    value_type=float,
                ),
            }],
        ),
    ])
