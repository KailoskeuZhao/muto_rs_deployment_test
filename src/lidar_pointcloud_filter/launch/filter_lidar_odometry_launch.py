import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    scan_converter_launch = os.path.join(
        get_package_share_directory("lidar_pointcloud_filter"),
        "launch",
        "camera_pointcloud_to_laserscan_launch.py",
    )

    input_topic_arg = DeclareLaunchArgument(
        "input_topic",
        default_value="lidar/PointCloud",
        description="Raw LiDAR PointCloud2 topic.",
    )
    filtered_topic_arg = DeclareLaunchArgument(
        "filtered_topic",
        default_value="lidar/PointCloudFiltered",
        description="Filtered and downsampled LiDAR PointCloud2 topic.",
    )
    filtered_no_downsample_topic_arg = DeclareLaunchArgument(
        "filtered_no_downsample_topic",
        default_value="lidar/PointCloudFilteredNoDownsample",
        description="Filtered LiDAR PointCloud2 topic before voxel downsampling.",
    )
    target_frame_arg = DeclareLaunchArgument(
        "target_frame",
        default_value="base_frame",
        description="Frame that filtered point clouds are transformed into.",
    )
    scan_topic_arg = DeclareLaunchArgument(
        "scan_topic",
        default_value="/lidar/filtered_laserscan",
        description="LaserScan topic generated from the filtered LiDAR cloud and consumed by rf2o.",
    )
    scan_min_z_arg = DeclareLaunchArgument(
        "scan_min_z",
        default_value="-0.4",
        description="Minimum z value kept when projecting filtered LiDAR cloud to LaserScan.",
    )
    scan_max_z_arg = DeclareLaunchArgument(
        "scan_max_z",
        default_value="0.2",
        description="Maximum z value kept when projecting filtered LiDAR cloud to LaserScan.",
    )
    scan_range_max_arg = DeclareLaunchArgument(
        "scan_range_max",
        default_value="10.0",
        description="Maximum range for the filtered LiDAR LaserScan.",
    )
    scan_angle_min_arg = DeclareLaunchArgument(
        "scan_angle_min",
        default_value="-3.141592653589793",
        description="Minimum filtered LiDAR scan angle in radians.",
    )
    scan_angle_max_arg = DeclareLaunchArgument(
        "scan_angle_max",
        default_value="3.141592653589793",
        description="Maximum filtered LiDAR scan angle in radians.",
    )
    odom_topic_arg = DeclareLaunchArgument(
        "odom_topic",
        default_value="scan_odom",
        description="Filtered odometry output topic consumed by downstream localization.",
    )
    raw_odom_topic_arg = DeclareLaunchArgument(
        "raw_odom_topic",
        default_value="scan_odom_raw",
        description="Internal raw rf2o odometry topic before translation deadband filtering.",
    )
    odom_frame_arg = DeclareLaunchArgument(
        "odom_frame",
        default_value="odom",
        description="rf2o odometry frame id.",
    )
    odom_child_frame_arg = DeclareLaunchArgument(
        "odom_child_frame",
        default_value="base_frame",
        description="rf2o base/child frame id.",
    )
    rf2o_publish_tf_arg = DeclareLaunchArgument(
        "rf2o_publish_tf",
        default_value="true",
        description=(
            "Whether filtered odometry should publish odom->base TF. "
            "Set false when an EKF consumes /scan_odom and publishes the authoritative TF."
        ),
    )
    rf2o_freq_arg = DeclareLaunchArgument(
        "rf2o_freq",
        default_value="20.0",
        description="rf2o processing frequency in Hz.",
    )
    rf2o_translation_deadband_arg = DeclareLaunchArgument(
        "rf2o_translation_deadband",
        default_value="0.003",
        description=(
            "Per-update RF2O planar translation deadband in meters. "
            "Set 0.0 to disable."
        ),
    )
    rf2o_translation_jump_rejection_threshold_arg = DeclareLaunchArgument(
        "rf2o_translation_jump_rejection_threshold",
        default_value="0.20",
        description=(
            "Reject an RF2O XY update above this per-message distance in meters "
            "when it also exceeds rf2o_max_translation_rate. Set 0.0 to disable."
        ),
    )
    rf2o_max_translation_rate_arg = DeclareLaunchArgument(
        "rf2o_max_translation_rate",
        default_value="1.5",
        description=(
            "Maximum plausible RF2O planar speed in m/s for translation-jump rejection. "
            "Set 0.0 to use only rf2o_translation_jump_rejection_threshold."
        ),
    )
    rf2o_yaw_deadband_arg = DeclareLaunchArgument(
        "rf2o_yaw_deadband",
        default_value="0.001",
        description=(
            "Per-update RF2O yaw deadband in radians. "
            "Set 0.0 to disable."
        ),
    )
    rf2o_yaw_jump_rejection_threshold_arg = DeclareLaunchArgument(
        "rf2o_yaw_jump_rejection_threshold",
        default_value="0.30",
        description=(
            "Reject an RF2O yaw update above this per-message delta in radians "
            "when it also exceeds rf2o_max_yaw_rate. Set 0.0 to disable."
        ),
    )
    rf2o_max_yaw_rate_arg = DeclareLaunchArgument(
        "rf2o_max_yaw_rate",
        default_value="4.0",
        description=(
            "Maximum plausible RF2O yaw rate in rad/s for yaw-jump rejection. "
            "Set 0.0 to use only rf2o_yaw_jump_rejection_threshold."
        ),
    )
    rf2o_init_pose_from_topic_arg = DeclareLaunchArgument(
        "rf2o_init_pose_from_topic",
        default_value="",
        description="Optional odometry topic used to initialize rf2o pose. Empty starts at zero.",
    )
    queue_size_arg = DeclareLaunchArgument(
        "queue_size",
        default_value="5",
        description="Point cloud and odometry queue size.",
    )
    voxel_leaf_size_arg = DeclareLaunchArgument(
        "voxel_leaf_size",
        default_value="0.02",
        description="LiDAR filter voxel leaf size in meters. 0.0 disables downsampling.",
    )
    transform_timeout_arg = DeclareLaunchArgument(
        "transform_timeout",
        default_value="0.05",
        description="TF lookup timeout in seconds.",
    )

    return LaunchDescription([
        input_topic_arg,
        filtered_topic_arg,
        filtered_no_downsample_topic_arg,
        target_frame_arg,
        scan_topic_arg,
        scan_min_z_arg,
        scan_max_z_arg,
        scan_range_max_arg,
        scan_angle_min_arg,
        scan_angle_max_arg,
        odom_topic_arg,
        raw_odom_topic_arg,
        odom_frame_arg,
        odom_child_frame_arg,
        rf2o_publish_tf_arg,
        rf2o_freq_arg,
        rf2o_translation_deadband_arg,
        rf2o_translation_jump_rejection_threshold_arg,
        rf2o_max_translation_rate_arg,
        rf2o_yaw_deadband_arg,
        rf2o_yaw_jump_rejection_threshold_arg,
        rf2o_max_yaw_rate_arg,
        rf2o_init_pose_from_topic_arg,
        queue_size_arg,
        voxel_leaf_size_arg,
        transform_timeout_arg,
        Node(
            package="lidar_pointcloud_filter",
            executable="lidar_pointcloud_filter_node",
            name="lidar_pointcloud_filter_node",
            output="screen",
            parameters=[{
                "input_topic": LaunchConfiguration("input_topic"),
                "output_topic": LaunchConfiguration("filtered_topic"),
                "no_downsample_output_topic": LaunchConfiguration("filtered_no_downsample_topic"),
                "target_frame": LaunchConfiguration("target_frame"),
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
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(scan_converter_launch),
            launch_arguments={
                "publish_fused_scan": "false",
                "publish_filtered_lidar_scan": "true",
                "lidar_topic": LaunchConfiguration("filtered_topic"),
                "filtered_lidar_scan_topic": LaunchConfiguration("scan_topic"),
                "filtered_lidar_scan_frame": LaunchConfiguration("target_frame"),
                "min_z": LaunchConfiguration("scan_min_z"),
                "max_z": LaunchConfiguration("scan_max_z"),
                "lidar_range_max": LaunchConfiguration("scan_range_max"),
                "angle_min": LaunchConfiguration("scan_angle_min"),
                "angle_max": LaunchConfiguration("scan_angle_max"),
                "queue_size": LaunchConfiguration("queue_size"),
                "transform_timeout": LaunchConfiguration("transform_timeout"),
            }.items(),
        ),
        Node(
            package="rf2o_laser_odometry",
            executable="rf2o_laser_odometry_node",
            output="screen",
            parameters=[{
                "laser_scan_topic": LaunchConfiguration("scan_topic"),
                "odom_topic": LaunchConfiguration("raw_odom_topic"),
                "publish_tf": False,
                "base_frame_id": LaunchConfiguration("odom_child_frame"),
                "odom_frame_id": LaunchConfiguration("odom_frame"),
                "init_pose_from_topic": LaunchConfiguration("rf2o_init_pose_from_topic"),
                "freq": ParameterValue(LaunchConfiguration("rf2o_freq"), value_type=float),
            }],
        ),
        Node(
            package="lidar_pointcloud_filter",
            executable="odometry_translation_deadband_node",
            name="rf2o_translation_deadband_filter",
            output="screen",
            parameters=[{
                "input_topic": LaunchConfiguration("raw_odom_topic"),
                "output_topic": LaunchConfiguration("odom_topic"),
                "translation_deadband": ParameterValue(
                    LaunchConfiguration("rf2o_translation_deadband"),
                    value_type=float,
                ),
                "translation_jump_rejection_threshold": ParameterValue(
                    LaunchConfiguration("rf2o_translation_jump_rejection_threshold"),
                    value_type=float,
                ),
                "max_translation_rate": ParameterValue(
                    LaunchConfiguration("rf2o_max_translation_rate"),
                    value_type=float,
                ),
                "yaw_deadband": ParameterValue(
                    LaunchConfiguration("rf2o_yaw_deadband"),
                    value_type=float,
                ),
                "yaw_jump_rejection_threshold": ParameterValue(
                    LaunchConfiguration("rf2o_yaw_jump_rejection_threshold"),
                    value_type=float,
                ),
                "max_yaw_rate": ParameterValue(
                    LaunchConfiguration("rf2o_max_yaw_rate"),
                    value_type=float,
                ),
                "publish_tf": ParameterValue(
                    LaunchConfiguration("rf2o_publish_tf"),
                    value_type=bool,
                ),
                "queue_size": ParameterValue(LaunchConfiguration("queue_size"), value_type=int),
            }],
        ),
    ])
