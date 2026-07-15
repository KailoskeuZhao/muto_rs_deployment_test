import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    slam_params_file = os.path.join(
        get_package_share_directory("muto_slam_mapping"),
        "config",
        "mapper_params_online_async.yaml",
    )
    slam_toolbox_launch = os.path.join(
        get_package_share_directory("slam_toolbox"),
        "launch",
        "online_async_launch.py",
    )
    fused_laserscan_launch = os.path.join(
        get_package_share_directory("lidar_pointcloud_filter"),
        "launch",
        "camera_pointcloud_to_laserscan_launch.py",
    )

    slam_params_file_arg = DeclareLaunchArgument(
        "slam_params_file",
        default_value=slam_params_file,
        description="Path to the slam_toolbox online async mapper parameter file.",
    )
    launch_fused_laserscan_arg = DeclareLaunchArgument(
        "launch_fused_laserscan",
        default_value="true",
        description=(
            "Whether to launch camera PointCloud2 to LaserScan conversion and fuse it with "
            "the filtered no-downsample LiDAR LaserScan."
        ),
    )
    camera_scan_topic_arg = DeclareLaunchArgument(
        "camera_scan_topic",
        default_value="/camera/filtered_laserscan",
        description="Intermediate downsampled camera LaserScan topic.",
    )
    lidar_scan_topic_arg = DeclareLaunchArgument(
        "lidar_scan_topic",
        default_value="/lidar/filtered_laserscan_no_downsample",
        description="Filtered full-resolution LiDAR LaserScan topic used by scan fusion.",
    )
    fused_scan_frame_arg = DeclareLaunchArgument(
        "fused_scan_frame",
        default_value="base_frame",
        description="Frame used for /fused/laserscan.",
    )
    depth_min_z_arg = DeclareLaunchArgument(
        "depth_min_z",
        default_value="-0.10",
        description="Minimum depth-camera point z in fused_scan_frame to keep.",
    )
    depth_max_z_arg = DeclareLaunchArgument(
        "depth_max_z",
        default_value="0.18",
        description="Maximum depth-camera point z in fused_scan_frame to keep.",
    )
    depth_min_x_arg = DeclareLaunchArgument(
        "depth_min_x",
        default_value="0.30",
        description="Minimum depth-camera point x in fused_scan_frame to keep.",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock if true.",
    )
    restamp_laserscan_output_arg = DeclareLaunchArgument(
        "restamp_laserscan_output",
        default_value="false",
        description=(
            "Use current ROS time for generated /fused/laserscan stamps. "
            "Leave false unless camera/LiDAR stamps are known bad while the data is fresh."
        ),
    )
    input_stamp_warning_age_arg = DeclareLaunchArgument(
        "input_stamp_warning_age",
        default_value="1.0",
        description="Warn when camera cloud or scan stamps differ from this node clock by more seconds.",
    )
    max_input_age_arg = DeclareLaunchArgument(
        "max_input_age",
        default_value="2.0",
        description=(
            "Drop camera clouds/scans whose stamps differ from this node clock by more seconds. "
            "Set 0.0 only for intentional non-live stamps."
        ),
    )
    fused_scan_queue_size_arg = DeclareLaunchArgument(
        "fused_scan_queue_size",
        default_value="1",
        description="Queue size for camera scan conversion and scan fusion.",
    )
    fused_scan_max_publish_rate_arg = DeclareLaunchArgument(
        "fused_scan_max_publish_rate",
        default_value="10.0",
        description="Maximum /fused/laserscan publish rate in Hz. Set 0.0 to process every cloud.",
    )
    fused_scan_input_point_stride_arg = DeclareLaunchArgument(
        "fused_scan_input_point_stride",
        default_value="8",
        description="Process every Nth depth-camera point when building the camera scan.",
    )
    camera_scan_angle_increment_arg = DeclareLaunchArgument(
        "camera_scan_angle_increment",
        default_value="0.017453292519943295",
        description="Intermediate camera LaserScan angular resolution in radians.",
    )
    fused_scan_angle_increment_arg = DeclareLaunchArgument(
        "fused_scan_angle_increment",
        default_value="0.004363323129985824",
        description="/fused/laserscan angular resolution in radians.",
    )
    fused_scan_processing_time_warning_arg = DeclareLaunchArgument(
        "fused_scan_processing_time_warning",
        default_value="0.05",
        description="Warn when one /fused/laserscan conversion takes longer than this many seconds.",
    )

    return LaunchDescription([
        slam_params_file_arg,
        launch_fused_laserscan_arg,
        camera_scan_topic_arg,
        lidar_scan_topic_arg,
        fused_scan_frame_arg,
        depth_min_z_arg,
        depth_max_z_arg,
        depth_min_x_arg,
        use_sim_time_arg,
        restamp_laserscan_output_arg,
        input_stamp_warning_age_arg,
        max_input_age_arg,
        fused_scan_queue_size_arg,
        fused_scan_max_publish_rate_arg,
        fused_scan_input_point_stride_arg,
        camera_scan_angle_increment_arg,
        fused_scan_angle_increment_arg,
        fused_scan_processing_time_warning_arg,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(fused_laserscan_launch),
            condition=IfCondition(LaunchConfiguration("launch_fused_laserscan")),
            launch_arguments={
                "publish_fused_scan": "true",
                "publish_camera_scan": "true",
                "publish_filtered_lidar_scan": "false",
                "camera_scan_topic": LaunchConfiguration("camera_scan_topic"),
                "lidar_scan_topic": LaunchConfiguration("lidar_scan_topic"),
                "fused_scan_frame": LaunchConfiguration("fused_scan_frame"),
                "processing_frame": LaunchConfiguration("fused_scan_frame"),
                "min_z": LaunchConfiguration("depth_min_z"),
                "max_z": LaunchConfiguration("depth_max_z"),
                "camera_min_x": LaunchConfiguration("depth_min_x"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "restamp_output": LaunchConfiguration("restamp_laserscan_output"),
                "input_stamp_warning_age": LaunchConfiguration("input_stamp_warning_age"),
                "max_input_age": LaunchConfiguration("max_input_age"),
                "queue_size": LaunchConfiguration("fused_scan_queue_size"),
                "max_publish_rate": LaunchConfiguration("fused_scan_max_publish_rate"),
                "input_point_stride": LaunchConfiguration("fused_scan_input_point_stride"),
                "camera_angle_increment": LaunchConfiguration("camera_scan_angle_increment"),
                "fused_angle_increment": LaunchConfiguration("fused_scan_angle_increment"),
                "processing_time_warning": LaunchConfiguration(
                    "fused_scan_processing_time_warning"
                ),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_toolbox_launch),
            launch_arguments={
                "slam_params_file": LaunchConfiguration("slam_params_file"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }.items(),
        ),
    ])
