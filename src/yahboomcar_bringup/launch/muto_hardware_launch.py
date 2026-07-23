from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    imu_gyro_lsb_per_dps_arg = DeclareLaunchArgument(
        "imu_gyro_lsb_per_dps",
        default_value="16.4",
        description="Raw gyro counts per degree/second used for processed IMU angular velocity.",
    )
    imu_yaw_rate_deadband_rad_s_arg = DeclareLaunchArgument(
        "imu_yaw_rate_deadband_rad_s",
        default_value="0.03",
        description="Processed IMU yaw-rate deadband in rad/s before publishing /imu/data_processed.",
    )
    imu_publish_rate_hz_arg = DeclareLaunchArgument(
        "imu_publish_rate_hz",
        default_value="50.0",
        description="Processed/raw IMU publish rate in Hz.",
    )
    imu_calibration_sample_count_arg = DeclareLaunchArgument(
        "imu_calibration_sample_count",
        default_value="1200",
        description="Number of valid startup IMU samples used for bias/scale calibration.",
    )
    imu_calibration_max_reads_arg = DeclareLaunchArgument(
        "imu_calibration_max_reads",
        default_value="3600",
        description="Maximum startup IMU read attempts while collecting calibration samples.",
    )
    lidar_scan_topic_arg = DeclareLaunchArgument(
        "lidar_scan_topic",
        default_value="lidar/raw_laserscan",
        description="Raw TG30 LaserScan topic.",
    )
    camera_width_arg = DeclareLaunchArgument(
        "camera_width",
        default_value="1280",
        description="Width in pixels for the Orbbec color stream.",
    )
    camera_height_arg = DeclareLaunchArgument(
        "camera_height",
        default_value="720",
        description="Height in pixels for the Orbbec color stream.",
    )
    color_fps_arg = DeclareLaunchArgument(
        "color_fps",
        default_value="30",
        description="Frame rate in Hz for the Orbbec color stream.",
    )
    depth_width_arg = DeclareLaunchArgument(
        "depth_width",
        default_value="320",
        description="Width in pixels for the Orbbec depth stream.",
    )
    depth_height_arg = DeclareLaunchArgument(
        "depth_height",
        default_value="240",
        description="Height in pixels for the Orbbec depth stream.",
    )
    depth_fps_arg = DeclareLaunchArgument(
        "depth_fps",
        default_value="30",
        description=(
            "Hardware frame rate for the Orbbec depth stream. Astra Pro Plus "
            "advertises 320x240 only at 30 FPS; downstream consumers cap processing at 7 Hz."
        ),
    )
    depth_info_url_arg = DeclareLaunchArgument(
        "depth_info_url",
        default_value="",
        description=(
            "Optional calibration URL for the exact selected depth profile. The upstream "
            "Astra launch uses its ir_info_url parameter for both IR and depth CameraInfo."
        ),
    )
    enable_point_cloud_arg = DeclareLaunchArgument(
        "enable_point_cloud",
        default_value="false",
        description=(
            "Publish the Orbbec XYZ PointCloud2. Disabled because fusion consumes "
            "the raw depth image directly."
        ),
    )
    enable_ir_arg = DeclareLaunchArgument(
        "enable_ir",
        default_value="false",
        description="Whether to enable the unused Orbbec IR stream.",
    )

    lidar_node = Node(
        package="lidar_tg30",
        executable="lidar_node",
        name="lidar_node",
        output="screen",
        parameters=[{
            "scan_topic": LaunchConfiguration("lidar_scan_topic"),
        }],
    )

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("orbbec_camera"),
                "launch",
                "astra_pro_plus.launch.py",
            ])
        ),
        launch_arguments={
            "color_width": LaunchConfiguration("camera_width"),
            "color_height": LaunchConfiguration("camera_height"),
            "color_fps": LaunchConfiguration("color_fps"),
            "depth_width": LaunchConfiguration("depth_width"),
            "depth_height": LaunchConfiguration("depth_height"),
            "depth_fps": LaunchConfiguration("depth_fps"),
            "ir_info_url": LaunchConfiguration("depth_info_url"),
            "enable_point_cloud": LaunchConfiguration("enable_point_cloud"),
            "enable_colored_point_cloud": "false",
            "enable_ir": LaunchConfiguration("enable_ir"),
        }.items(),
    )

    driver_node = Node(
        package="yahboomcar_bringup",
        executable="muto_driver",
        name="muto_driver",
        output="screen",
        parameters=[{
            "imu_gyro_lsb_per_dps": ParameterValue(
                LaunchConfiguration("imu_gyro_lsb_per_dps"),
                value_type=float,
            ),
            "imu_yaw_rate_deadband_rad_s": ParameterValue(
                LaunchConfiguration("imu_yaw_rate_deadband_rad_s"),
                value_type=float,
            ),
            "imu_publish_rate_hz": ParameterValue(
                LaunchConfiguration("imu_publish_rate_hz"),
                value_type=float,
            ),
            "imu_calibration_sample_count": ParameterValue(
                LaunchConfiguration("imu_calibration_sample_count"),
                value_type=int,
            ),
            "imu_calibration_max_reads": ParameterValue(
                LaunchConfiguration("imu_calibration_max_reads"),
                value_type=int,
            ),
        }],
    )

    return LaunchDescription([
        imu_gyro_lsb_per_dps_arg,
        imu_yaw_rate_deadband_rad_s_arg,
        imu_publish_rate_hz_arg,
        imu_calibration_sample_count_arg,
        imu_calibration_max_reads_arg,
        lidar_scan_topic_arg,
        camera_width_arg,
        camera_height_arg,
        color_fps_arg,
        depth_width_arg,
        depth_height_arg,
        depth_fps_arg,
        depth_info_url_arg,
        enable_point_cloud_arg,
        enable_ir_arg,
        lidar_node,
        camera_launch,
        driver_node,
    ])
