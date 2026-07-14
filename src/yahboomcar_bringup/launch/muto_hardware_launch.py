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
    lidar_publish_laserscan_arg = DeclareLaunchArgument(
        "lidar_publish_laserscan",
        default_value="true",
        description="Whether the TG30 driver publishes raw sensor_msgs/LaserScan.",
    )
    lidar_scan_topic_arg = DeclareLaunchArgument(
        "lidar_scan_topic",
        default_value="lidar/raw_laserscan",
        description="Raw TG30 LaserScan topic.",
    )
    lidar_publish_pointcloud_arg = DeclareLaunchArgument(
        "lidar_publish_pointcloud",
        default_value="true",
        description="Whether the TG30 driver also publishes the legacy PointCloud2 topic.",
    )
    lidar_pointcloud_topic_arg = DeclareLaunchArgument(
        "lidar_pointcloud_topic",
        default_value="lidar/PointCloud",
        description="Legacy TG30 PointCloud2 topic.",
    )

    lidar_node = Node(
        package="lidar_tg30",
        executable="lidar_node",
        name="lidar_node",
        output="screen",
        parameters=[{
            "publish_laserscan": ParameterValue(
                LaunchConfiguration("lidar_publish_laserscan"),
                value_type=bool,
            ),
            "scan_topic": LaunchConfiguration("lidar_scan_topic"),
            "publish_pointcloud": ParameterValue(
                LaunchConfiguration("lidar_publish_pointcloud"),
                value_type=bool,
            ),
            "pointcloud_topic": LaunchConfiguration("lidar_pointcloud_topic"),
        }],
    )

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("orbbec_camera"),
                "launch",
                "astra_pro_plus.launch.py",
            ])
        )
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
        lidar_publish_laserscan_arg,
        lidar_scan_topic_arg,
        lidar_publish_pointcloud_arg,
        lidar_pointcloud_topic_arg,
        lidar_node,
        camera_launch,
        driver_node,
    ])
