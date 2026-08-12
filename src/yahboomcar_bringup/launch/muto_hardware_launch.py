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
        default_value="10.0",
        description=(
            "Host polling rate for the controller-cached raw IMU snapshot. "
            "Ten Hz retains the prior timestamp-observation window for the "
            "0x60 comparison; runtime polls use separate slots in one "
            "coordinated post-gait scheduler. This does not configure the "
            "ICM-20948 output-data rate."
        ),
    )
    imu_attitude_publish_rate_hz_arg = DeclareLaunchArgument(
        "imu_attitude_publish_rate_hz",
        default_value="10.0",
        description=(
            "Host polling rate for the controller-fused 0x60 Euler attitude. "
            "The coordinated gait-slot scheduler gives it a separate serial "
            "opportunity from raw 0x61. Set 0.0 to disable. The localization "
            "launch normally consumes it through the guarded yaw adapter."
        ),
    )
    imu_suppress_identical_snapshots_arg = DeclareLaunchArgument(
        "imu_suppress_identical_snapshots",
        default_value="true",
        description=(
            "Publish only when accel/gyro values differ from the preceding "
            "controller snapshot. Disable only for protocol diagnostics."
        ),
    )
    imu_response_timeout_sec_arg = DeclareLaunchArgument(
        "imu_response_timeout_sec",
        default_value="0.008",
        description="Runtime IMU serial-response budget in seconds.",
    )
    imu_stale_warning_sec_arg = DeclareLaunchArgument(
        "imu_stale_warning_sec",
        default_value="2.0",
        description=(
            "Warn after this many seconds without a changed accel/gyro "
            "snapshot; zero disables the warning."
        ),
    )
    imu_locomotion_guard_sec_arg = DeclareLaunchArgument(
        "imu_locomotion_guard_sec",
        default_value="0.003",
        description=(
            "Reserve this much time before the next gait deadline instead "
            "of beginning an IMU serial transaction."
        ),
    )
    locomotion_update_rate_hz_arg = DeclareLaunchArgument(
        "locomotion_update_rate_hz",
        default_value="50.0",
        description=(
            "Fixed rate that advances one locomotion trajectory phase and "
            "publishes its commanded gait state."
        ),
    )
    batch_gait_phase_writes_arg = DeclareLaunchArgument(
        "batch_gait_phase_writes",
        default_value="true",
        description=(
            "Send the six unchanged vendor leg frames for each phase in one "
            "contiguous serial write. False restores per-leg writes."
        ),
    )
    cmd_vel_timeout_arg = DeclareLaunchArgument(
        "cmd_vel_timeout",
        default_value="0.5",
        description=(
            "Seconds without cmd_vel before the locomotion loop actively "
            "returns to standby. Set 0.0 to disable timeout handling."
        ),
    )
    locomotion_command_mapping_arg = DeclareLaunchArgument(
        "locomotion_command_mapping",
        default_value="geometric",
        description=(
            "cmd_vel conversion mode. 'geometric' derives amplitudes from "
            "the custom exact-SE(2) gait; 'calibrated' loads an external "
            "measured profile; 'legacy_100' is rollback-only."
        ),
    )
    locomotion_calibration_file_arg = DeclareLaunchArgument(
        "locomotion_calibration_file",
        default_value=PathJoinSubstitution([
            FindPackageShare("yahboomcar_bringup"),
            "config",
            "muto_locomotion_provisional_20260806.yaml",
        ]),
        description=(
            "Velocity-to-gait calibration profile used only when "
            "locomotion_command_mapping is 'calibrated'."
        ),
    )
    imu_calibration_sample_count_arg = DeclareLaunchArgument(
        "imu_calibration_sample_count",
        default_value="10",
        description=(
            "Number of changed accel/gyro snapshots used for startup "
            "bias/scale calibration."
        ),
    )
    imu_calibration_max_reads_arg = DeclareLaunchArgument(
        "imu_calibration_max_reads",
        default_value="150",
        description="Maximum startup IMU read attempts while collecting calibration samples.",
    )
    imu_calibration_timeout_sec_arg = DeclareLaunchArgument(
        "imu_calibration_timeout_sec",
        default_value="15.0",
        description="Maximum wall-clock seconds spent on startup IMU calibration.",
    )
    imu_calibration_read_interval_arg = DeclareLaunchArgument(
        "imu_calibration_read_interval",
        default_value="0.1",
        description="Seconds between startup IMU calibration polls.",
    )
    lidar_scan_topic_arg = DeclareLaunchArgument(
        "lidar_scan_topic",
        default_value="lidar/raw_laserscan",
        description="Raw TG30 LaserScan topic.",
    )
    camera_width_arg = DeclareLaunchArgument(
        "camera_width",
        default_value="640",
        description="Width in pixels for the Orbbec color stream.",
    )
    camera_height_arg = DeclareLaunchArgument(
        "camera_height",
        default_value="480",
        description="Height in pixels for the Orbbec color stream.",
    )
    color_fps_arg = DeclareLaunchArgument(
        "color_fps",
        default_value="30",
        description="Frame rate in Hz for the Orbbec color stream.",
    )
    color_info_url_arg = DeclareLaunchArgument(
        "color_info_url",
        default_value=(
            "package://yahboomcar_bringup/config/"
            "astra_pro_plus_acrf35300kr_color_640x480.yaml"
        ),
        description=(
            "Static color calibration fallback for Astra Pro Plus ACRF35300KR "
            "at 640x480."
        ),
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
            "Publish the Orbbec XYZ PointCloud2. The active costmap path consumes "
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
            "color_info_url": LaunchConfiguration("color_info_url"),
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
            "gait_state_topic": "/muto/commanded_gait_state",
            "gait_state_frame_id": "base_frame",
            "locomotion_update_rate_hz": ParameterValue(
                LaunchConfiguration("locomotion_update_rate_hz"),
                value_type=float,
            ),
            "batch_gait_phase_writes": ParameterValue(
                LaunchConfiguration("batch_gait_phase_writes"),
                value_type=bool,
            ),
            "cmd_vel_timeout": ParameterValue(
                LaunchConfiguration("cmd_vel_timeout"),
                value_type=float,
            ),
            "locomotion_command_mapping": ParameterValue(
                LaunchConfiguration("locomotion_command_mapping"),
                value_type=str,
            ),
            "locomotion_calibration_file": ParameterValue(
                LaunchConfiguration("locomotion_calibration_file"),
                value_type=str,
            ),
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
            "imu_attitude_publish_rate_hz": ParameterValue(
                LaunchConfiguration("imu_attitude_publish_rate_hz"),
                value_type=float,
            ),
            "imu_suppress_identical_snapshots": ParameterValue(
                LaunchConfiguration("imu_suppress_identical_snapshots"),
                value_type=bool,
            ),
            "imu_response_timeout_sec": ParameterValue(
                LaunchConfiguration("imu_response_timeout_sec"),
                value_type=float,
            ),
            "imu_stale_warning_sec": ParameterValue(
                LaunchConfiguration("imu_stale_warning_sec"),
                value_type=float,
            ),
            "imu_locomotion_guard_sec": ParameterValue(
                LaunchConfiguration("imu_locomotion_guard_sec"),
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
            "imu_calibration_timeout_sec": ParameterValue(
                LaunchConfiguration("imu_calibration_timeout_sec"),
                value_type=float,
            ),
            "imu_calibration_read_interval": ParameterValue(
                LaunchConfiguration("imu_calibration_read_interval"),
                value_type=float,
            ),
        }],
    )

    return LaunchDescription([
        imu_gyro_lsb_per_dps_arg,
        imu_yaw_rate_deadband_rad_s_arg,
        imu_publish_rate_hz_arg,
        imu_attitude_publish_rate_hz_arg,
        imu_suppress_identical_snapshots_arg,
        imu_response_timeout_sec_arg,
        imu_stale_warning_sec_arg,
        imu_locomotion_guard_sec_arg,
        locomotion_update_rate_hz_arg,
        batch_gait_phase_writes_arg,
        cmd_vel_timeout_arg,
        locomotion_command_mapping_arg,
        locomotion_calibration_file_arg,
        imu_calibration_sample_count_arg,
        imu_calibration_max_reads_arg,
        imu_calibration_timeout_sec_arg,
        imu_calibration_read_interval_arg,
        lidar_scan_topic_arg,
        camera_width_arg,
        camera_height_arg,
        color_fps_arg,
        color_info_url_arg,
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
