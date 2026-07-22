from ament_index_python.packages import get_package_share_directory
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    ekf_config = os.path.join(
        get_package_share_directory("yahboomcar_bringup"),
        "config",
        "ekf_lidar_imu.yaml"
    )
    ekf_imu_only_config = os.path.join(
        get_package_share_directory("yahboomcar_bringup"),
        "config",
        "ekf_imu_only.yaml"
    )
    ekf_with_foot_config = os.path.join(
        get_package_share_directory("yahboomcar_bringup"),
        "config",
        "ekf_lidar_imu_with_foot.yaml"
    )
    lidar_odometry_launch = os.path.join(
        get_package_share_directory("lidar_pointcloud_filter"),
        "launch",
        "filter_lidar_odometry_launch.py"
    )

    launch_lidar_odometry_arg = DeclareLaunchArgument(
        "launch_lidar_odometry",
        default_value="true",
        description="Whether to launch LiDAR filtering and odometry for /scan_odom.",
    )
    raw_scan_topic_arg = DeclareLaunchArgument(
        "raw_scan_topic",
        default_value="/lidar/raw_laserscan",
        description="Raw LiDAR LaserScan topic from the TG30 driver.",
    )
    scan_downsample_factor_arg = DeclareLaunchArgument(
        "scan_downsample_factor",
        default_value="2",
        description="LiDAR scan bin grouping factor for the RF2O input scan.",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock if true.",
    )
    rf2o_translation_deadband_arg = DeclareLaunchArgument(
        "rf2o_translation_deadband",
        default_value="0.0025",
        description=(
            "Per-update RF2O planar translation deadband in meters. "
            "At the default 20 Hz RF2O rate, 0.0025 m accepts roughly >=5 cm/s. "
            "Set 0.0 to disable."
        ),
    )
    rf2o_translation_jump_rejection_threshold_arg = DeclareLaunchArgument(
        "rf2o_translation_jump_rejection_threshold",
        default_value="0.03",
        description=(
            "Reject an RF2O XY update above this per-message distance in meters. "
            "If rf2o_max_translation_rate is positive, the update must also exceed "
            "that rate. Set 0.0 to disable the per-update cap."
        ),
    )
    rf2o_max_translation_rate_arg = DeclareLaunchArgument(
        "rf2o_max_translation_rate",
        default_value="0.0",
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
        default_value="0.087266",
        description=(
            "Reject an RF2O yaw update above this per-message delta in radians "
            "(0.087266 rad is 5 deg). Set 0.0 to disable the per-update cap."
        ),
    )
    rf2o_max_yaw_rate_arg = DeclareLaunchArgument(
        "rf2o_max_yaw_rate",
        default_value="0.0",
        description=(
            "Maximum plausible RF2O yaw rate in rad/s for yaw-jump rejection. "
            "Set 0.0 to use only rf2o_yaw_jump_rejection_threshold."
        ),
    )
    rf2o_use_cmd_vel_gate_arg = DeclareLaunchArgument(
        "rf2o_use_cmd_vel_gate",
        default_value="true",
        description=(
            "Apply RF2O deadbands and jump caps per axis only when recent cmd_vel for "
            "that axis is near zero. "
            "Set false to always apply the filters."
        ),
    )
    rf2o_cmd_vel_topic_arg = DeclareLaunchArgument(
        "rf2o_cmd_vel_topic",
        default_value="cmd_vel",
        description="cmd_vel topic used to gate stationary RF2O filtering.",
    )
    rf2o_cmd_vel_timeout_arg = DeclareLaunchArgument(
        "rf2o_cmd_vel_timeout",
        default_value="0.5",
        description="Seconds after the last cmd_vel before RF2O filtering assumes stationary.",
    )
    rf2o_cmd_vel_stationary_linear_threshold_arg = DeclareLaunchArgument(
        "rf2o_cmd_vel_stationary_linear_threshold",
        default_value="0.03",
        description="Planar cmd_vel magnitude at or below which translation filters apply.",
    )
    rf2o_cmd_vel_stationary_angular_threshold_arg = DeclareLaunchArgument(
        "rf2o_cmd_vel_stationary_angular_threshold",
        default_value="0.03",
        description="Angular cmd_vel magnitude at or below which yaw filters apply.",
    )
    launch_foot_odometry_arg = DeclareLaunchArgument(
        "launch_foot_odometry",
        default_value="false",
        description="Whether to launch rough Muto gait/cmd_vel foot odometry and fuse /foot_odom into the EKF.",
    )
    imu_only_arg = DeclareLaunchArgument(
        "imu_only",
        default_value="false",
        description="Run an IMU-only EKF test using /imu/data_processed and no LiDAR or foot odometry.",
    )

    use_lidar_odometry = PythonExpression([
        "'", LaunchConfiguration("launch_lidar_odometry"), "' == 'true' and '",
        LaunchConfiguration("imu_only"), "' == 'false'",
    ])
    use_foot_odometry = PythonExpression([
        "'", LaunchConfiguration("launch_foot_odometry"), "' == 'true' and '",
        LaunchConfiguration("imu_only"), "' == 'false'",
    ])
    use_lidar_imu_ekf = PythonExpression([
        "'", LaunchConfiguration("launch_foot_odometry"), "' == 'false' and '",
        LaunchConfiguration("imu_only"), "' == 'false'",
    ])
    use_foot_ekf = PythonExpression([
        "'", LaunchConfiguration("launch_foot_odometry"), "' == 'true' and '",
        LaunchConfiguration("imu_only"), "' == 'false'",
    ])

    return LaunchDescription([
        launch_lidar_odometry_arg,
        raw_scan_topic_arg,
        scan_downsample_factor_arg,
        use_sim_time_arg,
        rf2o_translation_deadband_arg,
        rf2o_translation_jump_rejection_threshold_arg,
        rf2o_max_translation_rate_arg,
        rf2o_yaw_deadband_arg,
        rf2o_yaw_jump_rejection_threshold_arg,
        rf2o_max_yaw_rate_arg,
        rf2o_use_cmd_vel_gate_arg,
        rf2o_cmd_vel_topic_arg,
        rf2o_cmd_vel_timeout_arg,
        rf2o_cmd_vel_stationary_linear_threshold_arg,
        rf2o_cmd_vel_stationary_angular_threshold_arg,
        launch_foot_odometry_arg,
        imu_only_arg,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidar_odometry_launch),
            condition=IfCondition(use_lidar_odometry),
            launch_arguments={
                "rf2o_publish_tf": "false",
                "raw_scan_topic": LaunchConfiguration("raw_scan_topic"),
                "scan_downsample_factor": LaunchConfiguration("scan_downsample_factor"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "rf2o_translation_deadband": LaunchConfiguration("rf2o_translation_deadband"),
                "rf2o_translation_jump_rejection_threshold": LaunchConfiguration(
                    "rf2o_translation_jump_rejection_threshold"
                ),
                "rf2o_max_translation_rate": LaunchConfiguration("rf2o_max_translation_rate"),
                "rf2o_yaw_deadband": LaunchConfiguration("rf2o_yaw_deadband"),
                "rf2o_yaw_jump_rejection_threshold": LaunchConfiguration(
                    "rf2o_yaw_jump_rejection_threshold"
                ),
                "rf2o_max_yaw_rate": LaunchConfiguration("rf2o_max_yaw_rate"),
                "rf2o_use_cmd_vel_gate": LaunchConfiguration("rf2o_use_cmd_vel_gate"),
                "rf2o_cmd_vel_topic": LaunchConfiguration("rf2o_cmd_vel_topic"),
                "rf2o_cmd_vel_timeout": LaunchConfiguration("rf2o_cmd_vel_timeout"),
                "rf2o_cmd_vel_stationary_linear_threshold": LaunchConfiguration(
                    "rf2o_cmd_vel_stationary_linear_threshold"
                ),
                "rf2o_cmd_vel_stationary_angular_threshold": LaunchConfiguration(
                    "rf2o_cmd_vel_stationary_angular_threshold"
                ),
            }.items(),
        ),
        Node(
            package='yahboomcar_bringup',
            executable='foot_odometry_node',
            name='foot_odometry_node',
            output='screen',
            condition=IfCondition(use_foot_odometry),
            parameters=[{
                'odom_topic': '/foot_odom',
                'frame_id': 'odom',
                'child_frame_id': 'base_frame',
                'publish_tf': False,
                'use_sim_time': ParameterValue(
                    LaunchConfiguration("use_sim_time"),
                    value_type=bool,
                ),
            }],
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            condition=IfCondition(use_lidar_imu_ekf),
            parameters=[
                ekf_config,
                {
                    'use_sim_time': ParameterValue(
                        LaunchConfiguration("use_sim_time"),
                        value_type=bool,
                    )
                },
            ],
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            condition=IfCondition(use_foot_ekf),
            parameters=[
                ekf_with_foot_config,
                {
                    'use_sim_time': ParameterValue(
                        LaunchConfiguration("use_sim_time"),
                        value_type=bool,
                    )
                },
            ],
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            condition=IfCondition(LaunchConfiguration("imu_only")),
            parameters=[
                ekf_imu_only_config,
                {
                    'use_sim_time': ParameterValue(
                        LaunchConfiguration("use_sim_time"),
                        value_type=bool,
                    )
                },
            ],
        ),
    ])
