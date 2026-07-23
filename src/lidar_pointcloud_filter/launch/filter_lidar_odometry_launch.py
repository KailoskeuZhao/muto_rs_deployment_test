from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    raw_scan_topic_arg = DeclareLaunchArgument(
        'raw_scan_topic',
        default_value='/lidar/raw_laserscan',
        description='Raw LiDAR LaserScan topic published by the TG30 driver.',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true.',
    )
    scan_topic_arg = DeclareLaunchArgument(
        'scan_topic',
        default_value='/lidar/filtered_laserscan',
        description='Downsampled filtered LiDAR LaserScan topic consumed by rf2o.',
    )
    no_downsample_scan_topic_arg = DeclareLaunchArgument(
        'no_downsample_scan_topic',
        default_value='/lidar/filtered_laserscan_no_downsample',
        description='Full-resolution filtered LiDAR LaserScan topic consumed by scan fusion.',
    )
    no_downsample_scan_range_max_arg = DeclareLaunchArgument(
        'no_downsample_scan_range_max',
        default_value='15.0',
        description='Maximum range for the full-resolution LiDAR LaserScan used by fusion.',
    )
    scan_range_max_arg = DeclareLaunchArgument(
        'scan_range_max',
        default_value='10.0',
        description='Maximum range for the filtered LiDAR LaserScan.',
    )
    scan_range_min_arg = DeclareLaunchArgument(
        'scan_range_min',
        default_value='0.05',
        description='Minimum range for the filtered LiDAR LaserScan.',
    )
    scan_angle_min_arg = DeclareLaunchArgument(
        'scan_angle_min',
        default_value='-3.141592653589793',
        description='Minimum filtered LiDAR scan angle in radians.',
    )
    scan_angle_max_arg = DeclareLaunchArgument(
        'scan_angle_max',
        default_value='3.141592653589793',
        description='Maximum filtered LiDAR scan angle in radians.',
    )
    scan_downsample_factor_arg = DeclareLaunchArgument(
        'scan_downsample_factor',
        default_value='2',
        description='Group this many LiDAR scan bins into one RF2O scan bin.',
    )
    scan_restamp_output_arg = DeclareLaunchArgument(
        'scan_restamp_output',
        default_value='false',
        description=(
            'Use current ROS time for the RF2O input LaserScan stamp. '
            'Leave false unless LiDAR scan stamps are known bad while the data is fresh.'
        ),
    )
    scan_input_stamp_warning_age_arg = DeclareLaunchArgument(
        'scan_input_stamp_warning_age',
        default_value='1.0',
        description='Warn when LiDAR scan stamps differ from this node clock by more seconds.',
    )
    scan_max_input_age_arg = DeclareLaunchArgument(
        'scan_max_input_age',
        default_value='2.0',
        description=(
            'Drop LiDAR scans whose stamps differ from this node clock by more seconds before '
            'publishing RF2O input scans. Set 0.0 only for intentional non-live stamps.'
        ),
    )
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='scan_odom',
        description='Filtered odometry output topic consumed by downstream localization.',
    )
    raw_odom_topic_arg = DeclareLaunchArgument(
        'raw_odom_topic',
        default_value='scan_odom_raw',
        description='Internal raw rf2o odometry topic before translation deadband filtering.',
    )
    odom_frame_arg = DeclareLaunchArgument(
        'odom_frame',
        default_value='odom',
        description='rf2o odometry frame id.',
    )
    odom_child_frame_arg = DeclareLaunchArgument(
        'odom_child_frame',
        default_value='base_frame',
        description='rf2o base/child frame id.',
    )
    rf2o_publish_tf_arg = DeclareLaunchArgument(
        'rf2o_publish_tf',
        default_value='true',
        description=(
            'Whether filtered odometry should publish odom->base TF. '
            'Set false when an EKF consumes /scan_odom and publishes the authoritative TF.'
        ),
    )
    rf2o_freq_arg = DeclareLaunchArgument(
        'rf2o_freq',
        default_value='20.0',
        description='rf2o processing frequency in Hz.',
    )
    rf2o_log_level_arg = DeclareLaunchArgument(
        'rf2o_log_level',
        default_value='warn',
        description='ROS log level for the RF2O process.',
    )
    rf2o_translation_deadband_arg = DeclareLaunchArgument(
        'rf2o_translation_deadband',
        default_value='0.0025',
        description=(
            'Per-update RF2O planar translation deadband in meters. '
            'At the default 20 Hz RF2O rate, 0.0025 m accepts roughly >=5 cm/s. '
            'Set 0.0 to disable.'
        ),
    )
    rf2o_translation_jump_rejection_threshold_arg = DeclareLaunchArgument(
        'rf2o_translation_jump_rejection_threshold',
        default_value='0.03',
        description=(
            'Reject an RF2O XY update above this per-message distance in meters. '
            'If rf2o_max_translation_rate is positive, the update must also exceed '
            'that rate. Set 0.0 to disable the per-update cap.'
        ),
    )
    rf2o_max_translation_rate_arg = DeclareLaunchArgument(
        'rf2o_max_translation_rate',
        default_value='0.0',
        description=(
            'Maximum plausible RF2O planar speed in m/s for translation-jump rejection. '
            'Set 0.0 to use only rf2o_translation_jump_rejection_threshold.'
        ),
    )
    rf2o_yaw_deadband_arg = DeclareLaunchArgument(
        'rf2o_yaw_deadband',
        default_value='0.001',
        description=(
            'Per-update RF2O yaw deadband in radians. '
            'Set 0.0 to disable.'
        ),
    )
    rf2o_yaw_jump_rejection_threshold_arg = DeclareLaunchArgument(
        'rf2o_yaw_jump_rejection_threshold',
        default_value='0.087266',
        description=(
            'Reject an RF2O yaw update above this per-message delta in radians '
            '(0.087266 rad is 5 deg). Set 0.0 to disable the per-update cap.'
        ),
    )
    rf2o_max_yaw_rate_arg = DeclareLaunchArgument(
        'rf2o_max_yaw_rate',
        default_value='0.0',
        description=(
            'Maximum plausible RF2O yaw rate in rad/s for yaw-jump rejection. '
            'Set 0.0 to use only rf2o_yaw_jump_rejection_threshold.'
        ),
    )
    rf2o_use_cmd_vel_gate_arg = DeclareLaunchArgument(
        'rf2o_use_cmd_vel_gate',
        default_value='true',
        description=(
            'Apply RF2O deadbands and jump caps per axis only when recent cmd_vel for '
            'that axis is near zero. '
            'Set false to always apply the filters.'
        ),
    )
    rf2o_cmd_vel_topic_arg = DeclareLaunchArgument(
        'rf2o_cmd_vel_topic',
        default_value='cmd_vel',
        description='cmd_vel topic used to gate stationary RF2O filtering.',
    )
    rf2o_cmd_vel_timeout_arg = DeclareLaunchArgument(
        'rf2o_cmd_vel_timeout',
        default_value='0.5',
        description='Seconds after the last cmd_vel before RF2O filtering assumes stationary.',
    )
    rf2o_cmd_vel_stationary_linear_threshold_arg = DeclareLaunchArgument(
        'rf2o_cmd_vel_stationary_linear_threshold',
        default_value='0.03',
        description='Planar cmd_vel magnitude at or below which translation filters apply.',
    )
    rf2o_cmd_vel_stationary_angular_threshold_arg = DeclareLaunchArgument(
        'rf2o_cmd_vel_stationary_angular_threshold',
        default_value='0.03',
        description='Angular cmd_vel magnitude at or below which yaw filters apply.',
    )
    rf2o_init_pose_from_topic_arg = DeclareLaunchArgument(
        'rf2o_init_pose_from_topic',
        default_value='',
        description='Optional odometry topic used to initialize rf2o pose. Empty starts at zero.',
    )
    queue_size_arg = DeclareLaunchArgument(
        'queue_size',
        default_value='5',
        description='LaserScan and odometry queue size.',
    )

    return LaunchDescription([
        raw_scan_topic_arg,
        use_sim_time_arg,
        scan_topic_arg,
        no_downsample_scan_topic_arg,
        no_downsample_scan_range_max_arg,
        scan_range_max_arg,
        scan_range_min_arg,
        scan_angle_min_arg,
        scan_angle_max_arg,
        scan_downsample_factor_arg,
        scan_restamp_output_arg,
        scan_input_stamp_warning_age_arg,
        scan_max_input_age_arg,
        odom_topic_arg,
        raw_odom_topic_arg,
        odom_frame_arg,
        odom_child_frame_arg,
        rf2o_publish_tf_arg,
        rf2o_freq_arg,
        rf2o_log_level_arg,
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
        rf2o_init_pose_from_topic_arg,
        queue_size_arg,
        Node(
            package='lidar_pointcloud_filter',
            executable='lidar_laserscan_filter_node',
            name='lidar_laserscan_filter_node',
            output='screen',
            parameters=[{
                'input_topic': LaunchConfiguration('raw_scan_topic'),
                'output_topic': LaunchConfiguration('scan_topic'),
                'no_downsample_output_topic': LaunchConfiguration('no_downsample_scan_topic'),
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'),
                    value_type=bool,
                ),
                'range_min': ParameterValue(
                    LaunchConfiguration('scan_range_min'),
                    value_type=float,
                ),
                'range_max': ParameterValue(
                    LaunchConfiguration('scan_range_max'),
                    value_type=float,
                ),
                'no_downsample_range_max': ParameterValue(
                    LaunchConfiguration('no_downsample_scan_range_max'),
                    value_type=float,
                ),
                'angle_min': ParameterValue(
                    LaunchConfiguration('scan_angle_min'),
                    value_type=float,
                ),
                'angle_max': ParameterValue(
                    LaunchConfiguration('scan_angle_max'),
                    value_type=float,
                ),
                'downsample_factor': ParameterValue(
                    LaunchConfiguration('scan_downsample_factor'),
                    value_type=int,
                ),
                'restamp_output': ParameterValue(
                    LaunchConfiguration('scan_restamp_output'),
                    value_type=bool,
                ),
                'queue_size': ParameterValue(LaunchConfiguration('queue_size'), value_type=int),
                'input_stamp_warning_age': ParameterValue(
                    LaunchConfiguration('scan_input_stamp_warning_age'),
                    value_type=float,
                ),
                'max_input_age': ParameterValue(
                    LaunchConfiguration('scan_max_input_age'),
                    value_type=float,
                ),
            }],
        ),
        Node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry_node',
            output='screen',
            arguments=[
                '--ros-args',
                '--log-level',
                LaunchConfiguration('rf2o_log_level'),
            ],
            parameters=[{
                'laser_scan_topic': LaunchConfiguration('scan_topic'),
                'odom_topic': LaunchConfiguration('raw_odom_topic'),
                'publish_tf': False,
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'),
                    value_type=bool,
                ),
                'base_frame_id': LaunchConfiguration('odom_child_frame'),
                'odom_frame_id': LaunchConfiguration('odom_frame'),
                'init_pose_from_topic': LaunchConfiguration('rf2o_init_pose_from_topic'),
                'freq': ParameterValue(LaunchConfiguration('rf2o_freq'), value_type=float),
            }],
        ),
        Node(
            package='lidar_pointcloud_filter',
            executable='odometry_translation_deadband_node',
            name='rf2o_translation_deadband_filter',
            output='screen',
            parameters=[{
                'input_topic': LaunchConfiguration('raw_odom_topic'),
                'output_topic': LaunchConfiguration('odom_topic'),
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'),
                    value_type=bool,
                ),
                'translation_deadband': ParameterValue(
                    LaunchConfiguration('rf2o_translation_deadband'),
                    value_type=float,
                ),
                'translation_jump_rejection_threshold': ParameterValue(
                    LaunchConfiguration('rf2o_translation_jump_rejection_threshold'),
                    value_type=float,
                ),
                'max_translation_rate': ParameterValue(
                    LaunchConfiguration('rf2o_max_translation_rate'),
                    value_type=float,
                ),
                'yaw_deadband': ParameterValue(
                    LaunchConfiguration('rf2o_yaw_deadband'),
                    value_type=float,
                ),
                'yaw_jump_rejection_threshold': ParameterValue(
                    LaunchConfiguration('rf2o_yaw_jump_rejection_threshold'),
                    value_type=float,
                ),
                'max_yaw_rate': ParameterValue(
                    LaunchConfiguration('rf2o_max_yaw_rate'),
                    value_type=float,
                ),
                'use_cmd_vel_gate': ParameterValue(
                    LaunchConfiguration('rf2o_use_cmd_vel_gate'),
                    value_type=bool,
                ),
                'cmd_vel_topic': LaunchConfiguration('rf2o_cmd_vel_topic'),
                'cmd_vel_timeout': ParameterValue(
                    LaunchConfiguration('rf2o_cmd_vel_timeout'),
                    value_type=float,
                ),
                'cmd_vel_stationary_linear_threshold': ParameterValue(
                    LaunchConfiguration('rf2o_cmd_vel_stationary_linear_threshold'),
                    value_type=float,
                ),
                'cmd_vel_stationary_angular_threshold': ParameterValue(
                    LaunchConfiguration('rf2o_cmd_vel_stationary_angular_threshold'),
                    value_type=float,
                ),
                'publish_tf': ParameterValue(
                    LaunchConfiguration('rf2o_publish_tf'),
                    value_type=bool,
                ),
                'queue_size': ParameterValue(LaunchConfiguration('queue_size'), value_type=int),
            }],
        ),
    ])
