import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    GroupAction,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def launch_file(package_name, launch_name):
    return os.path.join(
        get_package_share_directory(package_name),
        'launch',
        launch_name,
    )


def scoped_include(package_name, launch_name, launch_arguments=None, condition=None):
    """Include one subsystem without leaking its launch configurations."""
    include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file(package_name, launch_name)),
        launch_arguments=(launch_arguments or {}).items(),
        condition=condition,
    )
    return GroupAction(actions=[include], scoped=True)


def delayed_include(
    delay_arg,
    enabled_arg,
    package_name,
    launch_name,
    launch_arguments=None,
):
    include = scoped_include(package_name, launch_name, launch_arguments)
    return TimerAction(
        period=LaunchConfiguration(delay_arg),
        actions=[include],
        condition=IfCondition(LaunchConfiguration(enabled_arg)),
    )


def readiness_gated_include(
    stage,
    delay_arg,
    timeout_arg,
    enabled_condition,
    topics,
    transforms,
    package_name,
    launch_name,
    launch_arguments=None,
    additional_success_actions=None,
):
    gate_arguments = [
        '--stage',
        stage,
        '--timeout',
        LaunchConfiguration(timeout_arg),
    ]
    for topic_name, type_name in topics:
        gate_arguments.extend(['--topic', f'{topic_name}:{type_name}'])
    for target_frame, source_frame in transforms:
        gate_arguments.extend(
            ['--transform', f'{target_frame}:{source_frame}']
        )

    gate = Node(
        package='muto_slam_mapping',
        executable='pipeline_readiness_gate',
        name=f'{stage}_readiness_gate',
        output='screen',
        arguments=gate_arguments,
    )
    include = scoped_include(package_name, launch_name, launch_arguments)

    def handle_gate_exit(event, _context):
        if event.returncode == 0:
            return [include, *(additional_success_actions or [])]
        return [
            EmitEvent(
                event=Shutdown(
                    reason=(
                        f'{stage} readiness gate failed with exit code '
                        f'{event.returncode}'
                    )
                )
            )
        ]

    return [
        TimerAction(
            period=LaunchConfiguration(delay_arg),
            actions=[gate],
            condition=IfCondition(enabled_condition),
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=gate,
                on_exit=handle_gate_exit,
            )
        ),
    ]


def generate_launch_description():
    default_slam_params_file = os.path.join(
        get_package_share_directory('muto_slam_mapping'),
        'config',
        'mapper_params_online_async.yaml',
    )
    default_nav2_params_file = os.path.join(
        get_package_share_directory('muto_slam_mapping'),
        'config',
        'nav2_params.yaml',
    )
    default_nav_to_pose_bt_file = os.path.join(
        get_package_share_directory('muto_slam_mapping'),
        'behavior_trees',
        'muto_nav_to_pose.xml',
    )
    default_nav_through_poses_bt_file = os.path.join(
        get_package_share_directory('muto_slam_mapping'),
        'behavior_trees',
        'muto_nav_through_poses.xml',
    )
    default_nav2_bag_params_file = os.path.join(
        get_package_share_directory('muto_nav2_bag'),
        'config',
        'nav2_bag.yaml',
    )
    default_locomotion_calibration_file = os.path.join(
        get_package_share_directory('yahboomcar_bringup'),
        'config',
        'muto_locomotion_provisional_20260806.yaml',
    )
    localization_launch_arguments = {
        'use_sim_time': LaunchConfiguration('use_sim_time'),
        'launch_lidar_odometry': 'true',
        'launch_foot_odometry': LaunchConfiguration(
            'launch_foot_odometry'
        ),
        'foot_motor_poll_rate': LaunchConfiguration(
            'foot_motor_poll_rate'
        ),
        'allow_experimental_high_rate_motor_polling': LaunchConfiguration(
            'allow_experimental_high_rate_motor_polling'
        ),
        'foot_odometry_source': LaunchConfiguration(
            'foot_odometry_source'
        ),
        'foot_max_motor_sequence_gap': LaunchConfiguration(
            'foot_max_motor_sequence_gap'
        ),
        'fuse_controller_attitude_yaw': LaunchConfiguration(
            'fuse_controller_attitude_yaw'
        ),
        'controller_attitude_yaw_variance': LaunchConfiguration(
            'controller_attitude_yaw_variance'
        ),
        'controller_attitude_stationary_gate': LaunchConfiguration(
            'controller_attitude_stationary_gate'
        ),
        'controller_attitude_stationary_settle_sec': LaunchConfiguration(
            'controller_attitude_stationary_settle_sec'
        ),
        'controller_attitude_motion_state_timeout_sec': LaunchConfiguration(
            'controller_attitude_motion_state_timeout_sec'
        ),
        'controller_attitude_stability_window_sec': LaunchConfiguration(
            'controller_attitude_stability_window_sec'
        ),
        'controller_attitude_minimum_snapshots': LaunchConfiguration(
            'controller_attitude_minimum_snapshots'
        ),
        'controller_attitude_max_yaw_span_rad': LaunchConfiguration(
            'controller_attitude_max_yaw_span_rad'
        ),
        'controller_attitude_republish_interval_sec': LaunchConfiguration(
            'controller_attitude_republish_interval_sec'
        ),
        'rf2o_log_level': LaunchConfiguration('rf2o_log_level'),
        'rf2o_covariance_profile': LaunchConfiguration(
            'rf2o_covariance_profile'
        ),
        'rf2o_translation_deadband': LaunchConfiguration(
            'rf2o_translation_deadband'
        ),
        'rf2o_yaw_deadband': LaunchConfiguration(
            'rf2o_yaw_deadband'
        ),
    }

    localization_raw_imu_enabled = PythonExpression([
        "'", LaunchConfiguration('launch_localization'), "' == 'true' and '",
        LaunchConfiguration('fuse_controller_attitude_yaw'), "' == 'false'",
    ])
    localization_controller_attitude_enabled = PythonExpression([
        "'", LaunchConfiguration('launch_localization'), "' == 'true' and '",
        LaunchConfiguration('fuse_controller_attitude_yaw'), "' == 'true'",
    ])

    localization_raw_imu_actions = readiness_gated_include(
        'localization_raw_imu',
        'localization_delay',
        'localization_readiness_timeout',
        localization_raw_imu_enabled,
        [
            ('/lidar/raw_laserscan', 'sensor_msgs/msg/LaserScan'),
            ('/imu/data_processed', 'sensor_msgs/msg/Imu'),
        ],
        [('base_frame', 'lidar_frame'), ('base_frame', 'imu_link')],
        'yahboomcar_bringup',
        'ekf_imu_lidar_launch.py',
        localization_launch_arguments,
    )
    localization_controller_attitude_actions = readiness_gated_include(
        'localization_controller_attitude',
        'localization_delay',
        'localization_readiness_timeout',
        localization_controller_attitude_enabled,
        [
            ('/lidar/raw_laserscan', 'sensor_msgs/msg/LaserScan'),
            (
                '/imu/controller_attitude',
                'muto_hexapod_interfaces_custom/msg/ControllerAttitude',
            ),
            (
                '/muto/motion_command_state',
                'muto_hexapod_interfaces_custom/msg/MotionCommandState',
            ),
        ],
        [('base_frame', 'lidar_frame'), ('base_frame', 'imu_link')],
        'yahboomcar_bringup',
        'ekf_imu_lidar_launch.py',
        {
            **localization_launch_arguments,
            'fuse_controller_attitude_yaw': 'true',
        },
    )
    mapping_actions = readiness_gated_include(
        'mapping',
        'mapping_delay',
        'mapping_readiness_timeout',
        LaunchConfiguration('launch_mapping'),
        [
            ('/odometry/filtered', 'nav_msgs/msg/Odometry'),
            (
                '/lidar/filtered_laserscan_no_downsample',
                'sensor_msgs/msg/LaserScan',
            ),
        ],
        [('odom', 'base_frame')],
        'muto_slam_mapping',
        'online_async_mapping_launch.py',
        {
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'slam_params_file': LaunchConfiguration('slam_params_file'),
        },
    )
    nav2_bag_include = scoped_include(
        'muto_nav2_bag',
        'record_nav2_bag_launch.py',
        {
            'params_file': LaunchConfiguration('nav2_bag_params_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'output_directory': LaunchConfiguration(
                'nav2_bag_output_directory'
            ),
            'max_bag_directories': LaunchConfiguration(
                'max_bag_directories'
            ),
            'nav2_params_file': LaunchConfiguration('nav2_params_file'),
            'slam_params_file': LaunchConfiguration('slam_params_file'),
            'nav_to_pose_bt_file': LaunchConfiguration(
                'nav_to_pose_bt_file'
            ),
            'nav_through_poses_bt_file': LaunchConfiguration(
                'nav_through_poses_bt_file'
            ),
            'locomotion_calibration_file': LaunchConfiguration(
                'locomotion_calibration_file'
            ),
        },
        condition=IfCondition(LaunchConfiguration('launch_nav2_bag')),
    )
    nav2_actions = readiness_gated_include(
        'nav2',
        'nav2_delay',
        'nav2_readiness_timeout',
        LaunchConfiguration('launch_nav2'),
        [
            ('/map', 'nav_msgs/msg/OccupancyGrid'),
            ('/lidar/filtered_laserscan', 'sensor_msgs/msg/LaserScan'),
        ],
        [('map', 'base_frame')],
        'muto_slam_mapping',
        'nav2_planner_controller_launch.py',
        {
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('nav2_params_file'),
            'namespace': LaunchConfiguration('namespace'),
            'use_namespace': LaunchConfiguration('use_namespace'),
            'autostart': LaunchConfiguration('nav2_autostart'),
            'use_respawn': LaunchConfiguration('nav2_use_respawn'),
            'log_level': LaunchConfiguration('nav2_log_level'),
        },
        additional_success_actions=[nav2_bag_include],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true.',
        ),
        DeclareLaunchArgument(
            'launch_hardware',
            default_value='true',
            description='Start TG30 LiDAR, Orbbec camera, and Muto base driver.',
        ),
        DeclareLaunchArgument(
            'locomotion_update_rate_hz',
            default_value='50.0',
            description=(
                'Fixed locomotion trajectory phase rate used by the Muto '
                'driver and foot odometry.'
            ),
        ),
        DeclareLaunchArgument(
            'batch_gait_phase_writes',
            default_value='true',
            description=(
                'Write all six self-delimiting leg frames for one gait phase '
                'as a contiguous serial batch. False is the rollback mode.'
            ),
        ),
        DeclareLaunchArgument(
            'cmd_vel_timeout',
            default_value='0.5',
            description=(
                'Seconds without cmd_vel before locomotion returns to '
                'standby.'
            ),
        ),
        DeclareLaunchArgument(
            'locomotion_command_mapping',
            default_value='geometric',
            description=(
                'Map cmd_vel through custom gait geometry by default. Use '
                'calibrated for an external measured profile or legacy_100 '
                'only as an explicit rollback.'
            ),
        ),
        DeclareLaunchArgument(
            'locomotion_calibration_file',
            default_value=default_locomotion_calibration_file,
            description=(
                'Muto velocity profile used only by calibrated mapping.'
            ),
        ),
        DeclareLaunchArgument(
            'imu_publish_rate_hz',
            default_value='10.0',
            description=(
                'Host poll rate for the controller-cached IMU snapshot. Ten '
                'Hz retains the prior timestamp-observation window while raw '
                'and attitude use separate coordinated post-gait slots.'
            ),
        ),
        DeclareLaunchArgument(
            'imu_attitude_publish_rate_hz',
            default_value='10.0',
            description=(
                'Host poll rate for the controller-fused 0x60 Euler '
                'attitude. It receives separate coordinated gait slots from '
                'raw 0x61. Set 0.0 to disable. It is recorded by default and '
                'fused only with fuse_controller_attitude_yaw:=true.'
            ),
        ),
        DeclareLaunchArgument(
            'imu_suppress_identical_snapshots',
            default_value='true',
            description=(
                'Suppress consecutive identical accel/gyro snapshots '
                'instead of assigning cached data new ROS timestamps.'
            ),
        ),
        DeclareLaunchArgument(
            'imu_response_timeout_sec',
            default_value='0.008',
            description='Runtime IMU serial-response budget in seconds.',
        ),
        DeclareLaunchArgument(
            'imu_stale_warning_sec',
            default_value='2.0',
            description='Seconds without a changed IMU snapshot before warning.',
        ),
        DeclareLaunchArgument(
            'imu_locomotion_guard_sec',
            default_value='0.003',
            description='Time reserved before each locomotion phase deadline.',
        ),
        DeclareLaunchArgument(
            'imu_calibration_sample_count',
            default_value='10',
            description=(
                'Changed stationary accel/gyro snapshots used for startup '
                'calibration.'
            ),
        ),
        DeclareLaunchArgument(
            'imu_calibration_max_reads',
            default_value='150',
            description='Maximum serial read attempts during startup IMU calibration.',
        ),
        DeclareLaunchArgument(
            'imu_calibration_timeout_sec',
            default_value='15.0',
            description='Maximum wall-clock duration of startup IMU calibration.',
        ),
        DeclareLaunchArgument(
            'imu_calibration_read_interval',
            default_value='0.1',
            description='Seconds between startup IMU calibration polls.',
        ),
        DeclareLaunchArgument(
            'launch_sensor_tf',
            default_value='true',
            description='Start static sensor TF publishers.',
        ),
        DeclareLaunchArgument(
            'launch_localization',
            default_value='true',
            description='Start LiDAR filtering, RF2O odometry, and EKF.',
        ),
        DeclareLaunchArgument(
            'launch_foot_odometry',
            default_value='false',
            description=(
                'Opt in to diagnostic measured-joint foot odometry. Its '
                'blocking motor reads disrupt the 50 Hz gait on stock '
                'controller firmware.'
            ),
        ),
        DeclareLaunchArgument(
            'foot_motor_poll_rate',
            default_value='2.0',
            description=(
                'Rate in Hz for synchronized 18-joint motor feedback used '
                'by foot odometry. The current blocking hardware interface '
                'has a 2 Hz production limit.'
            ),
        ),
        DeclareLaunchArgument(
            'allow_experimental_high_rate_motor_polling',
            default_value='false',
            description=(
                'Required opt-in when foot_motor_poll_rate exceeds the 2 Hz '
                'production limit. Use only for controlled tests.'
            ),
        ),
        DeclareLaunchArgument(
            'foot_odometry_source',
            default_value='measured_joints',
            description=(
                'Foot odometry source: measured_joints or the '
                'regression-only commanded_targets mode.'
            ),
        ),
        DeclareLaunchArgument(
            'foot_max_motor_sequence_gap',
            default_value='10',
            description=(
                'Maximum gait-phase gap between measured joint snapshots '
                'used by foot odometry.'
            ),
        ),
        DeclareLaunchArgument(
            'fuse_controller_attitude_yaw',
            default_value='true',
            description=(
                'Replace the sparse raw-gyro EKF input with stable, stop-only '
                'relative controller-fused 0x60 yaw corrections.'
            ),
        ),
        DeclareLaunchArgument(
            'controller_attitude_yaw_variance',
            default_value='0.004873878716587337',
            description=(
                'Yaw variance in rad^2 for accepted 0x60 corrections; the '
                'default is (4 deg)^2.'
            ),
        ),
        DeclareLaunchArgument(
            'controller_attitude_stationary_gate',
            default_value='true',
            description=(
                'Only admit stable controller yaw while both selected and '
                'active locomotion states are stationary.'
            ),
        ),
        DeclareLaunchArgument(
            'controller_attitude_stationary_settle_sec',
            default_value='2.0',
            description='Required stationary dwell before correction.',
        ),
        DeclareLaunchArgument(
            'controller_attitude_motion_state_timeout_sec',
            default_value='0.25',
            description='Maximum accepted motion-command-state age.',
        ),
        DeclareLaunchArgument(
            'controller_attitude_stability_window_sec',
            default_value='1.0',
            description='Changed-attitude stability window in seconds.',
        ),
        DeclareLaunchArgument(
            'controller_attitude_minimum_snapshots',
            default_value='3',
            description='Minimum changed snapshots in the stable window.',
        ),
        DeclareLaunchArgument(
            'controller_attitude_max_yaw_span_rad',
            default_value='0.017453292519943295',
            description='Maximum stable-window yaw span (1 degree).',
        ),
        DeclareLaunchArgument(
            'controller_attitude_republish_interval_sec',
            default_value='0.0',
            description=(
                'Correction interval within one stop; zero means one '
                'correction per stationary episode.'
            ),
        ),
        DeclareLaunchArgument(
            'rf2o_log_level',
            default_value='error',
            description='ROS log level for the RF2O process.',
        ),
        DeclareLaunchArgument(
            'rf2o_covariance_profile',
            default_value='measured',
            description=(
                'RF2O odometry covariance profile: measured, relaxed, '
                'conservative, custom, or legacy_zero.'
            ),
        ),
        DeclareLaunchArgument(
            'rf2o_translation_deadband',
            default_value='0.0',
            description=(
                'RF2O planar translation deadband in meters. The production '
                'default is 0.0; jump rejection remains active.'
            ),
        ),
        DeclareLaunchArgument(
            'rf2o_yaw_deadband',
            default_value='0.0',
            description=(
                'RF2O yaw deadband in radians. The production default is '
                '0.0; jump rejection remains active.'
            ),
        ),
        DeclareLaunchArgument(
            'launch_mapping',
            default_value='true',
            description='Start SLAM Toolbox online asynchronous mapping.',
        ),
        DeclareLaunchArgument(
            'launch_nav2',
            default_value='true',
            description='Start Nav2 planner, controller, and navigator servers.',
        ),
        DeclareLaunchArgument(
            'launch_nav2_bag',
            default_value='true',
            description=(
                'Start the compact session-level Nav2 recorder after the '
                'Nav2 readiness gate succeeds.'
            ),
        ),
        DeclareLaunchArgument(
            'nav2_bag_params_file',
            default_value=default_nav2_bag_params_file,
            description='Nav2 recorder topic profile; compact by default.',
        ),
        DeclareLaunchArgument(
            'nav2_bag_output_directory',
            default_value='/opt/muto_rs_ws/bags',
            description='Parent directory for automatic Nav2 session bags.',
        ),
        DeclareLaunchArgument(
            'max_bag_directories',
            default_value='20',
            description='Shared maximum count of recognized Muto bag folders.',
        ),
        DeclareLaunchArgument(
            'sensor_tf_delay',
            default_value='1.0',
            description='Minimum delay before static sensor TF starts.',
        ),
        DeclareLaunchArgument(
            'localization_delay',
            default_value='3.0',
            description='Minimum delay before localization readiness checks.',
        ),
        DeclareLaunchArgument(
            'mapping_delay',
            default_value='8.0',
            description='Minimum delay before mapping readiness checks.',
        ),
        DeclareLaunchArgument(
            'nav2_delay',
            default_value='12.0',
            description='Minimum delay before Nav2 readiness checks.',
        ),
        DeclareLaunchArgument(
            'localization_readiness_timeout',
            default_value='120.0',
            description=(
                'Seconds to wait for raw scan, calibrated IMU output, and '
                'sensor TF.'
            ),
        ),
        DeclareLaunchArgument(
            'mapping_readiness_timeout',
            default_value='90.0',
            description='Seconds to wait for filtered odometry and odom TF.',
        ),
        DeclareLaunchArgument(
            'nav2_readiness_timeout',
            default_value='120.0',
            description='Seconds to wait for map, filtered LiDAR scan, and map TF.',
        ),
        DeclareLaunchArgument(
            'launch_camera_obstacle_scan',
            default_value='true',
            description=(
                'Start the independent camera obstacle source used by Nav2.'
            ),
        ),
        DeclareLaunchArgument(
            'camera_scan_max_publish_rate',
            default_value='7.0',
            description='Maximum camera-depth-to-scan processing rate in Hz.',
        ),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=default_slam_params_file,
            description='SLAM Toolbox online async parameter file.',
        ),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=default_nav2_params_file,
            description='Nav2 parameter file.',
        ),
        DeclareLaunchArgument(
            'nav_to_pose_bt_file',
            default_value=default_nav_to_pose_bt_file,
            description='NavigateToPose tree snapshotted by the Nav2 bag.',
        ),
        DeclareLaunchArgument(
            'nav_through_poses_bt_file',
            default_value=default_nav_through_poses_bt_file,
            description=(
                'NavigateThroughPoses tree snapshotted by the Nav2 bag.'
            ),
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Top-level Nav2 namespace.',
        ),
        DeclareLaunchArgument(
            'use_namespace',
            default_value='False',
            description='Whether to apply a namespace to Nav2.',
        ),
        DeclareLaunchArgument(
            'nav2_autostart',
            default_value='True',
            description='Automatically activate Nav2 lifecycle nodes.',
        ),
        DeclareLaunchArgument(
            'nav2_use_respawn',
            default_value='False',
            description='Whether to respawn crashed Nav2 nodes.',
        ),
        DeclareLaunchArgument(
            'nav2_log_level',
            default_value='info',
            description='Log level for Nav2 nodes.',
        ),
        delayed_include(
            'sensor_tf_delay',
            'launch_sensor_tf',
            'tf2_publisher',
            'all_tf2_publishers_launch.py',
            {'publish_odom_tf': 'false'},
        ),
        delayed_include(
            'sensor_tf_delay',
            'launch_camera_obstacle_scan',
            'lidar_pointcloud_filter',
            'camera_depth_to_laserscan_launch.py',
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'max_publish_rate': LaunchConfiguration(
                    'camera_scan_max_publish_rate'
                ),
            },
        ),
        scoped_include(
            'yahboomcar_bringup',
            'muto_hardware_launch.py',
            {
                'locomotion_update_rate_hz': LaunchConfiguration(
                    'locomotion_update_rate_hz'
                ),
                'batch_gait_phase_writes': LaunchConfiguration(
                    'batch_gait_phase_writes'
                ),
                'cmd_vel_timeout': LaunchConfiguration(
                    'cmd_vel_timeout'
                ),
                'locomotion_command_mapping': LaunchConfiguration(
                    'locomotion_command_mapping'
                ),
                'locomotion_calibration_file': LaunchConfiguration(
                    'locomotion_calibration_file'
                ),
                'imu_publish_rate_hz': LaunchConfiguration(
                    'imu_publish_rate_hz'
                ),
                'imu_attitude_publish_rate_hz': LaunchConfiguration(
                    'imu_attitude_publish_rate_hz'
                ),
                'imu_suppress_identical_snapshots': LaunchConfiguration(
                    'imu_suppress_identical_snapshots'
                ),
                'imu_response_timeout_sec': LaunchConfiguration(
                    'imu_response_timeout_sec'
                ),
                'imu_stale_warning_sec': LaunchConfiguration(
                    'imu_stale_warning_sec'
                ),
                'imu_locomotion_guard_sec': LaunchConfiguration(
                    'imu_locomotion_guard_sec'
                ),
                'imu_calibration_sample_count': LaunchConfiguration(
                    'imu_calibration_sample_count'
                ),
                'imu_calibration_max_reads': LaunchConfiguration(
                    'imu_calibration_max_reads'
                ),
                'imu_calibration_timeout_sec': LaunchConfiguration(
                    'imu_calibration_timeout_sec'
                ),
                'imu_calibration_read_interval': LaunchConfiguration(
                    'imu_calibration_read_interval'
                ),
            },
            condition=IfCondition(LaunchConfiguration('launch_hardware')),
        ),
        *localization_raw_imu_actions,
        *localization_controller_attitude_actions,
        *mapping_actions,
        *nav2_actions,
    ])
