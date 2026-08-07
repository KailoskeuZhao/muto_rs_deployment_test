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
from launch.substitutions import LaunchConfiguration
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
    enabled_arg,
    topics,
    transforms,
    package_name,
    launch_name,
    launch_arguments=None,
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
            return [include]
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
            condition=IfCondition(LaunchConfiguration(enabled_arg)),
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
    default_locomotion_calibration_file = os.path.join(
        get_package_share_directory('yahboomcar_bringup'),
        'config',
        'muto_locomotion_provisional_20260806.yaml',
    )

    localization_actions = readiness_gated_include(
        'localization',
        'localization_delay',
        'localization_readiness_timeout',
        'launch_localization',
        [
            ('/lidar/raw_laserscan', 'sensor_msgs/msg/LaserScan'),
            ('/imu/data_processed', 'sensor_msgs/msg/Imu'),
        ],
        [('base_frame', 'lidar_frame'), ('base_frame', 'imu_link')],
        'yahboomcar_bringup',
        'ekf_imu_lidar_launch.py',
        {
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'launch_lidar_odometry': 'true',
            'launch_foot_odometry': LaunchConfiguration(
                'launch_foot_odometry'
            ),
            'foot_motor_poll_rate': LaunchConfiguration(
                'foot_motor_poll_rate'
            ),
            'allow_experimental_high_rate_motor_polling': (
                LaunchConfiguration(
                    'allow_experimental_high_rate_motor_polling'
                )
            ),
            'foot_odometry_source': LaunchConfiguration(
                'foot_odometry_source'
            ),
            'foot_max_motor_sequence_gap': LaunchConfiguration(
                'foot_max_motor_sequence_gap'
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
        },
    )
    mapping_actions = readiness_gated_include(
        'mapping',
        'mapping_delay',
        'mapping_readiness_timeout',
        'launch_mapping',
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
    nav2_actions = readiness_gated_include(
        'nav2',
        'nav2_delay',
        'nav2_readiness_timeout',
        'launch_nav2',
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
            'cmd_vel_timeout',
            default_value='0.5',
            description=(
                'Seconds without cmd_vel before locomotion returns to '
                'standby.'
            ),
        ),
        DeclareLaunchArgument(
            'locomotion_command_mapping',
            default_value='calibrated',
            description=(
                'Map physical cmd_vel through the configured Muto velocity '
                'profile. Use legacy_100 only as an explicit rollback.'
            ),
        ),
        DeclareLaunchArgument(
            'locomotion_calibration_file',
            default_value=default_locomotion_calibration_file,
            description=(
                'Muto physical velocity-to-gait calibration profile.'
            ),
        ),
        DeclareLaunchArgument(
            'imu_calibration_sample_count',
            default_value='300',
            description='Valid stationary IMU samples used for startup calibration.',
        ),
        DeclareLaunchArgument(
            'imu_calibration_max_reads',
            default_value='600',
            description='Maximum serial read attempts during startup IMU calibration.',
        ),
        DeclareLaunchArgument(
            'imu_calibration_timeout_sec',
            default_value='30.0',
            description='Maximum wall-clock duration of startup IMU calibration.',
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
            default_value='true',
            description=(
                'Fuse continuity-gated measured-joint foot velocity into '
                'the localization EKF.'
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
                'cmd_vel_timeout': LaunchConfiguration(
                    'cmd_vel_timeout'
                ),
                'locomotion_command_mapping': LaunchConfiguration(
                    'locomotion_command_mapping'
                ),
                'locomotion_calibration_file': LaunchConfiguration(
                    'locomotion_calibration_file'
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
            },
            condition=IfCondition(LaunchConfiguration('launch_hardware')),
        ),
        *localization_actions,
        *mapping_actions,
        *nav2_actions,
    ])
