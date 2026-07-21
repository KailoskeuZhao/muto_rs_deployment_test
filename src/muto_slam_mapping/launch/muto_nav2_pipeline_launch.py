import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
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


def delayed_include(
    delay_arg,
    enabled_arg,
    package_name,
    launch_name,
    launch_arguments=None,
):
    include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file(package_name, launch_name)),
        launch_arguments=(launch_arguments or {}).items(),
    )
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
    include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file(package_name, launch_name)),
        launch_arguments=(launch_arguments or {}).items(),
    )

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

    localization_actions = readiness_gated_include(
        'localization',
        'localization_delay',
        'localization_readiness_timeout',
        'launch_localization',
        [('/lidar/raw_laserscan', 'sensor_msgs/msg/LaserScan')],
        [('base_frame', 'lidar_frame'), ('base_frame', 'imu_link')],
        'yahboomcar_bringup',
        'ekf_imu_lidar_launch.py',
        {
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'launch_lidar_odometry': 'true',
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
            'launch_fused_laserscan': LaunchConfiguration(
                'launch_fused_laserscan'
            ),
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
            ('/fused/laserscan', 'sensor_msgs/msg/LaserScan'),
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
            'launch_mapping',
            default_value='true',
            description='Start fused LaserScan generation and SLAM Toolbox.',
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
            default_value='60.0',
            description='Seconds to wait for raw scan and sensor TF.',
        ),
        DeclareLaunchArgument(
            'mapping_readiness_timeout',
            default_value='90.0',
            description='Seconds to wait for filtered odometry and odom TF.',
        ),
        DeclareLaunchArgument(
            'nav2_readiness_timeout',
            default_value='120.0',
            description='Seconds to wait for map, fused scan, and map TF.',
        ),
        DeclareLaunchArgument(
            'launch_fused_laserscan',
            default_value='true',
            description='Let mapping start /fused/laserscan generation.',
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
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                launch_file('yahboomcar_bringup', 'muto_hardware_launch.py')
            ),
            condition=IfCondition(LaunchConfiguration('launch_hardware')),
        ),
        *localization_actions,
        *mapping_actions,
        *nav2_actions,
    ])
