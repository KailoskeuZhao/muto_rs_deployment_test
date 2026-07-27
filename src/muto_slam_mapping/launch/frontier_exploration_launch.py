import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_params_file = os.path.join(
        get_package_share_directory('muto_slam_mapping'),
        'config',
        'frontier_exploration_params.yaml',
    )

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    control_service_enabled = LaunchConfiguration('control_service_enabled')
    log_level = LaunchConfiguration('log_level')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Muto frontier exploration parameter file.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true.',
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description=(
                'Start exploration immediately. Set false to wait for '
                'frontier_exploration_ctl start.'
            ),
        ),
        DeclareLaunchArgument(
            'control_service_enabled',
            default_value='true',
            description=(
                'Expose /control_exploration for frontier_exploration_ctl.'
            ),
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='ROS log level for the frontier explorer node.',
        ),
        Node(
            package='frontier_exploration_ros2',
            executable='frontier_explorer',
            name='frontier_explorer',
            output='screen',
            parameters=[
                params_file,
                {
                    'use_sim_time': ParameterValue(
                        use_sim_time,
                        value_type=bool,
                    ),
                    'autostart': ParameterValue(
                        autostart,
                        value_type=bool,
                    ),
                    'control_service_enabled': ParameterValue(
                        control_service_enabled,
                        value_type=bool,
                    ),
                },
            ],
            arguments=['--ros-args', '--log-level', log_level],
        ),
    ])
