# Copyright 2026 kailoskeuzhao
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_file(package_name, launch_name):
    return os.path.join(
        get_package_share_directory(package_name),
        'launch',
        launch_name,
    )


def generate_launch_description():
    sensor_tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file('tf2_publisher', 'all_tf2_publishers_launch.py')
        ),
        launch_arguments={'publish_odom_tf': 'false'}.items(),
        condition=UnlessCondition(
            LaunchConfiguration('replay_recorded_tf_static')
        ),
    )
    original_odometry = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file('yahboomcar_bringup', 'ekf_imu_lidar_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'launch_lidar_odometry': 'true',
            'launch_foot_odometry': LaunchConfiguration(
                'launch_foot_odometry'
            ),
            'foot_odometry_source': LaunchConfiguration(
                'foot_odometry_source'
            ),
            'foot_max_motor_sequence_gap': LaunchConfiguration(
                'foot_max_motor_sequence_gap'
            ),
            'foot_motor_poll_rate': LaunchConfiguration(
                'foot_motor_poll_rate'
            ),
            'allow_experimental_high_rate_motor_polling': 'true',
            'rf2o_log_level': LaunchConfiguration('rf2o_log_level'),
            'rf2o_freq': PythonExpression([
                LaunchConfiguration('playback_rate'), ' * 64.0'
            ]),
            'rf2o_covariance_profile': LaunchConfiguration(
                'rf2o_covariance_profile'
            ),
            'rf2o_translation_deadband': LaunchConfiguration(
                'rf2o_translation_deadband'
            ),
            'rf2o_yaw_deadband': LaunchConfiguration(
                'rf2o_yaw_deadband'
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
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'bag_path',
            description='Input rosbag2 directory produced by this package.',
        ),
        DeclareLaunchArgument(
            'playback_rate',
            default_value='1.0',
            description=(
                'Wall-clock replay multiplier. Simulated ROS time and sensor '
                'timestamps retain their recorded spacing.'
            ),
        ),
        DeclareLaunchArgument(
            'launch_foot_odometry',
            default_value='true',
            description=(
                'Run the original foot odometry node and fuse /foot_odom.'
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
            'foot_motor_poll_rate',
            default_value='2.0',
            description=(
                'Virtual motor-service polling rate during replay. High '
                'rates are safe here because no hardware serial read occurs.'
            ),
        ),
        DeclareLaunchArgument(
            'foot_max_motor_sequence_gap',
            default_value='10',
            description=(
                'Maximum gait-phase gap between measured joint snapshots.'
            ),
        ),
        DeclareLaunchArgument(
            'minimum_start_delay_sec',
            default_value='0.5',
            description=(
                'Minimum wall time before replay after consumers are ready.'
            ),
        ),
        DeclareLaunchArgument(
            'readiness_timeout_sec',
            default_value='30.0',
            description=(
                'Wall time allowed for the original odometry subscriptions.'
            ),
        ),
        DeclareLaunchArgument(
            'rf2o_log_level',
            default_value='error',
            description='ROS log level for the original RF2O process.',
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
            description='RF2O planar translation deadband in meters.',
        ),
        DeclareLaunchArgument(
            'rf2o_yaw_deadband',
            default_value='0.0',
            description='RF2O yaw deadband in radians.',
        ),
        DeclareLaunchArgument(
            'replay_processed_imu',
            default_value='true',
            description=(
                'Replay recorded /imu/data_processed. Set false when a new '
                'processor will consume replayed /imu/data_raw and publish '
                '/imu/data_processed.'
            ),
        ),
        DeclareLaunchArgument(
            'replay_recorded_tf_static',
            default_value='false',
            description=(
                'Publish /tf_static captured in the source bag instead of '
                'launching the current static sensor TF publishers.'
            ),
        ),
        DeclareLaunchArgument(
            'fuse_controller_attitude_yaw',
            default_value='true',
            description=(
                'Replace the sparse raw-gyro EKF input with stable, stop-only '
                'relative yaw adapted from recorded controller attitude.'
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
            description='Enable the stop-only stability gate.',
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
            description='Zero allows one correction per stationary episode.',
        ),
        sensor_tf,
        original_odometry,
        Node(
            package='muto_odometry_bag',
            executable='odometry_bag_replayer',
            name='odometry_bag_replayer',
            output='screen',
            parameters=[{
                'bag_path': LaunchConfiguration('bag_path'),
                'playback_rate': ParameterValue(
                    LaunchConfiguration('playback_rate'),
                    value_type=float,
                ),
                'minimum_start_delay_sec': ParameterValue(
                    LaunchConfiguration('minimum_start_delay_sec'),
                    value_type=float,
                ),
                'readiness_timeout_sec': ParameterValue(
                    LaunchConfiguration('readiness_timeout_sec'),
                    value_type=float,
                ),
                'require_foot_inputs': ParameterValue(
                    LaunchConfiguration('launch_foot_odometry'),
                    value_type=bool,
                ),
                'require_standard_imu_input': ParameterValue(
                    PythonExpression([
                        "'",
                        LaunchConfiguration(
                            'fuse_controller_attitude_yaw'
                        ),
                        "' != 'true'",
                    ]),
                    value_type=bool,
                ),
                'require_controller_attitude_input': ParameterValue(
                    LaunchConfiguration('fuse_controller_attitude_yaw'),
                    value_type=bool,
                ),
                'require_motion_command_state_input': ParameterValue(
                    PythonExpression([
                        "'",
                        LaunchConfiguration('fuse_controller_attitude_yaw'),
                        "' == 'true' and '",
                        LaunchConfiguration(
                            'controller_attitude_stationary_gate'
                        ),
                        "' == 'true'",
                    ]),
                    value_type=bool,
                ),
                'replay_processed_imu': ParameterValue(
                    LaunchConfiguration('replay_processed_imu'),
                    value_type=bool,
                ),
                'replay_recorded_tf_static': ParameterValue(
                    LaunchConfiguration('replay_recorded_tf_static'),
                    value_type=bool,
                ),
            }],
        ),
    ])
