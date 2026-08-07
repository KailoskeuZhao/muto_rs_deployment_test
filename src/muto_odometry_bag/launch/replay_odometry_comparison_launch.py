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

import copy
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml


def launch_file(package_name, launch_name):
    return os.path.join(
        get_package_share_directory(package_name),
        'launch',
        launch_name,
    )


def load_node_parameters(path, node_name):
    with open(path, encoding='utf-8') as parameter_file:
        document = yaml.safe_load(parameter_file)
    return copy.deepcopy(document[node_name]['ros__parameters'])


def without_sensor(parameters, sensor_name):
    """Return an EKF parameter copy without one numbered sensor input."""
    prefix = f'{sensor_name}_'
    return {
        name: copy.deepcopy(value)
        for name, value in parameters.items()
        if name != sensor_name and not name.startswith(prefix)
    }


def generate_launch_description():
    bringup_share = get_package_share_directory('yahboomcar_bringup')
    lidar_imu_parameters = load_node_parameters(
        os.path.join(bringup_share, 'config', 'ekf_lidar_imu.yaml'),
        'ekf_filter_node',
    )
    lidar_imu_parameters.update({
        'publish_tf': False,
        'use_sim_time': True,
    })
    lidar_only_parameters = without_sensor(lidar_imu_parameters, 'imu0')
    lidar_controller_attitude_parameters = copy.deepcopy(
        lidar_imu_parameters
    )
    lidar_controller_attitude_parameters.update(load_node_parameters(
        os.path.join(
            bringup_share,
            'config',
            'ekf_controller_attitude_yaw.yaml',
        ),
        'ekf_filter_node',
    ))
    raw_lidar_imu_parameters = copy.deepcopy(lidar_imu_parameters)
    raw_lidar_imu_parameters['odom0'] = '/scan_odom_raw_profiled'

    sensor_tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file('tf2_publisher', 'all_tf2_publishers_launch.py')
        ),
        launch_arguments={'publish_odom_tf': 'false'}.items(),
        condition=UnlessCondition(
            LaunchConfiguration('replay_recorded_tf_static')
        ),
    )
    original_fused_odometry = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file('yahboomcar_bringup', 'ekf_imu_lidar_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'launch_lidar_odometry': 'true',
            # Keep the normal comparison baseline on the sparse raw gyro even
            # though live bringup now defaults to stop-only controller yaw.
            'fuse_controller_attitude_yaw': 'false',
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
            'rf2o_profiled_raw_odom_topic': '/scan_odom_raw_profiled',
            'rf2o_covariance_profile': LaunchConfiguration(
                'rf2o_covariance_profile'
            ),
            'rf2o_translation_deadband': LaunchConfiguration(
                'rf2o_translation_deadband'
            ),
            'rf2o_yaw_deadband': LaunchConfiguration(
                'rf2o_yaw_deadband'
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
            'minimum_start_delay_sec',
            default_value='2.0',
            description=(
                'Minimum wall time before replay after consumers are ready; '
                'the comparison margin also lets output recorders discover '
                'all EKF topics.'
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
            'foot_odometry_source',
            default_value='measured_joints',
            description=(
                'Foot odometry source: measured_joints or the '
                'regression-only commanded_targets mode.'
            ),
        ),
        DeclareLaunchArgument(
            'launch_foot_odometry',
            default_value='false',
            description=(
                'Launch and compare measured-joint foot odometry. Leave '
                'false for source bags without motor-angle snapshots.'
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
            'lidar_imu_output_topic',
            default_value='/odometry/lidar_imu',
            description='Output from the comparison LiDAR plus IMU EKF.',
        ),
        DeclareLaunchArgument(
            'lidar_only_output_topic',
            default_value='/odometry/lidar_only',
            description='Output from the comparison filtered-LiDAR-only EKF.',
        ),
        DeclareLaunchArgument(
            'raw_lidar_imu_output_topic',
            default_value='/odometry/raw_lidar_imu',
            description='Output from the raw RF2O plus IMU comparison EKF.',
        ),
        DeclareLaunchArgument(
            'compare_controller_attitude',
            default_value='false',
            description=(
                'Add a comparison EKF that replaces the sparse raw gyro '
                'with relative controller-fused 0x60 yaw.'
            ),
        ),
        DeclareLaunchArgument(
            'lidar_controller_attitude_output_topic',
            default_value='/odometry/lidar_controller_attitude',
            description=(
                'Output from the filtered-LiDAR plus relative controller '
                '0x60 yaw comparison EKF.'
            ),
        ),
        DeclareLaunchArgument(
            'controller_attitude_yaw_variance',
            default_value='0.004873878716587337',
            description=(
                'Yaw variance in rad^2 for the controller-attitude '
                'comparison; default is (4 deg)^2.'
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
        original_fused_odometry,
        Node(
            package='yahboomcar_imu',
            executable='controller_attitude_yaw_adapter',
            name='controller_attitude_yaw_adapter_comparison',
            output='screen',
            condition=IfCondition(
                LaunchConfiguration('compare_controller_attitude')
            ),
            parameters=[{
                'input_topic': '/imu/controller_attitude',
                'output_topic': '/imu/controller_attitude_imu',
                'frame_id': 'imu_link',
                'yaw_variance_rad2': ParameterValue(
                    LaunchConfiguration(
                        'controller_attitude_yaw_variance'
                    ),
                    value_type=float,
                ),
                'suppress_identical_snapshots': True,
                'stationary_gate_enabled': ParameterValue(
                    LaunchConfiguration(
                        'controller_attitude_stationary_gate'
                    ),
                    value_type=bool,
                ),
                'motion_state_topic': '/muto/motion_command_state',
                'stationary_settle_sec': ParameterValue(
                    LaunchConfiguration(
                        'controller_attitude_stationary_settle_sec'
                    ),
                    value_type=float,
                ),
                'motion_state_timeout_sec': ParameterValue(
                    LaunchConfiguration(
                        'controller_attitude_motion_state_timeout_sec'
                    ),
                    value_type=float,
                ),
                'stability_window_sec': ParameterValue(
                    LaunchConfiguration(
                        'controller_attitude_stability_window_sec'
                    ),
                    value_type=float,
                ),
                'minimum_distinct_snapshots': ParameterValue(
                    LaunchConfiguration(
                        'controller_attitude_minimum_snapshots'
                    ),
                    value_type=int,
                ),
                'max_stationary_yaw_span_rad': ParameterValue(
                    LaunchConfiguration(
                        'controller_attitude_max_yaw_span_rad'
                    ),
                    value_type=float,
                ),
                'stationary_republish_interval_sec': ParameterValue(
                    LaunchConfiguration(
                        'controller_attitude_republish_interval_sec'
                    ),
                    value_type=float,
                ),
                'use_sim_time': True,
            }],
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_lidar_only_comparison',
            output='screen',
            parameters=[lidar_only_parameters],
            remappings=[
                ('odometry/filtered',
                 LaunchConfiguration('lidar_only_output_topic')),
            ],
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_lidar_controller_attitude_comparison',
            output='screen',
            condition=IfCondition(
                LaunchConfiguration('compare_controller_attitude')
            ),
            parameters=[lidar_controller_attitude_parameters],
            remappings=[
                (
                    'odometry/filtered',
                    LaunchConfiguration(
                        'lidar_controller_attitude_output_topic'
                    ),
                ),
            ],
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_lidar_imu_comparison',
            output='screen',
            parameters=[lidar_imu_parameters],
            remappings=[
                ('odometry/filtered',
                 LaunchConfiguration('lidar_imu_output_topic')),
            ],
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_raw_lidar_imu_comparison',
            output='screen',
            parameters=[raw_lidar_imu_parameters],
            remappings=[
                ('odometry/filtered',
                 LaunchConfiguration('raw_lidar_imu_output_topic')),
            ],
        ),
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
                # Baseline comparison EKFs always consume the standard IMU,
                # even when the controller-attitude branch is also enabled.
                'require_standard_imu_input': True,
                'require_controller_attitude_input': ParameterValue(
                    LaunchConfiguration('compare_controller_attitude'),
                    value_type=bool,
                ),
                'require_motion_command_state_input': ParameterValue(
                    PythonExpression([
                        "'",
                        LaunchConfiguration('compare_controller_attitude'),
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
