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
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
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
            'launch_foot_odometry': 'true',
            'rf2o_log_level': LaunchConfiguration('rf2o_log_level'),
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
        sensor_tf,
        original_fused_odometry,
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
                'require_foot_inputs': True,
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
