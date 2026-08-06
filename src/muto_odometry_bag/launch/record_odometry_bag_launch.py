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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'bag_path',
            default_value='',
            description=(
                'Output rosbag2 directory. Empty creates a timestamped '
                'directory in the current working directory.'
            ),
        ),
        DeclareLaunchArgument(
            'motor_poll_rate',
            default_value='2.0',
            description=(
                'Rate in Hz for recording get_motor_angles snapshots. The '
                'production limit is 2 Hz.'
            ),
        ),
        DeclareLaunchArgument(
            'allow_experimental_high_rate_motor_polling',
            default_value='false',
            description=(
                'Required opt-in above the 2 Hz production limit. Use only '
                'for a controlled hardware benchmark.'
            ),
        ),
        Node(
            package='muto_odometry_bag',
            executable='odometry_bag_recorder',
            name='odometry_bag_recorder',
            output='screen',
            parameters=[{
                'bag_path': LaunchConfiguration('bag_path'),
                'motor_poll_rate': ParameterValue(
                    LaunchConfiguration('motor_poll_rate'),
                    value_type=float,
                ),
                'allow_experimental_high_rate_motor_polling': ParameterValue(
                    LaunchConfiguration(
                        'allow_experimental_high_rate_motor_polling'
                    ),
                    value_type=bool,
                ),
            }],
        ),
    ])
