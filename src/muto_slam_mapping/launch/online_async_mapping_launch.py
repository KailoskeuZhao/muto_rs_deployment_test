import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def scoped_include(launch_path, launch_arguments, condition=None):
    """Include a component launch without exporting its generic arguments."""
    include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_path),
        launch_arguments=launch_arguments,
        condition=condition,
    )
    return GroupAction(actions=[include], scoped=True)


def generate_launch_description():
    slam_params_file = os.path.join(
        get_package_share_directory('muto_slam_mapping'),
        'config',
        'mapper_params_online_async.yaml',
    )
    slam_toolbox_launch = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'launch',
        'online_async_launch.py',
    )
    arguments = [
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=slam_params_file,
            description='Path to the slam_toolbox online async mapper parameter file.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true.',
        ),
    ]

    slam = scoped_include(
        slam_toolbox_launch,
        launch_arguments={
            'slam_params_file': LaunchConfiguration('slam_params_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )

    return LaunchDescription([*arguments, slam])
