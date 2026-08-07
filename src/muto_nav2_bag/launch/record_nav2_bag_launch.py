import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bag_share = get_package_share_directory('muto_nav2_bag')
    mapping_share = get_package_share_directory('muto_slam_mapping')
    bringup_share = get_package_share_directory('yahboomcar_bringup')

    default_params = os.path.join(bag_share, 'config', 'nav2_bag.yaml')
    default_nav2_params = os.path.join(
        mapping_share, 'config', 'nav2_params.yaml')
    default_frontier_params = os.path.join(
        mapping_share, 'config', 'frontier_exploration_params.yaml')
    default_slam_params = os.path.join(
        mapping_share, 'config', 'mapper_params_online_async.yaml')
    default_nav_to_pose_bt = os.path.join(
        mapping_share, 'behavior_trees', 'muto_nav_to_pose.xml')
    default_nav_through_poses_bt = os.path.join(
        mapping_share, 'behavior_trees', 'muto_nav_through_poses.xml')
    default_locomotion_calibration = os.path.join(
        bringup_share,
        'config',
        'muto_locomotion_provisional_20260806.yaml',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Nav2-bag recorder parameter file.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time while recording.',
        ),
        DeclareLaunchArgument(
            'output_directory',
            default_value='/opt/muto_rs_ws/bags',
            description='Parent directory for standalone Nav2 bags.',
        ),
        DeclareLaunchArgument(
            'bag_name',
            default_value='',
            description='Exact bag directory name; empty generates one.',
        ),
        DeclareLaunchArgument('storage_id', default_value='mcap'),
        DeclareLaunchArgument('storage_preset', default_value='zstd_fast'),
        DeclareLaunchArgument('max_cache_size', default_value='52428800'),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=default_nav2_params,
            description='Active Nav2 parameters copied into the bag.',
        ),
        DeclareLaunchArgument(
            'frontier_params_file',
            default_value=default_frontier_params,
            description='Active frontier parameters copied into the bag.',
        ),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=default_slam_params,
            description='Active SLAM parameters copied into the bag.',
        ),
        DeclareLaunchArgument(
            'nav_to_pose_bt_file',
            default_value=default_nav_to_pose_bt,
            description='NavigateToPose behavior tree copied into the bag.',
        ),
        DeclareLaunchArgument(
            'nav_through_poses_bt_file',
            default_value=default_nav_through_poses_bt,
            description=(
                'NavigateThroughPoses behavior tree copied into the bag.'
            ),
        ),
        DeclareLaunchArgument(
            'locomotion_calibration_file',
            default_value=default_locomotion_calibration,
            description='Active Muto velocity profile copied into the bag.',
        ),
        Node(
            package='muto_nav2_bag',
            executable='nav2_bag_recorder',
            name='nav2_bag_recorder',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {
                    'use_sim_time': ParameterValue(
                        LaunchConfiguration('use_sim_time'),
                        value_type=bool,
                    ),
                    'output_directory': ParameterValue(
                        LaunchConfiguration('output_directory'),
                        value_type=str,
                    ),
                    'bag_name': ParameterValue(
                        LaunchConfiguration('bag_name'), value_type=str),
                    'storage_id': ParameterValue(
                        LaunchConfiguration('storage_id'), value_type=str),
                    'storage_preset': ParameterValue(
                        LaunchConfiguration('storage_preset'), value_type=str),
                    'max_cache_size': ParameterValue(
                        LaunchConfiguration('max_cache_size'), value_type=int),
                    'nav2_params_file': ParameterValue(
                        LaunchConfiguration('nav2_params_file'),
                        value_type=str,
                    ),
                    'frontier_params_file': ParameterValue(
                        LaunchConfiguration('frontier_params_file'),
                        value_type=str,
                    ),
                    'slam_params_file': ParameterValue(
                        LaunchConfiguration('slam_params_file'),
                        value_type=str,
                    ),
                    'nav_to_pose_bt_file': ParameterValue(
                        LaunchConfiguration('nav_to_pose_bt_file'),
                        value_type=str,
                    ),
                    'nav_through_poses_bt_file': ParameterValue(
                        LaunchConfiguration('nav_through_poses_bt_file'),
                        value_type=str,
                    ),
                    'locomotion_calibration_file': ParameterValue(
                        LaunchConfiguration('locomotion_calibration_file'),
                        value_type=str,
                    ),
                },
            ],
        ),
    ])
