from ament_index_python.packages import get_package_share_directory
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    ekf_config = os.path.join(
        get_package_share_directory("yahboomcar_bringup"),
        "config",
        "ekf_lidar_imu.yaml"
    )
    lidar_odometry_launch = os.path.join(
        get_package_share_directory("lidar_pointcloud_filter"),
        "launch",
        "filter_lidar_odometry_launch.py"
    )

    launch_lidar_odometry_arg = DeclareLaunchArgument(
        "launch_lidar_odometry",
        default_value="true",
        description="Whether to launch LiDAR filtering and odometry for /scan_odom.",
    )
    launch_foot_odometry_arg = DeclareLaunchArgument(
        "launch_foot_odometry",
        default_value="true",
        description="Whether to launch rough Muto gait/cmd_vel foot odometry for EKF input on /foot_odom.",
    )

    return LaunchDescription([
        launch_lidar_odometry_arg,
        launch_foot_odometry_arg,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidar_odometry_launch),
            condition=IfCondition(LaunchConfiguration("launch_lidar_odometry")),
        ),
        Node(
            package='yahboomcar_bringup',
            executable='foot_odometry_node',
            name='foot_odometry_node',
            output='screen',
            condition=IfCondition(LaunchConfiguration("launch_foot_odometry")),
            parameters=[{
                'odom_topic': '/foot_odom',
                'frame_id': 'odom',
                'child_frame_id': 'base_frame',
                'publish_tf': False,
            }],
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config],
        ),
    ])
