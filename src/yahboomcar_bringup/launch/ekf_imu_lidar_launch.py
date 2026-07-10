from ament_index_python.packages import get_package_share_directory
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    ekf_config = os.path.join(
        get_package_share_directory("yahboomcar_bringup"),
        "config",
        "ekf_lidar_imu.yaml"
    )
    ekf_imu_only_config = os.path.join(
        get_package_share_directory("yahboomcar_bringup"),
        "config",
        "ekf_imu_only.yaml"
    )
    ekf_with_foot_config = os.path.join(
        get_package_share_directory("yahboomcar_bringup"),
        "config",
        "ekf_lidar_imu_with_foot.yaml"
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
    rf2o_translation_deadband_arg = DeclareLaunchArgument(
        "rf2o_translation_deadband",
        default_value="0.001",
        description=(
            "Per-update RF2O planar translation deadband in meters. "
            "Set 0.0 to disable."
        ),
    )
    rf2o_yaw_deadband_arg = DeclareLaunchArgument(
        "rf2o_yaw_deadband",
        default_value="0.001",
        description=(
            "Per-update RF2O yaw deadband in radians. "
            "Set 0.0 to disable."
        ),
    )
    launch_foot_odometry_arg = DeclareLaunchArgument(
        "launch_foot_odometry",
        default_value="false",
        description="Whether to launch rough Muto gait/cmd_vel foot odometry and fuse /foot_odom into the EKF.",
    )
    imu_only_arg = DeclareLaunchArgument(
        "imu_only",
        default_value="false",
        description="Run an IMU-only EKF test using /imu/data_processed and no LiDAR or foot odometry.",
    )

    use_lidar_odometry = PythonExpression([
        "'", LaunchConfiguration("launch_lidar_odometry"), "' == 'true' and '",
        LaunchConfiguration("imu_only"), "' == 'false'",
    ])
    use_foot_odometry = PythonExpression([
        "'", LaunchConfiguration("launch_foot_odometry"), "' == 'true' and '",
        LaunchConfiguration("imu_only"), "' == 'false'",
    ])
    use_lidar_imu_ekf = PythonExpression([
        "'", LaunchConfiguration("launch_foot_odometry"), "' == 'false' and '",
        LaunchConfiguration("imu_only"), "' == 'false'",
    ])
    use_foot_ekf = PythonExpression([
        "'", LaunchConfiguration("launch_foot_odometry"), "' == 'true' and '",
        LaunchConfiguration("imu_only"), "' == 'false'",
    ])

    return LaunchDescription([
        launch_lidar_odometry_arg,
        rf2o_translation_deadband_arg,
        rf2o_yaw_deadband_arg,
        launch_foot_odometry_arg,
        imu_only_arg,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidar_odometry_launch),
            condition=IfCondition(use_lidar_odometry),
            launch_arguments={
                "rf2o_publish_tf": "false",
                "rf2o_translation_deadband": LaunchConfiguration("rf2o_translation_deadband"),
                "rf2o_yaw_deadband": LaunchConfiguration("rf2o_yaw_deadband"),
            }.items(),
        ),
        Node(
            package='yahboomcar_bringup',
            executable='foot_odometry_node',
            name='foot_odometry_node',
            output='screen',
            condition=IfCondition(use_foot_odometry),
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
            condition=IfCondition(use_lidar_imu_ekf),
            parameters=[ekf_config],
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            condition=IfCondition(use_foot_ekf),
            parameters=[ekf_with_foot_config],
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            condition=IfCondition(LaunchConfiguration("imu_only")),
            parameters=[ekf_imu_only_config],
        ),
    ])
