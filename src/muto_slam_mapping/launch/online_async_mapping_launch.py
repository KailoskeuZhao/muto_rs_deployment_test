import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    slam_params_file = os.path.join(
        get_package_share_directory("muto_slam_mapping"),
        "config",
        "mapper_params_online_async.yaml",
    )
    slam_toolbox_launch = os.path.join(
        get_package_share_directory("slam_toolbox"),
        "launch",
        "online_async_launch.py",
    )
    fused_laserscan_launch = os.path.join(
        get_package_share_directory("lidar_pointcloud_filter"),
        "launch",
        "camera_pointcloud_to_laserscan_launch.py",
    )

    slam_params_file_arg = DeclareLaunchArgument(
        "slam_params_file",
        default_value=slam_params_file,
        description="Path to the slam_toolbox online async mapper parameter file.",
    )
    launch_fused_laserscan_arg = DeclareLaunchArgument(
        "launch_fused_laserscan",
        default_value="true",
        description=(
            "Whether to launch camera/LiDAR PointCloud2 to fused LaserScan conversion. "
            "This assumes the configured LiDAR cloud topic already exists."
        ),
    )

    return LaunchDescription([
        slam_params_file_arg,
        launch_fused_laserscan_arg,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(fused_laserscan_launch),
            condition=IfCondition(LaunchConfiguration("launch_fused_laserscan")),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_toolbox_launch),
            launch_arguments={
                "slam_params_file": LaunchConfiguration("slam_params_file"),
            }.items(),
        ),
    ])
