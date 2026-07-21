import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def launch_file(package_name, launch_name):
    return os.path.join(
        get_package_share_directory(package_name),
        "launch",
        launch_name,
    )


def delayed_include(delay_arg, enabled_arg, package_name, launch_name, launch_arguments=None):
    include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file(package_name, launch_name)),
        launch_arguments=(launch_arguments or {}).items(),
    )
    return TimerAction(
        period=LaunchConfiguration(delay_arg),
        actions=[include],
        condition=IfCondition(LaunchConfiguration(enabled_arg)),
    )


def generate_launch_description():
    default_slam_params_file = os.path.join(
        get_package_share_directory("muto_slam_mapping"),
        "config",
        "mapper_params_online_async.yaml",
    )
    default_nav2_params_file = os.path.join(
        get_package_share_directory("muto_slam_mapping"),
        "config",
        "nav2_params.yaml",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation clock if true.",
        ),
        DeclareLaunchArgument(
            "launch_hardware",
            default_value="true",
            description="Start TG30 LiDAR, Orbbec camera, and Muto base driver.",
        ),
        DeclareLaunchArgument(
            "launch_sensor_tf",
            default_value="true",
            description="Start static sensor TF publishers.",
        ),
        DeclareLaunchArgument(
            "launch_localization",
            default_value="true",
            description="Start LiDAR filtering, RF2O odometry, and EKF localization.",
        ),
        DeclareLaunchArgument(
            "launch_mapping",
            default_value="true",
            description="Start fused LaserScan generation and SLAM Toolbox mapping.",
        ),
        DeclareLaunchArgument(
            "launch_nav2",
            default_value="true",
            description="Start Nav2 planner, controller, smoother, behavior, and BT navigator.",
        ),
        DeclareLaunchArgument(
            "sensor_tf_delay",
            default_value="1.0",
            description="Seconds after launch before starting static sensor TF publishers.",
        ),
        DeclareLaunchArgument(
            "localization_delay",
            default_value="3.0",
            description="Seconds after launch before starting LiDAR odometry and EKF.",
        ),
        DeclareLaunchArgument(
            "mapping_delay",
            default_value="8.0",
            description="Seconds after launch before starting mapping and fused scan generation.",
        ),
        DeclareLaunchArgument(
            "nav2_delay",
            default_value="12.0",
            description="Seconds after launch before starting Nav2.",
        ),
        DeclareLaunchArgument(
            "launch_fused_laserscan",
            default_value="true",
            description="Let the mapping launch start /fused/laserscan generation.",
        ),
        DeclareLaunchArgument(
            "slam_params_file",
            default_value=default_slam_params_file,
            description="Path to the SLAM Toolbox online async mapper parameter file.",
        ),
        DeclareLaunchArgument(
            "nav2_params_file",
            default_value=default_nav2_params_file,
            description="Path to the Nav2 parameter file.",
        ),
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="Top-level Nav2 namespace.",
        ),
        DeclareLaunchArgument(
            "use_namespace",
            default_value="False",
            description="Whether to apply a namespace to Nav2.",
        ),
        DeclareLaunchArgument(
            "nav2_autostart",
            default_value="True",
            description="Automatically configure and activate Nav2 lifecycle nodes.",
        ),
        DeclareLaunchArgument(
            "nav2_use_respawn",
            default_value="False",
            description="Whether to respawn crashed Nav2 nodes.",
        ),
        DeclareLaunchArgument(
            "nav2_log_level",
            default_value="info",
            description="Log level for Nav2 nodes.",
        ),
        delayed_include(
            "sensor_tf_delay",
            "launch_sensor_tf",
            "tf2_publisher",
            "all_tf2_publishers_launch.py",
            {
                "publish_odom_tf": "false",
            },
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                launch_file("yahboomcar_bringup", "muto_hardware_launch.py")
            ),
            condition=IfCondition(LaunchConfiguration("launch_hardware")),
        ),
        delayed_include(
            "localization_delay",
            "launch_localization",
            "yahboomcar_bringup",
            "ekf_imu_lidar_launch.py",
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "launch_lidar_odometry": "true",
            },
        ),
        delayed_include(
            "mapping_delay",
            "launch_mapping",
            "muto_slam_mapping",
            "online_async_mapping_launch.py",
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "launch_fused_laserscan": LaunchConfiguration("launch_fused_laserscan"),
                "slam_params_file": LaunchConfiguration("slam_params_file"),
            },
        ),
        delayed_include(
            "nav2_delay",
            "launch_nav2",
            "muto_slam_mapping",
            "nav2_planner_controller_launch.py",
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "params_file": LaunchConfiguration("nav2_params_file"),
                "namespace": LaunchConfiguration("namespace"),
                "use_namespace": LaunchConfiguration("use_namespace"),
                "autostart": LaunchConfiguration("nav2_autostart"),
                "use_respawn": LaunchConfiguration("nav2_use_respawn"),
                "log_level": LaunchConfiguration("nav2_log_level"),
            },
        ),
    ])
