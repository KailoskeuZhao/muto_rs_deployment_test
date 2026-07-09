import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    default_params_file = os.path.join(
        get_package_share_directory("muto_slam_mapping"),
        "config",
        "nav2_params.yaml",
    )
    nav2_planner_controller_launch = os.path.join(
        get_package_share_directory("muto_slam_mapping"),
        "launch",
        "nav2_planner_controller_launch.py",
    )

    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Top-level namespace.",
    )
    use_namespace_arg = DeclareLaunchArgument(
        "use_namespace",
        default_value="False",
        description="Whether to apply a namespace to the Nav2 planner/controller bringup.",
    )
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params_file,
        description="Path to the Nav2 parameter file for controller, planner, and their costmaps.",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation clock if true.",
    )
    autostart_arg = DeclareLaunchArgument(
        "autostart",
        default_value="True",
        description="Automatically configure and activate Nav2 lifecycle nodes.",
    )
    use_respawn_arg = DeclareLaunchArgument(
        "use_respawn",
        default_value="False",
        description="Whether to respawn crashed Nav2 nodes.",
    )
    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Log level for launched Nav2 nodes.",
    )

    return LaunchDescription([
        namespace_arg,
        use_namespace_arg,
        params_file_arg,
        use_sim_time_arg,
        autostart_arg,
        use_respawn_arg,
        log_level_arg,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_planner_controller_launch),
            launch_arguments={
                "namespace": LaunchConfiguration("namespace"),
                "use_namespace": LaunchConfiguration("use_namespace"),
                "params_file": LaunchConfiguration("params_file"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "autostart": LaunchConfiguration("autostart"),
                "use_respawn": LaunchConfiguration("use_respawn"),
                "log_level": LaunchConfiguration("log_level"),
            }.items(),
        ),
    ])
