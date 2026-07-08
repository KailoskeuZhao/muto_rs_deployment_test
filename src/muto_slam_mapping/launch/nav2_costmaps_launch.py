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
        "nav2_costmap_params.yaml",
    )
    nav2_bringup_launch = os.path.join(
        get_package_share_directory("nav2_bringup"),
        "launch",
        "bringup_launch.py",
    )

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params_file,
        description="Path to the Nav2 parameter file.",
    )
    map_arg = DeclareLaunchArgument(
        "map",
        default_value="",
        description="Optional map YAML file passed to nav2_bringup.",
    )
    use_localization_arg = DeclareLaunchArgument(
        "use_localization",
        default_value="false",
        description="Whether nav2_bringup should launch localization. False assumes SLAM/localization is already running.",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock if true.",
    )
    autostart_arg = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically configure and activate Nav2 lifecycle nodes.",
    )
    use_composition_arg = DeclareLaunchArgument(
        "use_composition",
        default_value="false",
        description="Whether nav2_bringup should use component composition.",
    )
    use_respawn_arg = DeclareLaunchArgument(
        "use_respawn",
        default_value="false",
        description="Whether nav2_bringup should respawn crashed nodes.",
    )
    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Log level for launched Nav2 nodes.",
    )

    params_file = LaunchConfiguration("params_file")
    map_yaml = LaunchConfiguration("map")
    use_localization = LaunchConfiguration("use_localization")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    use_composition = LaunchConfiguration("use_composition")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")

    return LaunchDescription([
        params_file_arg,
        map_arg,
        use_localization_arg,
        use_sim_time_arg,
        autostart_arg,
        use_composition_arg,
        use_respawn_arg,
        log_level_arg,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_bringup_launch),
            launch_arguments={
                "params_file": params_file,
                "map": map_yaml,
                "slam": "false",
                "use_localization": use_localization,
                "use_sim_time": use_sim_time,
                "autostart": autostart,
                "use_composition": use_composition,
                "use_respawn": use_respawn,
                "log_level": log_level,
            }.items(),
        ),
    ])
