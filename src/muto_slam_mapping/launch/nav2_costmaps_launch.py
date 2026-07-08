import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params_file = os.path.join(
        get_package_share_directory("muto_slam_mapping"),
        "config",
        "nav2_params.yaml",
    )

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params_file,
        description="Path to the Nav2 parameter file. Only the costmap sections are launched here.",
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
    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Log level for launched Nav2 nodes.",
    )

    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    log_level = LaunchConfiguration("log_level")
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    return LaunchDescription([
        params_file_arg,
        use_sim_time_arg,
        autostart_arg,
        log_level_arg,
        Node(
            package="nav2_costmap_2d",
            executable="nav2_costmap_2d",
            output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_time}],
            arguments=[
                "--ros-args",
                "-r",
                "__node:=global_costmap",
                "-r",
                "__ns:=/global_costmap",
                "--log-level",
                log_level,
            ],
            remappings=remappings,
        ),
        Node(
            package="nav2_costmap_2d",
            executable="nav2_costmap_2d",
            output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_time}],
            arguments=[
                "--ros-args",
                "-r",
                "__node:=local_costmap",
                "-r",
                "__ns:=/local_costmap",
                "--log-level",
                log_level,
            ],
            remappings=remappings,
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_costmaps",
            output="screen",
            parameters=[
                {"use_sim_time": use_sim_time},
                {"autostart": autostart},
                {
                    "node_names": [
                        "/global_costmap/global_costmap",
                        "/local_costmap/local_costmap",
                    ]
                },
            ],
            arguments=["--ros-args", "--log-level", log_level],
        ),
    ])
