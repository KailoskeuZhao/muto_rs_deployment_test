import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_params_file = os.path.join(
        get_package_share_directory("muto_slam_mapping"),
        "config",
        "nav2_costmap_params.yaml",
    )

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params_file,
        description="Path to the Nav2 costmap parameter file.",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock if true.",
    )
    autostart_arg = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically configure and activate the costmap lifecycle nodes.",
    )
    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Log level for launched Nav2 costmap nodes.",
    )

    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    log_level = LaunchConfiguration("log_level")

    costmap_common_parameters = [
        params_file,
        {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
    ]

    return LaunchDescription([
        params_file_arg,
        use_sim_time_arg,
        autostart_arg,
        log_level_arg,
        Node(
            package="nav2_costmap_2d",
            executable="nav2_costmap_2d",
            namespace="global_costmap",
            name="global_costmap",
            output="screen",
            parameters=costmap_common_parameters,
            arguments=["--ros-args", "--log-level", log_level],
        ),
        Node(
            package="nav2_costmap_2d",
            executable="nav2_costmap_2d",
            namespace="local_costmap",
            name="local_costmap",
            output="screen",
            parameters=costmap_common_parameters,
            arguments=["--ros-args", "--log-level", log_level],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_costmaps",
            output="screen",
            parameters=[{
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "autostart": ParameterValue(autostart, value_type=bool),
                "node_names": [
                    "/global_costmap/global_costmap",
                    "/local_costmap/local_costmap",
                ],
            }],
            arguments=["--ros-args", "--log-level", log_level],
        ),
    ])
