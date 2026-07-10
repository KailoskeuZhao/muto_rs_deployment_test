import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
try:
    from launch_ros.actions import PushROSNamespace
except ImportError:
    from launch_ros.actions import PushRosNamespace as PushROSNamespace
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import ReplaceString, RewrittenYaml


def generate_launch_description():
    default_params_file = os.path.join(
        get_package_share_directory("muto_slam_mapping"),
        "config",
        "nav2_params.yaml",
    )

    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Top-level namespace.",
    )
    use_namespace_arg = DeclareLaunchArgument(
        "use_namespace",
        default_value="False",
        description="Whether to apply a namespace to the Nav2 action/planner/controller/smoother bringup.",
    )
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params_file,
        description="Path to the Nav2 parameter file for BT navigator, controller, planner, smoother, and costmaps.",
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

    namespace = LaunchConfiguration("namespace")
    use_namespace = LaunchConfiguration("use_namespace")
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]
    lifecycle_nodes = [
        "controller_server",
        "planner_server",
        "smoother_server",
        "behavior_server",
        "bt_navigator",
    ]

    params_file = ReplaceString(
        source_file=params_file,
        replacements={"<robot_namespace>": ("/", namespace)},
        condition=IfCondition(use_namespace),
    )

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={"autostart": autostart},
            convert_types=True,
        ),
        allow_substs=True,
    )

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )

    start_nav2_servers = GroupAction([
        PushROSNamespace(condition=IfCondition(use_namespace), namespace=namespace),
        SetParameter("use_sim_time", use_sim_time),
        Node(
            package="nav2_controller",
            executable="controller_server",
            output="screen",
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=["--ros-args", "--log-level", log_level],
            remappings=remappings + [("cmd_vel", "cmd_vel_nav")],
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=["--ros-args", "--log-level", log_level],
            remappings=remappings,
        ),
        Node(
            package="nav2_smoother",
            executable="smoother_server",
            name="smoother_server",
            output="screen",
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=["--ros-args", "--log-level", log_level],
            remappings=remappings,
        ),
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=["--ros-args", "--log-level", log_level],
            remappings=remappings,
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=["--ros-args", "--log-level", log_level],
            remappings=remappings,
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_costmaps",
            output="screen",
            parameters=[
                {"autostart": autostart},
                {"node_names": lifecycle_nodes},
            ],
            arguments=["--ros-args", "--log-level", log_level],
        ),
    ])

    return LaunchDescription([
        stdout_linebuf_envvar,
        namespace_arg,
        use_namespace_arg,
        params_file_arg,
        use_sim_time_arg,
        autostart_arg,
        use_respawn_arg,
        log_level_arg,
        start_nav2_servers,
    ])
