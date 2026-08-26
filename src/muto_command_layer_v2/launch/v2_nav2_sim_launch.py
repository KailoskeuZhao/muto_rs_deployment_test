"""Run the v2 executive against a reactive plant and real Nav2 in Humble.

The VLM, object registry, and frontier explorer remain external authorities;
this launch only supplies the independent v2 executive, the deterministic
environment fixture, and the existing Nav2 pipeline.  It never starts or
imports the legacy command layer.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    nav2_launch = os.path.join(
        get_package_share_directory("muto_slam_mapping"),
        "launch",
        "muto_nav2_pipeline_launch.py",
    )
    args = [
        DeclareLaunchArgument("mission_action", default_value="muto/mission"),
        DeclareLaunchArgument("vlm_action", default_value="/vlm/generate"),
        DeclareLaunchArgument("vlm_model", default_value=""),
        DeclareLaunchArgument("vlm_timeout_s", default_value="30.0"),
        DeclareLaunchArgument(
            "registry_query_service", default_value="/sam2/get_stored_objects"
        ),
        DeclareLaunchArgument(
            "frontier_control_service", default_value="/control_exploration"
        ),
        DeclareLaunchArgument(
            "frontier_goal_result_topic", default_value="/explore/frontier_goal_result"
        ),
        DeclareLaunchArgument("frontier_safety_watchdog_s", default_value="180.0"),
        DeclareLaunchArgument("navigate_action", default_value="/navigate_to_pose"),
        DeclareLaunchArgument("spin_action", default_value="/spin"),
        DeclareLaunchArgument("scenario_completion_policy", default_value="report_confirmed"),
        DeclareLaunchArgument("scenario_id", default_value=""),
        DeclareLaunchArgument("camera_topic", default_value="/camera/color/image_raw"),
        DeclareLaunchArgument("map_stale_after_s", default_value="2.0"),
        DeclareLaunchArgument("tf_stale_after_s", default_value="2.0"),
        DeclareLaunchArgument("record_bag", default_value="true"),
        DeclareLaunchArgument(
            "bag_output_uri",
            default_value="/tmp/muto_command_layer_v2_{run_id}_{mission_id}",
        ),
        DeclareLaunchArgument("bag_run_id", default_value=""),
        DeclareLaunchArgument("bag_storage_id", default_value="mcap"),
        DeclareLaunchArgument("start_x", default_value="0.0"),
        DeclareLaunchArgument("start_y", default_value="0.0"),
        DeclareLaunchArgument("start_yaw", default_value="0.0"),
        DeclareLaunchArgument("robot_radius_m", default_value="0.26"),
        DeclareLaunchArgument("footprint_radius_m", default_value="0.26"),
        DeclareLaunchArgument(
            "nav2_lifecycle_state_service", default_value="/bt_navigator/get_state"
        ),
        DeclareLaunchArgument("nav2_lifecycle_timeout_s", default_value="30.0"),
        DeclareLaunchArgument("obstacles_json", default_value="[]"),
        # The existing Nav2 pipeline declares these in its own launch file,
        # but its readiness timers are created before a scoped include can
        # resolve those declarations.  Keep the v2 harness explicit and pass
        # the values through so the launch is deterministic in Humble.
        DeclareLaunchArgument("localization_delay", default_value="3.0"),
        DeclareLaunchArgument("mapping_delay", default_value="8.0"),
        DeclareLaunchArgument("nav2_delay", default_value="0.0"),
        DeclareLaunchArgument("localization_readiness_timeout", default_value="30.0"),
        DeclareLaunchArgument("mapping_readiness_timeout", default_value="30.0"),
        DeclareLaunchArgument("nav2_readiness_timeout", default_value="30.0"),
    ]
    plant = Node(
        package="muto_command_layer_v2",
        executable="v2_sim_plant_node.py",
        name="muto_command_layer_v2_sim_plant",
        output="screen",
        parameters=[
            {
                "use_sim_time": False,
                "start_x": ParameterValue(LaunchConfiguration("start_x"), value_type=float),
                "start_y": ParameterValue(LaunchConfiguration("start_y"), value_type=float),
                "start_yaw": ParameterValue(LaunchConfiguration("start_yaw"), value_type=float),
                "robot_radius_m": ParameterValue(
                    LaunchConfiguration("robot_radius_m"), value_type=float
                ),
                "obstacles_json": ParameterValue(
                    LaunchConfiguration("obstacles_json"), value_type=str
                ),
            }
        ],
    )
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch),
        launch_arguments={
            "use_sim_time": "true",
            "launch_hardware": "false",
            "launch_localization": "false",
            "launch_mapping": "false",
            "launch_nav2": "true",
            "launch_nav2_bag": "false",
            "launch_sensor_tf": "false",
            "launch_camera_obstacle_scan": "false",
            "nav2_autostart": "true",
            "nav2_use_respawn": "false",
            "nav2_log_level": "warn",
            "localization_delay": LaunchConfiguration("localization_delay"),
            "mapping_delay": LaunchConfiguration("mapping_delay"),
            "nav2_delay": LaunchConfiguration("nav2_delay"),
            "localization_readiness_timeout": LaunchConfiguration(
                "localization_readiness_timeout"
            ),
            "mapping_readiness_timeout": LaunchConfiguration(
                "mapping_readiness_timeout"
            ),
            "nav2_readiness_timeout": LaunchConfiguration("nav2_readiness_timeout"),
        }.items(),
    )
    v2 = Node(
        package="muto_command_layer_v2",
        executable="v2_system_node.py",
        name="muto_command_layer_v2_system",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "mission_action": LaunchConfiguration("mission_action"),
                "vlm_action": LaunchConfiguration("vlm_action"),
                "vlm_model": LaunchConfiguration("vlm_model"),
                "vlm_timeout_s": LaunchConfiguration("vlm_timeout_s"),
                "registry_query_service": LaunchConfiguration("registry_query_service"),
                "frontier_control_service": LaunchConfiguration("frontier_control_service"),
                "frontier_goal_result_topic": LaunchConfiguration("frontier_goal_result_topic"),
                "frontier_safety_watchdog_s": LaunchConfiguration("frontier_safety_watchdog_s"),
                "navigate_action": LaunchConfiguration("navigate_action"),
                "spin_action": LaunchConfiguration("spin_action"),
                "scenario_completion_policy": LaunchConfiguration("scenario_completion_policy"),
                "scenario_id": LaunchConfiguration("scenario_id"),
                "camera_topic": LaunchConfiguration("camera_topic"),
                "map_stale_after_s": LaunchConfiguration("map_stale_after_s"),
                "tf_stale_after_s": LaunchConfiguration("tf_stale_after_s"),
                "footprint_radius_m": LaunchConfiguration("footprint_radius_m"),
                "nav2_lifecycle_state_service": LaunchConfiguration(
                    "nav2_lifecycle_state_service"
                ),
                "nav2_lifecycle_timeout_s": LaunchConfiguration(
                    "nav2_lifecycle_timeout_s"
                ),
            }
        ],
    )
    recorder = Node(
        package="muto_command_layer_v2",
        executable="high_level_recorder_node.py",
        name="muto_command_layer_v2_recorder",
        condition=IfCondition(LaunchConfiguration("record_bag")),
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "output_uri": LaunchConfiguration("bag_output_uri"),
                "run_id": LaunchConfiguration("bag_run_id"),
                "storage_id": LaunchConfiguration("bag_storage_id"),
                "commander_model": LaunchConfiguration("vlm_model"),
            }
        ],
    )
    return LaunchDescription(args + [plant, nav2, v2, recorder])
