"""Independent v2 command-layer composition for controlled validation."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument("use_sim_time", default_value="false"),
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
            "frontier_goal_result_topic",
            default_value="/explore/frontier_goal_result",
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
            default_value="",
            description=(
                "Optional rosbag2 URI template. Empty selects a unique "
                "persistent /opt/muto_rs_ws/bags URI per process and mission."
            ),
        ),
        DeclareLaunchArgument("bag_run_id", default_value=""),
        DeclareLaunchArgument("bag_storage_id", default_value="mcap"),
    ]
    node = Node(
        package="muto_command_layer_v2",
        executable="v2_system_node.py",
        name="muto_command_layer_v2_system",
        output="screen",
        parameters=[{
            "use_sim_time": LaunchConfiguration("use_sim_time"),
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
        }],
    )
    recorder = Node(
        package="muto_command_layer_v2",
        executable="high_level_recorder_node.py",
        name="muto_command_layer_v2_recorder",
        output="screen",
        condition=IfCondition(LaunchConfiguration("record_bag")),
        parameters=[
            {
                "output_uri": LaunchConfiguration("bag_output_uri"),
                "run_id": LaunchConfiguration("bag_run_id"),
                "storage_id": LaunchConfiguration("bag_storage_id"),
                "commander_model": LaunchConfiguration("vlm_model"),
            }
        ],
    )
    return LaunchDescription(arguments + [node, recorder])
