#!/usr/bin/env python3
"""Launchable v2 composition for controlled Humble validation."""

import rclpy
from rclpy.executors import MultiThreadedExecutor

from muto_command_layer_v2.composition import create_v2_node


def main(args=None) -> None:
    rclpy.init(args=args)
    # Parameters are read before composition so launch can select only the
    # transport names/model; authority and lifecycle policy stay in v2 code.
    # Match the launch action's node name so ROS parameter overrides are
    # applied before the real executive node is constructed.
    parameter_node = rclpy.create_node("muto_command_layer_v2_system")
    parameter_node.declare_parameter("mission_action", "muto/mission")
    parameter_node.declare_parameter("use_sim_time", False)
    parameter_node.declare_parameter("vlm_action", "/vlm/generate")
    parameter_node.declare_parameter("vlm_model", "")
    parameter_node.declare_parameter("vlm_timeout_s", 30.0)
    parameter_node.declare_parameter("registry_query_service", "/sam2/get_stored_objects")
    parameter_node.declare_parameter("frontier_control_service", "/control_exploration")
    parameter_node.declare_parameter(
        "frontier_completion_topic", "/explore/exploration_complete"
    )
    parameter_node.declare_parameter("frontier_observe_duration_s", 20.0)
    parameter_node.declare_parameter("navigate_action", "/navigate_to_pose")
    parameter_node.declare_parameter("spin_action", "/spin")
    parameter_node.declare_parameter("scenario_completion_policy", "report_confirmed")
    parameter_node.declare_parameter("scenario_id", "")
    parameter_node.declare_parameter("camera_topic", "/camera/color/image_raw")
    parameter_node.declare_parameter("map_stale_after_s", 2.0)
    parameter_node.declare_parameter("tf_stale_after_s", 2.0)
    parameter_node.declare_parameter("footprint_radius_m", 0.26)
    parameter_node.declare_parameter("nav2_lifecycle_state_service", "/bt_navigator/get_state")
    parameter_node.declare_parameter("nav2_lifecycle_timeout_s", 30.0)
    values = {
        name: parameter_node.get_parameter(name).value
        for name in (
            "mission_action", "vlm_action", "vlm_model", "vlm_timeout_s",
            "registry_query_service", "frontier_control_service",
            "frontier_completion_topic", "frontier_observe_duration_s",
            "navigate_action", "spin_action",
            "scenario_completion_policy", "camera_topic", "map_stale_after_s",
            "tf_stale_after_s",
            "footprint_radius_m",
            "nav2_lifecycle_state_service",
            "nav2_lifecycle_timeout_s",
            "scenario_id",
            "use_sim_time",
        )
    }
    parameter_node.destroy_node()
    node = create_v2_node(
        action_name=values["mission_action"],
        vlm_action=values["vlm_action"],
        vlm_model=values["vlm_model"],
        vlm_timeout_s=float(values["vlm_timeout_s"]),
        registry_query_service=values["registry_query_service"],
        frontier_control_service=values["frontier_control_service"],
        frontier_completion_topic=values["frontier_completion_topic"],
        frontier_observe_duration_s=float(values["frontier_observe_duration_s"]),
        nav_action=values["navigate_action"],
        spin_action=values["spin_action"],
        use_sim_time=bool(values["use_sim_time"]),
        scenario_completion_policy=str(values["scenario_completion_policy"]),
        scenario_id=str(values["scenario_id"]),
        camera_topic=str(values["camera_topic"]),
        map_stale_after_s=float(values["map_stale_after_s"]),
        tf_stale_after_s=float(values["tf_stale_after_s"]),
        footprint_radius_m=float(values["footprint_radius_m"]),
        nav2_lifecycle_state_service=str(values["nav2_lifecycle_state_service"]),
        nav2_lifecycle_timeout_s=float(values["nav2_lifecycle_timeout_s"]),
    )
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
