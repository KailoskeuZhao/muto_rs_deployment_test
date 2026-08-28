#!/usr/bin/env python3
"""Launchable v2 composition for controlled Humble validation."""

import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor

from muto_command_layer_v2.composition import create_v2_node


def main(args=None) -> None:
    rclpy.init(args=args)
    # Parameters are read before composition so launch can select only the
    # transport names/model; authority and lifecycle policy stay in v2 code.
    # Match the launch action's node name so ROS parameter overrides are
    # applied before the real executive node is constructed.
    parameter_node = rclpy.create_node("muto_command_layer_v2_system")
    parameter_node.declare_parameter("mission_action", "muto/mission")
    # rclpy's TimeSource declares ``use_sim_time`` while constructing every
    # node.  Launch already applies the override before this point, so
    # declaring it again raises ParameterAlreadyDeclaredException on Humble.
    parameter_node.declare_parameter("vlm_action", "/vlm/generate")
    parameter_node.declare_parameter("vlm_model", "")
    parameter_node.declare_parameter("vlm_timeout_s", 30.0)
    parameter_node.declare_parameter("registry_query_service", "/sam2/get_stored_objects")
    parameter_node.declare_parameter(
        "poi_grid_spacing_m", 1.0
    )
    parameter_node.declare_parameter("poi_grid_result_topic", "/muto/poi_grid/result")
    parameter_node.declare_parameter(
        "poi_grid_selected_pose_topic", "/muto/poi_grid/selected_pose"
    )
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
            "registry_query_service", "poi_grid_spacing_m",
            "poi_grid_result_topic", "poi_grid_selected_pose_topic",
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
        poi_grid_spacing_m=float(values["poi_grid_spacing_m"]),
        poi_grid_result_topic=values["poi_grid_result_topic"],
        poi_grid_selected_pose_topic=values["poi_grid_selected_pose_topic"],
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
    except (KeyboardInterrupt, ExternalShutdownException):
        # ROS 2 may deliver SIGINT either as an executor exception or as a
        # context shutdown while launch is tearing the graph down.  Both are
        # normal lifecycle exits, not node failures.
        pass
    finally:
        # Keep teardown best-effort: a launch supervisor can invalidate the
        # context before it reaches this finally block.  In that case calling
        # into an already-invalid subscription/guard condition only creates a
        # second traceback and turns a clean stop into a reported process
        # failure.
        try:
            executor.shutdown(timeout_sec=2.0)
        except (Exception, KeyboardInterrupt):
            pass
        try:
            executor.remove_node(node)
        except (Exception, KeyboardInterrupt):
            pass
        try:
            node.destroy_node()
        except (Exception, KeyboardInterrupt):
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except (Exception, KeyboardInterrupt):
            pass


if __name__ == "__main__":
    main()
