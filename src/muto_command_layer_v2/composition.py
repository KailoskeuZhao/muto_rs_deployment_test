"""Independent ROS composition for the v2 executive node."""

from __future__ import annotations

from typing import Callable, Optional

from .backend_adapters import V2ToolBackend
from .commander import CommanderAgent
from .mission_executive_node import MissionExecutiveNode
from .model_transport import VlmCandidateInspector, VlmCommanderPlanner
from .ros_authorities import (
    Nav2MotionAuthority,
    RosMapReachability,
    RosPoiGridAuthority,
    RosRegistryAuthority,
    RosVisualInput,
)
from .tools import ToolDispatcher


def create_v2_node(
    *,
    action_name: str = "muto/mission",
    vlm_action: str = "/vlm/generate",
    vlm_model: str = "",
    vlm_timeout_s: float = 30.0,
    registry_query_service: str = "/sam2/get_stored_objects",
    poi_grid_spacing_m: float = 1.0,
    poi_grid_result_topic: str = "/muto/poi_grid/result",
    poi_grid_selected_pose_topic: str = "/muto/poi_grid/selected_pose",
    nav_action: str = "/navigate_to_pose",
    spin_action: str = "/spin",
    use_sim_time: bool = False,
    observe_fn: Optional[Callable] = None,
    scenario_completion_policy: Optional[str] = None,
    scenario_id: str = "",
    camera_topic: str = "/camera/color/image_raw",
    map_stale_after_s: float = 2.0,
    tf_stale_after_s: Optional[float] = None,
    footprint_radius_m: float = 0.26,
    nav2_lifecycle_state_service: Optional[str] = None,
    nav2_lifecycle_timeout_s: float = 30.0,
) -> MissionExecutiveNode:
    """Construct the production-shaped v2 graph without legacy imports.

    The deterministic POI-grid authority selects one reachable known-free
    viewpoint per observation and hands it to Nav2. Each observation waits for
    a typed POI result before Commander regains control.
    """

    node = MissionExecutiveNode(
        action_name=action_name,
        scenario_completion_policy=scenario_completion_policy,
        scenario_id=scenario_id,
    )
    # The launchable wrapper reads parameters before constructing the real
    # composed node.  Apply the clock mode to that node explicitly; otherwise
    # a simulation plant's /clock is ignored by the executive's TF/pose
    # authority even when the launch sets use_sim_time=true.
    if use_sim_time:
        from rclpy.parameter import Parameter

        node.set_parameters(
            [Parameter("use_sim_time", Parameter.Type.BOOL, value=True)]
        )
    visual_input = RosVisualInput(node, topic=camera_topic)
    planner = CommanderAgent(
        VlmCommanderPlanner(
            node,
            action_name=vlm_action,
            model=vlm_model,
            timeout_s=vlm_timeout_s,
            jpeg_supplier=visual_input.jpeg,
        )
    )
    inspector = VlmCandidateInspector(
        node,
        action_name=vlm_action,
        model=vlm_model,
        timeout_s=vlm_timeout_s,
    )
    registry = RosRegistryAuthority(
        node,
        query_service=registry_query_service,
        timeout_s=vlm_timeout_s,
        visual_selector=inspector,
    )
    reachability = RosMapReachability(
        node,
        stale_after_s=map_stale_after_s,
        tf_stale_after_s=tf_stale_after_s,
        footprint_radius_m=footprint_radius_m,
    )
    node.configure_pose_supplier(reachability.current_pose)
    motion = Nav2MotionAuthority(
        node,
        navigate_action=nav_action,
        spin_action=spin_action,
        timeout_s=5.0,
        motion_timeout_s=120.0,
        observe_fn=observe_fn,
        reachability_fn=reachability.evaluate_point,
        pose_fn=reachability.current_pose,
        reachability_revision_fn=reachability.revision,
        lifecycle_state_service=nav2_lifecycle_state_service,
        lifecycle_timeout_s=nav2_lifecycle_timeout_s,
    )
    poi_grid = RosPoiGridAuthority(
        node,
        reachability=reachability,
        motion=motion,
        spacing_m=poi_grid_spacing_m,
        result_topic=poi_grid_result_topic,
        selected_pose_topic=poi_grid_selected_pose_topic,
    )
    if observe_fn is None:
        motion.set_observe_authority(poi_grid.observe)
    node.configure_dependencies(
        planner,
        ToolDispatcher(V2ToolBackend(registry, motion)),
    )
    return node
