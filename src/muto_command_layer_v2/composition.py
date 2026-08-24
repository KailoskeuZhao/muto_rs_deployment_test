"""Independent ROS composition for the v2 executive node."""

from __future__ import annotations

from typing import Callable, Optional

from .backend_adapters import V2ToolBackend
from .commander import CommanderAgent
from .mission_executive_node import MissionExecutiveNode
from .model_transport import VlmCandidateInspector, VlmCommanderPlanner
from .ros_authorities import (
    Nav2MotionAuthority,
    RosFrontierAuthority,
    RosMapReachability,
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
    frontier_control_service: str = "/control_exploration",
    frontier_completion_topic: str = "/explore/exploration_complete",
    frontier_observe_duration_s: float = 20.0,
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

    The frontier explorer is adapted through its independent control service;
    it is not imported from or wrapped by the legacy command layer. A bounded
    observation cycle is stopped cooperatively and reports completion-event
    evidence when the explorer exhausts its current frontier set.
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
    frontier = RosFrontierAuthority(
        node,
        control_service=frontier_control_service,
        completion_topic=frontier_completion_topic,
        service_timeout_s=min(vlm_timeout_s, 5.0),
        observe_duration_s=frontier_observe_duration_s,
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
        observe_fn=observe_fn or frontier.observe,
        reachability_fn=reachability.evaluate_point,
        pose_fn=reachability.current_pose,
        reachability_revision_fn=reachability.revision,
        lifecycle_state_service=nav2_lifecycle_state_service,
        lifecycle_timeout_s=nav2_lifecycle_timeout_s,
    )
    node.configure_dependencies(
        planner,
        ToolDispatcher(V2ToolBackend(registry, motion)),
    )
    return node
