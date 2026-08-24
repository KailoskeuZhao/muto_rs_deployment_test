"""ROS projections of the canonical v2 board and event contracts."""

from math import cos, sin
from geometry_msgs.msg import PoseStamped
from muto_command_layer_v2.msg import MissionBoard as MissionBoardMsg
from muto_command_layer_v2.msg import MissionEvent as MissionEventMsg

from .contracts import LifecycleState, MissionBoard, MissionEvent, ReachabilityState


_LIFECYCLE = {
    LifecycleState.IDLE: MissionBoardMsg.IDLE,
    LifecycleState.ACCEPTED: MissionBoardMsg.ACCEPTED,
    LifecycleState.RUNNING: MissionBoardMsg.RUNNING,
    LifecycleState.SUCCEEDED: MissionBoardMsg.SUCCEEDED,
    LifecycleState.CANCELED: MissionBoardMsg.CANCELED,
    LifecycleState.FAILED: MissionBoardMsg.FAILED,
}


_REACHABILITY = {
    ReachabilityState.UNKNOWN: 0,
    ReachabilityState.REACHABLE: 1,
    ReachabilityState.UNREACHABLE: 2,
}


def board_to_msg(board: MissionBoard, *, stamp=None) -> MissionBoardMsg:
    msg = MissionBoardMsg()
    if stamp is not None:
        msg.header.stamp = stamp
    msg.schema_version = board.schema_version
    msg.mission_id = board.mission_id
    msg.request_id = board.request_id
    msg.lifecycle_state = _LIFECYCLE[board.lifecycle_state]
    msg.objective = board.objective
    msg.object_request = board.object_request
    msg.completion_policy = board.completion_policy.value if board.completion_policy else ""
    msg.active_skill = board.active_skill.value if board.active_skill else ""
    msg.active_tool = board.active_tool.value if board.active_tool else ""
    msg.board_revision = board.board_revision
    msg.registry_revision = board.registry_revision
    msg.shortlisted_candidate_ids = list(board.shortlisted_candidate_ids)
    msg.rejected_candidate_ids = list(board.rejected_candidate_ids)
    msg.confirmed_target_id = board.confirmed_target_id
    msg.confirmed_registry_revision = board.confirmed_registry_revision
    if board.robot_pose is not None:
        msg.robot_pose = _pose_from_xyyaw(board.robot_pose)
    msg.motion_state = board.motion_state
    msg.search_progress = board.search_progress
    msg.approach_progress = board.approach_progress
    msg.consecutive_failures = board.consecutive_failures
    msg.no_progress_count = board.no_progress_count
    msg.last_event_type = board.last_event_type
    msg.last_outcome = board.last_outcome
    msg.last_reason_code = board.last_reason_code
    msg.recorder_available = board.recorder_available
    msg.recorder_uri = board.recorder_uri
    msg.scenario_id = board.scenario_id
    msg.coverage_summary = board.coverage_summary
    msg.visual_evidence_summary = board.visual_evidence_summary
    msg.active_command_status = board.active_command_status
    msg.active_limits = list(board.active_limits)
    msg.candidate_evidence_candidate_ids = [
        item.candidate_id for item in board.candidate_evidence
    ]
    msg.candidate_evidence_ids = [item.evidence_id for item in board.candidate_evidence]
    msg.candidate_evidence_sources = [item.source for item in board.candidate_evidence]
    msg.candidate_evidence_confidences = [
        item.confidence if item.confidence is not None else -1.0
        for item in board.candidate_evidence
    ]
    msg.candidate_evidence_reason_codes = [
        item.reason_code for item in board.candidate_evidence
    ]
    msg.candidate_evidence_timestamps_s = [
        item.observed_at_s if item.observed_at_s is not None else 0.0
        for item in board.candidate_evidence
    ]
    msg.reachability.state = _REACHABILITY[board.reachability.state]
    msg.reachability.reason_code = board.reachability.reason_code
    msg.reachability.path_length_m = (
        board.reachability.path_length_m
        if board.reachability.path_length_m is not None
        else 0.0
    )
    msg.reachability.estimated_time_s = (
        board.reachability.estimated_time_s
        if board.reachability.estimated_time_s is not None
        else 0.0
    )
    msg.reachability.costmap_revision = board.reachability.costmap_revision or 0
    msg.reachability.freshness = board.reachability.freshness
    msg.reachability.projected = board.reachability.projected
    if board.reachability.selected_pose is not None:
        msg.reachability.selected_pose = _pose_from_xyyaw(board.reachability.selected_pose)
    return msg


def event_to_msg(event: MissionEvent, *, stamp=None) -> MissionEventMsg:
    msg = MissionEventMsg()
    if stamp is not None:
        msg.stamp = stamp
    msg.sequence = event.sequence
    msg.schema_version = "muto_command_layer_v2"
    msg.mission_id = event.mission_id
    msg.request_id = event.request_id
    msg.board_revision = event.board_revision
    msg.event_type = event.event_type.value
    msg.lifecycle_state = event.lifecycle_state.value
    msg.skill = event.skill.value if event.skill else ""
    msg.tool = event.tool.value if event.tool else ""
    msg.outcome = event.outcome
    msg.reason_code = event.reason_code
    msg.candidate_id = event.candidate_id
    msg.registry_revision = event.registry_revision
    msg.detail = event.detail
    msg.evidence_id = event.evidence_id
    msg.evidence_source = event.evidence_source
    msg.evidence_confidence = (
        event.evidence_confidence if event.evidence_confidence is not None else -1.0
    )
    msg.evidence_timestamp_s = event.evidence_timestamp_s or 0.0
    return msg


def _pose_from_xyyaw(pose):
    msg = PoseStamped()
    msg.header.frame_id = "map"
    msg.pose.position.x = pose[0]
    msg.pose.position.y = pose[1]
    msg.pose.orientation.z = sin(pose[2] / 2.0)
    msg.pose.orientation.w = cos(pose[2] / 2.0)
    return msg
