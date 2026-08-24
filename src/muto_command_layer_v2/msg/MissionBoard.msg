# Canonical current mission snapshot. Detailed history is MissionEvent.
uint8 IDLE=0
uint8 ACCEPTED=1
uint8 RUNNING=2
uint8 SUCCEEDED=3
uint8 CANCELED=4
uint8 FAILED=5

std_msgs/Header header
string schema_version
string mission_id
string request_id
uint8 lifecycle_state
string objective
string object_request
string completion_policy
string active_skill
string active_tool
uint64 board_revision

string registry_revision
string[] shortlisted_candidate_ids
string[] rejected_candidate_ids
string confirmed_target_id
string confirmed_registry_revision

geometry_msgs/PoseStamped robot_pose
string motion_state
float32 search_progress
float32 approach_progress
uint32 consecutive_failures
uint32 no_progress_count

string last_event_type
string last_outcome
string last_reason_code
ReachabilityReport reachability
bool recorder_available
string recorder_uri
string scenario_id
string coverage_summary
string visual_evidence_summary
string active_command_status
string[] active_limits
# Parallel arrays keyed by candidate id in the current evidence set.
string[] candidate_evidence_candidate_ids
string[] candidate_evidence_ids
string[] candidate_evidence_sources
float32[] candidate_evidence_confidences
string[] candidate_evidence_reason_codes
float64[] candidate_evidence_timestamps_s
