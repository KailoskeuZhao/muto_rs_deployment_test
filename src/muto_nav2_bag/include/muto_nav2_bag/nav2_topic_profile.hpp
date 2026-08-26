#ifndef MUTO_NAV2_BAG__NAV2_TOPIC_PROFILE_HPP_
#define MUTO_NAV2_BAG__NAV2_TOPIC_PROFILE_HPP_

#include <algorithm>
#include <string>
#include <vector>

namespace muto_nav2_bag
{

inline std::vector<std::string> default_nav2_topics()
{
  // This is the always-on, session-level navigation allowlist. Keep enough
  // evidence to reconstruct goals, plans, controller output, robot response,
  // obstacle observations and Nav2 state without duplicating full costmaps,
  // raw sensors, perception products or high-rate action feedback.
  return {
    // Recorder provenance and operator milestones.
    "/muto/nav2_bag/metadata",
    "/muto/nav2_bag/event",
    "/muto/nav2_bag/status",
    "/muto/nav2_bag/path",
    "/clock",

    // Frame tree, map and the pose estimates needed to compare requested and
    // realized motion. The full odometry recorder owns raw source capture.
    "/tf",
    "/tf_static",
    "/map",
    "/map_metadata",
    "/pose",
    "/odometry/filtered",
    "/scan_odom",
    "/muto/motion_command_state",

    // Compact obstacle evidence actually consumed by Nav2.
    "/lidar/filtered_laserscan",
    "/camera/filtered_laserscan",

    // Operator goals, generated plans, controller lookahead, and commands
    // before and after the velocity smoother.
    "/clicked_point",
    "/goal_pose",
    "/initialpose",
    "/plan",
    "/plan_smoothed",
    "/received_global_plan",
    "/lookahead_point",
    "/cmd_vel_nav",
    "/cmd_vel",

    // Selected exploration/object targets and their terminal state. Candidate
    // marker arrays and child-bag bookkeeping belong in the scoped mission bag.
    "/explore/selected_frontier",
    "/explore/frontier_goal_result",
    "/explore/visibility_target_pose",
    "/frontier_goal_adapter/original_goal",
    "/frontier_goal_adapter/projected_goal",
    "/frontier_goal_adapter/status",
    "/frontier/navigate_to_pose/_action/status",
    "/muto/mission_board",
    "/muto/mission_event",
    "/muto/mission_rejection",
    "/muto/mission_recorder_status",

    // Compact action state. Goal/result payloads are service events on Humble;
    // high-rate feedback is derivable from pose, plans and target mirrors.
    "/navigate_to_pose/_action/status",
    "/navigate_through_poses/_action/status",
    "/compute_path_to_pose/_action/status",
    "/compute_path_through_poses/_action/status",
    "/follow_path/_action/status",
    "/smooth_path/_action/status",
    "/spin/_action/status",
    "/backup/_action/status",
    "/wait/_action/status",

    // Behaviour-tree decisions, lifecycle changes, and compact diagnostics.
    "/behavior_tree_log",
    "/diagnostics",
    "/rosout",
    "/parameter_events",
    "/behavior_server/transition_event",
    "/bt_navigator/transition_event",
    "/controller_server/transition_event",
    "/planner_server/transition_event",
    "/smoother_server/transition_event",
    "/velocity_smoother/transition_event",
    "/global_costmap/global_costmap/transition_event",
    "/local_costmap/local_costmap/transition_event",
  };
}

inline void append_topic_if_missing(
  std::vector<std::string> & topics, const std::string & topic)
{
  if (std::find(topics.begin(), topics.end(), topic) == topics.end()) {
    topics.push_back(topic);
  }
}

}  // namespace muto_nav2_bag

#endif  // MUTO_NAV2_BAG__NAV2_TOPIC_PROFILE_HPP_
