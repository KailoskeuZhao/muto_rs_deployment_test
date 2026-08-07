#ifndef MUTO_NAV2_BAG__NAV2_TOPIC_PROFILE_HPP_
#define MUTO_NAV2_BAG__NAV2_TOPIC_PROFILE_HPP_

#include <algorithm>
#include <string>
#include <vector>

namespace muto_nav2_bag
{

inline std::vector<std::string> default_nav2_topics()
{
  // This is deliberately an allowlist, derived from the deployed Muto graph.
  // Keep image, depth, point-cloud, detector, and /bond traffic out of this
  // profile: those belong in perception or full exploration recordings.
  return {
    // Recorder provenance and operator milestones.
    "/muto/nav2_bag/metadata",
    "/muto/nav2_bag/event",
    "/muto/nav2_bag/status",
    "/muto/nav2_bag/path",
    "/clock",

    // Frame tree, map, localization, and odometry sources used by Nav2.
    "/tf",
    "/tf_static",
    "/map",
    "/map_metadata",
    "/pose",
    "/odometry/filtered",
    "/scan_odom",
    "/scan_odom_raw",
    "/foot_odom",
    "/imu/data_processed",
    "/muto/commanded_gait_state",
    "/muto/motion_command_state",

    // Navigation observations (already reduced to LaserScan).
    "/lidar/filtered_laserscan",
    "/lidar/filtered_laserscan_no_downsample",
    "/camera/filtered_laserscan",

    // Global and local costmaps and the robot footprint they used.
    "/global_costmap/costmap",
    "/global_costmap/costmap_raw",
    "/global_costmap/costmap_updates",
    "/global_costmap/published_footprint",
    "/local_costmap/costmap",
    "/local_costmap/costmap_raw",
    "/local_costmap/costmap_updates",
    "/local_costmap/published_footprint",

    // Operator goals, generated plans, controller lookahead, and commands.
    "/clicked_point",
    "/goal_pose",
    "/initialpose",
    "/plan",
    "/plan_smoothed",
    "/received_global_plan",
    "/lookahead_point",
    "/lookahead_collision_arc",
    "/cmd_vel_nav",
    "/cmd_vel",

    // Exploration candidates and the target-pose mirrors used by this stack.
    "/explore/frontiers",
    "/explore/selected_frontier",
    "/explore/optimized_map",
    "/explore/exploration_complete",
    "/explore/visibility_target_pose",
    "/object_navigation/target_pose",
    "/explore_and_record/recording_event",
    "/explore_and_record/operator_event",
    "/explore_and_record/bag_status",
    "/explore_and_record/last_bag_path",
    "/explore_and_record/_action/feedback",
    "/explore_and_record/_action/status",
    "/go_to_object/_action/feedback",
    "/go_to_object/_action/status",

    // Nav2 action progress/status.  Goal and result payloads are service
    // events, not topics, on Humble; the target mirrors above retain the
    // mission-level destination selected by this workspace.
    "/navigate_to_pose/_action/feedback",
    "/navigate_to_pose/_action/status",
    "/navigate_through_poses/_action/feedback",
    "/navigate_through_poses/_action/status",
    "/compute_path_to_pose/_action/feedback",
    "/compute_path_to_pose/_action/status",
    "/compute_path_through_poses/_action/feedback",
    "/compute_path_through_poses/_action/status",
    "/follow_path/_action/feedback",
    "/follow_path/_action/status",
    "/smooth_path/_action/feedback",
    "/smooth_path/_action/status",
    "/spin/_action/feedback",
    "/spin/_action/status",
    "/backup/_action/feedback",
    "/backup/_action/status",
    "/wait/_action/feedback",
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
