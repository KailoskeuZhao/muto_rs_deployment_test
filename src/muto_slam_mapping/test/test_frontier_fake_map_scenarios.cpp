/*
Copyright 2026 Zhao Tianyi

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

#include <gtest/gtest.h>

#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "frontier_exploration_ros2/decision_map.hpp"
#include "frontier_exploration_ros2/frontier_explorer_core.hpp"
#include "frontier_exploration_ros2/frontier_policy.hpp"
#include "frontier_exploration_ros2/frontier_search.hpp"
#include "frontier_exploration_ros2/mrtsp_ordering.hpp"

namespace frontier_exploration_ros2
{
namespace
{

constexpr double kResolution = 0.04;

geometry_msgs::msg::Pose make_pose_for_cell(int x, int y, double yaw = 0.0)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = (static_cast<double>(x) + 0.5) * kResolution;
  pose.position.y = (static_cast<double>(y) + 0.5) * kResolution;
  pose.orientation.w = std::cos(yaw * 0.5);
  pose.orientation.z = std::sin(yaw * 0.5);
  return pose;
}

nav_msgs::msg::OccupancyGrid build_grid(int width, int height, int default_value)
{
  nav_msgs::msg::OccupancyGrid msg;
  msg.info.width = static_cast<uint32_t>(width);
  msg.info.height = static_cast<uint32_t>(height);
  msg.info.resolution = kResolution;
  msg.info.origin.orientation.w = 1.0;
  msg.data.assign(
    static_cast<std::size_t>(width * height),
    static_cast<int8_t>(default_value));
  return msg;
}

void set_cell(nav_msgs::msg::OccupancyGrid & msg, int x, int y, int value)
{
  ASSERT_GE(x, 0);
  ASSERT_GE(y, 0);
  ASSERT_LT(x, static_cast<int>(msg.info.width));
  ASSERT_LT(y, static_cast<int>(msg.info.height));
  msg.data[
    static_cast<std::size_t>(y) * static_cast<std::size_t>(msg.info.width) +
    static_cast<std::size_t>(x)] = static_cast<int8_t>(value);
}

void set_rect(
  nav_msgs::msg::OccupancyGrid & msg,
  int min_x,
  int min_y,
  int max_x,
  int max_y,
  int value)
{
  for (int y = min_y; y <= max_y; ++y) {
    for (int x = min_x; x <= max_x; ++x) {
      set_cell(msg, x, y, value);
    }
  }
}

nav_msgs::msg::OccupancyGrid costmap_from_known_obstacles(
  const nav_msgs::msg::OccupancyGrid & map_msg)
{
  auto costmap = build_grid(
    static_cast<int>(map_msg.info.width),
    static_cast<int>(map_msg.info.height),
    0);
  for (std::size_t index = 0; index < map_msg.data.size(); ++index) {
    if (map_msg.data[index] >= 65) {
      costmap.data[index] = 100;
    }
  }
  return costmap;
}

FrontierExplorerCoreParams muto_params()
{
  FrontierExplorerCoreParams params;
  params.frontier_map_optimization_enabled = true;
  params.sigma_s = 2.0;
  params.sigma_r = 30.0;
  params.dilation_kernel_radius_cells = 1;
  params.occ_threshold = 65;
  params.min_frontier_size_cells = 5;
  params.frontier_candidate_min_goal_distance_m = 0.50;
  params.frontier_selection_min_distance = 0.50;
  params.frontier_visit_tolerance = 0.40;
  params.mrtsp_solver = "dp";
  params.dp_solver_candidate_limit = 12;
  params.dp_planning_horizon = 8;
  params.weight_distance_wd = 1.0;
  params.weight_gain_ws = 1.0;
  params.sensor_effective_range_m = 1.5;
  params.max_linear_speed_vmax = 0.30;
  params.max_angular_speed_wmax = 0.50;
  return params;
}

struct ScenarioResult
{
  OccupancyGrid2d decision_map;
  FrontierSequence frontiers;
  FrontierSequence order;
};

ScenarioResult run_scenario(
  const nav_msgs::msg::OccupancyGrid & map_msg,
  const nav_msgs::msg::OccupancyGrid & costmap_msg,
  const geometry_msgs::msg::Pose & robot_pose,
  bool optimization_enabled = true)
{
  FrontierExplorerCoreParams params = muto_params();
  params.frontier_map_optimization_enabled = optimization_enabled;
  FrontierExplorerCore core(params, FrontierExplorerCoreCallbacks{});
  const OccupancyGrid2d raw_map(map_msg);
  const OccupancyGrid2d costmap(costmap_msg);
  const auto decision_result = build_decision_map(raw_map, core.decision_map_config());
  const auto search = get_frontier(
    robot_pose,
    decision_result.decision_map,
    costmap,
    std::nullopt,
    params.frontier_candidate_min_goal_distance_m,
    true,
    core.frontier_search_options());
  const FrontierSequence frontiers = FrontierExplorerCore::to_frontier_sequence(search.frontiers);
  return {
    decision_result.decision_map,
    frontiers,
    core.build_mrtsp_frontier_sequence(frontiers, robot_pose),
  };
}

double distance_to_robot(
  const std::pair<double, double> & point,
  const geometry_msgs::msg::Pose & robot_pose)
{
  return std::hypot(
    point.first - robot_pose.position.x,
    point.second - robot_pose.position.y);
}

void print_result(
  const std::string & name,
  const ScenarioResult & result,
  const geometry_msgs::msg::Pose & robot_pose)
{
  std::cout << "\n[fake-map] " << name << ": " << result.frontiers.size()
            << " frontier(s), DP order=" << result.order.size() << '\n';
  for (std::size_t index = 0; index < result.frontiers.size(); ++index) {
    const auto & frontier = result.frontiers[index];
    const auto goal = frontier_position(frontier);
    const auto order_it = std::find_if(
      result.order.begin(), result.order.end(),
      [&frontier](const FrontierCandidate & ordered) {
        return ordered.center_cell == frontier.center_cell;
      });
    const auto rank = order_it == result.order.end() ? -1 :
      static_cast<int>(std::distance(result.order.begin(), order_it));
    std::cout << std::fixed << std::setprecision(2)
              << "  candidate " << index
              << ": centroid=(" << frontier.centroid.first << ", " << frontier.centroid.second <<
      ')'
              << ", goal=(" << goal.first << ", " << goal.second << ')'
              << ", robot_distance=" << distance_to_robot(goal, robot_pose)
              << " m, cells=" << frontier.size
              << ", DP rank=" << rank << '\n';
  }
}

nav_msgs::msg::OccupancyGrid build_three_exit_map()
{
  auto map_msg = build_grid(120, 100, 100);

  // A known T corridor with three separated unknown continuations.
  set_rect(map_msg, 10, 45, 64, 54, 0);
  set_rect(map_msg, 55, 15, 64, 84, 0);
  set_rect(map_msg, 65, 45, 119, 54, -1);
  set_rect(map_msg, 55, 85, 64, 99, -1);
  set_rect(map_msg, 55, 0, 64, 14, -1);
  return map_msg;
}

TEST(FakeMapScenarioTests, OpenKnownPatchProducesOneReachableBoundaryGoal)
{
  auto map_msg = build_grid(100, 80, -1);
  set_rect(map_msg, 25, 20, 74, 59, 0);
  const auto costmap_msg = costmap_from_known_obstacles(map_msg);
  const auto robot_pose = make_pose_for_cell(50, 40);

  const auto result = run_scenario(map_msg, costmap_msg, robot_pose);
  print_result("open known patch", result, robot_pose);

  ASSERT_EQ(result.frontiers.size(), 1U);
  ASSERT_EQ(result.order.size(), 1U);
  ASSERT_TRUE(result.order.front().goal_point.has_value());
  const auto goal = frontier_position(result.order.front());
  EXPECT_GE(distance_to_robot(goal, robot_pose), 0.50 - kResolution);
  const auto [goal_x, goal_y] = result.decision_map.worldToMap(goal.first, goal.second);
  EXPECT_EQ(result.decision_map.getCost(goal_x, goal_y), 0);
}

TEST(FakeMapScenarioTests, DecisionMapOptimizationCanMoveGoalIntoRawUnknown)
{
  auto map_msg = build_grid(100, 80, -1);
  set_rect(map_msg, 25, 20, 74, 59, 0);
  const auto costmap_msg = costmap_from_known_obstacles(map_msg);
  const auto robot_pose = make_pose_for_cell(50, 40);
  const OccupancyGrid2d raw_map(map_msg);

  const auto optimized = run_scenario(
    map_msg, costmap_msg, robot_pose, true);
  ASSERT_EQ(optimized.frontiers.size(), 1U);
  const auto optimized_goal = frontier_position(optimized.frontiers.front());
  const auto [optimized_x, optimized_y] = raw_map.worldToMap(
    optimized_goal.first, optimized_goal.second);
  EXPECT_EQ(
    raw_map.getCost(optimized_x, optimized_y),
    static_cast<int>(OccupancyGrid2d::CostValues::NoInformation));

  const auto raw = run_scenario(map_msg, costmap_msg, robot_pose, false);
  ASSERT_EQ(raw.frontiers.size(), 1U);
  const auto raw_goal = frontier_position(raw.frontiers.front());
  const auto [raw_x, raw_y] = raw_map.worldToMap(raw_goal.first, raw_goal.second);
  EXPECT_EQ(
    raw_map.getCost(raw_x, raw_y),
    static_cast<int>(OccupancyGrid2d::CostValues::FreeSpace));

  bool touches_unknown = false;
  for (int dy = -1; dy <= 1; ++dy) {
    for (int dx = -1; dx <= 1; ++dx) {
      if (dx == 0 && dy == 0) {
        continue;
      }
      if (
        raw_map.getCost(raw_x + dx, raw_y + dy) ==
        static_cast<int>(OccupancyGrid2d::CostValues::NoInformation))
      {
        touches_unknown = true;
      }
    }
  }
  EXPECT_TRUE(touches_unknown);
}

TEST(FakeMapScenarioTests, ThreeExitCorridorFindsAllSeparatedOpenings)
{
  const auto map_msg = build_three_exit_map();
  const auto costmap_msg = costmap_from_known_obstacles(map_msg);
  const auto robot_pose = make_pose_for_cell(20, 49, 0.0);

  const auto result = run_scenario(map_msg, costmap_msg, robot_pose);
  print_result("three-exit corridor", result, robot_pose);

  ASSERT_EQ(result.frontiers.size(), 3U);
  ASSERT_EQ(result.order.size(), 3U);
  for (const auto & frontier : result.frontiers) {
    ASSERT_TRUE(frontier.goal_point.has_value());
    const auto goal = frontier_position(frontier);
    const auto [goal_x, goal_y] = result.decision_map.worldToMap(goal.first, goal.second);
    EXPECT_EQ(result.decision_map.getCost(goal_x, goal_y), 0);
    EXPECT_GE(distance_to_robot(goal, robot_pose), 0.50 - kResolution);
  }
}

TEST(FakeMapScenarioTests, UnreachableFreeIslandDoesNotCreateCandidate)
{
  auto map_msg = build_grid(120, 80, 100);

  // Reachable corridor and its unknown continuation.
  set_rect(map_msg, 10, 30, 55, 39, 0);
  set_rect(map_msg, 56, 30, 70, 39, -1);

  // A disconnected free island also touches unknown, but occupied cells separate it.
  set_rect(map_msg, 85, 50, 100, 59, 0);
  set_rect(map_msg, 101, 50, 115, 59, -1);

  const auto costmap_msg = costmap_from_known_obstacles(map_msg);
  const auto robot_pose = make_pose_for_cell(20, 34);
  const auto result = run_scenario(map_msg, costmap_msg, robot_pose);
  print_result("unreachable island", result, robot_pose);

  ASSERT_EQ(result.frontiers.size(), 1U);
  EXPECT_LT(result.frontiers.front().centroid.first, 3.0);
}

TEST(FakeMapScenarioTests, GlobalCostmapBlockRemovesOnlyThatOpening)
{
  const auto map_msg = build_three_exit_map();
  auto costmap_msg = costmap_from_known_obstacles(map_msg);

  // Close the east mouth in the navigation costmap while leaving north/south open.
  set_rect(costmap_msg, 63, 44, 67, 55, 100);
  const auto robot_pose = make_pose_for_cell(20, 49, 0.0);
  const auto result = run_scenario(map_msg, costmap_msg, robot_pose);
  print_result("east opening cost-blocked", result, robot_pose);

  ASSERT_EQ(result.frontiers.size(), 2U);
  ASSERT_EQ(result.order.size(), 2U);
  for (const auto & frontier : result.frontiers) {
    EXPECT_LT(frontier.centroid.first, 3.0);
  }
}

TEST(FakeMapScenarioTests, ReplanningDropsRevealedExitAndChoosesRemainingFrontier)
{
  auto map_msg = build_three_exit_map();
  auto costmap_msg = costmap_from_known_obstacles(map_msg);
  const auto start_pose = make_pose_for_cell(20, 49, 0.0);

  const auto initial = run_scenario(map_msg, costmap_msg, start_pose);
  ASSERT_EQ(initial.order.size(), 3U);
  print_result("replan step 0", initial, start_pose);

  // Simulate SLAM revealing the east continuation after the first goal.
  set_rect(map_msg, 65, 45, 119, 54, 0);
  costmap_msg = costmap_from_known_obstacles(map_msg);
  const auto east_pose = make_pose_for_cell(66, 49, 0.0);
  const auto after_east = run_scenario(map_msg, costmap_msg, east_pose);
  print_result("replan after east reveal", after_east, east_pose);

  ASSERT_EQ(after_east.frontiers.size(), 2U);
  ASSERT_EQ(after_east.order.size(), 2U);
  for (const auto & frontier : after_east.frontiers) {
    EXPECT_LT(frontier.centroid.first, 3.0);
  }

  // Reveal the south continuation; north should be the only remaining goal.
  set_rect(map_msg, 55, 0, 64, 14, 0);
  costmap_msg = costmap_from_known_obstacles(map_msg);
  const auto south_pose = make_pose_for_cell(59, 13, -1.5707963267948966);
  const auto after_south = run_scenario(map_msg, costmap_msg, south_pose);
  print_result("replan after east and south reveal", after_south, south_pose);

  ASSERT_EQ(after_south.frontiers.size(), 1U);
  ASSERT_EQ(after_south.order.size(), 1U);
  EXPECT_GT(after_south.frontiers.front().centroid.second, 3.0);
}

TEST(FakeMapScenarioTests, DoorwayWidthSweepShowsPracticalFrontierCutoff)
{
  std::vector<int> accepted_widths;
  for (int width_cells = 3; width_cells <= 12; ++width_cells) {
    auto map_msg = build_grid(100, 80, 100);
    const int min_y = 40 - (width_cells / 2);
    const int max_y = min_y + width_cells - 1;
    set_rect(map_msg, 10, min_y, 60, max_y, 0);
    set_rect(map_msg, 61, min_y, 99, max_y, -1);
    const auto costmap_msg = costmap_from_known_obstacles(map_msg);
    const auto robot_pose = make_pose_for_cell(20, min_y + (width_cells / 2), 0.0);
    const auto result = run_scenario(map_msg, costmap_msg, robot_pose);
    const bool accepted = !result.frontiers.empty();
    if (accepted) {
      accepted_widths.push_back(width_cells);
    }
    std::cout << "\n[fake-map] doorway width=" << width_cells
              << " cells (" << std::fixed << std::setprecision(2)
              << static_cast<double>(width_cells) * kResolution << " m): "
              << (accepted ? "accepted" : "ignored");
    if (accepted) {
      std::cout << ", extracted frontier cells=" << result.frontiers.front().size;
    }
    std::cout << '\n';
  }

  ASSERT_FALSE(accepted_widths.empty());
  EXPECT_EQ(accepted_widths.front(), 7);
  EXPECT_EQ(accepted_widths.back(), 12);
}

TEST(FakeMapScenarioTests, FullyKnownMapCompletesWithoutGoal)
{
  auto map_msg = build_grid(80, 60, 100);
  set_rect(map_msg, 10, 10, 69, 49, 0);
  const auto costmap_msg = costmap_from_known_obstacles(map_msg);
  const auto robot_pose = make_pose_for_cell(40, 30);
  const auto result = run_scenario(map_msg, costmap_msg, robot_pose);
  print_result("fully known room", result, robot_pose);

  EXPECT_TRUE(result.frontiers.empty());
  EXPECT_TRUE(result.order.empty());
}

}  // namespace
}  // namespace frontier_exploration_ros2
