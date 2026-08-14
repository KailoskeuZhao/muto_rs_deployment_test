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

#include <geometry_msgs/msg/point.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/footprint_collision_checker.hpp>
#include <nav2_navfn_planner/navfn.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <utility>
#include <vector>

namespace
{

using nav2_costmap_2d::FREE_SPACE;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

constexpr int kWidth = 90;
constexpr int kHeight = 60;
constexpr double kResolution = 0.04;
constexpr double kRobotRadius = 0.26;

struct Grid
{
  std::vector<unsigned char> costs = std::vector<unsigned char>(
    static_cast<std::size_t>(kWidth * kHeight), LETHAL_OBSTACLE);

  unsigned char & at(int x, int y)
  {
    return costs[static_cast<std::size_t>(y * kWidth + x)];
  }

  unsigned char at(int x, int y) const
  {
    return costs[static_cast<std::size_t>(y * kWidth + x)];
  }

  void fill_rect(int min_x, int min_y, int max_x, int max_y, unsigned char value)
  {
    for (int y = min_y; y <= max_y; ++y) {
      for (int x = min_x; x <= max_x; ++x) {
        at(x, y) = value;
      }
    }
  }
};

struct NavFnResult
{
  bool propagated{false};
  int path_length{0};
  int unknown_path_samples{0};
};

template<typename PlannerT>
auto calculate_dijkstra(PlannerT & planner, int)
-> decltype(
  planner.calcNavFnDijkstra(std::function<bool()> {}, true),
  bool{})
{
  return planner.calcNavFnDijkstra([]() {return false;}, true);
}

template<typename PlannerT>
auto calculate_dijkstra(PlannerT & planner, long)
-> decltype(planner.calcNavFnDijkstra(true), bool{})
{
  return planner.calcNavFnDijkstra(true);
}

NavFnResult run_navfn(
  const Grid & grid,
  const std::pair<int, int> & robot,
  const std::pair<int, int> & goal,
  bool allow_unknown)
{
  nav2_navfn_planner::NavFn planner(kWidth, kHeight);
  planner.setCostmap(grid.costs.data(), true, allow_unknown);

  // NavFn computes a potential from the robot cell and extracts the path from
  // the requested endpoint, matching NavfnPlanner's intentionally reversed
  // internal start/goal convention.
  int navfn_start[2] = {goal.first, goal.second};
  int navfn_goal[2] = {robot.first, robot.second};
  planner.setStart(navfn_start);
  planner.setGoal(navfn_goal);

  NavFnResult result;
  result.propagated = calculate_dijkstra(planner, 0);
  if (!result.propagated) {
    return result;
  }
  result.path_length = planner.calcPath(kWidth * kHeight);
  for (int index = 0; index < result.path_length; ++index) {
    const int x = std::clamp(
      static_cast<int>(std::floor(planner.getPathX()[index])), 0, kWidth - 1);
    const int y = std::clamp(
      static_cast<int>(std::floor(planner.getPathY()[index])), 0, kHeight - 1);
    if (grid.at(x, y) == NO_INFORMATION) {
      ++result.unknown_path_samples;
    }
  }
  return result;
}

nav2_costmap_2d::Footprint circular_footprint(double radius)
{
  nav2_costmap_2d::Footprint footprint;
  constexpr int kSamples = 32;
  footprint.reserve(kSamples);
  for (int index = 0; index < kSamples; ++index) {
    const double angle =
      2.0 * M_PI * static_cast<double>(index) / static_cast<double>(kSamples);
    geometry_msgs::msg::Point point;
    point.x = radius * std::cos(angle);
    point.y = radius * std::sin(angle);
    footprint.push_back(point);
  }
  return footprint;
}

double cell_center(int cell)
{
  return (static_cast<double>(cell) + 0.5) * kResolution;
}

TEST(FrontierNavFnPolicy, KnownFreeRouteWorksWithUnknownTraversalDisabled)
{
  Grid grid;
  grid.fill_rect(5, 20, 80, 39, FREE_SPACE);

  const auto permissive = run_navfn(grid, {10, 30}, {75, 30}, true);
  const auto known_only = run_navfn(grid, {10, 30}, {75, 30}, false);

  ASSERT_GT(permissive.path_length, 0);
  ASSERT_GT(known_only.path_length, 0);
  EXPECT_EQ(permissive.unknown_path_samples, 0);
  EXPECT_EQ(known_only.unknown_path_samples, 0);
}

TEST(FrontierNavFnPolicy, UnknownGapIsTraversableOnlyWhenExplicitlyAllowed)
{
  Grid grid;
  grid.fill_rect(5, 20, 35, 39, FREE_SPACE);
  grid.fill_rect(36, 20, 44, 39, NO_INFORMATION);
  grid.fill_rect(45, 20, 80, 39, FREE_SPACE);

  const auto permissive = run_navfn(grid, {10, 30}, {75, 30}, true);
  const auto known_only = run_navfn(grid, {10, 30}, {75, 30}, false);

  ASSERT_GT(permissive.path_length, 0);
  EXPECT_GT(permissive.unknown_path_samples, 0);
  EXPECT_EQ(known_only.path_length, 0);
}

TEST(FrontierNavFnPolicy, FreeBoundaryGoalStillOverlapsUnknownWithRobotFootprint)
{
  nav2_costmap_2d::Costmap2D costmap(
    kWidth, kHeight, kResolution, 0.0, 0.0, NO_INFORMATION);
  for (int y = 10; y <= 49; ++y) {
    for (int x = 10; x <= 60; ++x) {
      costmap.setCost(x, y, FREE_SPACE);
    }
  }

  nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> checker(
    &costmap);
  const auto footprint = circular_footprint(kRobotRadius);

  // This is the goal style produced by the frontier selector: a free cell
  // immediately adjacent to an unknown frontier cell.
  const double boundary_score = checker.footprintCostAtPose(
    cell_center(60), cell_center(30), 0.0, footprint);

  // Seven 4 cm cells place the same center more than 0.26 m inside known free
  // space, allowing the complete footprint to remain known-free.
  const double standoff_score = checker.footprintCostAtPose(
    cell_center(53), cell_center(30), 0.0, footprint);

  EXPECT_EQ(boundary_score, static_cast<double>(NO_INFORMATION));
  EXPECT_LT(standoff_score, static_cast<double>(LETHAL_OBSTACLE));
}

}  // namespace
