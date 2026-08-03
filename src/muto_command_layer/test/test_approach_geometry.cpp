#include <cmath>
#include <stdexcept>

#include "gtest/gtest.h"
#include "muto_command_layer/approach_geometry.hpp"

namespace
{

constexpr double kTolerance = 1.0e-9;

muto_command_layer::ApproachGrid free_grid(
  const int width = 12,
  const int height = 12)
{
  muto_command_layer::ApproachGrid grid;
  grid.width = width;
  grid.height = height;
  grid.resolution = 1.0;
  grid.costs.assign(
    static_cast<std::size_t>(width * height), static_cast<uint8_t>(0));
  return grid;
}

muto_command_layer::ApproachPlannerConfig config()
{
  muto_command_layer::ApproachPlannerConfig value;
  value.maximum_traversable_cost = 252;
  value.robot_radius = 0.4;
  value.minimum_standoff = 2.0;
  value.start_snap_distance = 1.1;
  return value;
}

void set_cost(
  muto_command_layer::ApproachGrid & grid,
  const int x,
  const int y,
  const uint8_t value = 254)
{
  grid.costs[
    static_cast<std::size_t>(y * grid.width + x)] = value;
}

void expect_faces_object(
  const muto_command_layer::PlanarPose & pose,
  const double object_x,
  const double object_y)
{
  EXPECT_NEAR(
    pose.yaw, std::atan2(object_y - pose.y, object_x - pose.x),
    kTolerance);
}

TEST(ApproachGeometry, ChoosesShortestReachableCellOnMinimumRing)
{
  const auto pose = muto_command_layer::compute_object_approach_pose(
    free_grid(), 5.5, 5.5, 10.5, 5.5, config());

  EXPECT_NEAR(pose.x, 7.5, kTolerance);
  EXPECT_NEAR(pose.y, 5.5, kTolerance);
  EXPECT_NEAR(std::hypot(pose.x - 5.5, pose.y - 5.5), 2.0, kTolerance);
  expect_faces_object(pose, 5.5, 5.5);
}

TEST(ApproachGeometry, SearchesAroundBlockedRobotFacingSide)
{
  auto grid = free_grid();
  set_cost(grid, 7, 5);

  const auto pose = muto_command_layer::compute_object_approach_pose(
    grid, 5.5, 5.5, 10.5, 5.5, config());

  EXPECT_FALSE(
    std::abs(pose.x - 7.5) < kTolerance &&
    std::abs(pose.y - 5.5) < kTolerance);
  const double radius = std::hypot(pose.x - 5.5, pose.y - 5.5);
  EXPECT_GE(radius, 2.0 - kTolerance);
  EXPECT_LT(radius, 3.0 + kTolerance);
  expect_faces_object(pose, 5.5, 5.5);
}

TEST(ApproachGeometry, KeepsTargetOnRobotReachableSideOfCostmapWall)
{
  auto grid = free_grid();
  for (int y = 0; y < grid.height; ++y) {
    set_cost(grid, 7, y);
  }

  const auto pose = muto_command_layer::compute_object_approach_pose(
    grid, 3.5, 5.5, 10.5, 5.5, config());

  EXPECT_GT(pose.x, 7.5);
  expect_faces_object(pose, 3.5, 5.5);
}

TEST(ApproachGeometry, BlocksAllNav2NonTraversableCostValues)
{
  for (const uint8_t blocked_cost : {uint8_t{253}, uint8_t{254}, uint8_t{255}}) {
    SCOPED_TRACE(static_cast<int>(blocked_cost));
    auto grid = free_grid();
    for (int y = 0; y < grid.height; ++y) {
      set_cost(grid, 7, y, blocked_cost);
    }

    const auto pose = muto_command_layer::compute_object_approach_pose(
      grid, 3.5, 5.5, 10.5, 5.5, config());
    EXPECT_GT(pose.x, 7.5);
  }
}

TEST(ApproachGeometry, AllowsNav2MaximumNonObstacleCost)
{
  auto grid = free_grid();
  set_cost(grid, 7, 5, 252);

  const auto pose = muto_command_layer::compute_object_approach_pose(
    grid, 5.5, 5.5, 10.5, 5.5, config());

  EXPECT_NEAR(pose.x, 7.5, kTolerance);
  EXPECT_NEAR(pose.y, 5.5, kTolerance);
}

TEST(ApproachGeometry, SnapsRobotFromLethalCostCell)
{
  auto grid = free_grid();
  set_cost(grid, 10, 5);

  EXPECT_NO_THROW({
      const auto pose = muto_command_layer::compute_object_approach_pose(
      grid, 5.5, 5.5, 10.5, 5.5, config());
      expect_faces_object(pose, 5.5, 5.5);
  });
}

TEST(ApproachGeometry, MinimumRadiusCannotBeSmallerThanRobot)
{
  auto planner_config = config();
  planner_config.robot_radius = 1.1;
  planner_config.minimum_standoff = 0.1;
  const auto pose = muto_command_layer::compute_object_approach_pose(
    free_grid(), 5.5, 5.5, 10.5, 5.5, planner_config);

  EXPECT_GE(
    std::hypot(pose.x - 5.5, pose.y - 5.5),
    planner_config.robot_radius - kTolerance);
  expect_faces_object(pose, 5.5, 5.5);
}

TEST(ApproachGeometry, SupportsRotatedCostmapOrigin)
{
  auto grid = free_grid();
  grid.origin_yaw = std::acos(-1.0) / 2.0;
  const auto pose = muto_command_layer::compute_object_approach_pose(
    grid, -5.5, 5.5, -5.5, 10.5, config());

  EXPECT_NEAR(pose.x, -5.5, kTolerance);
  EXPECT_NEAR(pose.y, 7.5, kTolerance);
  EXPECT_NEAR(pose.yaw, -std::acos(-1.0) / 2.0, kTolerance);
}

TEST(ApproachGeometry, AllowsLethalObjectCentroidCell)
{
  auto grid = free_grid();
  set_cost(grid, 5, 5);
  const auto pose = muto_command_layer::compute_object_approach_pose(
    grid, 5.5, 5.5, 10.5, 5.5, config());

  EXPECT_GE(std::hypot(pose.x - 5.5, pose.y - 5.5), 2.0 - kTolerance);
  expect_faces_object(pose, 5.5, 5.5);
}

TEST(ApproachGeometry, RejectsInvalidDistanceConfiguration)
{
  auto invalid = config();
  invalid.robot_radius = 0.0;

  EXPECT_THROW(
    muto_command_layer::compute_object_approach_pose(
      free_grid(), 5.5, 5.5, 10.5, 5.5, invalid),
    std::invalid_argument);
}

TEST(ApproachGeometry, RejectsInscribedCostAsTraversableThreshold)
{
  auto invalid = config();
  invalid.maximum_traversable_cost = 253;

  EXPECT_THROW(
    muto_command_layer::compute_object_approach_pose(
      free_grid(), 5.5, 5.5, 10.5, 5.5, invalid),
    std::invalid_argument);
}

}  // namespace
