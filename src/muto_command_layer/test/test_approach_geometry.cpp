#include <cmath>
#include <stdexcept>

#include "gtest/gtest.h"
#include "muto_command_layer/approach_geometry.hpp"

namespace
{

constexpr double kTolerance = 1.0e-9;

TEST(ApproachGeometry, ApproachesFromRobotFacingSide)
{
  const auto pose = muto_command_layer::compute_object_approach_pose(
    0.0, 0.0, 2.0, 0.0, 0.0, 0.75);

  EXPECT_NEAR(pose.x, 0.75, kTolerance);
  EXPECT_NEAR(pose.y, 0.0, kTolerance);
  EXPECT_NEAR(std::abs(pose.yaw), std::acos(-1.0), kTolerance);
}

TEST(ApproachGeometry, FacesObjectFromNorth)
{
  const auto pose = muto_command_layer::compute_object_approach_pose(
    1.0, 2.0, 1.0, 5.0, 0.0, 1.25);

  EXPECT_NEAR(pose.x, 1.0, kTolerance);
  EXPECT_NEAR(pose.y, 3.25, kTolerance);
  EXPECT_NEAR(pose.yaw, -std::acos(-1.0) / 2.0, kTolerance);
}

TEST(ApproachGeometry, HandlesCoincidentRobotAndObject)
{
  const auto pose = muto_command_layer::compute_object_approach_pose(
    4.0, -2.0, 4.0, -2.0, 0.0, 0.5);

  EXPECT_NEAR(pose.x, 3.5, kTolerance);
  EXPECT_NEAR(pose.y, -2.0, kTolerance);
  EXPECT_NEAR(pose.yaw, 0.0, kTolerance);
}

TEST(ApproachGeometry, RejectsInvalidDistance)
{
  EXPECT_THROW(
    muto_command_layer::compute_object_approach_pose(
      0.0, 0.0, 1.0, 0.0, 0.0, 0.0),
    std::invalid_argument);
}

TEST(ApproachGeometry, OutputRemainsAtRequestedRadiusAndFacesCentroid)
{
  constexpr double object_x = -1.2;
  constexpr double object_y = 3.7;
  constexpr double standoff = 0.9;
  const auto pose = muto_command_layer::compute_object_approach_pose(
    object_x, object_y, 2.4, -0.8, 0.4, standoff);

  EXPECT_NEAR(
    std::hypot(pose.x - object_x, pose.y - object_y),
    standoff, kTolerance);
  EXPECT_NEAR(
    pose.yaw, std::atan2(object_y - pose.y, object_x - pose.x),
    kTolerance);
}

}  // namespace
