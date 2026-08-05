#include <array>
#include <cmath>
#include <limits>
#include <string>

#include "gtest/gtest.h"
#include "lidar_pointcloud_filter/odometry_covariance_profiles.hpp"

namespace
{

constexpr double kPi = 3.14159265358979323846;

lidar_pointcloud_filter::OdometryCovarianceProfile customProfile()
{
  return {
    "custom",
    0.01,
    0.02,
    0.03,
    0.04,
    0.05,
    0.06,
    0.7,
    0.8,
  };
}

TEST(OdometryCovarianceProfiles, MeasuredProfileUsesBagEstimate)
{
  const auto profile =
    lidar_pointcloud_filter::measuredCovarianceProfile();

  EXPECT_EQ(profile.name, "measured");
  EXPECT_DOUBLE_EQ(profile.pose_x_variance, 2.5e-4);
  EXPECT_DOUBLE_EQ(profile.pose_y_variance, 2.5e-4);
  EXPECT_DOUBLE_EQ(profile.pose_yaw_variance, 1.0e-4);
  EXPECT_DOUBLE_EQ(profile.twist_x_variance, 2.5e-4);
  EXPECT_DOUBLE_EQ(profile.twist_y_variance, 1.0);
  EXPECT_DOUBLE_EQ(profile.twist_yaw_variance, 1.1e-4);
}

TEST(OdometryCovarianceProfiles, NamedProfilesResolve)
{
  auto custom = customProfile();
  lidar_pointcloud_filter::OdometryCovarianceProfile resolved;

  ASSERT_TRUE(lidar_pointcloud_filter::resolveCovarianceProfile(
      "relaxed", custom, resolved));
  EXPECT_DOUBLE_EQ(resolved.pose_x_variance, 1.0e-3);
  EXPECT_DOUBLE_EQ(resolved.pose_yaw_variance, 4.0e-4);

  ASSERT_TRUE(lidar_pointcloud_filter::resolveCovarianceProfile(
      "conservative", custom, resolved));
  EXPECT_DOUBLE_EQ(resolved.pose_x_variance, 2.5e-3);
  EXPECT_NEAR(resolved.pose_yaw_variance, std::pow(kPi / 60.0, 2), 1e-12);

  ASSERT_TRUE(lidar_pointcloud_filter::resolveCovarianceProfile(
      "legacy_zero", custom, resolved));
  EXPECT_DOUBLE_EQ(resolved.pose_x_variance, 0.0);
  EXPECT_DOUBLE_EQ(resolved.pose_yaw_variance, 0.0);
  EXPECT_DOUBLE_EQ(resolved.unobserved_pose_variance, 0.0);
}

TEST(OdometryCovarianceProfiles, CustomProfileMustBeFiniteAndNonnegative)
{
  auto custom = customProfile();
  lidar_pointcloud_filter::OdometryCovarianceProfile resolved;

  ASSERT_TRUE(lidar_pointcloud_filter::resolveCovarianceProfile(
      "custom", custom, resolved));
  EXPECT_DOUBLE_EQ(resolved.pose_x_variance, 0.01);

  custom.pose_x_variance = -1.0;
  EXPECT_FALSE(lidar_pointcloud_filter::resolveCovarianceProfile(
      "custom", custom, resolved));

  custom = customProfile();
  custom.pose_x_variance = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(lidar_pointcloud_filter::resolveCovarianceProfile(
      "custom", custom, resolved));
  EXPECT_FALSE(lidar_pointcloud_filter::resolveCovarianceProfile(
      "missing", custom, resolved));
}

TEST(OdometryCovarianceProfiles, CovarianceArraysUseRosPlanarIndices)
{
  const auto profile = customProfile();
  std::array<double, 36> pose;
  std::array<double, 36> twist;

  lidar_pointcloud_filter::setPoseCovariance(pose, profile);
  lidar_pointcloud_filter::setTwistCovariance(twist, profile);

  EXPECT_DOUBLE_EQ(pose[0], 0.01);
  EXPECT_DOUBLE_EQ(pose[7], 0.02);
  EXPECT_DOUBLE_EQ(pose[14], 0.7);
  EXPECT_DOUBLE_EQ(pose[21], 0.7);
  EXPECT_DOUBLE_EQ(pose[28], 0.7);
  EXPECT_DOUBLE_EQ(pose[35], 0.03);

  EXPECT_DOUBLE_EQ(twist[0], 0.04);
  EXPECT_DOUBLE_EQ(twist[7], 0.05);
  EXPECT_DOUBLE_EQ(twist[14], 0.8);
  EXPECT_DOUBLE_EQ(twist[21], 0.8);
  EXPECT_DOUBLE_EQ(twist[28], 0.8);
  EXPECT_DOUBLE_EQ(twist[35], 0.06);
}

}  // namespace
