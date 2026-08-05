#include "lidar_pointcloud_filter/odometry_covariance_profiles.hpp"

#include <cmath>

namespace lidar_pointcloud_filter
{

namespace
{

OdometryCovarianceProfile relaxedCovarianceProfile()
{
  return {
    "relaxed",
    1.0e-3,
    1.0e-3,
    4.0e-4,
    1.0e-3,
    1.0,
    4.4e-4,
    1.0,
    1.0,
  };
}

OdometryCovarianceProfile conservativeCovarianceProfile()
{
  return {
    "conservative",
    2.5e-3,
    2.5e-3,
    2.741556778e-3,
    2.5e-3,
    1.0,
    2.741556778e-3,
    1.0,
    1.0,
  };
}

OdometryCovarianceProfile legacyZeroCovarianceProfile()
{
  return {
    "legacy_zero",
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
  };
}

bool isFiniteAndNonnegative(double value)
{
  return std::isfinite(value) && value >= 0.0;
}

}  // namespace

OdometryCovarianceProfile measuredCovarianceProfile()
{
  // Estimated from six stationary windows (about 182 seconds total) in
  // odom_test_001. X/Y are deliberately isotropic using the larger measured
  // planar variance. Twist Y is not observed by RF2O.
  return {
    "measured",
    2.5e-4,
    2.5e-4,
    1.0e-4,
    2.5e-4,
    1.0,
    1.1e-4,
    1.0,
    1.0,
  };
}

bool covarianceProfileIsValid(const OdometryCovarianceProfile & profile)
{
  return
    isFiniteAndNonnegative(profile.pose_x_variance) &&
    isFiniteAndNonnegative(profile.pose_y_variance) &&
    isFiniteAndNonnegative(profile.pose_yaw_variance) &&
    isFiniteAndNonnegative(profile.twist_x_variance) &&
    isFiniteAndNonnegative(profile.twist_y_variance) &&
    isFiniteAndNonnegative(profile.twist_yaw_variance) &&
    isFiniteAndNonnegative(profile.unobserved_pose_variance) &&
    isFiniteAndNonnegative(profile.unobserved_twist_variance);
}

bool resolveCovarianceProfile(
  const std::string & name,
  const OdometryCovarianceProfile & custom_profile,
  OdometryCovarianceProfile & resolved_profile)
{
  if (name == "measured") {
    resolved_profile = measuredCovarianceProfile();
  } else if (name == "relaxed") {
    resolved_profile = relaxedCovarianceProfile();
  } else if (name == "conservative") {
    resolved_profile = conservativeCovarianceProfile();
  } else if (name == "legacy_zero") {
    resolved_profile = legacyZeroCovarianceProfile();
  } else if (name == "custom") {
    resolved_profile = custom_profile;
    resolved_profile.name = "custom";
  } else {
    return false;
  }

  return covarianceProfileIsValid(resolved_profile);
}

void setPoseCovariance(
  std::array<double, 36> & covariance,
  const OdometryCovarianceProfile & profile)
{
  covariance.fill(0.0);
  covariance[0] = profile.pose_x_variance;
  covariance[7] = profile.pose_y_variance;
  covariance[14] = profile.unobserved_pose_variance;
  covariance[21] = profile.unobserved_pose_variance;
  covariance[28] = profile.unobserved_pose_variance;
  covariance[35] = profile.pose_yaw_variance;
}

void setTwistCovariance(
  std::array<double, 36> & covariance,
  const OdometryCovarianceProfile & profile)
{
  covariance.fill(0.0);
  covariance[0] = profile.twist_x_variance;
  covariance[7] = profile.twist_y_variance;
  covariance[14] = profile.unobserved_twist_variance;
  covariance[21] = profile.unobserved_twist_variance;
  covariance[28] = profile.unobserved_twist_variance;
  covariance[35] = profile.twist_yaw_variance;
}

}  // namespace lidar_pointcloud_filter
