#ifndef LIDAR_POINTCLOUD_FILTER__ODOMETRY_COVARIANCE_PROFILES_HPP_
#define LIDAR_POINTCLOUD_FILTER__ODOMETRY_COVARIANCE_PROFILES_HPP_

#include <array>
#include <string>

namespace lidar_pointcloud_filter
{

struct OdometryCovarianceProfile
{
  std::string name;
  double pose_x_variance;
  double pose_y_variance;
  double pose_yaw_variance;
  double twist_x_variance;
  double twist_y_variance;
  double twist_yaw_variance;
  double unobserved_pose_variance;
  double unobserved_twist_variance;
};

OdometryCovarianceProfile measuredCovarianceProfile();

bool resolveCovarianceProfile(
  const std::string & name,
  const OdometryCovarianceProfile & custom_profile,
  OdometryCovarianceProfile & resolved_profile);

bool covarianceProfileIsValid(const OdometryCovarianceProfile & profile);

void setPoseCovariance(
  std::array<double, 36> & covariance,
  const OdometryCovarianceProfile & profile);

void setTwistCovariance(
  std::array<double, 36> & covariance,
  const OdometryCovarianceProfile & profile);

}  // namespace lidar_pointcloud_filter

#endif  // LIDAR_POINTCLOUD_FILTER__ODOMETRY_COVARIANCE_PROFILES_HPP_
