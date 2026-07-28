#ifndef MUTO_COMMAND_LAYER__APPROACH_GEOMETRY_HPP_
#define MUTO_COMMAND_LAYER__APPROACH_GEOMETRY_HPP_

#include <cmath>
#include <stdexcept>

namespace muto_command_layer
{

struct PlanarPose
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

inline PlanarPose compute_object_approach_pose(
  const double object_x,
  const double object_y,
  const double robot_x,
  const double robot_y,
  const double robot_yaw,
  const double approach_distance)
{
  if (!std::isfinite(object_x) || !std::isfinite(object_y) ||
    !std::isfinite(robot_x) || !std::isfinite(robot_y) ||
    !std::isfinite(robot_yaw) || !std::isfinite(approach_distance))
  {
    throw std::invalid_argument("approach geometry inputs must be finite");
  }
  if (approach_distance <= 0.0) {
    throw std::invalid_argument("approach_distance must be positive");
  }

  const double object_to_robot_x = robot_x - object_x;
  const double object_to_robot_y = robot_y - object_y;
  const double current_distance = std::hypot(
    object_to_robot_x, object_to_robot_y);

  double outward_x;
  double outward_y;
  constexpr double kCoincidentTolerance = 1.0e-6;
  if (current_distance > kCoincidentTolerance) {
    outward_x = object_to_robot_x / current_distance;
    outward_y = object_to_robot_y / current_distance;
  } else {
    // If the 2-D centroids coincide, place the target behind the robot and
    // retain its current viewing direction toward the object.
    outward_x = -std::cos(robot_yaw);
    outward_y = -std::sin(robot_yaw);
  }

  PlanarPose result;
  result.x = object_x + approach_distance * outward_x;
  result.y = object_y + approach_distance * outward_y;
  result.yaw = std::atan2(object_y - result.y, object_x - result.x);
  return result;
}

}  // namespace muto_command_layer

#endif  // MUTO_COMMAND_LAYER__APPROACH_GEOMETRY_HPP_
