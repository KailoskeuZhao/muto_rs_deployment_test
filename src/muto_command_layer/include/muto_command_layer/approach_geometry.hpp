#ifndef MUTO_COMMAND_LAYER__APPROACH_GEOMETRY_HPP_
#define MUTO_COMMAND_LAYER__APPROACH_GEOMETRY_HPP_

#include <cstdint>
#include <vector>

namespace muto_command_layer
{

struct PlanarPose
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct ApproachGrid
{
  int width{0};
  int height{0};
  double resolution{0.0};
  double origin_x{0.0};
  double origin_y{0.0};
  double origin_yaw{0.0};
  std::vector<uint8_t> costs;
};

struct ApproachPlannerConfig
{
  int maximum_traversable_cost{252};
  double robot_radius{0.16};
  double minimum_standoff{0.75};
  double start_snap_distance{0.5};
};

PlanarPose compute_object_approach_pose(
  const ApproachGrid & grid,
  const double object_x,
  const double object_y,
  const double robot_x,
  const double robot_y,
  const ApproachPlannerConfig & config = ApproachPlannerConfig{});

}  // namespace muto_command_layer

#endif  // MUTO_COMMAND_LAYER__APPROACH_GEOMETRY_HPP_
