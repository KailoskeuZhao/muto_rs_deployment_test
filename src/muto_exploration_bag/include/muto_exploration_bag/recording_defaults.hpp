#ifndef MUTO_EXPLORATION_BAG__RECORDING_DEFAULTS_HPP_
#define MUTO_EXPLORATION_BAG__RECORDING_DEFAULTS_HPP_

namespace muto_exploration_bag
{

// Keep navigation, odometry, action, log, and structured object-result topics
// while omitting the high-bandwidth perception payloads that dominate bag size.
inline constexpr char kDefaultExcludeRegex[] =
  "^(/camera/[^/]+/(image_raw|points)(/.*)?|"
  "/sam2/(annotated_image|mask|instance_mask|instance_pointcloud)(/.*)?|"
  "/lidar/PointCloud.*)$";

}  // namespace muto_exploration_bag

#endif  // MUTO_EXPLORATION_BAG__RECORDING_DEFAULTS_HPP_
