#include <gtest/gtest.h>

#include <regex>
#include <string>
#include <vector>

#include "muto_exploration_bag/recording_defaults.hpp"

namespace
{

bool excluded_by_default(const std::string & topic)
{
  const std::regex filter(muto_exploration_bag::kDefaultExcludeRegex);
  return std::regex_match(topic, filter);
}

TEST(RecordingDefaults, ExcludesHighBandwidthPerceptionPayloads)
{
  const std::vector<std::string> excluded_topics = {
    "/camera/color/image_raw",
    "/camera/color/image_raw/compressed",
    "/camera/color/image_raw/theora",
    "/camera/depth/image_raw",
    "/camera/depth/image_raw/compressedDepth",
    "/camera/depth/points",
    "/sam2/annotated_image",
    "/sam2/annotated_image/compressed",
    "/sam2/mask",
    "/sam2/instance_mask",
    "/sam2/instance_pointcloud",
    "/lidar/PointCloudFiltered",
  };

  for (const auto & topic : excluded_topics) {
    EXPECT_TRUE(excluded_by_default(topic)) << topic;
  }
}

TEST(RecordingDefaults, RetainsMissionAndStructuredResultTopics)
{
  const std::vector<std::string> retained_topics = {
    "/camera/color/camera_info",
    "/camera/depth/camera_info",
    "/camera/filtered_laserscan",
    "/lidar/raw_laserscan",
    "/lidar/filtered_laserscan",
    "/map",
    "/tf",
    "/tf_static",
    "/scan_odom",
    "/odometry/filtered",
    "/navigate_to_pose/_action/status",
    "/explore_and_record/recording_event",
    "/explore_and_record/operator_event",
    "/sam2/detections",
    "/sam2/segments",
    "/sam2/stored_objects",
    "/sam2/stored_object_markers",
  };

  for (const auto & topic : retained_topics) {
    EXPECT_FALSE(excluded_by_default(topic)) << topic;
  }
}

}  // namespace
