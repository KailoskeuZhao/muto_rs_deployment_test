#include <gtest/gtest.h>

#include <algorithm>
#include <set>
#include <string>
#include <vector>

#include "muto_nav2_bag/nav2_topic_profile.hpp"

namespace
{

bool contains(
  const std::vector<std::string> & topics, const std::string & topic)
{
  return std::find(topics.begin(), topics.end(), topic) != topics.end();
}

TEST(Nav2TopicProfile, IsAnAbsoluteUniqueAllowlist)
{
  const auto topics = muto_nav2_bag::default_nav2_topics();
  const std::set<std::string> unique(topics.begin(), topics.end());

  ASSERT_FALSE(topics.empty());
  EXPECT_EQ(unique.size(), topics.size());
  for (const auto & topic : topics) {
    ASSERT_FALSE(topic.empty());
    EXPECT_EQ(topic.front(), '/');
  }
}

TEST(Nav2TopicProfile, KeepsRecorderAndNavigationEvidence)
{
  const auto topics = muto_nav2_bag::default_nav2_topics();

  EXPECT_TRUE(contains(topics, "/muto/nav2_bag/metadata"));
  EXPECT_TRUE(contains(topics, "/muto/nav2_bag/event"));
  EXPECT_TRUE(contains(topics, "/muto/nav2_bag/status"));
  EXPECT_TRUE(contains(topics, "/muto/nav2_bag/path"));
  EXPECT_TRUE(contains(topics, "/clock"));
  EXPECT_TRUE(contains(topics, "/tf"));
  EXPECT_TRUE(contains(topics, "/odometry/filtered"));
  EXPECT_TRUE(contains(topics, "/muto/motion_command_state"));
  EXPECT_TRUE(contains(topics, "/plan"));
  EXPECT_TRUE(contains(topics, "/cmd_vel_nav"));
  EXPECT_TRUE(contains(topics, "/cmd_vel"));
  EXPECT_TRUE(contains(topics, "/navigate_to_pose/_action/status"));
  EXPECT_TRUE(contains(topics, "/explore/selected_frontier"));
}

TEST(Nav2TopicProfile, ExcludesKnownHighVolumeOrUnrelatedStreams)
{
  const auto topics = muto_nav2_bag::default_nav2_topics();

  EXPECT_FALSE(contains(topics, "/camera/color/image_raw"));
  EXPECT_FALSE(contains(topics, "/camera/depth/image_raw"));
  EXPECT_FALSE(contains(topics, "/camera/depth/points"));
  EXPECT_FALSE(contains(topics, "/sam2/instance_mask"));
  EXPECT_FALSE(contains(topics, "/bond"));
  EXPECT_FALSE(contains(topics, "/navigate_to_pose/_action/feedback"));
  EXPECT_FALSE(contains(topics, "/global_costmap/costmap_raw"));
  EXPECT_FALSE(contains(topics, "/local_costmap/costmap_raw"));
}

TEST(Nav2TopicProfile, AppendsOnlyOnce)
{
  std::vector<std::string> topics = {"/tf"};

  muto_nav2_bag::append_topic_if_missing(topics, "/tf");
  muto_nav2_bag::append_topic_if_missing(topics, "/map");

  ASSERT_EQ(topics.size(), 2U);
  EXPECT_EQ(topics[0], "/tf");
  EXPECT_EQ(topics[1], "/map");
}

}  // namespace
