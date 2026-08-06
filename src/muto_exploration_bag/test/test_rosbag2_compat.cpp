#include <gtest/gtest.h>

#include <string>
#include <unordered_map>

#include "muto_exploration_bag/rosbag2_compat.hpp"

namespace
{

struct HumbleRecordOptions
{
  bool all{false};
  std::string exclude;
};

struct JazzyRecordOptions
{
  bool all_topics{false};
  bool all_services{false};
  std::string exclude_regex;
  bool disable_keyboard_controls{false};
};

struct HumbleStorageOptions {};

struct JazzyStorageOptions
{
  std::unordered_map<std::string, std::string> custom_data;
};

TEST(Rosbag2Compat, ConfiguresHumbleRecordOptions)
{
  HumbleRecordOptions options;

  muto_exploration_bag::rosbag2_compat::set_all_topics(options, true);
  muto_exploration_bag::rosbag2_compat::set_exclude_regex(
    options, "/camera/.*");

  EXPECT_TRUE(options.all);
  EXPECT_EQ(options.exclude, "/camera/.*");
  EXPECT_FALSE(
    muto_exploration_bag::rosbag2_compat::set_all_services(options, true));
  EXPECT_FALSE(
    muto_exploration_bag::rosbag2_compat::disable_keyboard_controls(options));
}

TEST(Rosbag2Compat, ConfiguresJazzyRecordOptions)
{
  JazzyRecordOptions options;

  muto_exploration_bag::rosbag2_compat::set_all_topics(options, true);
  muto_exploration_bag::rosbag2_compat::set_exclude_regex(
    options, "/camera/.*");

  EXPECT_TRUE(options.all_topics);
  EXPECT_EQ(options.exclude_regex, "/camera/.*");
  EXPECT_TRUE(
    muto_exploration_bag::rosbag2_compat::set_all_services(options, true));
  EXPECT_TRUE(options.all_services);
  EXPECT_TRUE(
    muto_exploration_bag::rosbag2_compat::disable_keyboard_controls(options));
  EXPECT_TRUE(options.disable_keyboard_controls);
}

TEST(Rosbag2Compat, PreservesCustomDataWhenStorageSupportsIt)
{
  const std::unordered_map<std::string, std::string> custom_data = {
    {"goal_id", "test-goal"},
  };
  HumbleStorageOptions humble_options;
  JazzyStorageOptions jazzy_options;

  EXPECT_FALSE(
    muto_exploration_bag::rosbag2_compat::set_custom_data(
      humble_options, custom_data));
  EXPECT_TRUE(
    muto_exploration_bag::rosbag2_compat::set_custom_data(
      jazzy_options, custom_data));
  EXPECT_EQ(jazzy_options.custom_data, custom_data);
}

}  // namespace
