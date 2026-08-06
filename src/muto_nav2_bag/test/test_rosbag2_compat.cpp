#include <gtest/gtest.h>

#include <string>
#include <unordered_map>

#include "muto_nav2_bag/rosbag2_compat.hpp"

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

  muto_nav2_bag::rosbag2_compat::set_all_topics(options, false);
  muto_nav2_bag::rosbag2_compat::set_exclude_regex(options, "");

  EXPECT_FALSE(options.all);
  EXPECT_TRUE(options.exclude.empty());
  EXPECT_FALSE(
    muto_nav2_bag::rosbag2_compat::set_all_services(options, true));
  EXPECT_FALSE(
    muto_nav2_bag::rosbag2_compat::disable_keyboard_controls(options));
}

TEST(Rosbag2Compat, ConfiguresJazzyRecordOptions)
{
  JazzyRecordOptions options;

  muto_nav2_bag::rosbag2_compat::set_all_topics(options, false);
  muto_nav2_bag::rosbag2_compat::set_exclude_regex(options, "");

  EXPECT_FALSE(options.all_topics);
  EXPECT_TRUE(options.exclude_regex.empty());
  EXPECT_TRUE(
    muto_nav2_bag::rosbag2_compat::set_all_services(options, true));
  EXPECT_TRUE(options.all_services);
  EXPECT_TRUE(
    muto_nav2_bag::rosbag2_compat::disable_keyboard_controls(options));
  EXPECT_TRUE(options.disable_keyboard_controls);
}

TEST(Rosbag2Compat, PreservesCustomDataWhenStorageSupportsIt)
{
  const std::unordered_map<std::string, std::string> custom_data = {
    {"topic_scope", "explicit_nav2_allowlist"},
  };
  HumbleStorageOptions humble_options;
  JazzyStorageOptions jazzy_options;

  EXPECT_FALSE(
    muto_nav2_bag::rosbag2_compat::set_custom_data(
      humble_options, custom_data));
  EXPECT_TRUE(
    muto_nav2_bag::rosbag2_compat::set_custom_data(
      jazzy_options, custom_data));
  EXPECT_EQ(jazzy_options.custom_data, custom_data);
}

}  // namespace
