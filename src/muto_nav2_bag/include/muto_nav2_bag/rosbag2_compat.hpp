#ifndef MUTO_NAV2_BAG__ROSBAG2_COMPAT_HPP_
#define MUTO_NAV2_BAG__ROSBAG2_COMPAT_HPP_

#include <string>
#include <type_traits>
#include <utility>

namespace muto_nav2_bag
{
namespace rosbag2_compat
{

// rosbag2 renamed a handful of RecordOptions fields between Humble and
// Jazzy.  Keep those differences here so the recorder itself has one code
// path on both the deployed robot and development machine.
template<typename OptionsT, typename = void>
struct HasAllTopics : std::false_type {};

template<typename OptionsT>
struct HasAllTopics<
  OptionsT, std::void_t<decltype(std::declval<OptionsT &>().all_topics)>>
  : std::true_type {};

template<typename OptionsT, typename = void>
struct HasAll : std::false_type {};

template<typename OptionsT>
struct HasAll<OptionsT, std::void_t<decltype(std::declval<OptionsT &>().all)>>
  : std::true_type {};

template<typename OptionsT, typename = void>
struct HasAllServices : std::false_type {};

template<typename OptionsT>
struct HasAllServices<
  OptionsT, std::void_t<decltype(std::declval<OptionsT &>().all_services)>>
  : std::true_type {};

template<typename OptionsT, typename = void>
struct HasExcludeRegex : std::false_type {};

template<typename OptionsT>
struct HasExcludeRegex<
  OptionsT, std::void_t<decltype(std::declval<OptionsT &>().exclude_regex)>>
  : std::true_type {};

template<typename OptionsT, typename = void>
struct HasExclude : std::false_type {};

template<typename OptionsT>
struct HasExclude<
  OptionsT, std::void_t<decltype(std::declval<OptionsT &>().exclude)>>
  : std::true_type {};

template<typename OptionsT, typename = void>
struct HasDisableKeyboardControls : std::false_type {};

template<typename OptionsT>
struct HasDisableKeyboardControls<
  OptionsT,
  std::void_t<decltype(
    std::declval<OptionsT &>().disable_keyboard_controls)>>
  : std::true_type {};

template<typename OptionsT, typename = void>
struct HasCustomData : std::false_type {};

template<typename OptionsT>
struct HasCustomData<
  OptionsT, std::void_t<decltype(std::declval<OptionsT &>().custom_data)>>
  : std::true_type {};

template<typename RecordOptionsT>
void set_all_topics(RecordOptionsT & options, const bool enabled)
{
  if constexpr (HasAllTopics<RecordOptionsT>::value) {
    options.all_topics = enabled;
  } else {
    static_assert(
      HasAll<RecordOptionsT>::value,
      "Unsupported rosbag2 RecordOptions: no all-topic selector");
    options.all = enabled;
  }
}

template<typename RecordOptionsT>
bool set_all_services(RecordOptionsT & options, const bool enabled)
{
  if constexpr (HasAllServices<RecordOptionsT>::value) {
    options.all_services = enabled;
    return true;
  }
  (void)options;
  (void)enabled;
  return false;
}

template<typename RecordOptionsT>
void set_exclude_regex(RecordOptionsT & options, const std::string & regex)
{
  if constexpr (HasExcludeRegex<RecordOptionsT>::value) {
    options.exclude_regex = regex;
  } else {
    static_assert(
      HasExclude<RecordOptionsT>::value,
      "Unsupported rosbag2 RecordOptions: no exclusion regex");
    options.exclude = regex;
  }
}

template<typename RecordOptionsT>
bool disable_keyboard_controls(RecordOptionsT & options)
{
  if constexpr (HasDisableKeyboardControls<RecordOptionsT>::value) {
    options.disable_keyboard_controls = true;
    return true;
  }
  (void)options;
  return false;
}

template<typename StorageOptionsT, typename CustomDataT>
bool set_custom_data(StorageOptionsT & options, const CustomDataT & custom_data)
{
  if constexpr (HasCustomData<StorageOptionsT>::value) {
    options.custom_data = custom_data;
    return true;
  }
  (void)options;
  (void)custom_data;
  return false;
}

}  // namespace rosbag2_compat
}  // namespace muto_nav2_bag

#endif  // MUTO_NAV2_BAG__ROSBAG2_COMPAT_HPP_
