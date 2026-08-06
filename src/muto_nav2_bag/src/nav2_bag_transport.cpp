#include "muto_nav2_bag/nav2_bag_transport.hpp"

#include <exception>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "muto_nav2_bag/rosbag2_compat.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node_options.hpp"
#include "rmw/rmw.h"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_storage/storage_options.hpp"
#include "rosbag2_transport/record_options.hpp"

namespace muto_nav2_bag
{

Nav2BagTransport::Nav2BagTransport(rclcpp::Context::SharedPtr context)
: context_(std::move(context))
{
  if (!context_) {
    throw std::invalid_argument("Nav2 bag transport requires a ROS context");
  }
}

Nav2BagTransport::~Nav2BagTransport()
{
  try {
    stop();
  } catch (const std::exception & error) {
    RCLCPP_ERROR(
      rclcpp::get_logger("nav2_bag_transport"),
      "Could not finalize Nav2 bag: %s", error.what());
  }
}

void Nav2BagTransport::start(const Nav2BagOptions & options)
{
  if (active()) {
    throw std::logic_error("a Nav2 bag is already recording");
  }
  if (options.uri.empty()) {
    throw std::invalid_argument("Nav2 bag URI must not be empty");
  }
  if (options.storage_id.empty()) {
    throw std::invalid_argument("Nav2 bag storage ID must not be empty");
  }
  if (options.topics.empty()) {
    throw std::invalid_argument(
            "Nav2 bag requires an explicit, non-empty topic allowlist");
  }

  const std::filesystem::path uri =
    std::filesystem::path(options.uri).lexically_normal();
  if (std::filesystem::exists(uri)) {
    throw std::runtime_error("Nav2 bag path already exists: " + uri.string());
  }
  if (uri.has_parent_path()) {
    std::filesystem::create_directories(uri.parent_path());
  }

  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = uri.string();
  storage_options.storage_id = options.storage_id;
  storage_options.max_cache_size = options.max_cache_size;
  storage_options.storage_preset_profile = options.storage_preset_profile;
  const bool storage_supports_custom_data =
    rosbag2_compat::set_custom_data(storage_options, options.custom_data);

  rosbag2_transport::RecordOptions record_options;
  // Never fall back to recording the full graph.  Missing topics are simply
  // discovered when/if their publishers appear during this recording.
  rosbag2_compat::set_all_topics(record_options, false);
  const bool recorder_supports_all_services =
    rosbag2_compat::set_all_services(
    record_options, options.record_all_services);
  record_options.topics = options.topics;
  record_options.regex.clear();
  rosbag2_compat::set_exclude_regex(record_options, "");
  record_options.include_hidden_topics = options.include_hidden_topics;
  rosbag2_compat::disable_keyboard_controls(record_options);
  record_options.use_sim_time = options.use_sim_time;
  record_options.rmw_serialization_format = rmw_get_serialization_format();

  if (!storage_supports_custom_data) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("nav2_bag_transport"),
      "This rosbag2 version has no StorageOptions custom_data; recording "
      "provenance remains available in the portable manifest");
  }
  if (options.record_all_services && !recorder_supports_all_services) {
    RCLCPP_WARN(
      rclcpp::get_logger("nav2_bag_transport"),
      "This rosbag2 version cannot record all service events. Nav2 action "
      "feedback/status and this stack's target-pose mirrors remain in the "
      "explicit topic allowlist");
  }

  rclcpp::NodeOptions node_options;
  node_options.context(context_);
  node_options.use_global_arguments(false);
  auto writer = std::make_shared<rosbag2_cpp::Writer>();
  auto recorder = std::make_shared<rosbag2_transport::Recorder>(
    writer, storage_options, record_options, "muto_nav2_rosbag2", node_options);

  rclcpp::ExecutorOptions executor_options;
  executor_options.context = context_;
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>(
    executor_options);
  executor->add_node(recorder);
  std::thread executor_thread(
    [executor]()
    {
      try {
        executor->spin();
      } catch (const std::exception & error) {
        RCLCPP_ERROR(
          rclcpp::get_logger("nav2_bag_transport"),
          "Nav2 bag executor stopped unexpectedly: %s", error.what());
      }
    });

  try {
    recorder->record();
  } catch (...) {
    executor->cancel();
    if (executor_thread.joinable()) {
      executor_thread.join();
    }
    executor->remove_node(recorder);
    throw;
  }

  recorder_ = std::move(recorder);
  executor_ = std::move(executor);
  executor_thread_ = std::move(executor_thread);
  path_ = uri.string();
}

void Nav2BagTransport::stop()
{
  if (!recorder_) {
    return;
  }

  std::exception_ptr stop_error;
  try {
    recorder_->stop();
  } catch (...) {
    stop_error = std::current_exception();
  }

  if (executor_) {
    executor_->cancel();
  }
  if (executor_thread_.joinable()) {
    executor_thread_.join();
  }
  if (executor_) {
    try {
      executor_->remove_node(recorder_);
    } catch (const std::exception & error) {
      RCLCPP_WARN(
        rclcpp::get_logger("nav2_bag_transport"),
        "Could not remove finalized recorder node: %s", error.what());
    }
  }
  recorder_.reset();
  executor_.reset();

  if (stop_error) {
    std::rethrow_exception(stop_error);
  }
}

bool Nav2BagTransport::active() const noexcept
{
  return recorder_ != nullptr;
}

const std::string & Nav2BagTransport::path() const noexcept
{
  return path_;
}

}  // namespace muto_nav2_bag
