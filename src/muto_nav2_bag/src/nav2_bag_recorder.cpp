#include <algorithm>
#include <chrono>
#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <memory>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "muto_nav2_bag/nav2_bag_transport.hpp"
#include "muto_nav2_bag/nav2_topic_profile.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rmw/rmw.h"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "recording_build_info.hpp"

using namespace std::chrono_literals;

namespace
{

constexpr char kDefaultOutputDirectory[] = "/opt/muto_rs_ws/bags";
constexpr char kDefaultMetadataTopic[] = "/muto/nav2_bag/metadata";
constexpr char kDefaultEventTopic[] = "/muto/nav2_bag/event";
constexpr char kDefaultStatusTopic[] = "/muto/nav2_bag/status";
constexpr char kDefaultPathTopic[] = "/muto/nav2_bag/path";
constexpr char kDefaultStopService[] = "/muto/nav2_bag/stop";
constexpr char kManifestFilename[] = "muto_nav2_recording_manifest.json";

struct ConfigSnapshot
{
  std::string role;
  std::string source;
  std::string snapshot;
  std::string status;
  std::string detail;
};

std::string json_escape(const std::string & value)
{
  std::ostringstream escaped;
  for (const unsigned char character : value) {
    switch (character) {
      case '"':
        escaped << "\\\"";
        break;
      case '\\':
        escaped << "\\\\";
        break;
      case '\n':
        escaped << "\\n";
        break;
      case '\r':
        escaped << "\\r";
        break;
      case '\t':
        escaped << "\\t";
        break;
      default:
        if (character < 0x20U) {
          escaped << "\\u" << std::hex << std::setw(4) <<
            std::setfill('0') << static_cast<unsigned int>(character) <<
            std::dec;
        } else {
          escaped << character;
        }
    }
  }
  return escaped.str();
}

std::string wall_time_string(const std::chrono::system_clock::time_point & time)
{
  const std::time_t wall_time = std::chrono::system_clock::to_time_t(time);
  std::tm utc_time{};
  gmtime_r(&wall_time, &utc_time);
  std::ostringstream value;
  value << std::put_time(&utc_time, "%Y-%m-%dT%H:%M:%SZ");
  return value.str();
}

std::string generated_bag_name()
{
  const auto now = std::chrono::system_clock::now();
  const std::time_t wall_time = std::chrono::system_clock::to_time_t(now);
  std::tm local_time{};
  localtime_r(&wall_time, &local_time);

  const auto clock_seed = static_cast<uint64_t>(
    now.time_since_epoch().count());
  std::random_device random_device;
  std::mt19937_64 generator(
    clock_seed ^ (static_cast<uint64_t>(random_device()) << 32U) ^
    static_cast<uint64_t>(random_device()));

  std::ostringstream name;
  name << "muto_nav2_" << std::put_time(&local_time, "%Y%m%d_%H%M%S") <<
    "_" << std::hex << std::setw(8) << std::setfill('0') <<
    static_cast<uint32_t>(generator());
  return name.str();
}

bool is_safe_bag_name(const std::string & name)
{
  if (name.empty() || name == "." || name == "..") {
    return false;
  }
  return std::all_of(
    name.begin(), name.end(),
    [](const unsigned char character) {
      return std::isalnum(character) != 0 || character == '_' ||
             character == '-' || character == '.';
    });
}

std::string join_topics(const std::vector<std::string> & topics)
{
  std::ostringstream value;
  for (std::size_t index = 0U; index < topics.size(); ++index) {
    if (index != 0U) {
      value << ';';
    }
    value << topics[index];
  }
  return value.str();
}

}  // namespace

class Nav2BagRecorderNode : public rclcpp::Node
{
public:
  Nav2BagRecorderNode()
  : Node("nav2_bag_recorder")
  {
    output_directory_ = declare_parameter<std::string>(
      "output_directory", kDefaultOutputDirectory);
    bag_name_ = declare_parameter<std::string>("bag_name", "");
    storage_id_ = declare_parameter<std::string>("storage_id", "mcap");
    storage_preset_ = declare_parameter<std::string>(
      "storage_preset", "zstd_fast");
    max_cache_size_ =
      declare_parameter<int64_t>("max_cache_size", 52428800);
    topics_ = declare_parameter<std::vector<std::string>>(
      "topics", muto_nav2_bag::default_nav2_topics());
    include_hidden_topics_ =
      declare_parameter<bool>("include_hidden_topics", true);
    record_all_services_ =
      declare_parameter<bool>("record_all_services", false);
    metadata_topic_ = declare_parameter<std::string>(
      "metadata_topic", kDefaultMetadataTopic);
    event_topic_ = declare_parameter<std::string>(
      "event_topic", kDefaultEventTopic);
    status_topic_ = declare_parameter<std::string>(
      "status_topic", kDefaultStatusTopic);
    path_topic_ = declare_parameter<std::string>(
      "path_topic", kDefaultPathTopic);
    stop_service_name_ = declare_parameter<std::string>(
      "stop_service", kDefaultStopService);
    nav2_params_file_ =
      declare_parameter<std::string>("nav2_params_file", "");
    frontier_params_file_ =
      declare_parameter<std::string>("frontier_params_file", "");
    slam_params_file_ =
      declare_parameter<std::string>("slam_params_file", "");
    nav_to_pose_bt_file_ =
      declare_parameter<std::string>("nav_to_pose_bt_file", "");
    nav_through_poses_bt_file_ =
      declare_parameter<std::string>("nav_through_poses_bt_file", "");
    get_parameter("use_sim_time", use_sim_time_);

    resolve_and_validate_parameters();

    transport_ = std::make_unique<muto_nav2_bag::Nav2BagTransport>(
      get_node_base_interface()->get_context());

    const auto transient_qos =
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    metadata_publisher_ =
      create_publisher<std_msgs::msg::String>(metadata_topic_, transient_qos);
    status_publisher_ =
      create_publisher<std_msgs::msg::String>(status_topic_, transient_qos);
    path_publisher_ =
      create_publisher<std_msgs::msg::String>(path_topic_, transient_qos);

    // Advertise and consume the event topic continuously. This prevents a
    // `ros2 topic pub --once` milestone from beating rosbag2 discovery.
    event_topic_anchor_ = create_publisher<std_msgs::msg::String>(
      event_topic_, rclcpp::QoS(100).reliable().durability_volatile());
    event_subscription_ = create_subscription<std_msgs::msg::String>(
      event_topic_, rclcpp::QoS(100).reliable().durability_volatile(),
      std::bind(
        &Nav2BagRecorderNode::handle_event, this, std::placeholders::_1));
    stop_service_ = create_service<std_srvs::srv::Trigger>(
      stop_service_name_,
      std::bind(
        &Nav2BagRecorderNode::handle_stop, this, std::placeholders::_1,
        std::placeholders::_2));

    start_recording();

    // The three provenance publishers are transient-local. Republish once as
    // well, so old and new rosbag2 QoS-selection implementations both capture
    // them after discovery completes.
    announcement_timer_ = create_wall_timer(
      750ms,
      [this]() {
        publish_announcements();
        announcement_timer_->cancel();
      });
  }

  ~Nav2BagRecorderNode() override
  {
    try {
      finalize_recording("node_shutdown", false);
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        get_logger(), "Unexpected error while closing Nav2 bag: %s",
        error.what());
    }
  }

private:
  void resolve_and_validate_parameters()
  {
    if (output_directory_.empty()) {
      output_directory_ = kDefaultOutputDirectory;
    }
    output_directory_ = std::filesystem::path(
      output_directory_).lexically_normal().string();
    if (!std::filesystem::path(output_directory_).is_absolute()) {
      throw std::invalid_argument("output_directory must be absolute");
    }
    if (bag_name_.empty()) {
      bag_name_ = generated_bag_name();
    }
    if (!is_safe_bag_name(bag_name_)) {
      throw std::invalid_argument(
              "bag_name must be one safe path component using only letters, "
              "digits, '.', '-' or '_'");
    }
    const auto session_separator = bag_name_.find_last_of('_');
    session_id_ =
      session_separator == std::string::npos ? bag_name_ :
      bag_name_.substr(session_separator + 1U);
    if (storage_id_.empty() || storage_preset_.empty()) {
      throw std::invalid_argument(
              "storage_id and storage_preset must not be empty");
    }
    if (max_cache_size_ < 0) {
      throw std::invalid_argument("max_cache_size must be nonnegative");
    }
    if (topics_.empty()) {
      topics_ = muto_nav2_bag::default_nav2_topics();
    }

    std::vector<std::string> unique_topics;
    unique_topics.reserve(topics_.size() + 4U);
    for (const auto & topic : topics_) {
      if (topic.empty() || topic.front() != '/') {
        throw std::invalid_argument(
                "topics entries must be absolute topic names");
      }
      muto_nav2_bag::append_topic_if_missing(unique_topics, topic);
    }
    topics_ = std::move(unique_topics);

    const std::vector<std::string *> recorder_names = {
      &metadata_topic_, &event_topic_, &status_topic_, &path_topic_};
    for (const auto * name : recorder_names) {
      if (name->empty() || name->front() != '/') {
        throw std::invalid_argument(
                "recorder topic names must be absolute topic names");
      }
      muto_nav2_bag::append_topic_if_missing(topics_, *name);
    }
    if (stop_service_name_.empty() || stop_service_name_.front() != '/') {
      throw std::invalid_argument(
              "stop_service must be an absolute service name");
    }
  }

  void start_recording()
  {
    const auto bag_path = std::filesystem::path(output_directory_) / bag_name_;
    recording_started_at_ = std::chrono::system_clock::now();
    const char * ros_distro = std::getenv("ROS_DISTRO");
    ros_distro_ = ros_distro == nullptr ? "unknown" : ros_distro;
    const char * rmw_implementation = rmw_get_implementation_identifier();
    rmw_implementation_ =
      rmw_implementation == nullptr ? "unknown" : rmw_implementation;

    muto_nav2_bag::Nav2BagOptions options;
    options.uri = bag_path.string();
    options.storage_id = storage_id_;
    options.storage_preset_profile = storage_preset_;
    options.topics = topics_;
    options.max_cache_size = static_cast<uint64_t>(max_cache_size_);
    options.include_hidden_topics = include_hidden_topics_;
    options.record_all_services = record_all_services_;
    options.use_sim_time = use_sim_time_;
    options.custom_data = {
      {"muto_schema", "nav2_recording_v1"},
      {"bag_path", bag_path.string()},
      {"bag_name", bag_name_},
      {"session_id", session_id_},
      {"git_revision", muto_nav2_bag_build::kGitRevision},
      {"git_dirty", muto_nav2_bag_build::kGitDirty ? "true" : "false"},
      {"ros_distro", ros_distro_},
      {"rmw_implementation", rmw_implementation_},
      {"topic_scope", "explicit_nav2_allowlist"},
      {"topics", join_topics(topics_)},
      {"event_topic", event_topic_},
      {"manifest_file", kManifestFilename},
    };

    try {
      transport_->start(options);
      prepare_config_snapshots();
      write_manifest();
    } catch (...) {
      if (transport_->active()) {
        try {
          transport_->stop();
        } catch (const std::exception & stop_error) {
          RCLCPP_ERROR(
            get_logger(), "Could not close incomplete Nav2 bag: %s",
            stop_error.what());
        }
      }
      throw;
    }

    publish_announcements();
    RCLCPP_INFO(
      get_logger(),
      "Recording the Nav2 allowlist (%zu topics) to %s",
      topics_.size(), transport_->path().c_str());
  }

  void prepare_config_snapshots()
  {
    snapshots_.clear();
    snapshot_config(
      "nav2", nav2_params_file_, "nav2_params.snapshot.yaml");
    snapshot_config(
      "frontier", frontier_params_file_,
      "frontier_exploration_params.snapshot.yaml");
    snapshot_config(
      "slam", slam_params_file_, "slam_params.snapshot.yaml");
    snapshot_config(
      "navigate_to_pose_behavior_tree", nav_to_pose_bt_file_,
      "muto_nav_to_pose.snapshot.xml");
    snapshot_config(
      "navigate_through_poses_behavior_tree", nav_through_poses_bt_file_,
      "muto_nav_through_poses.snapshot.xml");
  }

  void snapshot_config(
    const std::string & role, const std::string & source,
    const std::string & snapshot_name)
  {
    ConfigSnapshot record;
    record.role = role;
    record.source = source;
    if (source.empty()) {
      record.status = "not_configured";
      snapshots_.push_back(std::move(record));
      return;
    }

    const std::filesystem::path source_path =
      std::filesystem::path(source).lexically_normal();
    const std::filesystem::path snapshot_path =
      std::filesystem::path(transport_->path()) / snapshot_name;
    record.snapshot = snapshot_name;

    std::error_code error;
    if (!std::filesystem::is_regular_file(source_path, error)) {
      record.status = "missing";
      record.detail = error ? error.message() : "not a regular file";
      RCLCPP_WARN(
        get_logger(), "Could not snapshot %s config %s: %s", role.c_str(),
        source_path.string().c_str(), record.detail.c_str());
      snapshots_.push_back(std::move(record));
      return;
    }

    error.clear();
    std::filesystem::copy_file(
      source_path, snapshot_path,
      std::filesystem::copy_options::overwrite_existing, error);
    if (error) {
      record.status = "copy_error";
      record.detail = error.message();
      RCLCPP_WARN(
        get_logger(), "Could not snapshot %s config %s: %s", role.c_str(),
        source_path.string().c_str(), record.detail.c_str());
    } else {
      record.status = "copied";
    }
    snapshots_.push_back(std::move(record));
  }

  void write_manifest()
  {
    const auto manifest_path =
      std::filesystem::path(transport_->path()) / kManifestFilename;
    std::ofstream manifest(manifest_path, std::ios::out | std::ios::trunc);
    if (!manifest.is_open()) {
      throw std::runtime_error(
              "could not open Nav2 recording manifest: " +
              manifest_path.string());
    }

    manifest << "{\n";
    manifest << "  \"schema\": \"muto_nav2_recording_manifest_v1\",\n";
    manifest << "  \"bag_path\": \"" <<
      json_escape(transport_->path()) << "\",\n";
    manifest << "  \"bag_name\": \"" << json_escape(bag_name_) <<
      "\",\n";
    manifest << "  \"session_id\": \"" << json_escape(session_id_) <<
      "\",\n";
    manifest << "  \"recording_started_at\": \"" <<
      wall_time_string(recording_started_at_) << "\",\n";
    manifest << "  \"recording_ended_at\": ";
    if (recording_ended_at_.time_since_epoch().count() == 0) {
      manifest << "null,\n";
    } else {
      manifest << "\"" << wall_time_string(recording_ended_at_) << "\",\n";
    }
    manifest << "  \"finalized\": " << (finalized_ ? "true" : "false") <<
      ",\n";
    manifest << "  \"stop_reason\": \"" << json_escape(stop_reason_) <<
      "\",\n";
    manifest << "  \"finalization_detail\": \"" <<
      json_escape(finalization_detail_) << "\",\n";
    manifest << "  \"git_revision\": \"" <<
      json_escape(muto_nav2_bag_build::kGitRevision) << "\",\n";
    manifest << "  \"git_dirty\": " <<
      (muto_nav2_bag_build::kGitDirty ? "true" : "false") << ",\n";
    manifest << "  \"ros_distro\": \"" << json_escape(ros_distro_) <<
      "\",\n";
    manifest << "  \"rmw_implementation\": \"" <<
      json_escape(rmw_implementation_) << "\",\n";
    manifest << "  \"storage_id\": \"" << json_escape(storage_id_) <<
      "\",\n";
    manifest << "  \"storage_preset\": \"" <<
      json_escape(storage_preset_) << "\",\n";
    manifest << "  \"max_cache_size_bytes\": " << max_cache_size_ << ",\n";
    manifest << "  \"include_hidden_topics\": " <<
      (include_hidden_topics_ ? "true" : "false") << ",\n";
    manifest << "  \"record_all_services_requested\": " <<
      (record_all_services_ ? "true" : "false") << ",\n";
    manifest << "  \"use_sim_time\": " <<
      (use_sim_time_ ? "true" : "false") << ",\n";
    manifest << "  \"metadata_topic\": \"" <<
      json_escape(metadata_topic_) << "\",\n";
    manifest << "  \"event_topic\": \"" << json_escape(event_topic_) <<
      "\",\n";
    manifest << "  \"status_topic\": \"" << json_escape(status_topic_) <<
      "\",\n";
    manifest << "  \"path_topic\": \"" << json_escape(path_topic_) <<
      "\",\n";
    manifest << "  \"stop_service\": \"" <<
      json_escape(stop_service_name_) << "\",\n";

    manifest << "  \"topics\": [\n";
    for (std::size_t index = 0U; index < topics_.size(); ++index) {
      manifest << "    \"" << json_escape(topics_[index]) << "\"" <<
        (index + 1U == topics_.size() ? "\n" : ",\n");
    }
    manifest << "  ],\n";

    manifest << "  \"config_snapshots\": [\n";
    for (std::size_t index = 0U; index < snapshots_.size(); ++index) {
      const auto & snapshot = snapshots_[index];
      manifest << "    {\"role\": \"" << json_escape(snapshot.role) <<
        "\", \"source\": \"" << json_escape(snapshot.source) <<
        "\", \"snapshot\": \"" << json_escape(snapshot.snapshot) <<
        "\", \"status\": \"" << json_escape(snapshot.status) <<
        "\", \"detail\": \"" << json_escape(snapshot.detail) << "\"}" <<
        (index + 1U == snapshots_.size() ? "\n" : ",\n");
    }
    manifest << "  ]\n";
    manifest << "}\n";
    manifest.flush();
    if (!manifest.good()) {
      throw std::runtime_error(
              "could not write Nav2 recording manifest: " +
              manifest_path.string());
    }
  }

  std::string metadata_json() const
  {
    std::ostringstream metadata;
    metadata << "{\"schema\":\"muto_nav2_bag_metadata_v1\","
      "\"bag_path\":\"" << json_escape(transport_->path()) <<
      "\",\"bag_name\":\"" << json_escape(bag_name_) <<
      "\",\"session_id\":\"" << json_escape(session_id_) <<
      "\",\"manifest_file\":\"" << kManifestFilename <<
      "\",\"git_revision\":\"" <<
      json_escape(muto_nav2_bag_build::kGitRevision) <<
      "\",\"git_dirty\":" <<
      (muto_nav2_bag_build::kGitDirty ? "true" : "false") <<
      ",\"ros_distro\":\"" << json_escape(ros_distro_) <<
      "\",\"rmw_implementation\":\"" <<
      json_escape(rmw_implementation_) <<
      "\",\"topic_count\":" << topics_.size() << "}";
    return metadata.str();
  }

  void publish_announcements()
  {
    if (!transport_ || !transport_->active()) {
      return;
    }
    std_msgs::msg::String path_message;
    path_message.data = transport_->path();
    path_publisher_->publish(path_message);

    std_msgs::msg::String metadata_message;
    metadata_message.data = metadata_json();
    metadata_publisher_->publish(metadata_message);
    publish_status("recording", "");
  }

  void publish_status(const std::string & event, const std::string & detail)
  {
    std_msgs::msg::String message;
    message.data =
      "{\"schema\":\"muto_nav2_bag_status_v1\",\"event\":\"" +
      json_escape(event) + "\",\"bag_path\":\"" +
      json_escape(transport_ ? transport_->path() : "") +
      "\",\"detail\":\"" + json_escape(detail) + "\"}";
    status_publisher_->publish(message);
  }

  void handle_event(const std_msgs::msg::String::ConstSharedPtr message)
  {
    if (transport_ && transport_->active()) {
      RCLCPP_INFO(get_logger(), "Nav2 milestone recorded: %s", message->data.c_str());
    } else {
      RCLCPP_WARN(
        get_logger(), "Nav2 milestone arrived after recording stopped: %s",
        message->data.c_str());
    }
  }

  void handle_stop(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    if (!transport_ || !transport_->active()) {
      response->success = true;
      response->message = "Nav2 bag is already finalized: " +
        (transport_ ? transport_->path() : std::string{});
      return;
    }

    response->success = finalize_recording("stop_service", true);
    response->message = response->success ?
      "Finalized Nav2 bag: " + transport_->path() :
      "Could not finalize Nav2 bag: " + finalization_detail_;
  }

  bool finalize_recording(const std::string & reason, const bool publish_final)
  {
    if (!transport_ || !transport_->active()) {
      return true;
    }

    const std::string finalized_path = transport_->path();
    if (status_publisher_) {
      publish_status("recording_stopping", reason);
    }

    bool stopped_cleanly = true;
    finalization_detail_.clear();
    try {
      transport_->stop();
    } catch (const std::exception & error) {
      stopped_cleanly = false;
      finalization_detail_ = error.what();
      RCLCPP_ERROR(
        get_logger(), "Could not finalize Nav2 bag %s: %s",
        finalized_path.c_str(), error.what());
    }

    finalized_ = stopped_cleanly;
    stop_reason_ = reason;
    recording_ended_at_ = std::chrono::system_clock::now();
    try {
      write_manifest();
    } catch (const std::exception & error) {
      stopped_cleanly = false;
      finalized_ = false;
      if (!finalization_detail_.empty()) {
        finalization_detail_ += "; ";
      }
      finalization_detail_ += error.what();
      RCLCPP_ERROR(get_logger(), "%s", error.what());
    }

    if (publish_final && status_publisher_) {
      publish_status(
        stopped_cleanly ? "recording_finalized" : "recording_error",
        finalization_detail_);
    }
    if (stopped_cleanly) {
      RCLCPP_INFO(
        get_logger(), "Finalized Nav2 bag after %s: %s", reason.c_str(),
        finalized_path.c_str());
    }
    return stopped_cleanly;
  }

  std::unique_ptr<muto_nav2_bag::Nav2BagTransport> transport_;
  std::string output_directory_;
  std::string bag_name_;
  std::string session_id_;
  std::string storage_id_;
  std::string storage_preset_;
  int64_t max_cache_size_{52428800};
  std::vector<std::string> topics_;
  bool include_hidden_topics_{true};
  bool record_all_services_{false};
  bool use_sim_time_{false};
  std::string metadata_topic_;
  std::string event_topic_;
  std::string status_topic_;
  std::string path_topic_;
  std::string stop_service_name_;
  std::string nav2_params_file_;
  std::string frontier_params_file_;
  std::string slam_params_file_;
  std::string nav_to_pose_bt_file_;
  std::string nav_through_poses_bt_file_;
  std::string ros_distro_;
  std::string rmw_implementation_;
  std::vector<ConfigSnapshot> snapshots_;
  std::chrono::system_clock::time_point recording_started_at_{};
  std::chrono::system_clock::time_point recording_ended_at_{};
  bool finalized_{false};
  std::string stop_reason_;
  std::string finalization_detail_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr metadata_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr path_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr event_topic_anchor_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr event_subscription_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
  rclcpp::TimerBase::SharedPtr announcement_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<Nav2BagRecorderNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("nav2_bag_recorder"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
