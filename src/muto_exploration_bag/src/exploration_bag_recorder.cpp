#include <algorithm>
#include <chrono>
#include <cmath>
#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iterator>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "muto_exploration_bag/exploration_bag_transport.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "recording_build_info.hpp"

using namespace std::chrono_literals;

namespace
{

constexpr char kStartedEvent[] = "mission_started";

bool is_terminal_event(const std::string & event)
{
  return event == "succeeded" || event == "canceled" || event == "aborted";
}

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

std::optional<std::string> json_string_field(
  const std::string & json, const std::string & key)
{
  const std::string quoted_key = "\"" + key + "\"";
  const auto key_position = json.find(quoted_key);
  if (key_position == std::string::npos) {
    return std::nullopt;
  }
  const auto colon = json.find(':', key_position + quoted_key.size());
  if (colon == std::string::npos) {
    return std::nullopt;
  }
  const auto opening_quote = json.find('"', colon + 1U);
  if (opening_quote == std::string::npos) {
    return std::nullopt;
  }

  std::string value;
  bool escaped = false;
  for (std::size_t index = opening_quote + 1U; index < json.size(); ++index) {
    const char character = json[index];
    if (escaped) {
      switch (character) {
        case 'n':
          value.push_back('\n');
          break;
        case 'r':
          value.push_back('\r');
          break;
        case 't':
          value.push_back('\t');
          break;
        default:
          value.push_back(character);
          break;
      }
      escaped = false;
    } else if (character == '\\') {
      escaped = true;
    } else if (character == '"') {
      return value;
    } else {
      value.push_back(character);
    }
  }
  return std::nullopt;
}

std::string timestamped_bag_name(const std::string & goal_id)
{
  const auto now = std::chrono::system_clock::now();
  const std::time_t wall_time = std::chrono::system_clock::to_time_t(now);
  std::tm local_time{};
  localtime_r(&wall_time, &local_time);

  std::string safe_goal_id;
  std::copy_if(
    goal_id.begin(), goal_id.end(), std::back_inserter(safe_goal_id),
    [](const unsigned char character) {
      return std::isalnum(character) != 0;
    });
  if (safe_goal_id.empty()) {
    safe_goal_id = "unknown";
  }

  std::ostringstream name;
  name << "muto_explore_" << std::put_time(&local_time, "%Y%m%d_%H%M%S") <<
    "_" << safe_goal_id.substr(0U, 8U);
  return name.str();
}

void write_recording_manifest(
  const std::filesystem::path & bag_path,
  const std::unordered_map<std::string, std::string> & custom_data)
{
  std::vector<std::pair<std::string, std::string>> sorted_data(
    custom_data.begin(), custom_data.end());
  std::sort(sorted_data.begin(), sorted_data.end());

  const auto manifest_path = bag_path / "muto_recording_manifest.json";
  std::ofstream manifest(manifest_path, std::ios::out | std::ios::trunc);
  if (!manifest.is_open()) {
    throw std::runtime_error(
            "could not open recording manifest: " + manifest_path.string());
  }

  manifest << "{\n";
  for (std::size_t index = 0U; index < sorted_data.size(); ++index) {
    manifest << "  \"" << json_escape(sorted_data[index].first) << "\": \"" <<
      json_escape(sorted_data[index].second) << "\"";
    manifest << (index + 1U == sorted_data.size() ? "\n" : ",\n");
  }
  manifest << "}\n";
  manifest.flush();
  if (!manifest.good()) {
    throw std::runtime_error(
            "could not write recording manifest: " + manifest_path.string());
  }
}

}  // namespace

class ExplorationBagRecorderNode : public rclcpp::Node
{
public:
  ExplorationBagRecorderNode()
  : Node("exploration_bag_recorder")
  {
    output_directory_ =
      declare_parameter<std::string>("output_directory", "");
    storage_id_ = declare_parameter<std::string>("storage_id", "mcap");
    storage_preset_ =
      declare_parameter<std::string>("storage_preset", "none");
    topics_ = declare_parameter<std::vector<std::string>>(
      "topics", std::vector<std::string>{});
    topics_regex_ = declare_parameter<std::string>("topics_regex", "");
    exclude_regex_ = declare_parameter<std::string>("exclude_regex", "");
    max_cache_size_ =
      declare_parameter<int64_t>("max_cache_size", 104857600);
    post_terminal_delay_ =
      declare_parameter<double>("post_terminal_delay", 0.25);
    lifecycle_event_topic_ = declare_parameter<std::string>(
      "lifecycle_event_topic", "/explore_and_record/recording_event");
    status_topic_ = declare_parameter<std::string>(
      "status_topic", "/explore_and_record/bag_status");
    path_topic_ = declare_parameter<std::string>(
      "path_topic", "/explore_and_record/last_bag_path");
    operator_event_topic_ = declare_parameter<std::string>(
      "operator_event_topic", "/explore_and_record/operator_event");
    include_hidden_topics_ =
      declare_parameter<bool>("include_hidden_topics", true);
    record_all_services_ =
      declare_parameter<bool>("record_all_services", true);

    resolve_and_validate_parameters();
    transport_ =
      std::make_unique<muto_exploration_bag::ExplorationBagTransport>(
      get_node_base_interface()->get_context());

    const auto transient_qos =
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    status_publisher_ =
      create_publisher<std_msgs::msg::String>(status_topic_, transient_qos);
    path_publisher_ =
      create_publisher<std_msgs::msg::String>(path_topic_, transient_qos);
    // Keep the manual event topic advertised before a bag starts. Otherwise a
    // one-shot CLI publisher can send its message before rosbag2's periodic
    // topic discovery has time to create a subscription.
    operator_event_topic_anchor_ = create_publisher<std_msgs::msg::String>(
      operator_event_topic_, rclcpp::QoS(100).reliable().durability_volatile());
    lifecycle_subscription_ = create_subscription<std_msgs::msg::String>(
      lifecycle_event_topic_, transient_qos,
      std::bind(
        &ExplorationBagRecorderNode::handle_lifecycle_event, this,
        std::placeholders::_1));
    operator_event_subscription_ = create_subscription<std_msgs::msg::String>(
      operator_event_topic_, rclcpp::QoS(100).reliable().durability_volatile(),
      std::bind(
        &ExplorationBagRecorderNode::handle_operator_event, this,
        std::placeholders::_1));
    stop_timer_ = create_wall_timer(
      20ms, std::bind(&ExplorationBagRecorderNode::check_pending_stop, this));
    stop_timer_->cancel();

    RCLCPP_INFO(
      get_logger(),
      "Armed for %s; bags=%s operator_events=%s status=%s",
      lifecycle_event_topic_.c_str(), output_directory_.c_str(),
      operator_event_topic_.c_str(), status_topic_.c_str());
  }

  ~ExplorationBagRecorderNode() override
  {
    finalize_recording("node_shutdown");
  }

private:
  void resolve_and_validate_parameters()
  {
    if (output_directory_.empty()) {
      const char * home = std::getenv("HOME");
      if (home == nullptr || *home == '\0') {
        throw std::invalid_argument(
                "output_directory is empty and HOME is unavailable");
      }
      output_directory_ =
        (std::filesystem::path(home) / ".ros" / "bags" /
        "explore_and_record").string();
    }
    output_directory_ = std::filesystem::path(
      output_directory_).lexically_normal().string();
    if (!std::filesystem::path(output_directory_).is_absolute()) {
      throw std::invalid_argument("output_directory must be absolute");
    }
    if (storage_id_.empty() || storage_preset_.empty()) {
      throw std::invalid_argument(
              "storage_id and storage_preset must not be empty");
    }
    if (!topics_.empty() && !topics_regex_.empty()) {
      throw std::invalid_argument(
              "topics and topics_regex are mutually exclusive");
    }
    for (const auto & topic : topics_) {
      if (topic.empty() || topic.front() != '/') {
        throw std::invalid_argument(
                "topics entries must be absolute topic names");
      }
    }
    if (max_cache_size_ < 0) {
      throw std::invalid_argument("max_cache_size must be nonnegative");
    }
    if (!std::isfinite(post_terminal_delay_) || post_terminal_delay_ < 0.0) {
      throw std::invalid_argument(
              "post_terminal_delay must be finite and nonnegative");
    }
    if (lifecycle_event_topic_.empty() || status_topic_.empty() ||
      path_topic_.empty() || operator_event_topic_.empty())
    {
      throw std::invalid_argument("recorder topic names must not be empty");
    }
  }

  void handle_lifecycle_event(const std_msgs::msg::String::ConstSharedPtr message)
  {
    const auto event = json_string_field(message->data, "event");
    const auto goal_id = json_string_field(message->data, "goal_id");
    if (!event || !goal_id) {
      RCLCPP_WARN(
        get_logger(), "Ignoring malformed exploration lifecycle event: %s",
        message->data.c_str());
      return;
    }

    if (*event == kStartedEvent) {
      start_recording(*goal_id, message->data);
      return;
    }
    if (!is_terminal_event(*event)) {
      return;
    }
    if (!transport_->active()) {
      RCLCPP_DEBUG(
        get_logger(), "No active bag for terminal event %s (%s)",
        event->c_str(), goal_id->c_str());
      return;
    }
    if (*goal_id != active_goal_id_) {
      RCLCPP_WARN(
        get_logger(), "Ignoring terminal event for goal %s; recording goal %s",
        goal_id->c_str(), active_goal_id_.c_str());
      return;
    }

    pending_terminal_event_ = *event;
    stop_deadline_ = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(post_terminal_delay_));
    publish_status("recording_finishing", active_goal_id_, "");
    stop_timer_->reset();
  }

  void start_recording(
    const std::string & goal_id, const std::string & start_event)
  {
    if (transport_->active() && goal_id == active_goal_id_) {
      publish_status("recording_ready", active_goal_id_, "");
      return;
    }
    if (transport_->active()) {
      RCLCPP_WARN(
        get_logger(), "New exploration goal arrived before goal %s finalized",
        active_goal_id_.c_str());
      finalize_recording("superseded");
    }

    const auto bag_path = std::filesystem::path(output_directory_) /
      timestamped_bag_name(goal_id);
    bool use_sim_time = false;
    get_parameter("use_sim_time", use_sim_time);

    muto_exploration_bag::ExplorationBagOptions options;
    options.uri = bag_path.string();
    options.storage_id = storage_id_;
    options.storage_preset_profile = storage_preset_;
    options.topics = topics_;
    options.topics_regex = topics_regex_;
    options.exclude_regex = exclude_regex_;
    options.max_cache_size = static_cast<uint64_t>(max_cache_size_);
    options.include_hidden_topics = include_hidden_topics_;
    options.record_all_services = record_all_services_;
    options.use_sim_time = use_sim_time;
    const char * ros_distro = std::getenv("ROS_DISTRO");
    options.custom_data = {
      {"muto_schema", "explore_and_record_v1"},
      {"goal_id", goal_id},
      {"start_event", start_event},
      {"bag_path", bag_path.string()},
      {"git_revision", muto_exploration_bag_build::kGitRevision},
      {"git_dirty",
        muto_exploration_bag_build::kGitDirty ? "true" : "false"},
      {"ros_distro", ros_distro == nullptr ? "unknown" : ros_distro},
      {"topic_scope", topics_.empty() ?
        (topics_regex_.empty() ? "all_topics" : "regex") :
        "configured_topics"},
      {"operator_event_topic", operator_event_topic_},
      {"manifest_file", "muto_recording_manifest.json"},
    };

    try {
      transport_->start(options);
      write_recording_manifest(bag_path, options.custom_data);
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        get_logger(), "Could not prepare exploration bag for goal %s: %s",
        goal_id.c_str(), error.what());
      if (transport_->active()) {
        try {
          transport_->stop();
        } catch (const std::exception & stop_error) {
          RCLCPP_ERROR(
            get_logger(), "Could not close incomplete exploration bag: %s",
            stop_error.what());
        }
      }
      publish_status("recording_error", goal_id, error.what());
      return;
    }

    active_goal_id_ = goal_id;
    pending_terminal_event_.clear();
    std_msgs::msg::String path_message;
    path_message.data = transport_->path();
    path_publisher_->publish(path_message);
    publish_status("recording_ready", active_goal_id_, "");
    RCLCPP_INFO(
      get_logger(), "Recording exploration goal %s to %s",
      active_goal_id_.c_str(), transport_->path().c_str());
  }

  void handle_operator_event(const std_msgs::msg::String::ConstSharedPtr message)
  {
    if (transport_->active()) {
      RCLCPP_INFO(
        get_logger(), "Operator event recorded: %s", message->data.c_str());
    } else {
      RCLCPP_WARN(
        get_logger(), "Operator event was not recorded because no mission bag is active: %s",
        message->data.c_str());
    }
  }

  void check_pending_stop()
  {
    if (pending_terminal_event_.empty()) {
      stop_timer_->cancel();
      return;
    }
    if (std::chrono::steady_clock::now() < stop_deadline_) {
      return;
    }
    stop_timer_->cancel();
    const std::string terminal_event = pending_terminal_event_;
    pending_terminal_event_.clear();
    finalize_recording(terminal_event);
  }

  void finalize_recording(const std::string & reason)
  {
    if (!transport_ || !transport_->active()) {
      return;
    }

    const std::string finalized_goal = active_goal_id_;
    const std::string finalized_path = transport_->path();
    try {
      transport_->stop();
      RCLCPP_INFO(
        get_logger(), "Finalized exploration bag after %s: %s",
        reason.c_str(), finalized_path.c_str());
      publish_status("recording_finalized", finalized_goal, "");
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        get_logger(), "Could not finalize exploration bag %s: %s",
        finalized_path.c_str(), error.what());
      publish_status("recording_error", finalized_goal, error.what());
    }
    active_goal_id_.clear();
  }

  void publish_status(
    const std::string & event, const std::string & goal_id,
    const std::string & detail)
  {
    std_msgs::msg::String message;
    message.data =
      "{\"schema\":\"muto_exploration_bag_status_v1\",\"event\":\"" +
      json_escape(event) + "\",\"goal_id\":\"" + json_escape(goal_id) +
      "\",\"bag_path\":\"" +
      json_escape(transport_ ? transport_->path() : "") +
      "\",\"detail\":\"" + json_escape(detail) + "\"}";
    status_publisher_->publish(message);
  }

  std::unique_ptr<muto_exploration_bag::ExplorationBagTransport> transport_;
  std::string output_directory_;
  std::string storage_id_;
  std::string storage_preset_;
  std::vector<std::string> topics_;
  std::string topics_regex_;
  std::string exclude_regex_;
  int64_t max_cache_size_{104857600};
  double post_terminal_delay_{0.25};
  std::string lifecycle_event_topic_;
  std::string status_topic_;
  std::string path_topic_;
  std::string operator_event_topic_;
  bool include_hidden_topics_{true};
  bool record_all_services_{true};
  std::string active_goal_id_;
  std::string pending_terminal_event_;
  std::chrono::steady_clock::time_point stop_deadline_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr path_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
    operator_event_topic_anchor_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
    lifecycle_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
    operator_event_subscription_;
  rclcpp::TimerBase::SharedPtr stop_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<ExplorationBagRecorderNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("exploration_bag_recorder"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
