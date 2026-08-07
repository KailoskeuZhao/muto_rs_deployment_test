// Copyright 2026 kailoskeuzhao
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <filesystem>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
#include <utility>

#include "geometry_msgs/msg/twist.hpp"
#include "muto_hexapod_interfaces_custom/msg/commanded_gait_state.hpp"
#include "muto_hexapod_interfaces_custom/msg/controller_attitude.hpp"
#include "muto_hexapod_interfaces_custom/msg/motion_command_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_storage/serialized_bag_message.hpp"
#include "rosbag2_storage/storage_options.hpp"
#include "rosbag2_transport/reader_writer_factory.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

namespace
{

constexpr char kScanTopic[] = "/lidar/raw_laserscan";
constexpr char kScanType[] = "sensor_msgs/msg/LaserScan";
constexpr char kImuTopic[] = "/imu/data_processed";
constexpr char kImuType[] = "sensor_msgs/msg/Imu";
constexpr char kRawImuTopic[] = "/imu/data_raw";
constexpr char kRawImuType[] = "sensor_msgs/msg/Imu";
constexpr char kControllerAttitudeTopic[] = "/imu/controller_attitude";
constexpr char kControllerAttitudeType[] =
  "muto_hexapod_interfaces_custom/msg/ControllerAttitude";
constexpr char kImuTelemetryStatusTopic[] = "/muto/imu_telemetry_status";
constexpr char kImuTelemetryStatusType[] = "std_msgs/msg/String";
constexpr char kGaitTopic[] = "/muto/commanded_gait_state";
constexpr char kGaitType[] =
  "muto_hexapod_interfaces_custom/msg/CommandedGaitState";
constexpr char kMotionCommandTopic[] = "/muto/motion_command_state";
constexpr char kMotionCommandType[] =
  "muto_hexapod_interfaces_custom/msg/MotionCommandState";
constexpr char kCmdVelTopic[] = "/cmd_vel";
constexpr char kCmdVelType[] = "geometry_msgs/msg/Twist";
constexpr char kMotorTopic[] = "/muto/measured_motor_state";
constexpr char kMotorType[] = "std_msgs/msg/String";
constexpr char kEventTopic[] = "/muto/odometry_test_event";
constexpr char kEventType[] = "std_msgs/msg/String";
constexpr char kMetadataTopic[] = "/muto/odometry_recording_metadata";
constexpr char kMetadataType[] = "std_msgs/msg/String";
constexpr char kTfStaticTopic[] = "/tf_static";
constexpr char kTfStaticType[] = "tf2_msgs/msg/TFMessage";

template<typename MessageT, typename = void>
struct HasReceiveTimestamp : std::false_type {};

template<typename MessageT>
struct HasReceiveTimestamp<
  MessageT,
  std::void_t<decltype(std::declval<MessageT>().recv_timestamp)>>
  : std::true_type {};

template<typename MessageT>
rcutils_time_point_value_t bag_timestamp(const MessageT & message)
{
  if constexpr (HasReceiveTimestamp<MessageT>::value) {
    return message.recv_timestamp;
  } else {
    return message.time_stamp;
  }
}

}  // namespace

class OdometryBagReplayer : public rclcpp::Node
{
public:
  OdometryBagReplayer()
  : Node("odometry_bag_replayer")
  {
    const auto bag_path = declare_parameter<std::string>("bag_path", "");
    playback_rate_ = declare_parameter<double>("playback_rate", 1.0);
    minimum_start_delay_sec_ =
      declare_parameter<double>("minimum_start_delay_sec", 0.5);
    readiness_timeout_sec_ =
      declare_parameter<double>("readiness_timeout_sec", 30.0);
    require_foot_inputs_ =
      declare_parameter<bool>("require_foot_inputs", true);
    replay_processed_imu_ =
      declare_parameter<bool>("replay_processed_imu", true);
    replay_recorded_tf_static_ =
      declare_parameter<bool>("replay_recorded_tf_static", false);
    motor_service_name_ =
      declare_parameter<std::string>("motor_service_name", "get_motor_angles");

    if (bag_path.empty()) {
      throw std::invalid_argument("bag_path must identify a rosbag2 directory");
    }
    if (!std::filesystem::exists(bag_path)) {
      throw std::invalid_argument("bag_path does not exist: " + bag_path);
    }
    if (playback_rate_ <= 0.0) {
      throw std::invalid_argument("playback_rate must be positive");
    }
    if (minimum_start_delay_sec_ < 0.0 || readiness_timeout_sec_ <= 0.0) {
      throw std::invalid_argument(
              "start delay must be non-negative and readiness timeout positive");
    }
    if (motor_service_name_.empty()) {
      throw std::invalid_argument("motor_service_name must not be empty");
    }

    const auto scan_qos =
      rclcpp::QoS(rclcpp::KeepLast(20)).best_effort().durability_volatile();
    const auto reliable_qos =
      rclcpp::QoS(rclcpp::KeepLast(100)).reliable().durability_volatile();
    const auto gait_qos =
      rclcpp::QoS(rclcpp::KeepLast(100)).reliable().transient_local();
    const auto motor_qos =
      rclcpp::QoS(rclcpp::KeepLast(20)).reliable().durability_volatile();
    const auto clock_qos =
      rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();
    const auto telemetry_status_qos =
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    const auto retained_qos =
      rclcpp::QoS(rclcpp::KeepLast(100)).reliable().transient_local();

    scan_publisher_ =
      create_publisher<sensor_msgs::msg::LaserScan>(kScanTopic, scan_qos);
    imu_publisher_ =
      create_publisher<sensor_msgs::msg::Imu>(kImuTopic, reliable_qos);
    raw_imu_publisher_ =
      create_publisher<sensor_msgs::msg::Imu>(kRawImuTopic, reliable_qos);
    controller_attitude_publisher_ = create_publisher<
      muto_hexapod_interfaces_custom::msg::ControllerAttitude>(
      kControllerAttitudeTopic, reliable_qos);
    imu_telemetry_status_publisher_ =
      create_publisher<std_msgs::msg::String>(
      kImuTelemetryStatusTopic, telemetry_status_qos);
    gait_publisher_ =
      create_publisher<muto_hexapod_interfaces_custom::msg::CommandedGaitState>(
      kGaitTopic, gait_qos);
    motion_command_publisher_ =
      create_publisher<muto_hexapod_interfaces_custom::msg::MotionCommandState>(
      kMotionCommandTopic, gait_qos);
    cmd_vel_publisher_ =
      create_publisher<geometry_msgs::msg::Twist>(kCmdVelTopic, reliable_qos);
    motor_publisher_ =
      create_publisher<std_msgs::msg::String>(kMotorTopic, motor_qos);
    event_publisher_ =
      create_publisher<std_msgs::msg::String>(kEventTopic, retained_qos);
    metadata_publisher_ =
      create_publisher<std_msgs::msg::String>(kMetadataTopic, retained_qos);
    if (replay_recorded_tf_static_) {
      tf_static_publisher_ =
        create_publisher<tf2_msgs::msg::TFMessage>(kTfStaticTopic, retained_qos);
    }
    clock_publisher_ =
      create_publisher<rosgraph_msgs::msg::Clock>("/clock", clock_qos);

    motor_service_ = create_service<std_srvs::srv::Trigger>(
      motor_service_name_,
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
      {
        std::lock_guard<std::mutex> lock(motor_mutex_);
        if (latest_motor_payload_.empty()) {
          response->success = false;
          response->message =
          R"({"error":"replay_motor_sample_unavailable"})";
          return;
        }
        response->success = true;
        response->message = latest_motor_payload_;
      });

    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = std::filesystem::absolute(bag_path).string();
    reader_ =
      rosbag2_transport::ReaderWriterFactory::make_reader(storage_options);
    reader_->open(storage_options);
    bootstrap_timestamp_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
      reader_->get_metadata().starting_time.time_since_epoch()).count();
    validate_bag_topics();

    RCLCPP_INFO(
      get_logger(), "Loaded odometry source bag %s at %.2fx",
      storage_options.uri.c_str(), playback_rate_);
    playback_thread_ =
      std::thread(&OdometryBagReplayer::playback, this);
  }

  ~OdometryBagReplayer() override
  {
    stop_requested_ = true;
    wait_condition_.notify_all();
    if (playback_thread_.joinable()) {
      playback_thread_.join();
    }
  }

private:
  void validate_bag_topics()
  {
    std::map<std::string, std::string> topics;
    for (const auto & topic : reader_->get_all_topics_and_types()) {
      topics[topic.name] = topic.type;
    }
    std::map<std::string, std::uint64_t> message_counts;
    for (const auto & topic : reader_->get_metadata().topics_with_message_count) {
      message_counts[topic.topic_metadata.name] = topic.message_count;
    }

    require_topic(topics, message_counts, kScanTopic, kScanType);
    require_topic(topics, message_counts, kImuTopic, kImuType);
    if (require_foot_inputs_) {
      require_topic(topics, message_counts, kGaitTopic, kGaitType);
      require_topic(topics, message_counts, kMotorTopic, kMotorType);
    }

    const auto command = topics.find(kCmdVelTopic);
    if (command != topics.end() && command->second != kCmdVelType) {
      throw std::runtime_error(
              std::string(kCmdVelTopic) + " has type " + command->second +
              ", expected " + kCmdVelType);
    }
    if (command == topics.end() || message_counts[kCmdVelTopic] == 0) {
      RCLCPP_WARN(
        get_logger(),
        "%s has no samples; RF2O deadband replay will assume stationary",
        kCmdVelTopic);
    }

    const bool have_raw_imu = validate_optional_topic(
      topics, message_counts, kRawImuTopic, kRawImuType);
    if (!replay_processed_imu_ && !have_raw_imu) {
      throw std::runtime_error(
              "replay_processed_imu is false but /imu/data_raw is absent or empty");
    }
    validate_optional_topic(
      topics,
      message_counts,
      kControllerAttitudeTopic,
      kControllerAttitudeType);
    validate_optional_topic(
      topics,
      message_counts,
      kImuTelemetryStatusTopic,
      kImuTelemetryStatusType);
    validate_optional_topic(topics, message_counts, kEventTopic, kEventType);
    validate_optional_topic(topics, message_counts, kMetadataTopic, kMetadataType);
    validate_optional_topic(
      topics, message_counts, kMotionCommandTopic, kMotionCommandType);
    const bool have_recorded_tf = validate_optional_topic(
      topics, message_counts, kTfStaticTopic, kTfStaticType);
    if (replay_recorded_tf_static_ && !have_recorded_tf) {
      throw std::runtime_error(
              "replay_recorded_tf_static is true but /tf_static is absent or empty");
    }
  }

  static bool validate_optional_topic(
    const std::map<std::string, std::string> & topics,
    const std::map<std::string, std::uint64_t> & message_counts,
    const std::string & name,
    const std::string & type)
  {
    const auto found = topics.find(name);
    if (found == topics.end()) {
      return false;
    }
    if (found->second != type) {
      throw std::runtime_error(
              name + " has type " + found->second + ", expected " + type);
    }
    const auto count = message_counts.find(name);
    return count != message_counts.end() && count->second > 0;
  }

  static void require_topic(
    const std::map<std::string, std::string> & topics,
    const std::map<std::string, std::uint64_t> & message_counts,
    const std::string & name,
    const std::string & type)
  {
    const auto found = topics.find(name);
    if (found == topics.end()) {
      throw std::runtime_error("required bag topic is absent: " + name);
    }
    if (found->second != type) {
      throw std::runtime_error(
              name + " has type " + found->second + ", expected " + type);
    }
    const auto count = message_counts.find(name);
    if (count == message_counts.end() || count->second == 0) {
      throw std::runtime_error("required bag topic has no messages: " + name);
    }
  }

  bool subscribers_ready() const
  {
    const bool imu_ready = replay_processed_imu_ ?
      imu_publisher_->get_subscription_count() > 0 :
      raw_imu_publisher_->get_subscription_count() > 0;
    const bool base_ready =
      scan_publisher_->get_subscription_count() > 0 &&
      imu_ready &&
      cmd_vel_publisher_->get_subscription_count() > 0;
    return base_ready &&
           (!require_foot_inputs_ ||
           gait_publisher_->get_subscription_count() > 0);
  }

  bool wait_for_original_nodes()
  {
    using SteadyClock = std::chrono::steady_clock;
    const auto started = SteadyClock::now();
    const auto minimum_ready_time =
      started + std::chrono::duration_cast<SteadyClock::duration>(
      std::chrono::duration<double>(minimum_start_delay_sec_));
    const auto deadline =
      started + std::chrono::duration_cast<SteadyClock::duration>(
      std::chrono::duration<double>(readiness_timeout_sec_));

    std::unique_lock<std::mutex> lock(wait_mutex_);
    while (!stop_requested_) {
      publish_clock(bootstrap_timestamp_);
      const auto now = SteadyClock::now();
      if (now >= minimum_ready_time && subscribers_ready()) {
        RCLCPP_INFO(
          get_logger(),
          "Original LiDAR, IMU, command%s consumers are ready",
          require_foot_inputs_ ? ", and foot" : "");
        return true;
      }
      if (now >= deadline) {
        RCLCPP_ERROR(
          get_logger(),
          "Timed out waiting for original odometry node subscriptions");
        return false;
      }
      wait_condition_.wait_for(
        lock, std::chrono::milliseconds(100),
        [this]() {return stop_requested_.load();});
    }
    return false;
  }

  template<typename MessageT>
  MessageT deserialize(
    const rosbag2_storage::SerializedBagMessageSharedPtr & bag_message)
  {
    rclcpp::SerializedMessage serialized(*bag_message->serialized_data);
    rclcpp::Serialization<MessageT> serialization;
    MessageT message;
    serialization.deserialize_message(&serialized, &message);
    return message;
  }

  void publish_clock(rcutils_time_point_value_t timestamp)
  {
    rosgraph_msgs::msg::Clock clock;
    clock.clock = rclcpp::Time(timestamp, RCL_ROS_TIME);
    clock_publisher_->publish(clock);
  }

  void update_motor_snapshot(
    const rosbag2_storage::SerializedBagMessageSharedPtr & bag_message)
  {
    const auto snapshot = deserialize<std_msgs::msg::String>(bag_message);
    {
      std::lock_guard<std::mutex> lock(motor_mutex_);
      latest_motor_payload_ = snapshot.data;
    }
    motor_publisher_->publish(snapshot);
  }

  void publish_source_message(
    const rosbag2_storage::SerializedBagMessageSharedPtr & bag_message)
  {
    if (bag_message->topic_name == kScanTopic) {
      scan_publisher_->publish(
        deserialize<sensor_msgs::msg::LaserScan>(bag_message));
    } else if (bag_message->topic_name == kImuTopic) {
      if (replay_processed_imu_) {
        imu_publisher_->publish(
          deserialize<sensor_msgs::msg::Imu>(bag_message));
      }
    } else if (bag_message->topic_name == kRawImuTopic) {
      raw_imu_publisher_->publish(
        deserialize<sensor_msgs::msg::Imu>(bag_message));
    } else if (bag_message->topic_name == kControllerAttitudeTopic) {
      controller_attitude_publisher_->publish(
        deserialize<
          muto_hexapod_interfaces_custom::msg::ControllerAttitude>(
          bag_message));
    } else if (bag_message->topic_name == kImuTelemetryStatusTopic) {
      imu_telemetry_status_publisher_->publish(
        deserialize<std_msgs::msg::String>(bag_message));
    } else if (bag_message->topic_name == kGaitTopic) {
      gait_publisher_->publish(
        deserialize<muto_hexapod_interfaces_custom::msg::CommandedGaitState>(
          bag_message));
    } else if (bag_message->topic_name == kMotionCommandTopic) {
      motion_command_publisher_->publish(
        deserialize<muto_hexapod_interfaces_custom::msg::MotionCommandState>(
          bag_message));
    } else if (bag_message->topic_name == kCmdVelTopic) {
      cmd_vel_publisher_->publish(
        deserialize<geometry_msgs::msg::Twist>(bag_message));
    } else if (bag_message->topic_name == kMotorTopic) {
      update_motor_snapshot(bag_message);
    } else if (bag_message->topic_name == kEventTopic) {
      event_publisher_->publish(
        deserialize<std_msgs::msg::String>(bag_message));
    } else if (bag_message->topic_name == kMetadataTopic) {
      metadata_publisher_->publish(
        deserialize<std_msgs::msg::String>(bag_message));
    } else if (bag_message->topic_name == kTfStaticTopic) {
      if (replay_recorded_tf_static_) {
        tf_static_publisher_->publish(
          deserialize<tf2_msgs::msg::TFMessage>(bag_message));
      }
    }
  }

  bool wait_until(std::chrono::steady_clock::time_point deadline)
  {
    std::unique_lock<std::mutex> lock(wait_mutex_);
    return !wait_condition_.wait_until(
      lock, deadline, [this]() {return stop_requested_.load();});
  }

  void playback()
  {
    if (!wait_for_original_nodes()) {
      return;
    }

    bool have_first_timestamp = false;
    rcutils_time_point_value_t first_timestamp = 0;
    auto wall_start = std::chrono::steady_clock::now();
    std::uint64_t published_count = 0;
    bool recorded_tf_delivery_wait_done = false;

    try {
      while (!stop_requested_ && reader_->has_next()) {
        auto bag_message = reader_->read_next();
        const auto timestamp = bag_timestamp(*bag_message);
        if (timestamp < 0) {
          RCLCPP_WARN(
            get_logger(), "Skipping message with negative bag timestamp");
          continue;
        }
        if (!have_first_timestamp) {
          first_timestamp = timestamp;
          wall_start = std::chrono::steady_clock::now();
          have_first_timestamp = true;
        }

        const auto elapsed =
          std::max<rcutils_time_point_value_t>(
          0, timestamp - first_timestamp);
        const auto scaled_nanoseconds = static_cast<std::int64_t>(
          std::llround(static_cast<double>(elapsed) / playback_rate_));
        const auto deadline =
          wall_start + std::chrono::nanoseconds(scaled_nanoseconds);
        if (!wait_until(deadline)) {
          return;
        }

        if (bag_message->topic_name == kMotorTopic) {
          update_motor_snapshot(bag_message);
        }
        publish_clock(timestamp);
        if (published_count == 0) {
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        if (bag_message->topic_name != kMotorTopic) {
          publish_source_message(bag_message);
        }
        if (
          bag_message->topic_name == kTfStaticTopic &&
          replay_recorded_tf_static_ && !recorded_tf_delivery_wait_done)
        {
          const auto delivery_delay = std::chrono::milliseconds(50);
          std::this_thread::sleep_for(delivery_delay);
          wall_start += delivery_delay;
          recorded_tf_delivery_wait_done = true;
        }
        ++published_count;
      }
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Replay failed: %s", error.what());
      return;
    }

    RCLCPP_INFO(
      get_logger(), "Replay complete: %" PRIu64 " source messages published",
      published_count);
  }

  double playback_rate_{1.0};
  double minimum_start_delay_sec_{0.5};
  double readiness_timeout_sec_{30.0};
  bool require_foot_inputs_{true};
  bool replay_processed_imu_{true};
  bool replay_recorded_tf_static_{false};
  std::string motor_service_name_;
  rcutils_time_point_value_t bootstrap_timestamp_{0};

  std::unique_ptr<rosbag2_cpp::Reader> reader_;
  std::thread playback_thread_;
  std::atomic<bool> stop_requested_{false};
  std::condition_variable wait_condition_;
  std::mutex wait_mutex_;
  std::mutex motor_mutex_;
  std::string latest_motor_payload_;

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr raw_imu_publisher_;
  rclcpp::Publisher<
    muto_hexapod_interfaces_custom::msg::ControllerAttitude>::SharedPtr
    controller_attitude_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
    imu_telemetry_status_publisher_;
  rclcpp::Publisher<
    muto_hexapod_interfaces_custom::msg::CommandedGaitState>::SharedPtr
    gait_publisher_;
  rclcpp::Publisher<
    muto_hexapod_interfaces_custom::msg::MotionCommandState>::SharedPtr
    motion_command_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr motor_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr event_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr metadata_publisher_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_publisher_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr motor_service_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<OdometryBagReplayer>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("odometry_bag_replayer"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
