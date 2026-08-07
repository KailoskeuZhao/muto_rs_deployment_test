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

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <filesystem>
#include <functional>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

#include "geometry_msgs/msg/twist.hpp"
#include "muto_hexapod_interfaces_custom/msg/commanded_gait_state.hpp"
#include "muto_hexapod_interfaces_custom/msg/motion_command_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rmw/rmw.h"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_storage/topic_metadata.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

#include "recording_build_info.hpp"

namespace
{

constexpr char kScanTopic[] = "/lidar/raw_laserscan";
constexpr char kScanType[] = "sensor_msgs/msg/LaserScan";
constexpr char kImuTopic[] = "/imu/data_processed";
constexpr char kImuType[] = "sensor_msgs/msg/Imu";
constexpr char kRawImuTopic[] = "/imu/data_raw";
constexpr char kRawImuType[] = "sensor_msgs/msg/Imu";
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
constexpr double kProductionMotorPollRateHz = 2.0;

std::string timestamped_bag_name()
{
  const auto now = std::chrono::system_clock::now();
  const std::time_t time = std::chrono::system_clock::to_time_t(now);
  std::tm local_time{};
  localtime_r(&time, &local_time);

  std::ostringstream name;
  name << "muto_odometry_" << std::put_time(&local_time, "%Y%m%d_%H%M%S");
  return name.str();
}

}  // namespace

class OdometryBagRecorder : public rclcpp::Node
{
public:
  OdometryBagRecorder()
  : Node("odometry_bag_recorder")
  {
    auto bag_path = declare_parameter<std::string>("bag_path", "");
    motor_service_name_ =
      declare_parameter<std::string>("motor_service_name", "get_motor_angles");
    motor_poll_rate_ = declare_parameter<double>("motor_poll_rate", 2.0);
    const bool allow_experimental_high_rate = declare_parameter<bool>(
      "allow_experimental_high_rate_motor_polling", false);

    if (bag_path.empty()) {
      bag_path = timestamped_bag_name();
    }
    if (motor_service_name_.empty()) {
      throw std::invalid_argument("motor_service_name must not be empty");
    }
    if (!std::isfinite(motor_poll_rate_) || motor_poll_rate_ <= 0.0) {
      throw std::invalid_argument("motor_poll_rate must be finite and positive");
    }
    if (motor_poll_rate_ > kProductionMotorPollRateHz &&
      !allow_experimental_high_rate)
    {
      throw std::invalid_argument(
              "motor_poll_rate exceeds the 2 Hz production limit; set "
              "allow_experimental_high_rate_motor_polling:=true only for a "
              "controlled hardware benchmark");
    }
    if (motor_poll_rate_ > kProductionMotorPollRateHz) {
      RCLCPP_WARN(
        get_logger(),
        "Experimental %.1f Hz motor recording exceeds the %.1f Hz production "
        "limit. The 2026-08-05 10 Hz test produced approximately 40 ms p95 "
        "gait and IMU intervals",
        motor_poll_rate_, kProductionMotorPollRateHz);
    }

    const auto absolute_path = std::filesystem::absolute(bag_path);
    if (std::filesystem::exists(absolute_path)) {
      throw std::runtime_error(
              "bag_path already exists: " + absolute_path.string());
    }
    if (absolute_path.has_parent_path()) {
      std::filesystem::create_directories(absolute_path.parent_path());
    }

    writer_ = std::make_unique<rosbag2_cpp::Writer>();
    writer_->open(absolute_path.string());
    register_topic(kScanTopic, kScanType);
    register_topic(kImuTopic, kImuType);
    register_topic(kRawImuTopic, kRawImuType);
    register_topic(kGaitTopic, kGaitType);
    register_topic(kMotionCommandTopic, kMotionCommandType);
    register_topic(kCmdVelTopic, kCmdVelType);
    register_topic(kMotorTopic, kMotorType);
    register_topic(kEventTopic, kEventType);
    register_topic(kMetadataTopic, kMetadataType);
    register_topic(kTfStaticTopic, kTfStaticType);

    const auto scan_qos =
      rclcpp::QoS(rclcpp::KeepLast(20)).best_effort().durability_volatile();
    const auto reliable_qos =
      rclcpp::QoS(rclcpp::KeepLast(100)).reliable().durability_volatile();
    const auto gait_qos =
      rclcpp::QoS(rclcpp::KeepLast(100)).reliable().durability_volatile();
    const auto motor_qos =
      rclcpp::QoS(rclcpp::KeepLast(20)).reliable().durability_volatile();
    const auto static_tf_qos =
      rclcpp::QoS(rclcpp::KeepLast(100)).reliable().transient_local();

    // Subscribe to latched static transforms before live sensor streams so
    // the source bag normally contains its frame geometry before its first
    // scan.
    tf_static_subscription_ = create_subscription<tf2_msgs::msg::TFMessage>(
      kTfStaticTopic,
      static_tf_qos,
      [this](std::shared_ptr<rclcpp::SerializedMessage> message) {
        write_serialized(std::move(message), kTfStaticTopic, kTfStaticType);
      });

    scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
      kScanTopic,
      scan_qos,
      [this](std::shared_ptr<rclcpp::SerializedMessage> message) {
        write_serialized(std::move(message), kScanTopic, kScanType);
      });
    imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      kImuTopic,
      reliable_qos,
      [this](std::shared_ptr<rclcpp::SerializedMessage> message) {
        write_serialized(std::move(message), kImuTopic, kImuType);
      });
    raw_imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      kRawImuTopic,
      reliable_qos,
      [this](std::shared_ptr<rclcpp::SerializedMessage> message) {
        write_serialized(std::move(message), kRawImuTopic, kRawImuType);
      });
    gait_subscription_ =
      create_subscription<muto_hexapod_interfaces_custom::msg::CommandedGaitState>(
      kGaitTopic,
      gait_qos,
      [this](std::shared_ptr<rclcpp::SerializedMessage> message) {
        write_serialized(std::move(message), kGaitTopic, kGaitType);
      });
    motion_command_subscription_ =
      create_subscription<muto_hexapod_interfaces_custom::msg::MotionCommandState>(
      kMotionCommandTopic,
      gait_qos,
      [this](std::shared_ptr<rclcpp::SerializedMessage> message) {
        write_serialized(
          std::move(message), kMotionCommandTopic, kMotionCommandType);
      });
    cmd_vel_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      kCmdVelTopic,
      reliable_qos,
      [this](std::shared_ptr<rclcpp::SerializedMessage> message) {
        write_serialized(std::move(message), kCmdVelTopic, kCmdVelType);
      });
    event_subscription_ = create_subscription<std_msgs::msg::String>(
      kEventTopic,
      reliable_qos,
      [this](std::shared_ptr<rclcpp::SerializedMessage> message) {
        write_serialized(std::move(message), kEventTopic, kEventType);
      });

    motor_publisher_ =
      create_publisher<std_msgs::msg::String>(kMotorTopic, motor_qos);
    metadata_publisher_ =
      create_publisher<std_msgs::msg::String>(kMetadataTopic, static_tf_qos);
    motor_client_ =
      create_client<std_srvs::srv::Trigger>(motor_service_name_);
    motor_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / motor_poll_rate_),
      std::bind(&OdometryBagRecorder::poll_motor_service, this));

    record_build_metadata();

    RCLCPP_INFO(
      get_logger(),
      "Recording Muto odometry source data to %s",
      absolute_path.c_str());
    RCLCPP_INFO(
      get_logger(),
      "Motor snapshots: %s -> %s at %.2f Hz",
      motor_service_name_.c_str(), kMotorTopic, motor_poll_rate_);
  }

  ~OdometryBagRecorder() override
  {
    if (writer_) {
      try {
        writer_->close();
      } catch (const std::exception & error) {
        RCLCPP_ERROR(get_logger(), "Failed to close bag: %s", error.what());
      }
    }
  }

private:
  void register_topic(const std::string & name, const std::string & type)
  {
    rosbag2_storage::TopicMetadata metadata;
    metadata.name = name;
    metadata.type = type;
    metadata.serialization_format = rmw_get_serialization_format();
    writer_->create_topic(metadata);
  }

  void write_serialized(
    std::shared_ptr<rclcpp::SerializedMessage> message,
    const std::string & topic,
    const std::string & type)
  {
    try {
      writer_->write(std::move(message), topic, type, now());
      ++message_count_;
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        get_logger(), "Failed writing %s: %s", topic.c_str(), error.what());
    }
  }

  void record_build_metadata()
  {
    std_msgs::msg::String metadata;
    std::ostringstream json;
    json << "{\"schema_version\":1,\"git_revision\":\""
         << muto_odometry_bag_build::kGitRevision
         << "\",\"git_dirty\":"
         << (muto_odometry_bag_build::kGitDirty ? "true" : "false")
         << ",\"tf_static_capture_enabled\":true}";
    metadata.data = json.str();
    const auto stamp = now();
    writer_->write(metadata, kMetadataTopic, stamp);
    ++message_count_;
    metadata_publisher_->publish(metadata);
    RCLCPP_INFO(
      get_logger(), "Recording build metadata: git=%s dirty=%s",
      muto_odometry_bag_build::kGitRevision,
      muto_odometry_bag_build::kGitDirty ? "true" : "false");
  }

  void poll_motor_service()
  {
    if (motor_request_pending_.exchange(true)) {
      return;
    }
    if (!motor_client_->service_is_ready()) {
      motor_request_pending_ = false;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Waiting for motor service %s", motor_service_name_.c_str());
      return;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    motor_client_->async_send_request(
      request,
      [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        motor_request_pending_ = false;
        std_srvs::srv::Trigger::Response::SharedPtr response;
        try {
          response = future.get();
        } catch (const std::exception & error) {
          RCLCPP_WARN(
            get_logger(), "Motor service request failed: %s", error.what());
          return;
        }

        if (!response || !response->success || response->message.empty()) {
          const std::string detail = response ? response->message : "no response";
          RCLCPP_WARN(
            get_logger(), "Motor snapshot unavailable: %s", detail.c_str());
          return;
        }

        std_msgs::msg::String snapshot;
        snapshot.data = response->message;
        const auto stamp = now();
        try {
          writer_->write(snapshot, kMotorTopic, stamp);
          ++message_count_;
          motor_publisher_->publish(snapshot);
        } catch (const std::exception & error) {
          RCLCPP_ERROR(
            get_logger(), "Failed writing motor snapshot: %s", error.what());
        }
      });
  }

  std::unique_ptr<rosbag2_cpp::Writer> writer_;
  std::atomic<bool> motor_request_pending_{false};
  std::atomic<std::uint64_t> message_count_{0};
  std::string motor_service_name_;
  double motor_poll_rate_{2.0};

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr raw_imu_subscription_;
  rclcpp::Subscription<
    muto_hexapod_interfaces_custom::msg::CommandedGaitState>::SharedPtr
    gait_subscription_;
  rclcpp::Subscription<
    muto_hexapod_interfaces_custom::msg::MotionCommandState>::SharedPtr
    motion_command_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
    cmd_vel_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr event_subscription_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr
    tf_static_subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr motor_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr metadata_publisher_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr motor_client_;
  rclcpp::TimerBase::SharedPtr motor_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<OdometryBagRecorder>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("odometry_bag_recorder"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
