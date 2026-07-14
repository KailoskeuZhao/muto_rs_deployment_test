#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class LaserScanFusionNode : public rclcpp::Node
{
public:
  LaserScanFusionNode()
  : Node("laserscan_fusion_node")
  {
    camera_scan_topic_ = declare_parameter<std::string>(
      "camera_scan_topic", "/camera/filtered_laserscan");
    lidar_scan_topic_ = declare_parameter<std::string>(
      "lidar_scan_topic", "/lidar/filtered_laserscan_no_downsample");
    output_topic_ = declare_parameter<std::string>("output_topic", "/fused/laserscan");
    output_frame_ = declare_parameter<std::string>("output_frame", "base_frame");

    range_min_ = declare_parameter<double>("range_min", 0.05);
    range_max_ = declare_parameter<double>("range_max", 15.0);
    angle_min_ = declare_parameter<double>("angle_min", -M_PI);
    angle_max_ = declare_parameter<double>("angle_max", M_PI);
    angle_increment_ = declare_parameter<double>("angle_increment", M_PI / 720.0);
    scan_time_ = declare_parameter<double>("scan_time", 0.0);
    time_increment_ = declare_parameter<double>("time_increment", 0.0);

    queue_size_ = declare_parameter<int>("queue_size", 1);
    max_lidar_age_ = declare_parameter<double>("max_lidar_age", 0.5);
    max_publish_rate_ = declare_parameter<double>("max_publish_rate", 0.0);
    require_lidar_ = declare_parameter<bool>("require_lidar", true);
    restamp_output_ = declare_parameter<bool>("restamp_output", false);
    input_stamp_warning_age_ = declare_parameter<double>("input_stamp_warning_age", 1.0);
    max_input_age_ = declare_parameter<double>("max_input_age", 2.0);
    processing_time_warning_ = declare_parameter<double>("processing_time_warning", 0.0);
    transform_timeout_ = rclcpp::Duration::from_seconds(
      declare_parameter<double>("transform_timeout", 0.05));
    log_filter_stats_ = declare_parameter<bool>("log_filter_stats", false);

    normalizeParameters();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    const auto scan_qos =
      rclcpp::QoS(rclcpp::KeepLast(queue_size_)).best_effort().durability_volatile();
    const auto output_qos = rclcpp::QoS(rclcpp::KeepLast(queue_size_));
    publisher_ = create_publisher<sensor_msgs::msg::LaserScan>(output_topic_, output_qos);

    camera_scan_subscriber_ = create_subscription<sensor_msgs::msg::LaserScan>(
      camera_scan_topic_, scan_qos,
      std::bind(&LaserScanFusionNode::cameraScanCallback, this, std::placeholders::_1));
    lidar_scan_subscriber_ = create_subscription<sensor_msgs::msg::LaserScan>(
      lidar_scan_topic_, scan_qos,
      std::bind(&LaserScanFusionNode::lidarScanCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Fusing camera scan %s with LiDAR scan %s -> %s in frame %s, range[%.3f, %.3f], "
      "angle[%.3f, %.3f], angle_increment=%.6f, require_lidar=%s",
      camera_scan_topic_.c_str(), lidar_scan_topic_.c_str(), output_topic_.c_str(),
      output_frame_.empty() ? "<camera scan frame>" : output_frame_.c_str(),
      range_min_, range_max_, angle_min_, angle_max_, angle_increment_,
      require_lidar_ ? "true" : "false");
  }

private:
  struct Transform3D
  {
    std::array<double, 9> rotation{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    double x{0.0};
    double y{0.0};
    double z{0.0};
  };

  struct FilterStats
  {
    std::size_t kept{0};
    std::size_t invalid{0};
    std::size_t range_filtered{0};
    std::size_t angle_filtered{0};
  };

  void normalizeParameters()
  {
    if (queue_size_ < 1) {
      RCLCPP_WARN(get_logger(), "queue_size must be positive; using 1");
      queue_size_ = 1;
    }
    if (range_min_ < 0.0) {
      RCLCPP_WARN(get_logger(), "range_min must be non-negative; using 0.0");
      range_min_ = 0.0;
    }
    if (range_max_ < range_min_) {
      RCLCPP_WARN(get_logger(), "range_max is smaller than range_min; swapping them");
      std::swap(range_max_, range_min_);
    }
    if (angle_max_ < angle_min_) {
      RCLCPP_WARN(get_logger(), "angle_max is smaller than angle_min; swapping them");
      std::swap(angle_max_, angle_min_);
    }
    if (angle_increment_ <= 0.0) {
      RCLCPP_WARN(get_logger(), "angle_increment must be positive; using 0.25 degrees");
      angle_increment_ = M_PI / 720.0;
    }
    if (max_lidar_age_ < 0.0) {
      RCLCPP_WARN(get_logger(), "max_lidar_age must be non-negative; using 0.0");
      max_lidar_age_ = 0.0;
    }
    if (max_publish_rate_ < 0.0) {
      RCLCPP_WARN(get_logger(), "max_publish_rate must be non-negative; using 0.0");
      max_publish_rate_ = 0.0;
    }
    if (input_stamp_warning_age_ < 0.0) {
      RCLCPP_WARN(get_logger(), "input_stamp_warning_age must be non-negative; using 0.0");
      input_stamp_warning_age_ = 0.0;
    }
    if (max_input_age_ < 0.0) {
      RCLCPP_WARN(get_logger(), "max_input_age must be non-negative; using 0.0");
      max_input_age_ = 0.0;
    }
    if (processing_time_warning_ < 0.0) {
      RCLCPP_WARN(get_logger(), "processing_time_warning must be non-negative; using 0.0");
      processing_time_warning_ = 0.0;
    }
  }

  void lidarScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    warnIfStampFarFromNow(*msg, "lidar");
    if (isStampTooFarFromNow(*msg, "lidar")) {
      return;
    }
    if (msg->header.frame_id.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "LiDAR LaserScan has an empty frame_id; waiting before fusion");
      return;
    }

    if (!received_lidar_scan_) {
      received_lidar_scan_ = true;
      RCLCPP_INFO(
        get_logger(), "Received first LiDAR scan on %s with frame_id '%s'",
        lidar_scan_topic_.c_str(), msg->header.frame_id.c_str());
    }

    std::lock_guard<std::mutex> lock(latest_lidar_mutex_);
    latest_lidar_scan_ = msg;
  }

  void cameraScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    const rclcpp::Time now = get_clock()->now();
    if (shouldThrottle(now)) {
      return;
    }
    const auto processing_start = std::chrono::steady_clock::now();

    warnIfStampFarFromNow(*msg, "camera");
    if (isStampTooFarFromNow(*msg, "camera")) {
      return;
    }
    if (msg->header.frame_id.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Camera LaserScan has an empty frame_id; dropping scan");
      return;
    }
    if (msg->ranges.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Camera LaserScan has no ranges; dropping scan");
      return;
    }
    last_process_time_ = now;

    sensor_msgs::msg::LaserScan::SharedPtr lidar_msg;
    {
      std::lock_guard<std::mutex> lock(latest_lidar_mutex_);
      lidar_msg = latest_lidar_scan_;
    }

    if (require_lidar_ && !lidar_msg) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "No LiDAR LaserScan received yet on %s; waiting before publishing fused scan",
        lidar_scan_topic_.c_str());
      return;
    }
    if (lidar_msg && max_lidar_age_ > 0.0) {
      const double age = std::abs(
        (rclcpp::Time(msg->header.stamp) - rclcpp::Time(lidar_msg->header.stamp)).seconds());
      if (age > max_lidar_age_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Latest LiDAR LaserScan is %.3f seconds from the camera scan; waiting before fusion",
          age);
        return;
      }
    }

    sensor_msgs::msg::LaserScan output_scan;
    initializeOutputScan(*msg, output_scan);

    FilterStats camera_stats;
    if (!addScanToOutput(*msg, output_scan, "camera", camera_stats)) {
      return;
    }

    FilterStats lidar_stats;
    bool lidar_used = false;
    if (lidar_msg) {
      lidar_used = addScanToOutput(*lidar_msg, output_scan, "lidar", lidar_stats);
      if (require_lidar_ && !lidar_used) {
        return;
      }
    }

    publisher_->publish(output_scan);
    warnIfProcessingWasSlow(processing_start, *msg, camera_stats, lidar_stats, lidar_used);

    if (log_filter_stats_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Fused scans: camera_kept=%zu lidar_used=%s lidar_kept=%zu bins=%zu",
        camera_stats.kept, lidar_used ? "true" : "false", lidar_stats.kept,
        output_scan.ranges.size());
    }
  }

  bool shouldThrottle(const rclcpp::Time & now) const
  {
    if (max_publish_rate_ <= 0.0) {
      return false;
    }
    if (last_process_time_.nanoseconds() == 0) {
      return false;
    }

    const double elapsed = (now - last_process_time_).seconds();
    return elapsed >= 0.0 && elapsed < 1.0 / max_publish_rate_;
  }

  void initializeOutputScan(
    const sensor_msgs::msg::LaserScan & camera_msg,
    sensor_msgs::msg::LaserScan & output_msg) const
  {
    const auto bin_count = static_cast<std::size_t>(
      std::floor((angle_max_ - angle_min_) / angle_increment_)) + 1U;

    output_msg.header = camera_msg.header;
    output_msg.header.frame_id = output_frame_.empty() ? camera_msg.header.frame_id : output_frame_;
    if (restamp_output_) {
      output_msg.header.stamp = get_clock()->now();
    }
    output_msg.angle_min = static_cast<float>(angle_min_);
    output_msg.angle_increment = static_cast<float>(angle_increment_);
    output_msg.angle_max = static_cast<float>(angle_min_ + (bin_count - 1U) * angle_increment_);
    output_msg.time_increment = static_cast<float>(time_increment_);
    output_msg.scan_time = static_cast<float>(scan_time_);
    output_msg.range_min = static_cast<float>(range_min_);
    output_msg.range_max = static_cast<float>(range_max_);
    output_msg.ranges.assign(bin_count, std::numeric_limits<float>::infinity());
    output_msg.intensities.clear();
  }

  bool addScanToOutput(
    const sensor_msgs::msg::LaserScan & input_scan,
    sensor_msgs::msg::LaserScan & output_scan,
    const char * scan_name,
    FilterStats & stats)
  {
    if (input_scan.angle_increment <= 0.0F) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "%s LaserScan angle_increment must be positive; skipping scan", scan_name);
      return false;
    }

    Transform3D transform;
    if (!lookupTransform(
        input_scan.header.frame_id, output_scan.header.frame_id, input_scan.header.stamp,
        scan_name, transform))
    {
      return false;
    }

    for (std::size_t input_index = 0; input_index < input_scan.ranges.size(); ++input_index) {
      const float input_range = input_scan.ranges[input_index];
      if (!std::isfinite(input_range)) {
        ++stats.invalid;
        continue;
      }
      if (input_range < input_scan.range_min || input_range > input_scan.range_max) {
        ++stats.range_filtered;
        continue;
      }

      const double source_angle =
        input_scan.angle_min + static_cast<double>(input_index) * input_scan.angle_increment;
      const double source_x = static_cast<double>(input_range) * std::cos(source_angle);
      const double source_y = static_cast<double>(input_range) * std::sin(source_angle);

      const double target_x =
        transform.x + transform.rotation[0] * source_x + transform.rotation[1] * source_y;
      const double target_y =
        transform.y + transform.rotation[3] * source_x + transform.rotation[4] * source_y;

      const double target_range = std::hypot(target_x, target_y);
      if (target_range < range_min_ || target_range > range_max_) {
        ++stats.range_filtered;
        continue;
      }

      const double target_angle = std::atan2(target_y, target_x);
      if (target_angle < angle_min_ || target_angle > angle_max_) {
        ++stats.angle_filtered;
        continue;
      }

      const auto output_index = static_cast<std::int64_t>(
        std::floor((target_angle - angle_min_) / angle_increment_));
      if (output_index < 0 ||
        static_cast<std::size_t>(output_index) >= output_scan.ranges.size())
      {
        ++stats.angle_filtered;
        continue;
      }

      auto & output_range = output_scan.ranges[static_cast<std::size_t>(output_index)];
      if (!std::isfinite(output_range) || target_range < output_range) {
        output_range = static_cast<float>(target_range);
      }
      ++stats.kept;
    }

    return true;
  }

  bool lookupTransform(
    const std::string & source_frame,
    const std::string & target_frame,
    const builtin_interfaces::msg::Time & stamp,
    const char * scan_name,
    Transform3D & transform)
  {
    if (source_frame == target_frame) {
      transform = Transform3D{};
      return true;
    }

    try {
      const geometry_msgs::msg::TransformStamped transform_msg =
        tf_buffer_->lookupTransform(target_frame, source_frame, stamp, transform_timeout_);
      setTransform(transform_msg, transform);
      return true;
    } catch (const tf2::TransformException & stamped_ex) {
      try {
        const geometry_msgs::msg::TransformStamped transform_msg =
          tf_buffer_->lookupTransform(
          target_frame, source_frame, rclcpp::Time(0, 0, get_clock()->get_clock_type()),
          transform_timeout_);
        setTransform(transform_msg, transform);
        return true;
      } catch (const tf2::TransformException & latest_ex) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Failed to transform %s scan from %s to %s. stamped lookup: %s; latest lookup: %s",
          scan_name, source_frame.c_str(), target_frame.c_str(), stamped_ex.what(),
          latest_ex.what());
        return false;
      }
    }
  }

  void setTransform(
    const geometry_msgs::msg::TransformStamped & transform_msg,
    Transform3D & transform) const
  {
    const auto & translation = transform_msg.transform.translation;
    const auto & rotation = transform_msg.transform.rotation;
    const tf2::Quaternion quaternion(rotation.x, rotation.y, rotation.z, rotation.w);
    const tf2::Matrix3x3 matrix(quaternion);

    transform.x = translation.x;
    transform.y = translation.y;
    transform.z = translation.z;
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        transform.rotation[static_cast<std::size_t>(row * 3 + col)] = matrix[row][col];
      }
    }
  }

  void warnIfProcessingWasSlow(
    const std::chrono::steady_clock::time_point & processing_start,
    const sensor_msgs::msg::LaserScan & camera_msg,
    const FilterStats & camera_stats,
    const FilterStats & lidar_stats,
    bool lidar_used)
  {
    if (processing_time_warning_ <= 0.0) {
      return;
    }

    const auto elapsed = std::chrono::steady_clock::now() - processing_start;
    const double elapsed_s = std::chrono::duration<double>(elapsed).count();
    if (elapsed_s <= processing_time_warning_) {
      return;
    }

    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "LaserScan fusion took %.3f s for %zu camera bins; kept camera=%zu lidar_used=%s "
      "lidar=%zu",
      elapsed_s, camera_msg.ranges.size(), camera_stats.kept, lidar_used ? "true" : "false",
      lidar_stats.kept);
  }

  void warnIfStampFarFromNow(const sensor_msgs::msg::LaserScan & msg, const char * scan_name)
  {
    if (input_stamp_warning_age_ <= 0.0) {
      return;
    }

    const rclcpp::Time now = get_clock()->now();
    const rclcpp::Time stamp(msg.header.stamp, get_clock()->get_clock_type());
    const double age = (now - stamp).seconds();
    if (std::fabs(age) <= input_stamp_warning_age_) {
      return;
    }

    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "%s LaserScan stamp is %.3f seconds from this node clock; restamp_output=%s",
      scan_name, age, restamp_output_ ? "true" : "false");
  }

  bool isStampTooFarFromNow(const sensor_msgs::msg::LaserScan & msg, const char * scan_name)
  {
    if (max_input_age_ <= 0.0) {
      return false;
    }

    const rclcpp::Time now = get_clock()->now();
    const rclcpp::Time stamp(msg.header.stamp, get_clock()->get_clock_type());
    const double age = (now - stamp).seconds();
    if (std::fabs(age) <= max_input_age_) {
      return false;
    }

    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Dropping %s LaserScan because stamp is %.3f seconds from this node clock; "
      "max_input_age=%.3f",
      scan_name, age, max_input_age_);
    return true;
  }

  std::string camera_scan_topic_;
  std::string lidar_scan_topic_;
  std::string output_topic_;
  std::string output_frame_;
  double range_min_;
  double range_max_;
  double angle_min_;
  double angle_max_;
  double angle_increment_;
  double scan_time_;
  double time_increment_;
  int queue_size_;
  double max_lidar_age_;
  double max_publish_rate_;
  bool require_lidar_;
  bool restamp_output_;
  double input_stamp_warning_age_;
  double max_input_age_;
  double processing_time_warning_;
  rclcpp::Duration transform_timeout_{0, 0};
  bool log_filter_stats_;
  bool received_lidar_scan_{false};
  rclcpp::Time last_process_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr camera_scan_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_scan_subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
  std::mutex latest_lidar_mutex_;
  sensor_msgs::msg::LaserScan::SharedPtr latest_lidar_scan_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaserScanFusionNode>());
  rclcpp::shutdown();
  return 0;
}
