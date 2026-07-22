#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class CameraPointCloudToLaserScanNode : public rclcpp::Node
{
public:
  CameraPointCloudToLaserScanNode()
  : Node("camera_pointcloud_to_laserscan_node")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/camera/depth/points");
    output_topic_ = declare_parameter<std::string>("output_topic", "/camera/filtered_laserscan");
    processing_frame_ = declare_parameter<std::string>("processing_frame", "camera_link");

    min_z_ = declare_parameter<double>("min_z", -0.2);
    max_z_ = declare_parameter<double>("max_z", 0.05);
    camera_min_x_ = declare_parameter<double>("camera_min_x", -100.0);
    range_min_ = declare_parameter<double>("range_min", 0.05);
    range_max_ = declare_parameter<double>("range_max", 3.0);

    angle_min_ = declare_parameter<double>("angle_min", -M_PI);
    angle_max_ = declare_parameter<double>("angle_max", M_PI);
    angle_increment_ = declare_parameter<double>("angle_increment", M_PI / 720.0);
    scan_time_ = declare_parameter<double>("scan_time", 0.0);
    time_increment_ = declare_parameter<double>("time_increment", 0.0);

    queue_size_ = declare_parameter<int>("queue_size", 5);
    max_publish_rate_ = declare_parameter<double>("max_publish_rate", 0.0);
    input_point_stride_ = declare_parameter<int>("input_point_stride", 8);
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

    const auto input_qos =
      rclcpp::QoS(rclcpp::KeepLast(queue_size_)).best_effort().durability_volatile();
    const auto output_qos = rclcpp::QoS(rclcpp::KeepLast(queue_size_));
    publisher_ = create_publisher<sensor_msgs::msg::LaserScan>(output_topic_, output_qos);
    camera_subscriber_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, input_qos,
      std::bind(
        &CameraPointCloudToLaserScanNode::cameraPointCloudCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Converting %s -> %s in processing frame %s, z[%.3f, %.3f], camera_x>=%.3f, "
      "range[%.3f, %.3f], restamp_output=%s",
      input_topic_.c_str(), output_topic_.c_str(),
      processing_frame_.empty() ? "<input frame>" : processing_frame_.c_str(),
      min_z_, max_z_, camera_min_x_, range_min_, range_max_,
      restamp_output_ ? "true" : "false");
    if (max_publish_rate_ > 0.0 || input_point_stride_ > 1) {
      RCLCPP_INFO(
        get_logger(), "Cost controls: max_publish_rate=%.3f Hz, input_point_stride=%d",
        max_publish_rate_, input_point_stride_);
    }
  }

private:
  struct FilterStats
  {
    std::size_t kept{0};
    std::size_t invalid{0};
    std::size_t x_filtered{0};
    std::size_t z_filtered{0};
    std::size_t range_filtered{0};
    std::size_t angle_filtered{0};
  };

  void normalizeParameters()
  {
    if (queue_size_ < 1) {
      RCLCPP_WARN(get_logger(), "queue_size must be positive; using 1");
      queue_size_ = 1;
    }
    if (max_publish_rate_ < 0.0) {
      RCLCPP_WARN(get_logger(), "max_publish_rate must be non-negative; using 0.0");
      max_publish_rate_ = 0.0;
    }
    if (input_point_stride_ < 1) {
      RCLCPP_WARN(get_logger(), "input_point_stride must be positive; using 1");
      input_point_stride_ = 1;
    }
    if (min_z_ > max_z_) {
      RCLCPP_WARN(get_logger(), "min_z is greater than max_z; swapping them");
      std::swap(min_z_, max_z_);
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
    if (processing_frame_.empty()) {
      RCLCPP_WARN(get_logger(), "processing_frame is empty; using each input cloud frame");
    }
  }

  void cameraPointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    const rclcpp::Time now = get_clock()->now();
    if (shouldThrottle(now)) {
      return;
    }
    const auto processing_start = std::chrono::steady_clock::now();

    warnIfStampFarFromNow(*msg, "input");
    if (isStampTooFarFromNow(*msg, "input")) {
      return;
    }

    if (msg->header.frame_id.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Input cloud has an empty frame_id; dropping cloud");
      return;
    }

    const bool cloud_has_points =
      msg->width > 0U && msg->height > 0U && msg->point_step > 0U && !msg->data.empty();
    if (!cloud_has_points) {
      last_process_time_ = now;
      const std::string scan_frame =
        processing_frame_.empty() ? msg->header.frame_id : processing_frame_;

      sensor_msgs::msg::LaserScan scan;
      initializeScan(*msg, scan_frame, scan);

      FilterStats camera_stats;
      publisher_->publish(scan);
      warnIfProcessingWasSlow(processing_start, *msg, camera_stats);

      if (log_filter_stats_) {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Converted empty camera cloud to empty scan: input=%u bins=%zu",
          msg->width * msg->height, scan.ranges.size());
      }
      return;
    }

    if (!hasField(*msg, "x") || !hasField(*msg, "y") || !hasField(*msg, "z")) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Input cloud is missing x/y/z fields; dropping cloud");
      return;
    }
    last_process_time_ = now;

    const std::string scan_frame =
      processing_frame_.empty() ? msg->header.frame_id : processing_frame_;
    tf2::Transform camera_transform;
    if (!lookupCloudTransform(*msg, scan_frame, "camera", camera_transform)) {
      return;
    }

    sensor_msgs::msg::LaserScan scan;
    initializeScan(*msg, scan_frame, scan);

    FilterStats camera_stats;
    addCloudToScan(
      *msg, camera_transform, scan, camera_stats, camera_min_x_, range_min_, range_max_,
      input_point_stride_);

    publisher_->publish(scan);
    warnIfProcessingWasSlow(processing_start, *msg, camera_stats);

    if (log_filter_stats_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Converted camera cloud to scan: input=%u kept=%zu x_filtered=%zu "
        "z_filtered=%zu bins=%zu",
        msg->width * msg->height, camera_stats.kept, camera_stats.x_filtered,
        camera_stats.z_filtered, scan.ranges.size());
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

  void initializeScan(
    const sensor_msgs::msg::PointCloud2 & input_msg,
    const std::string & scan_frame,
    sensor_msgs::msg::LaserScan & scan)
  {
    const auto bin_count = static_cast<std::size_t>(
      std::floor((angle_max_ - angle_min_) / angle_increment_)) + 1U;

    scan.header = input_msg.header;
    scan.header.frame_id = scan_frame;
    if (restamp_output_) {
      scan.header.stamp = get_clock()->now();
    }
    scan.angle_min = static_cast<float>(angle_min_);
    scan.angle_max = static_cast<float>(angle_min_ + (bin_count - 1U) * angle_increment_);
    scan.angle_increment = static_cast<float>(angle_increment_);
    scan.time_increment = static_cast<float>(time_increment_);
    scan.scan_time = static_cast<float>(scan_time_);
    scan.range_min = static_cast<float>(range_min_);
    scan.range_max = static_cast<float>(range_max_);
    scan.ranges.assign(bin_count, std::numeric_limits<float>::infinity());
    scan.intensities.clear();
  }

  bool lookupCloudTransform(
    const sensor_msgs::msg::PointCloud2 & msg,
    const std::string & target_frame,
    const char * cloud_name,
    tf2::Transform & transform)
  {
    if (msg.header.frame_id == target_frame) {
      transform.setIdentity();
      return true;
    }

    try {
      const geometry_msgs::msg::TransformStamped transform_msg =
        tf_buffer_->lookupTransform(
        target_frame, msg.header.frame_id, msg.header.stamp, transform_timeout_);
      setTfTransform(transform_msg, transform);
      return true;
    } catch (const tf2::TransformException & ex) {
      const std::string stamped_error = ex.what();
      try {
        const geometry_msgs::msg::TransformStamped transform_msg =
          tf_buffer_->lookupTransform(
          target_frame, msg.header.frame_id, rclcpp::Time(0, 0, get_clock()->get_clock_type()),
          transform_timeout_);
        setTfTransform(transform_msg, transform);
        return true;
      } catch (const tf2::TransformException & latest_ex) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Failed to transform %s cloud from %s to %s. stamped lookup: %s; latest lookup: %s",
          cloud_name, msg.header.frame_id.c_str(), target_frame.c_str(), stamped_error.c_str(),
          latest_ex.what());
        return false;
      }
    }
  }

  void setTfTransform(
    const geometry_msgs::msg::TransformStamped & transform_msg,
    tf2::Transform & transform) const
  {
    const auto & translation = transform_msg.transform.translation;
    const auto & rotation = transform_msg.transform.rotation;
    transform.setOrigin(tf2::Vector3(translation.x, translation.y, translation.z));
    transform.setRotation(tf2::Quaternion(rotation.x, rotation.y, rotation.z, rotation.w));
  }

  void addCloudToScan(
    const sensor_msgs::msg::PointCloud2 & msg,
    const tf2::Transform & transform,
    sensor_msgs::msg::LaserScan & scan,
    FilterStats & stats,
    const double min_x,
    const double range_min,
    const double range_max,
    const int point_stride)
  {
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(msg, "z");

    std::size_t point_index = 0;
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++point_index) {
      if (point_stride > 1 && point_index % static_cast<std::size_t>(point_stride) != 0U) {
        continue;
      }

      if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) || !std::isfinite(*iter_z)) {
        ++stats.invalid;
        continue;
      }

      const tf2::Vector3 transformed_point =
        transform * tf2::Vector3(*iter_x, *iter_y, *iter_z);
      const double x = transformed_point.x();
      const double y = transformed_point.y();
      const double z = transformed_point.z();

      if (x < min_x) {
        ++stats.x_filtered;
        continue;
      }

      if (z < min_z_ || z > max_z_) {
        ++stats.z_filtered;
        continue;
      }

      const double range = std::hypot(x, y);
      if (range < range_min || range > range_max) {
        ++stats.range_filtered;
        continue;
      }

      const double angle = std::atan2(y, x);
      if (angle < angle_min_ || angle > angle_max_) {
        ++stats.angle_filtered;
        continue;
      }

      const auto index = static_cast<std::size_t>(
        std::floor((angle - angle_min_) / angle_increment_));
      if (index >= scan.ranges.size()) {
        ++stats.angle_filtered;
        continue;
      }

      const float range_f = static_cast<float>(range);
      if (range_f < scan.ranges[index]) {
        scan.ranges[index] = range_f;
      }
      ++stats.kept;
    }
  }

  bool hasField(const sensor_msgs::msg::PointCloud2 & msg, const std::string & name) const
  {
    return std::any_of(
      msg.fields.begin(), msg.fields.end(),
      [&name](const sensor_msgs::msg::PointField & field) {
        return field.name == name;
      });
  }

  void warnIfProcessingWasSlow(
    const std::chrono::steady_clock::time_point & processing_start,
    const sensor_msgs::msg::PointCloud2 & input_msg,
    const FilterStats & camera_stats)
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
      "Camera scan conversion took %.3f s for %u input points; kept=%zu",
      elapsed_s, input_msg.width * input_msg.height, camera_stats.kept);
  }

  void warnIfStampFarFromNow(const sensor_msgs::msg::PointCloud2 & msg, const char * cloud_name)
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
      "%s cloud stamp is %.3f seconds from this node clock; restamp_output=%s",
      cloud_name, age, restamp_output_ ? "true" : "false");
  }

  bool isStampTooFarFromNow(const sensor_msgs::msg::PointCloud2 & msg, const char * cloud_name)
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
      "Dropping %s cloud because stamp is %.3f seconds from this node clock; "
      "max_input_age=%.3f. Set max_input_age:=0.0 only if this driver has bad stamps "
      "and the data is known to be fresh.",
      cloud_name, age, max_input_age_);
    return true;
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string processing_frame_;
  double min_z_;
  double max_z_;
  double camera_min_x_;
  double range_min_;
  double range_max_;
  double angle_min_;
  double angle_max_;
  double angle_increment_;
  double scan_time_;
  double time_increment_;
  int queue_size_;
  double max_publish_rate_;
  int input_point_stride_;
  bool restamp_output_;
  double input_stamp_warning_age_;
  double max_input_age_;
  double processing_time_warning_;
  rclcpp::Duration transform_timeout_{0, 0};
  bool log_filter_stats_;
  rclcpp::Time last_process_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr camera_subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraPointCloudToLaserScanNode>());
  rclcpp::shutdown();
  return 0;
}
