#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <string>

#include <Eigen/Geometry>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/exceptions.h"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class CameraPointCloudToLaserScanNode : public rclcpp::Node
{
public:
  CameraPointCloudToLaserScanNode()
  : Node("camera_pointcloud_to_laserscan_node")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/camera/depth/points");
    lidar_topic_ = declare_parameter<std::string>("lidar_topic", "lidar/PointCloud");
    output_topic_ = declare_parameter<std::string>("output_topic", "/camera/depth/scan");
    use_lidar_ = declare_parameter<bool>("use_lidar", true);

    min_z_ = declare_parameter<double>("min_z", -0.4);
    max_z_ = declare_parameter<double>("max_z", 0.2);
    range_min_ = declare_parameter<double>("range_min", 0.05);
    range_max_ = declare_parameter<double>("range_max", 3.0);

    angle_min_ = declare_parameter<double>("angle_min", -29.2 * M_PI / 180.0);
    angle_max_ = declare_parameter<double>("angle_max", 29.2 * M_PI / 180.0);
    angle_increment_ = declare_parameter<double>("angle_increment", M_PI / 720.0);
    scan_time_ = declare_parameter<double>("scan_time", 0.0);
    time_increment_ = declare_parameter<double>("time_increment", 0.0);

    queue_size_ = declare_parameter<int>("queue_size", 5);
    max_lidar_age_ = declare_parameter<double>("max_lidar_age", 0.5);
    transform_timeout_ = rclcpp::Duration::from_seconds(
      declare_parameter<double>("transform_timeout", 0.05));
    log_filter_stats_ = declare_parameter<bool>("log_filter_stats", false);

    normalizeParameters();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(queue_size_));
    publisher_ = create_publisher<sensor_msgs::msg::LaserScan>(output_topic_, qos);
    camera_subscriber_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, qos,
      std::bind(
        &CameraPointCloudToLaserScanNode::cameraPointCloudCallback, this, std::placeholders::_1));
    if (use_lidar_) {
      lidar_subscriber_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        lidar_topic_, qos,
        std::bind(
          &CameraPointCloudToLaserScanNode::lidarPointCloudCallback, this, std::placeholders::_1));
    }

    RCLCPP_INFO(
      get_logger(),
      "Converting %s -> %s in original cloud frame, z[%.3f, %.3f], range[%.3f, %.3f]",
      input_topic_.c_str(), output_topic_.c_str(), min_z_, max_z_, range_min_, range_max_);
    if (use_lidar_) {
      RCLCPP_INFO(
        get_logger(), "Merging latest %s into each scan with TF into the camera cloud frame",
        lidar_topic_.c_str());
    }
  }

private:
  struct FilterStats
  {
    std::size_t kept{0};
    std::size_t invalid{0};
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
    if (max_lidar_age_ < 0.0) {
      RCLCPP_WARN(get_logger(), "max_lidar_age must be non-negative; using 0.0");
      max_lidar_age_ = 0.0;
    }
  }

  void lidarPointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(latest_lidar_mutex_);
    latest_lidar_cloud_ = msg;
  }

  void cameraPointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (msg->header.frame_id.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Input cloud has an empty frame_id; dropping cloud");
      return;
    }
    if (!hasField(*msg, "x") || !hasField(*msg, "y") || !hasField(*msg, "z")) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Input cloud is missing x/y/z fields; dropping cloud");
      return;
    }

    const auto bin_count = static_cast<std::size_t>(
      std::floor((angle_max_ - angle_min_) / angle_increment_)) + 1U;

    sensor_msgs::msg::LaserScan scan;
    scan.header = msg->header;
    scan.angle_min = static_cast<float>(angle_min_);
    scan.angle_max = static_cast<float>(angle_min_ + (bin_count - 1U) * angle_increment_);
    scan.angle_increment = static_cast<float>(angle_increment_);
    scan.time_increment = static_cast<float>(time_increment_);
    scan.scan_time = static_cast<float>(scan_time_);
    scan.range_min = static_cast<float>(range_min_);
    scan.range_max = static_cast<float>(range_max_);
    scan.ranges.assign(bin_count, std::numeric_limits<float>::infinity());

    FilterStats camera_stats;
    addCloudToScan(*msg, Eigen::Isometry3d::Identity(), scan, camera_stats);

    FilterStats lidar_stats;
    const bool lidar_used = addLatestLidarCloudToScan(scan, *msg, lidar_stats);

    publisher_->publish(scan);

    if (log_filter_stats_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Converted clouds to scan: camera_input=%u camera_kept=%zu lidar_used=%s lidar_kept=%zu "
        "camera_z_filtered=%zu lidar_z_filtered=%zu bins=%zu",
        msg->width * msg->height, camera_stats.kept, lidar_used ? "true" : "false",
        lidar_stats.kept, camera_stats.z_filtered, lidar_stats.z_filtered, scan.ranges.size());
    }
  }

  bool addLatestLidarCloudToScan(
    sensor_msgs::msg::LaserScan & scan,
    const sensor_msgs::msg::PointCloud2 & camera_msg,
    FilterStats & stats)
  {
    if (!use_lidar_) {
      return false;
    }

    sensor_msgs::msg::PointCloud2::SharedPtr lidar_msg;
    {
      std::lock_guard<std::mutex> lock(latest_lidar_mutex_);
      lidar_msg = latest_lidar_cloud_;
    }

    if (!lidar_msg) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "No lidar cloud received yet on %s; publishing camera-only scan", lidar_topic_.c_str());
      return false;
    }
    if (lidar_msg->header.frame_id.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Latest lidar cloud has an empty frame_id; publishing camera-only scan");
      return false;
    }
    if (!hasField(*lidar_msg, "x") || !hasField(*lidar_msg, "y") || !hasField(*lidar_msg, "z")) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Latest lidar cloud is missing x/y/z fields; publishing camera-only scan");
      return false;
    }

    if (max_lidar_age_ > 0.0) {
      const double age = std::abs(
        (rclcpp::Time(camera_msg.header.stamp) - rclcpp::Time(lidar_msg->header.stamp)).seconds());
      if (age > max_lidar_age_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Latest lidar cloud is %.3f seconds from the camera cloud; publishing camera-only scan",
          age);
        return false;
      }
    }

    Eigen::Isometry3d lidar_to_scan_frame = Eigen::Isometry3d::Identity();
    if (lidar_msg->header.frame_id != scan.header.frame_id) {
      try {
        const geometry_msgs::msg::TransformStamped transform =
          tf_buffer_->lookupTransform(
          scan.header.frame_id, lidar_msg->header.frame_id, lidar_msg->header.stamp,
          transform_timeout_);
        lidar_to_scan_frame = tf2::transformToEigen(transform);
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Failed to transform lidar cloud from %s to %s: %s; publishing camera-only scan",
          lidar_msg->header.frame_id.c_str(), scan.header.frame_id.c_str(), ex.what());
        return false;
      }
    }

    addCloudToScan(*lidar_msg, lidar_to_scan_frame, scan, stats);
    return true;
  }

  void addCloudToScan(
    const sensor_msgs::msg::PointCloud2 & msg,
    const Eigen::Isometry3d & transform,
    sensor_msgs::msg::LaserScan & scan,
    FilterStats & stats)
  {
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(msg, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) || !std::isfinite(*iter_z)) {
        ++stats.invalid;
        continue;
      }

      const Eigen::Vector3d point =
        transform * Eigen::Vector3d(*iter_x, *iter_y, *iter_z);
      const double x = point.x();
      const double y = point.y();
      const double z = point.z();

      if (z < min_z_ || z > max_z_) {
        ++stats.z_filtered;
        continue;
      }

      const double range = std::hypot(x, y);
      if (range < range_min_ || range > range_max_) {
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

  std::string input_topic_;
  std::string lidar_topic_;
  std::string output_topic_;
  bool use_lidar_;
  double min_z_;
  double max_z_;
  double range_min_;
  double range_max_;
  double angle_min_;
  double angle_max_;
  double angle_increment_;
  double scan_time_;
  double time_increment_;
  int queue_size_;
  double max_lidar_age_;
  rclcpp::Duration transform_timeout_{0, 0};
  bool log_filter_stats_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr camera_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
  std::mutex latest_lidar_mutex_;
  sensor_msgs::msg::PointCloud2::SharedPtr latest_lidar_cloud_;
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
