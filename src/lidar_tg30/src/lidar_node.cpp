#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <string>

#include "builtin_interfaces/msg/time.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "CYdLidar.h"

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kRadToDeg = 180.0 / kPi;
constexpr std::int64_t kNanosecondsPerSecond = 1000000000LL;

builtin_interfaces::msg::Time toBuiltinTime(const rclcpp::Time & time)
{
  builtin_interfaces::msg::Time stamp;
  const std::int64_t nanoseconds = time.nanoseconds();
  stamp.sec = static_cast<std::int32_t>(nanoseconds / kNanosecondsPerSecond);
  stamp.nanosec = static_cast<std::uint32_t>(nanoseconds % kNanosecondsPerSecond);
  return stamp;
}

void packLaserScanMsg(
  const LaserScan & scan,
  const std::string & frame_id,
  const builtin_interfaces::msg::Time & stamp,
  const double angle_min,
  const double angle_max,
  const double angle_increment,
  const double range_min,
  const double range_max,
  const double scan_time,
  sensor_msgs::msg::LaserScan & msg)
{
  const std::size_t bin_count = static_cast<std::size_t>(
    std::floor((angle_max - angle_min) / angle_increment)) + 1U;
  const float infinity = std::numeric_limits<float>::infinity();

  msg.header.frame_id = frame_id;
  msg.header.stamp = stamp;
  msg.angle_min = static_cast<float>(angle_min);
  msg.angle_increment = static_cast<float>(angle_increment);
  msg.angle_max = static_cast<float>(angle_min + (bin_count - 1U) * angle_increment);
  msg.time_increment = bin_count > 0U ? static_cast<float>(scan_time / bin_count) : 0.0F;
  msg.scan_time = static_cast<float>(scan_time);
  msg.range_min = static_cast<float>(range_min);
  msg.range_max = static_cast<float>(range_max);
  msg.ranges.assign(bin_count, infinity);

  for (const auto & point : scan.points) {
    const double range = point.range;
    if (!std::isfinite(range) || range < range_min || range > range_max) {
      continue;
    }

    const double bin_position = (point.angle - angle_min) / angle_increment;
    const auto index = static_cast<std::int64_t>(std::llround(bin_position));
    if (index < 0 || static_cast<std::size_t>(index) >= bin_count) {
      continue;
    }

    auto & bin_range = msg.ranges[static_cast<std::size_t>(index)];
    if (!std::isfinite(bin_range) || range < bin_range) {
      bin_range = static_cast<float>(range);
    }
  }
}
}  // namespace

class LidarNode : public rclcpp::Node
{
public:
  LidarNode()
  : Node("lidar_node")
  {
    frame_id_ = declare_parameter<std::string>("frame_id", "lidar_frame");
    scan_topic_ = declare_parameter<std::string>("scan_topic", "lidar/raw_laserscan");
    queue_size_ = declare_parameter<int>("queue_size", 5);
    range_min_ = declare_parameter<double>("range_min", 0.05);
    range_max_ = declare_parameter<double>("range_max", 64.0);
    angle_min_ = declare_parameter<double>("angle_min", -kPi);
    angle_max_ = declare_parameter<double>("angle_max", kPi);
    angle_increment_ = declare_parameter<double>("angle_increment", kPi / 720.0);
    scan_frequency_ = declare_parameter<double>("scan_frequency", 16.0);

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
    if (angle_max_ <= angle_min_) {
      RCLCPP_WARN(get_logger(), "angle_max must be greater than angle_min; using full circle");
      angle_min_ = -kPi;
      angle_max_ = kPi;
    }
    if (angle_increment_ <= 0.0) {
      RCLCPP_WARN(get_logger(), "angle_increment must be positive; using 0.25 degrees");
      angle_increment_ = kPi / 720.0;
    }
    if (scan_frequency_ <= 0.0) {
      RCLCPP_WARN(get_logger(), "scan_frequency must be positive; using 16 Hz");
      scan_frequency_ = 16.0;
    }

    auto qos = rclcpp::QoS(rclcpp::KeepLast(queue_size_)).best_effort().durability_volatile();
    scan_publisher_ = create_publisher<sensor_msgs::msg::LaserScan>(scan_topic_, qos);

    ydlidar::os_init();
    configureLidar();

    const auto timer_period = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::duration<double>(1.0 / scan_frequency_));
    timer_ = create_wall_timer(timer_period, std::bind(&LidarNode::publish_lidar_info, this));
  }

private:
  void configureLidar()
  {
    std::string port = "/dev/mylidar";
    laser_.setlidaropt(LidarPropSerialPort, port.c_str(), port.size());

    std::string ignore_array;
    laser_.setlidaropt(LidarPropIgnoreArray, ignore_array.c_str(), ignore_array.size());

    int baudrate = 512000;
    laser_.setlidaropt(LidarPropSerialBaudrate, &baudrate, sizeof(int));

    int optval = TYPE_TOF;
    laser_.setlidaropt(LidarPropLidarType, &optval, sizeof(int));

    optval = YDLIDAR_TYPE_SERIAL;
    laser_.setlidaropt(LidarPropDeviceType, &optval, sizeof(int));

    optval = 20;
    laser_.setlidaropt(LidarPropSampleRate, &optval, sizeof(int));

    optval = 4;
    laser_.setlidaropt(LidarPropAbnormalCheckCount, &optval, sizeof(int));

    optval = 0;
    laser_.setlidaropt(LidarPropIntenstiyBit, &optval, sizeof(int));

    bool b_optvalue = false;
    laser_.setlidaropt(LidarPropFixedResolution, &b_optvalue, sizeof(bool));

    b_optvalue = false;
    laser_.setlidaropt(LidarPropReversion, &b_optvalue, sizeof(bool));

    b_optvalue = false;
    laser_.setlidaropt(LidarPropInverted, &b_optvalue, sizeof(bool));

    b_optvalue = true;
    laser_.setlidaropt(LidarPropAutoReconnect, &b_optvalue, sizeof(bool));

    b_optvalue = false;
    laser_.setlidaropt(LidarPropSingleChannel, &b_optvalue, sizeof(bool));
    laser_.setlidaropt(LidarPropIntenstiy, &b_optvalue, sizeof(bool));
    laser_.setlidaropt(LidarPropSupportMotorDtrCtrl, &b_optvalue, sizeof(bool));

    float f_optvalue = static_cast<float>(angle_max_ * kRadToDeg);
    laser_.setlidaropt(LidarPropMaxAngle, &f_optvalue, sizeof(float));

    f_optvalue = static_cast<float>(angle_min_ * kRadToDeg);
    laser_.setlidaropt(LidarPropMinAngle, &f_optvalue, sizeof(float));

    f_optvalue = static_cast<float>(range_max_);
    laser_.setlidaropt(LidarPropMaxRange, &f_optvalue, sizeof(float));

    f_optvalue = static_cast<float>(range_min_);
    laser_.setlidaropt(LidarPropMinRange, &f_optvalue, sizeof(float));

    f_optvalue = static_cast<float>(scan_frequency_);
    laser_.setlidaropt(LidarPropScanFrequency, &f_optvalue, sizeof(float));

    bool ret = laser_.initialize();
    if (ret) {
      ret = laser_.turnOn();
      if (!ret) {
        RCLCPP_ERROR(get_logger(), "%s", laser_.DescribeError());
      }
    } else {
      RCLCPP_ERROR(get_logger(), "%s", laser_.DescribeError());
    }
  }

  void publish_lidar_info()
  {
    if (!laser_.doProcessSimple(scan_)) {
      return;
    }

    const auto stamp = toBuiltinTime(get_clock()->now());
    if (scan_publisher_) {
      sensor_msgs::msg::LaserScan scan_msg;
      packLaserScanMsg(
        scan_, frame_id_, stamp, angle_min_, angle_max_, angle_increment_, range_min_, range_max_,
        1.0 / scan_frequency_, scan_msg);
      scan_publisher_->publish(scan_msg);
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_publisher_;
  CYdLidar laser_;
  LaserScan scan_;
  std::string frame_id_;
  std::string scan_topic_;
  int queue_size_;
  double range_min_;
  double range_max_;
  double angle_min_;
  double angle_max_;
  double angle_increment_;
  double scan_frequency_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarNode>());
  rclcpp::shutdown();
  return 0;
}
