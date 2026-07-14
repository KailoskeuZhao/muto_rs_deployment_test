#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class LidarLaserScanFilterNode : public rclcpp::Node
{
public:
  LidarLaserScanFilterNode()
  : Node("lidar_laserscan_filter_node")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/lidar/raw_laserscan");
    output_topic_ = declare_parameter<std::string>("output_topic", "/lidar/filtered_laserscan");
    no_downsample_output_topic_ = declare_parameter<std::string>(
      "no_downsample_output_topic", "/lidar/filtered_laserscan_no_downsample");

    range_min_ = declare_parameter<double>("range_min", 0.05);
    range_max_ = declare_parameter<double>("range_max", 64.0);
    no_downsample_range_max_ = declare_parameter<double>("no_downsample_range_max", range_max_);
    angle_min_ = declare_parameter<double>("angle_min", -M_PI);
    angle_max_ = declare_parameter<double>("angle_max", M_PI);
    downsample_factor_ = declare_parameter<int>("downsample_factor", 2);
    queue_size_ = declare_parameter<int>("queue_size", 5);
    restamp_output_ = declare_parameter<bool>("restamp_output", false);
    input_stamp_warning_age_ = declare_parameter<double>("input_stamp_warning_age", 1.0);
    max_input_age_ = declare_parameter<double>("max_input_age", 2.0);
    log_filter_stats_ = declare_parameter<bool>("log_filter_stats", false);

    normalizeParameters();

    const auto input_qos =
      rclcpp::QoS(rclcpp::KeepLast(queue_size_)).best_effort().durability_volatile();
    const auto output_qos = rclcpp::QoS(rclcpp::KeepLast(queue_size_));

    publisher_ = create_publisher<sensor_msgs::msg::LaserScan>(output_topic_, output_qos);
    if (!no_downsample_output_topic_.empty()) {
      no_downsample_publisher_ = create_publisher<sensor_msgs::msg::LaserScan>(
        no_downsample_output_topic_, output_qos);
    }

    subscriber_ = create_subscription<sensor_msgs::msg::LaserScan>(
      input_topic_, input_qos,
      std::bind(&LidarLaserScanFilterNode::scanCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Filtering LaserScan %s -> %s, no-downsample=%s, range[%.3f, %.3f], "
      "angle[%.3f, %.3f], downsample_factor=%d, no_downsample_range_max=%.3f",
      input_topic_.c_str(), output_topic_.c_str(),
      no_downsample_output_topic_.empty() ? "<disabled>" : no_downsample_output_topic_.c_str(),
      range_min_, range_max_, angle_min_, angle_max_, downsample_factor_,
      no_downsample_range_max_);
  }

private:
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
    if (no_downsample_range_max_ < range_min_) {
      RCLCPP_WARN(
        get_logger(), "no_downsample_range_max is smaller than range_min; using range_max");
      no_downsample_range_max_ = range_max_;
    }
    if (angle_max_ < angle_min_) {
      RCLCPP_WARN(get_logger(), "angle_max is smaller than angle_min; swapping them");
      std::swap(angle_max_, angle_min_);
    }
    if (downsample_factor_ < 1) {
      RCLCPP_WARN(get_logger(), "downsample_factor must be positive; using 1");
      downsample_factor_ = 1;
    }
    if (input_stamp_warning_age_ < 0.0) {
      RCLCPP_WARN(get_logger(), "input_stamp_warning_age must be non-negative; using 0.0");
      input_stamp_warning_age_ = 0.0;
    }
    if (max_input_age_ < 0.0) {
      RCLCPP_WARN(get_logger(), "max_input_age must be non-negative; using 0.0");
      max_input_age_ = 0.0;
    }
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    warnIfStampFarFromNow(*msg);
    if (isStampTooFarFromNow(*msg)) {
      return;
    }
    if (msg->header.frame_id.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Input LaserScan has an empty frame_id; dropping scan");
      return;
    }
    if (msg->ranges.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Input LaserScan has no ranges; dropping scan");
      return;
    }
    if (msg->angle_increment <= 0.0F) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Input LaserScan angle_increment must be positive; dropping scan");
      return;
    }

    FilterStats no_downsample_stats;
    sensor_msgs::msg::LaserScan full_resolution_scan;
    if (!buildFilteredScan(*msg, no_downsample_range_max_, full_resolution_scan, no_downsample_stats)) {
      return;
    }
    if (restamp_output_) {
      full_resolution_scan.header.stamp = get_clock()->now();
    }

    if (no_downsample_publisher_) {
      no_downsample_publisher_->publish(full_resolution_scan);
    }

    FilterStats downsample_stats;
    sensor_msgs::msg::LaserScan rf2o_resolution_scan;
    if (!buildFilteredScan(*msg, range_max_, rf2o_resolution_scan, downsample_stats)) {
      return;
    }
    if (restamp_output_) {
      rf2o_resolution_scan.header.stamp = full_resolution_scan.header.stamp;
    }

    sensor_msgs::msg::LaserScan downsampled_scan;
    downsampleScan(rf2o_resolution_scan, downsampled_scan);
    publisher_->publish(downsampled_scan);

    if (log_filter_stats_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Filtered scan: input=%zu rf2o_kept=%zu no_downsample_kept=%zu "
        "rf2o_range_filtered=%zu no_downsample_range_filtered=%zu downsampled=%zu",
        msg->ranges.size(), downsample_stats.kept, no_downsample_stats.kept,
        downsample_stats.range_filtered, no_downsample_stats.range_filtered,
        downsampled_scan.ranges.size());
    }
  }

  bool buildFilteredScan(
    const sensor_msgs::msg::LaserScan & input,
    const double range_max,
    sensor_msgs::msg::LaserScan & output,
    FilterStats & stats)
  {
    const double input_increment = input.angle_increment;
    const double selected_angle_min = std::max(angle_min_, static_cast<double>(input.angle_min));
    const double selected_angle_max = std::min(angle_max_, static_cast<double>(input.angle_max));
    if (selected_angle_max < selected_angle_min) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Configured angle window does not overlap input scan; dropping scan");
      return false;
    }

    const auto first_index = static_cast<std::int64_t>(
      std::ceil((selected_angle_min - input.angle_min) / input_increment));
    const auto last_index = static_cast<std::int64_t>(
      std::floor((selected_angle_max - input.angle_min) / input_increment));
    const auto clamped_first = std::max<std::int64_t>(0, first_index);
    const auto clamped_last = std::min<std::int64_t>(
      static_cast<std::int64_t>(input.ranges.size()) - 1, last_index);
    if (clamped_last < clamped_first) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Configured angle window selected no input scan bins; dropping scan");
      return false;
    }

    const std::size_t output_count =
      static_cast<std::size_t>(clamped_last - clamped_first + 1);
    const double effective_range_min = std::max(range_min_, static_cast<double>(input.range_min));
    double effective_range_max = range_max;
    if (input.range_max > 0.0F) {
      effective_range_max = std::min(range_max, static_cast<double>(input.range_max));
    }

    output = input;
    output.angle_min = static_cast<float>(input.angle_min + clamped_first * input_increment);
    output.angle_increment = input.angle_increment;
    output.angle_max = static_cast<float>(
      output.angle_min + (output_count - 1U) * input_increment);
    output.range_min = static_cast<float>(effective_range_min);
    output.range_max = static_cast<float>(effective_range_max);
    output.ranges.assign(output_count, std::numeric_limits<float>::infinity());
    output.intensities.clear();

    for (std::size_t output_index = 0; output_index < output_count; ++output_index) {
      const std::size_t input_index =
        static_cast<std::size_t>(clamped_first) + output_index;
      const double angle = input.angle_min + input_index * input_increment;
      if (angle < angle_min_ || angle > angle_max_) {
        ++stats.angle_filtered;
        continue;
      }

      const float range = input.ranges[input_index];
      if (!std::isfinite(range)) {
        ++stats.invalid;
        continue;
      }
      if (range < effective_range_min || range > effective_range_max) {
        ++stats.range_filtered;
        continue;
      }

      output.ranges[output_index] = range;
      ++stats.kept;
    }

    return true;
  }

  void downsampleScan(
    const sensor_msgs::msg::LaserScan & input,
    sensor_msgs::msg::LaserScan & output) const
  {
    if (downsample_factor_ <= 1 || input.ranges.size() < static_cast<std::size_t>(downsample_factor_)) {
      output = input;
      return;
    }

    const std::size_t factor = static_cast<std::size_t>(downsample_factor_);
    const std::size_t output_count = input.ranges.size() / factor;
    const double input_increment = input.angle_increment;
    const double output_angle_increment = input_increment * static_cast<double>(factor);
    const double output_angle_min =
      input.angle_min + 0.5 * static_cast<double>(factor - 1U) * input_increment;

    output = input;
    output.angle_min = static_cast<float>(output_angle_min);
    output.angle_increment = static_cast<float>(output_angle_increment);
    output.angle_max = static_cast<float>(
      output_angle_min + (output_count - 1U) * output_angle_increment);
    output.time_increment = static_cast<float>(input.time_increment * static_cast<float>(factor));
    output.ranges.assign(output_count, std::numeric_limits<float>::infinity());
    output.intensities.clear();

    for (std::size_t output_index = 0; output_index < output_count; ++output_index) {
      const std::size_t input_start = output_index * factor;
      float min_range = std::numeric_limits<float>::infinity();
      for (std::size_t offset = 0; offset < factor; ++offset) {
        const float range = input.ranges[input_start + offset];
        if (std::isfinite(range) && range < min_range) {
          min_range = range;
        }
      }
      output.ranges[output_index] = min_range;
    }
  }

  void warnIfStampFarFromNow(const sensor_msgs::msg::LaserScan & msg)
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
      "Input LaserScan stamp is %.3f seconds from this node clock; restamp_output=%s",
      age, restamp_output_ ? "true" : "false");
  }

  bool isStampTooFarFromNow(const sensor_msgs::msg::LaserScan & msg)
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
      "Dropping input LaserScan because stamp is %.3f seconds from this node clock; "
      "max_input_age=%.3f",
      age, max_input_age_);
    return true;
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string no_downsample_output_topic_;
  double range_min_;
  double range_max_;
  double no_downsample_range_max_;
  double angle_min_;
  double angle_max_;
  int downsample_factor_;
  int queue_size_;
  bool restamp_output_;
  double input_stamp_warning_age_;
  double max_input_age_;
  bool log_filter_stats_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr no_downsample_publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarLaserScanFilterNode>());
  rclcpp::shutdown();
  return 0;
}
