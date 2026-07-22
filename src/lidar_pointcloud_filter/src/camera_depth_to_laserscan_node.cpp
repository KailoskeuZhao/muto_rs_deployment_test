#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class CameraDepthToLaserScanNode : public rclcpp::Node
{
public:
  CameraDepthToLaserScanNode()
  : Node("camera_depth_to_laserscan_node")
  {
    depth_image_topic_ = declare_parameter<std::string>(
      "depth_image_topic", "/camera/depth/image_raw");
    camera_info_topic_ = declare_parameter<std::string>(
      "camera_info_topic", "/camera/depth/camera_info");
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/camera/filtered_laserscan");
    processing_frame_ = declare_parameter<std::string>("processing_frame", "base_frame");

    depth_scale_ = declare_parameter<double>("depth_scale", 0.001);
    pixel_stride_x_ = declare_parameter<int>("pixel_stride_x", 4);
    pixel_stride_y_ = declare_parameter<int>("pixel_stride_y", 4);
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

    queue_size_ = declare_parameter<int>("queue_size", 1);
    max_publish_rate_ = declare_parameter<double>("max_publish_rate", 0.0);
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
    camera_info_subscriber_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, input_qos,
      std::bind(&CameraDepthToLaserScanNode::cameraInfoCallback, this, std::placeholders::_1));
    depth_subscriber_ = create_subscription<sensor_msgs::msg::Image>(
      depth_image_topic_, input_qos,
      std::bind(&CameraDepthToLaserScanNode::depthImageCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Converting %s + %s -> %s in processing frame %s, nearest pixel per %dx%d block, "
      "z[%.3f, %.3f], camera_x>=%.3f, range[%.3f, %.3f], restamp_output=%s",
      depth_image_topic_.c_str(), camera_info_topic_.c_str(), output_topic_.c_str(),
      processing_frame_.c_str(), pixel_stride_x_, pixel_stride_y_, min_z_, max_z_, camera_min_x_,
      range_min_, range_max_, restamp_output_ ? "true" : "false");
  }

private:
  struct FilterStats
  {
    std::size_t blocks{0};
    std::size_t empty_blocks{0};
    std::size_t projected{0};
    std::size_t x_filtered{0};
    std::size_t z_filtered{0};
    std::size_t range_filtered{0};
    std::size_t angle_filtered{0};
    std::size_t updated_bins{0};
  };

  struct Ray
  {
    float x{0.0F};
    float y{0.0F};
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
    if (pixel_stride_x_ < 1) {
      RCLCPP_WARN(get_logger(), "pixel_stride_x must be positive; using 1");
      pixel_stride_x_ = 1;
    }
    if (pixel_stride_y_ < 1) {
      RCLCPP_WARN(get_logger(), "pixel_stride_y must be positive; using 1");
      pixel_stride_y_ = 1;
    }
    if (!(depth_scale_ > 0.0) || !std::isfinite(depth_scale_)) {
      RCLCPP_WARN(get_logger(), "depth_scale must be finite and positive; using 0.001");
      depth_scale_ = 0.001;
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
    if (!(angle_increment_ > 0.0) || !std::isfinite(angle_increment_)) {
      RCLCPP_WARN(get_logger(), "angle_increment must be finite and positive; using 0.25 degrees");
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
      RCLCPP_WARN(get_logger(), "processing_frame is empty; using base_frame");
      processing_frame_ = "base_frame";
    }
  }

  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    camera_info_ = msg;
  }

  void depthImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    const rclcpp::Time now = get_clock()->now();
    if (shouldThrottle(now)) {
      return;
    }
    const auto processing_start = std::chrono::steady_clock::now();

    warnIfStampFarFromNow(*msg);
    if (isStampTooFarFromNow(*msg) || !validateDepthImage(*msg)) {
      return;
    }
    if (!camera_info_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for depth CameraInfo on %s", camera_info_topic_.c_str());
      return;
    }
    if (!validateCameraInfo(*camera_info_, *msg)) {
      return;
    }

    const std::string source_frame =
      msg->header.frame_id.empty() ? camera_info_->header.frame_id : msg->header.frame_id;
    if (source_frame.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Depth image and CameraInfo both have empty frame_id values; dropping image");
      return;
    }
    if (!msg->header.frame_id.empty() && !camera_info_->header.frame_id.empty() &&
      msg->header.frame_id != camera_info_->header.frame_id)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Depth image frame %s does not match CameraInfo frame %s; dropping image",
        msg->header.frame_id.c_str(), camera_info_->header.frame_id.c_str());
      return;
    }

    last_process_time_ = now;
    tf2::Transform depth_transform;
    if (!lookupDepthTransform(*msg, source_frame, depth_transform) ||
      !ensureRayCache(*camera_info_, msg->width, msg->height))
    {
      return;
    }

    sensor_msgs::msg::LaserScan scan;
    initializeScan(*msg, scan);
    FilterStats stats;
    addDepthImageToScan(*msg, depth_transform, scan, stats);
    publisher_->publish(scan);

    warnIfProcessingWasSlow(processing_start, *msg, stats);
    if (log_filter_stats_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Converted depth image to scan: pixels=%u blocks=%zu empty=%zu projected=%zu "
        "updated_bins=%zu bins=%zu",
        msg->width * msg->height, stats.blocks, stats.empty_blocks, stats.projected,
        stats.updated_bins, scan.ranges.size());
    }
  }

  bool shouldThrottle(const rclcpp::Time & now) const
  {
    if (max_publish_rate_ <= 0.0 || last_process_time_.nanoseconds() == 0) {
      return false;
    }
    const double elapsed = (now - last_process_time_).seconds();
    return elapsed >= 0.0 && elapsed < 1.0 / max_publish_rate_;
  }

  bool validateDepthImage(const sensor_msgs::msg::Image & msg)
  {
    if (msg.encoding != "16UC1") {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Depth image encoding is %s; expected 16UC1", msg.encoding.c_str());
      return false;
    }
    if (msg.width == 0U || msg.height == 0U) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Depth image is empty");
      return false;
    }
    const std::size_t minimum_step = static_cast<std::size_t>(msg.width) * sizeof(std::uint16_t);
    const std::size_t required_size = static_cast<std::size_t>(msg.step) * msg.height;
    if (msg.step < minimum_step || msg.data.size() < required_size) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Depth image storage is invalid: width=%u height=%u step=%u data=%zu",
        msg.width, msg.height, msg.step, msg.data.size());
      return false;
    }
    return true;
  }

  bool validateCameraInfo(
    const sensor_msgs::msg::CameraInfo & info,
    const sensor_msgs::msg::Image & image)
  {
    if ((info.width != 0U && info.width != image.width) ||
      (info.height != 0U && info.height != image.height))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Depth CameraInfo dimensions %ux%u do not match image dimensions %ux%u",
        info.width, info.height, image.width, image.height);
      return false;
    }
    const double fx = info.k[0];
    const double fy = info.k[4];
    const double cx = info.k[2];
    const double cy = info.k[5];
    if (!(fx > 0.0) || !(fy > 0.0) || !std::isfinite(fx) || !std::isfinite(fy) ||
      !std::isfinite(cx) || !std::isfinite(cy))
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Depth CameraInfo has invalid K");
      return false;
    }
    return true;
  }

  bool ensureRayCache(
    const sensor_msgs::msg::CameraInfo & info,
    const std::uint32_t width,
    const std::uint32_t height)
  {
    if (ray_width_ == width && ray_height_ == height && ray_k_ == info.k &&
      ray_d_ == info.d && ray_distortion_model_ == info.distortion_model)
    {
      return true;
    }

    ray_distortion_supported_ =
      info.distortion_model.empty() || info.distortion_model == "plumb_bob" ||
      info.distortion_model == "rational_polynomial";
    if (!ray_distortion_supported_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Unsupported depth distortion model %s; using pinhole rays without distortion correction",
        info.distortion_model.c_str());
    }

    const std::size_t pixel_count = static_cast<std::size_t>(width) * height;
    ray_table_.assign(pixel_count, Ray{});
    ray_valid_.assign(pixel_count, 0U);
    ray_width_ = width;
    ray_height_ = height;
    ray_k_ = info.k;
    ray_d_ = info.d;
    ray_distortion_model_ = info.distortion_model;
    RCLCPP_INFO(
      get_logger(), "Initialized lazy depth ray cache for %ux%u CameraInfo (%s)", width, height,
      info.distortion_model.empty() ? "pinhole" : info.distortion_model.c_str());
    return true;
  }

  const Ray & rayForPixel(
    const std::uint32_t u,
    const std::uint32_t v,
    const std::uint32_t width)
  {
    const std::size_t index = static_cast<std::size_t>(v) * width + u;
    if (ray_valid_[index] != 0U) {
      return ray_table_[index];
    }

    const double distorted_x = (static_cast<double>(u) - ray_k_[2]) / ray_k_[0];
    const double distorted_y = (static_cast<double>(v) - ray_k_[5]) / ray_k_[4];
    double ray_x = distorted_x;
    double ray_y = distorted_y;
    if (ray_distortion_supported_ && !ray_d_.empty()) {
      undistortNormalizedPoint(distorted_x, distorted_y, ray_d_, ray_x, ray_y);
    }
    if (!std::isfinite(ray_x) || !std::isfinite(ray_y)) {
      ray_x = distorted_x;
      ray_y = distorted_y;
    }
    ray_table_[index] = Ray{static_cast<float>(ray_x), static_cast<float>(ray_y)};
    ray_valid_[index] = 1U;
    return ray_table_[index];
  }

  static void undistortNormalizedPoint(
    const double distorted_x,
    const double distorted_y,
    const std::vector<double> & distortion,
    double & x,
    double & y)
  {
    const auto coefficient = [&distortion](const std::size_t index) {
        return index < distortion.size() ? distortion[index] : 0.0;
      };
    const double k1 = coefficient(0U);
    const double k2 = coefficient(1U);
    const double p1 = coefficient(2U);
    const double p2 = coefficient(3U);
    const double k3 = coefficient(4U);
    const double k4 = coefficient(5U);
    const double k5 = coefficient(6U);
    const double k6 = coefficient(7U);

    x = distorted_x;
    y = distorted_y;
    for (int iteration = 0; iteration < 5; ++iteration) {
      const double x2 = x * x;
      const double y2 = y * y;
      const double xy = x * y;
      const double r2 = x2 + y2;
      const double r4 = r2 * r2;
      const double r6 = r4 * r2;
      const double numerator = 1.0 + k1 * r2 + k2 * r4 + k3 * r6;
      const double denominator = 1.0 + k4 * r2 + k5 * r4 + k6 * r6;
      if (std::fabs(numerator) < 1.0e-12 || !std::isfinite(numerator) ||
        !std::isfinite(denominator))
      {
        break;
      }
      const double delta_x = 2.0 * p1 * xy + p2 * (r2 + 2.0 * x2);
      const double delta_y = p1 * (r2 + 2.0 * y2) + 2.0 * p2 * xy;
      const double inverse_radial = denominator / numerator;
      x = (distorted_x - delta_x) * inverse_radial;
      y = (distorted_y - delta_y) * inverse_radial;
    }
  }

  bool lookupDepthTransform(
    const sensor_msgs::msg::Image & msg,
    const std::string & source_frame,
    tf2::Transform & transform)
  {
    if (source_frame == processing_frame_) {
      transform.setIdentity();
      return true;
    }
    try {
      const auto transform_msg = tf_buffer_->lookupTransform(
        processing_frame_, source_frame, msg.header.stamp, transform_timeout_);
      setTfTransform(transform_msg, transform);
      return true;
    } catch (const tf2::TransformException & stamped_exception) {
      const std::string stamped_error = stamped_exception.what();
      try {
        const auto transform_msg = tf_buffer_->lookupTransform(
          processing_frame_, source_frame,
          rclcpp::Time(0, 0, get_clock()->get_clock_type()), transform_timeout_);
        setTfTransform(transform_msg, transform);
        return true;
      } catch (const tf2::TransformException & latest_exception) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Failed to transform depth image from %s to %s. stamped lookup: %s; latest lookup: %s",
          source_frame.c_str(), processing_frame_.c_str(), stamped_error.c_str(),
          latest_exception.what());
        return false;
      }
    }
  }

  static void setTfTransform(
    const geometry_msgs::msg::TransformStamped & transform_msg,
    tf2::Transform & transform)
  {
    const auto & translation = transform_msg.transform.translation;
    const auto & rotation = transform_msg.transform.rotation;
    transform.setOrigin(tf2::Vector3(translation.x, translation.y, translation.z));
    transform.setRotation(tf2::Quaternion(rotation.x, rotation.y, rotation.z, rotation.w));
  }

  void initializeScan(const sensor_msgs::msg::Image & image, sensor_msgs::msg::LaserScan & scan)
  {
    const auto bin_count = static_cast<std::size_t>(
      std::floor((angle_max_ - angle_min_) / angle_increment_)) + 1U;
    scan.header = image.header;
    scan.header.frame_id = processing_frame_;
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
  }

  void addDepthImageToScan(
    const sensor_msgs::msg::Image & image,
    const tf2::Transform & transform,
    sensor_msgs::msg::LaserScan & scan,
    FilterStats & stats)
  {
    const auto stride_x = static_cast<std::uint32_t>(pixel_stride_x_);
    const auto stride_y = static_cast<std::uint32_t>(pixel_stride_y_);
    for (std::uint32_t block_v = 0; block_v < image.height; block_v += stride_y) {
      const std::uint32_t end_v = std::min(image.height, block_v + stride_y);
      for (std::uint32_t block_u = 0; block_u < image.width; block_u += stride_x) {
        const std::uint32_t end_u = std::min(image.width, block_u + stride_x);
        ++stats.blocks;
        std::uint16_t nearest_depth = 0U;
        std::uint32_t nearest_u = block_u;
        std::uint32_t nearest_v = block_v;
        for (std::uint32_t v = block_v; v < end_v; ++v) {
          for (std::uint32_t u = block_u; u < end_u; ++u) {
            const std::uint16_t depth = readDepth(image, u, v);
            if (depth != 0U && (nearest_depth == 0U || depth < nearest_depth)) {
              nearest_depth = depth;
              nearest_u = u;
              nearest_v = v;
            }
          }
        }
        if (nearest_depth == 0U) {
          ++stats.empty_blocks;
          continue;
        }

        const double optical_z = static_cast<double>(nearest_depth) * depth_scale_;
        const Ray & ray = rayForPixel(nearest_u, nearest_v, image.width);
        const tf2::Vector3 point = transform * tf2::Vector3(
          static_cast<double>(ray.x) * optical_z,
          static_cast<double>(ray.y) * optical_z,
          optical_z);
        ++stats.projected;

        const double x = point.x();
        const double y = point.y();
        const double z = point.z();
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
          ++stats.range_filtered;
          continue;
        }
        if (x < camera_min_x_) {
          ++stats.x_filtered;
          continue;
        }
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
          ++stats.updated_bins;
        }
      }
    }
  }

  static std::uint16_t readDepth(
    const sensor_msgs::msg::Image & image,
    const std::uint32_t u,
    const std::uint32_t v)
  {
    const std::size_t offset =
      static_cast<std::size_t>(v) * image.step + static_cast<std::size_t>(u) * 2U;
    if (image.is_bigendian != 0U) {
      return static_cast<std::uint16_t>(
        (static_cast<std::uint16_t>(image.data[offset]) << 8U) |
        static_cast<std::uint16_t>(image.data[offset + 1U]));
    }
    return static_cast<std::uint16_t>(
      static_cast<std::uint16_t>(image.data[offset]) |
      (static_cast<std::uint16_t>(image.data[offset + 1U]) << 8U));
  }

  void warnIfProcessingWasSlow(
    const std::chrono::steady_clock::time_point & processing_start,
    const sensor_msgs::msg::Image & image,
    const FilterStats & stats)
  {
    if (processing_time_warning_ <= 0.0) {
      return;
    }
    const double elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - processing_start).count();
    if (elapsed <= processing_time_warning_) {
      return;
    }
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Depth scan conversion took %.3f s for %u pixels (%zu blocks, %zu projected, %zu bins updated)",
      elapsed, image.width * image.height, stats.blocks, stats.projected, stats.updated_bins);
  }

  void warnIfStampFarFromNow(const sensor_msgs::msg::Image & image)
  {
    if (input_stamp_warning_age_ <= 0.0) {
      return;
    }
    const double age = (
      get_clock()->now() - rclcpp::Time(image.header.stamp, get_clock()->get_clock_type())).seconds();
    if (std::fabs(age) <= input_stamp_warning_age_) {
      return;
    }
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Depth image stamp is %.3f seconds from this node clock; restamp_output=%s",
      age, restamp_output_ ? "true" : "false");
  }

  bool isStampTooFarFromNow(const sensor_msgs::msg::Image & image)
  {
    if (max_input_age_ <= 0.0) {
      return false;
    }
    const double age = (
      get_clock()->now() - rclcpp::Time(image.header.stamp, get_clock()->get_clock_type())).seconds();
    if (std::fabs(age) <= max_input_age_) {
      return false;
    }
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Dropping depth image because stamp is %.3f seconds from this node clock; max_input_age=%.3f",
      age, max_input_age_);
    return true;
  }

  std::string depth_image_topic_;
  std::string camera_info_topic_;
  std::string output_topic_;
  std::string processing_frame_;
  double depth_scale_{};
  int pixel_stride_x_{};
  int pixel_stride_y_{};
  double min_z_{};
  double max_z_{};
  double camera_min_x_{};
  double range_min_{};
  double range_max_{};
  double angle_min_{};
  double angle_max_{};
  double angle_increment_{};
  double scan_time_{};
  double time_increment_{};
  int queue_size_{};
  double max_publish_rate_{};
  bool restamp_output_{};
  double input_stamp_warning_age_{};
  double max_input_age_{};
  double processing_time_warning_{};
  rclcpp::Duration transform_timeout_{0, 0};
  bool log_filter_stats_{};
  rclcpp::Time last_process_time_{0, 0, RCL_ROS_TIME};

  sensor_msgs::msg::CameraInfo::SharedPtr camera_info_;
  std::vector<Ray> ray_table_;
  std::vector<std::uint8_t> ray_valid_;
  std::uint32_t ray_width_{0U};
  std::uint32_t ray_height_{0U};
  std::array<double, 9> ray_k_{};
  std::vector<double> ray_d_;
  std::string ray_distortion_model_;
  bool ray_distortion_supported_{true};

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraDepthToLaserScanNode>());
  rclcpp::shutdown();
  return 0;
}
