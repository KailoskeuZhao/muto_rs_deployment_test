#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

class OdometryTranslationDeadbandNode : public rclcpp::Node
{
public:
  OdometryTranslationDeadbandNode()
  : Node("odometry_translation_deadband_node")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "scan_odom_raw");
    output_topic_ = declare_parameter<std::string>("output_topic", "scan_odom");
    translation_deadband_ = declare_parameter<double>("translation_deadband", 0.003);
    yaw_deadband_ = declare_parameter<double>("yaw_deadband", 0.001);
    translation_jump_rejection_threshold_ =
      declare_parameter<double>("translation_jump_rejection_threshold", 0.20);
    max_translation_rate_ = declare_parameter<double>("max_translation_rate", 1.5);
    yaw_jump_rejection_threshold_ =
      declare_parameter<double>("yaw_jump_rejection_threshold", 0.30);
    max_yaw_rate_ = declare_parameter<double>("max_yaw_rate", 4.0);
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    queue_size_ = declare_parameter<int>("queue_size", 5);

    if (translation_deadband_ < 0.0) {
      RCLCPP_WARN(get_logger(), "translation_deadband must be non-negative; using 0.0");
      translation_deadband_ = 0.0;
    }
    if (yaw_deadband_ < 0.0) {
      RCLCPP_WARN(get_logger(), "yaw_deadband must be non-negative; using 0.0");
      yaw_deadband_ = 0.0;
    }
    if (translation_jump_rejection_threshold_ < 0.0) {
      RCLCPP_WARN(
        get_logger(), "translation_jump_rejection_threshold must be non-negative; using 0.0");
      translation_jump_rejection_threshold_ = 0.0;
    }
    if (max_translation_rate_ < 0.0) {
      RCLCPP_WARN(get_logger(), "max_translation_rate must be non-negative; using 0.0");
      max_translation_rate_ = 0.0;
    }
    if (yaw_jump_rejection_threshold_ < 0.0) {
      RCLCPP_WARN(
        get_logger(), "yaw_jump_rejection_threshold must be non-negative; using 0.0");
      yaw_jump_rejection_threshold_ = 0.0;
    }
    if (max_yaw_rate_ < 0.0) {
      RCLCPP_WARN(get_logger(), "max_yaw_rate must be non-negative; using 0.0");
      max_yaw_rate_ = 0.0;
    }
    if (queue_size_ < 1) {
      RCLCPP_WARN(get_logger(), "queue_size must be positive; using 1");
      queue_size_ = 1;
    }

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(queue_size_));
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(output_topic_, qos);
    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      input_topic_, qos,
      std::bind(&OdometryTranslationDeadbandNode::odomCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Filtering odometry %s -> %s with %.6f m translation and %.6f rad yaw deadbands; "
      "rejecting translation jumps above %.6f m and %.6f m/s, "
      "yaw jumps above %.6f rad and %.6f rad/s; publish_tf=%s",
      input_topic_.c_str(), output_topic_.c_str(), translation_deadband_, yaw_deadband_,
      translation_jump_rejection_threshold_, max_translation_rate_,
      yaw_jump_rejection_threshold_, max_yaw_rate_, publish_tf_ ? "true" : "false");
  }

private:
  static double normalizeAngle(double angle)
  {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  static double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
  {
    return std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  static geometry_msgs::msg::Quaternion quaternionFromYaw(double yaw)
  {
    geometry_msgs::msg::Quaternion q;
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(yaw * 0.5);
    q.w = std::cos(yaw * 0.5);
    return q;
  }

  bool shouldRejectTranslationJump(double translation, double dt) const
  {
    if (translation_jump_rejection_threshold_ <= 0.0) {
      return false;
    }

    if (translation <= translation_jump_rejection_threshold_) {
      return false;
    }

    if (max_translation_rate_ <= 0.0 || dt <= 0.0) {
      return true;
    }

    return translation / dt > max_translation_rate_;
  }

  bool shouldRejectYawJump(double dyaw, double dt) const
  {
    if (yaw_jump_rejection_threshold_ <= 0.0) {
      return false;
    }

    const double abs_dyaw = std::fabs(dyaw);
    if (abs_dyaw <= yaw_jump_rejection_threshold_) {
      return false;
    }

    if (max_yaw_rate_ <= 0.0 || dt <= 0.0) {
      return true;
    }

    return abs_dyaw / dt > max_yaw_rate_;
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    nav_msgs::msg::Odometry filtered = *msg;

    const double raw_x = msg->pose.pose.position.x;
    const double raw_y = msg->pose.pose.position.y;
    const double raw_yaw = yawFromQuaternion(msg->pose.pose.orientation);
    const rclcpp::Time stamp(msg->header.stamp);
    bool accept_translation = true;
    bool accept_yaw = true;

    if (!have_last_odom_) {
      filtered_x_ = raw_x;
      filtered_y_ = raw_y;
      filtered_yaw_ = raw_yaw;
      have_last_odom_ = true;
    } else {
      const double dt = (stamp - last_raw_stamp_).seconds();
      const double dx = raw_x - last_raw_x_;
      const double dy = raw_y - last_raw_y_;
      const double translation = std::hypot(dx, dy);
      const double dyaw = normalizeAngle(raw_yaw - last_raw_yaw_);

      if (shouldRejectTranslationJump(translation, dt)) {
        accept_translation = false;
        filtered.twist.twist.linear.x = 0.0;
        filtered.twist.twist.linear.y = 0.0;
        const double translation_rate = dt > 0.0 ? translation / dt : 0.0;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Rejected RF2O translation jump %.6f m over %.6f s (%.6f m/s); holding position",
          translation, dt, translation_rate);
      } else if (translation_deadband_ <= 0.0 || translation > translation_deadband_) {
        filtered_x_ += dx;
        filtered_y_ += dy;
      } else {
        filtered.twist.twist.linear.x = 0.0;
        filtered.twist.twist.linear.y = 0.0;
        RCLCPP_DEBUG(
          get_logger(),
          "Suppressed %.6f m odometry translation below %.6f m deadband",
          translation, translation_deadband_);
      }

      if (shouldRejectYawJump(dyaw, dt)) {
        accept_yaw = false;
        filtered.twist.twist.angular.z = 0.0;
        const double yaw_rate = dt > 0.0 ? std::fabs(dyaw) / dt : 0.0;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Rejected RF2O yaw jump %.6f rad over %.6f s (%.6f rad/s); holding yaw",
          dyaw, dt, yaw_rate);
      } else if (yaw_deadband_ <= 0.0 || std::fabs(dyaw) > yaw_deadband_) {
        filtered_yaw_ = normalizeAngle(filtered_yaw_ + dyaw);
      } else {
        filtered.twist.twist.angular.z = 0.0;
        RCLCPP_DEBUG(
          get_logger(),
          "Suppressed %.6f rad odometry yaw below %.6f rad deadband",
          dyaw, yaw_deadband_);
      }
    }

    if (accept_translation) {
      last_raw_x_ = raw_x;
      last_raw_y_ = raw_y;
    }
    if (accept_yaw) {
      last_raw_yaw_ = raw_yaw;
    }
    last_raw_stamp_ = stamp;

    filtered.pose.pose.position.x = filtered_x_;
    filtered.pose.pose.position.y = filtered_y_;
    filtered.pose.pose.orientation = quaternionFromYaw(filtered_yaw_);
    odom_pub_->publish(filtered);

    if (tf_broadcaster_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header = filtered.header;
      transform.child_frame_id = filtered.child_frame_id;
      transform.transform.translation.x = filtered.pose.pose.position.x;
      transform.transform.translation.y = filtered.pose.pose.position.y;
      transform.transform.translation.z = filtered.pose.pose.position.z;
      transform.transform.rotation = filtered.pose.pose.orientation;
      tf_broadcaster_->sendTransform(transform);
    }
  }

  std::string input_topic_;
  std::string output_topic_;
  double translation_deadband_;
  double yaw_deadband_;
  double translation_jump_rejection_threshold_;
  double max_translation_rate_;
  double yaw_jump_rejection_threshold_;
  double max_yaw_rate_;
  bool publish_tf_;
  int queue_size_;

  bool have_last_odom_{false};
  double last_raw_x_{0.0};
  double last_raw_y_{0.0};
  double last_raw_yaw_{0.0};
  double filtered_x_{0.0};
  double filtered_y_{0.0};
  double filtered_yaw_{0.0};
  rclcpp::Time last_raw_stamp_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdometryTranslationDeadbandNode>());
  rclcpp::shutdown();
  return 0;
}
