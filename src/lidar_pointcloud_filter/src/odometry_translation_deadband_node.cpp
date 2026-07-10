#include <cmath>
#include <functional>
#include <memory>
#include <string>

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
    translation_deadband_ = declare_parameter<double>("translation_deadband", 0.001);
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    queue_size_ = declare_parameter<int>("queue_size", 5);

    if (translation_deadband_ < 0.0) {
      RCLCPP_WARN(get_logger(), "translation_deadband must be non-negative; using 0.0");
      translation_deadband_ = 0.0;
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
      "Filtering odometry translation %s -> %s with %.6f m deadband; publish_tf=%s",
      input_topic_.c_str(), output_topic_.c_str(), translation_deadband_,
      publish_tf_ ? "true" : "false");
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    nav_msgs::msg::Odometry filtered = *msg;

    const double raw_x = msg->pose.pose.position.x;
    const double raw_y = msg->pose.pose.position.y;

    if (!have_last_odom_) {
      filtered_x_ = raw_x;
      filtered_y_ = raw_y;
      have_last_odom_ = true;
    } else {
      const double dx = raw_x - last_raw_x_;
      const double dy = raw_y - last_raw_y_;
      const double translation = std::hypot(dx, dy);

      if (translation_deadband_ <= 0.0 || translation > translation_deadband_) {
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
    }

    last_raw_x_ = raw_x;
    last_raw_y_ = raw_y;

    filtered.pose.pose.position.x = filtered_x_;
    filtered.pose.pose.position.y = filtered_y_;
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
  bool publish_tf_;
  int queue_size_;

  bool have_last_odom_{false};
  double last_raw_x_{0.0};
  double last_raw_y_{0.0};
  double filtered_x_{0.0};
  double filtered_y_{0.0};

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
