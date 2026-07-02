#include <chrono>
#include <functional>
#include <memory>

#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"


using namespace std::chrono_literals;

class OdomBroadcaster : public rclcpp::Node
{
public:
  OdomBroadcaster()
  : Node("imu_tf2_broadcaster")
  {

    this->declare_parameter<std::string>("odom_topic", "scan_odom");
	odom_topic = this->get_parameter("odom_topic").as_string();	  

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(odom_topic, 10, std::bind(&OdomBroadcaster::broadcast_callback, this, std::placeholders::_1));
  }

private:
  void broadcast_callback(const std::shared_ptr<const nav_msgs::msg::Odometry> msg)
  {
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = msg->header.stamp;
    t.header.frame_id = msg->header.frame_id;
    t.child_frame_id = msg->child_frame_id;

    t.transform.translation.x = msg->pose.pose.position.x;
    t.transform.translation.y = msg->pose.pose.position.y;
    t.transform.translation.z = msg->pose.pose.position.z;

    t.transform.rotation = msg->pose.pose.orientation;

    tf_broadcaster_->sendTransform(t);

  }

  std::string odom_topic;
std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomBroadcaster>());
  rclcpp::shutdown();
  return 0;
}
