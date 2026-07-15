#include <memory>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"

class FixedFrameBroadcaster : public rclcpp::Node
{
public:
  FixedFrameBroadcaster()
  : Node("camera_tf2_broadcaster")
  {
    tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    broadcast_static_transform();
  }

private:
  void broadcast_static_transform()
  {
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "base_frame";
    t.child_frame_id = "camera_link";
    t.transform.translation.x = 0.13;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 0.115;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.408407, 0.0);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(t);
  }

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FixedFrameBroadcaster>());
  rclcpp::shutdown();
  return 0;
}
