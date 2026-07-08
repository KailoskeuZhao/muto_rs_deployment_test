#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2/exceptions.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class LidarPointCloudFilterNode : public rclcpp::Node
{
public:
  LidarPointCloudFilterNode()
  : Node("lidar_pointcloud_filter_node")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "lidar/PointCloud");
    output_topic_ = declare_parameter<std::string>("output_topic", "lidar/PointCloudFiltered");
    no_downsample_output_topic_ = declare_parameter<std::string>(
      "no_downsample_output_topic", "lidar/PointCloudFilteredNoDownsample");
    target_frame_ = declare_parameter<std::string>("target_frame", "base_frame");

    min_range_ = declare_parameter<double>("min_range", 0.05);
    max_range_ = declare_parameter<double>("max_range", 64.0);

    min_x_ = declare_parameter<double>("min_x", -100.0);
    max_x_ = declare_parameter<double>("max_x", 100.0);
    min_y_ = declare_parameter<double>("min_y", -100.0);
    max_y_ = declare_parameter<double>("max_y", 100.0);
    min_z_ = declare_parameter<double>("min_z", -1.0);
    max_z_ = declare_parameter<double>("max_z", 1.0);

    voxel_leaf_size_ = declare_parameter<double>("voxel_leaf_size", 0.02);
    queue_size_ = declare_parameter<int>("queue_size", 5);
    transform_timeout_ = rclcpp::Duration::from_seconds(
      declare_parameter<double>("transform_timeout", 0.05));
    log_filter_stats_ = declare_parameter<bool>("log_filter_stats", false);

    if (queue_size_ < 1) {
      RCLCPP_WARN(get_logger(), "queue_size must be positive; using 1");
      queue_size_ = 1;
    }
    if (min_range_ < 0.0) {
      RCLCPP_WARN(get_logger(), "min_range must be non-negative; using 0.0");
      min_range_ = 0.0;
    }
    if (max_range_ < min_range_) {
      RCLCPP_WARN(get_logger(), "max_range is smaller than min_range; swapping them");
      std::swap(max_range_, min_range_);
    }
    if (voxel_leaf_size_ < 0.0) {
      RCLCPP_WARN(get_logger(), "voxel_leaf_size must be non-negative; using 0.0");
      voxel_leaf_size_ = 0.0;
    }
    if (target_frame_.empty()) {
      RCLCPP_WARN(get_logger(), "target_frame is empty; using input cloud frame");
    }

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(queue_size_));
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, qos);
    if (!no_downsample_output_topic_.empty()) {
      no_downsample_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        no_downsample_output_topic_, qos);
    }
    subscriber_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, qos,
      std::bind(&LidarPointCloudFilterNode::pointCloudCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "Filtering %s -> %s in target frame %s",
      input_topic_.c_str(), output_topic_.c_str(),
      target_frame_.empty() ? "<input frame>" : target_frame_.c_str());
    if (no_downsample_publisher_) {
      RCLCPP_INFO(
        get_logger(), "Publishing no-downsample filtered cloud to %s",
        no_downsample_output_topic_.c_str());
    }
    RCLCPP_INFO(
      get_logger(),
      "Range [%.3f, %.3f], ROI x[%.3f, %.3f] y[%.3f, %.3f] z[%.3f, %.3f], voxel %.3f",
      min_range_, max_range_, min_x_, max_x_, min_y_, max_y_, min_z_, max_z_, voxel_leaf_size_);
  }

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    const std::string output_frame = target_frame_.empty() ? msg->header.frame_id : target_frame_;
    if (output_frame.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Input cloud has an empty frame_id and target_frame is empty; dropping cloud");
      return;
    }

    sensor_msgs::msg::PointCloud2 target_frame_msg;
    if (!transformCloudToTargetFrame(*msg, output_frame, target_frame_msg)) {
      return;
    }

    pcl::PointCloud<pcl::PointXYZ> input_cloud;
    pcl::fromROSMsg(target_frame_msg, input_cloud);

    pcl::PointCloud<pcl::PointXYZ> target_frame_cloud;
    std::vector<int> finite_indices;
    pcl::removeNaNFromPointCloud(input_cloud, target_frame_cloud, finite_indices);

    auto cropped_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    cropped_cloud->reserve(target_frame_cloud.size());

    const double min_range_sq = min_range_ * min_range_;
    const double max_range_sq = max_range_ * max_range_;

    for (const auto & point : target_frame_cloud.points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        continue;
      }

      const double range_sq =
        static_cast<double>(point.x) * point.x +
        static_cast<double>(point.y) * point.y +
        static_cast<double>(point.z) * point.z;

      if (range_sq < min_range_sq || range_sq > max_range_sq) {
        continue;
      }
      if (point.x < min_x_ || point.x > max_x_) {
        continue;
      }
      if (point.y < min_y_ || point.y > max_y_) {
        continue;
      }
      if (point.z < min_z_ || point.z > max_z_) {
        continue;
      }

      cropped_cloud->push_back(point);
    }

    cropped_cloud->height = 1;
    cropped_cloud->width = static_cast<std::uint32_t>(cropped_cloud->size());
    cropped_cloud->is_dense = true;

    if (no_downsample_publisher_) {
      publishCloud(*cropped_cloud, *msg, output_frame, no_downsample_publisher_);
    }

    auto output_cloud = cropped_cloud;
    if (voxel_leaf_size_ > 0.0 && !cropped_cloud->empty()) {
      auto downsampled_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
      pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
      voxel_grid.setInputCloud(cropped_cloud);
      voxel_grid.setLeafSize(
        static_cast<float>(voxel_leaf_size_),
        static_cast<float>(voxel_leaf_size_),
        static_cast<float>(voxel_leaf_size_));
      voxel_grid.filter(*downsampled_cloud);
      downsampled_cloud->height = 1;
      downsampled_cloud->width = static_cast<std::uint32_t>(downsampled_cloud->size());
      downsampled_cloud->is_dense = true;
      output_cloud = downsampled_cloud;
    }

    publishCloud(*output_cloud, *msg, output_frame, publisher_);

    if (log_filter_stats_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Filtered cloud: input=%zu finite=%zu roi_range=%zu output=%zu",
        input_cloud.size(), target_frame_cloud.size(), cropped_cloud->size(),
        output_cloud->size());
    }
  }

  void publishCloud(
    const pcl::PointCloud<pcl::PointXYZ> & cloud,
    const sensor_msgs::msg::PointCloud2 & input_msg,
    const std::string & output_frame,
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & publisher)
  {
    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(cloud, output_msg);
    output_msg.header = input_msg.header;
    output_msg.header.frame_id = output_frame;
    publisher->publish(output_msg);
  }

  bool transformCloudToTargetFrame(
    const sensor_msgs::msg::PointCloud2 & input_msg,
    const std::string & target_frame,
    sensor_msgs::msg::PointCloud2 & output_msg)
  {
    const std::string & source_frame = input_msg.header.frame_id;
    if (source_frame == target_frame) {
      output_msg = input_msg;
      return true;
    }

    if (source_frame.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Input cloud has an empty frame_id; dropping cloud");
      return false;
    }

    try {
      const geometry_msgs::msg::TransformStamped transform =
        tf_buffer_->lookupTransform(
        target_frame, source_frame, input_msg.header.stamp, transform_timeout_);
      tf2::doTransform(input_msg, output_msg, transform);
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Failed to transform cloud from %s to %s: %s",
        source_frame.c_str(), target_frame.c_str(), ex.what());
      return false;
    }
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string no_downsample_output_topic_;
  std::string target_frame_;
  double min_range_;
  double max_range_;
  double min_x_;
  double max_x_;
  double min_y_;
  double max_y_;
  double min_z_;
  double max_z_;
  double voxel_leaf_size_;
  rclcpp::Duration transform_timeout_{0, 0};
  int queue_size_;
  bool log_filter_stats_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr no_downsample_publisher_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarPointCloudFilterNode>());
  rclcpp::shutdown();
  return 0;
}
