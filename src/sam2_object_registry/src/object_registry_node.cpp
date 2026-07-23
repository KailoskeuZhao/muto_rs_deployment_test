#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <filesystem>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <fcntl.h>
#include <unistd.h>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/exceptions.hpp"
#include "tf2/time.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "yaml-cpp/yaml.h"

#include "sam2_object_registry/msg/detected_object_array.hpp"
#include "sam2_object_registry/msg/stored_object.hpp"
#include "sam2_object_registry/msg/stored_object_array.hpp"
#include "sam2_object_registry/srv/get_stored_objects.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace sam2_object_registry
{

namespace fs = std::filesystem;
using DetectedObject = msg::DetectedObject;
using DetectedObjectArray = msg::DetectedObjectArray;
using StoredObject = msg::StoredObject;
using StoredObjectArray = msg::StoredObjectArray;
using GetStoredObjects = srv::GetStoredObjects;
using Trigger = std_srvs::srv::Trigger;
using PointCloud2 = sensor_msgs::msg::PointCloud2;
using Marker = visualization_msgs::msg::Marker;
using MarkerArray = visualization_msgs::msg::MarkerArray;

struct CellKey
{
  std::int64_t x{};
  std::int64_t y{};
  std::int64_t z{};

  bool operator==(const CellKey & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct CellKeyHash
{
  std::size_t operator()(const CellKey & key) const
  {
    auto combine = [](std::size_t seed, std::int64_t value) {
        const auto hashed = std::hash<std::int64_t>{}(value);
        return seed ^ (hashed + 0x9e3779b97f4a7c15ULL + (seed << 6U) + (seed >> 2U));
      };
    std::size_t seed = combine(0U, key.x);
    seed = combine(seed, key.y);
    return combine(seed, key.z);
  }
};

struct ObjectRecord
{
  std::uint64_t id{};
  std::string name;
  std::string label;
  std::int32_t class_id{-1};
  std::array<double, 3> position{};
  std::uint64_t observation_count{1};
  std::uint32_t point_count{};
  float last_confidence{};
  builtin_interfaces::msg::Time last_seen;
  CellKey cell;
};

struct Accumulator
{
  double x{};
  double y{};
  double z{};
  std::uint32_t count{};
};

class ObjectRegistryNode : public rclcpp::Node
{
public:
  ObjectRegistryNode()
  : Node("object_registry")
  {
    pointcloud_topic_ = declare_parameter<std::string>(
      "pointcloud_topic", "/sam2/instance_pointcloud");
    detections_topic_ = declare_parameter<std::string>(
      "detections_topic", "/sam2/detections");
    objects_topic_ = declare_parameter<std::string>(
      "objects_topic", "/sam2/stored_objects");
    marker_topic_ = declare_parameter<std::string>(
      "marker_topic", "/sam2/stored_object_markers");
    query_service_name_ = declare_parameter<std::string>(
      "query_service", "/sam2/get_stored_objects");
    save_service_name_ = declare_parameter<std::string>(
      "save_service", "/sam2/save_stored_objects");
    output_path_ = expand_user_path(declare_parameter<std::string>(
      "output_yaml", "~/.ros/sam2_objects.yaml"));
    target_frame_ = declare_parameter<std::string>("target_frame", "map");
    duplicate_distance_threshold_ = declare_parameter<double>(
      "duplicate_distance_threshold", 0.25);
    metadata_sync_tolerance_ = declare_parameter<double>(
      "metadata_sync_tolerance", 0.2);
    sync_queue_size_ = declare_parameter<int>("sync_queue_size", 10);
    min_points_ = declare_parameter<int>("min_points", 20);
    yolo_confidence_ = declare_parameter<double>("yolo_confidence", 0.4);
    tf_timeout_ = declare_parameter<double>("tf_timeout", 0.1);
    tf_cache_time_ = declare_parameter<double>("tf_cache_time", 30.0);
    snapshot_publish_rate_ = declare_parameter<double>("snapshot_publish_rate", 2.0);
    marker_scale_ = declare_parameter<double>("marker_scale", 0.12);
    marker_text_height_ = declare_parameter<double>("marker_text_height", 0.12);
    marker_text_offset_ = declare_parameter<double>("marker_text_offset", 0.15);
    save_on_shutdown_ = declare_parameter<bool>("save_on_shutdown", true);
    validate_parameters();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(
      get_clock(), tf2::durationFromSec(tf_cache_time_));
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    const auto input_qos = rclcpp::QoS(
      rclcpp::KeepLast(static_cast<std::size_t>(sync_queue_size_))).best_effort();
    pointcloud_sub_ = create_subscription<PointCloud2>(
      pointcloud_topic_, input_qos,
      std::bind(&ObjectRegistryNode::pointcloud_callback, this, std::placeholders::_1));
    detections_sub_ = create_subscription<DetectedObjectArray>(
      detections_topic_, input_qos,
      std::bind(&ObjectRegistryNode::detections_callback, this, std::placeholders::_1));
    objects_pub_ = create_publisher<StoredObjectArray>(
      objects_topic_, rclcpp::QoS(1).reliable().transient_local());
    markers_pub_ = create_publisher<MarkerArray>(
      marker_topic_, rclcpp::QoS(1).reliable().transient_local());
    query_service_ = create_service<GetStoredObjects>(
      query_service_name_,
      std::bind(
        &ObjectRegistryNode::query_callback, this,
        std::placeholders::_1, std::placeholders::_2));
    save_service_ = create_service<Trigger>(
      save_service_name_,
      std::bind(
        &ObjectRegistryNode::save_callback, this,
        std::placeholders::_1, std::placeholders::_2));

    load_database();
    publish_snapshot();
    snapshot_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / snapshot_publish_rate_)),
      std::bind(&ObjectRegistryNode::publish_snapshot_if_changed, this));
    RCLCPP_INFO(
      get_logger(),
      "Indexing %s + %s in %s; query %s, save %s, snapshot %s + markers %s at %.2f Hz "
      "(new objects publish immediately), shutdown YAML %s",
      pointcloud_topic_.c_str(), detections_topic_.c_str(), target_frame_.c_str(),
      query_service_name_.c_str(), save_service_name_.c_str(),
      objects_topic_.c_str(), marker_topic_.c_str(), snapshot_publish_rate_, output_path_.c_str());
  }

  ~ObjectRegistryNode() override
  {
    flush_on_shutdown();
  }

  void flush_on_shutdown()
  {
    if (!save_on_shutdown_ || shutdown_save_attempted_.exchange(true)) {
      return;
    }
    try {
      save_database();
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Failed to persist object registry: %s", error.what());
    }
  }

private:
  using CloudPtr = PointCloud2::ConstSharedPtr;
  using DetectionsPtr = DetectedObjectArray::ConstSharedPtr;
  using SpatialCells = std::unordered_map<
    CellKey, std::unordered_set<std::uint64_t>, CellKeyHash>;

  static fs::path expand_user_path(const std::string & value)
  {
    if (value == "~" || value.rfind("~/", 0) == 0) {
      const char * home = std::getenv("HOME");
      if (home == nullptr || std::strlen(home) == 0U) {
        throw std::runtime_error("HOME is unavailable for output_yaml expansion");
      }
      if (value == "~") {
        return fs::path(home);
      }
      return fs::path(home) / value.substr(2);
    }
    return fs::absolute(fs::path(value));
  }

  void validate_parameters()
  {
    if (target_frame_.empty()) {
      RCLCPP_WARN(get_logger(), "target_frame is empty; using map");
      target_frame_ = "map";
    }
    if (!(duplicate_distance_threshold_ > 0.0)) {
      RCLCPP_WARN(get_logger(), "duplicate_distance_threshold must be positive; using 0.25");
      duplicate_distance_threshold_ = 0.25;
    }
    if (metadata_sync_tolerance_ < 0.0) {
      RCLCPP_WARN(get_logger(), "metadata_sync_tolerance must be non-negative; using 0.2");
      metadata_sync_tolerance_ = 0.2;
    }
    if (sync_queue_size_ < 1) {
      RCLCPP_WARN(get_logger(), "sync_queue_size must be positive; using 10");
      sync_queue_size_ = 10;
    }
    if (min_points_ < 1) {
      RCLCPP_WARN(get_logger(), "min_points must be positive; using 20");
      min_points_ = 20;
    }
    if (!(yolo_confidence_ >= 0.0 && yolo_confidence_ <= 1.0)) {
      RCLCPP_WARN(get_logger(), "yolo_confidence must be in [0, 1]; using 0.4");
      yolo_confidence_ = 0.4;
    }
    if (tf_timeout_ < 0.0) {
      RCLCPP_WARN(get_logger(), "tf_timeout must be non-negative; using 0.1");
      tf_timeout_ = 0.1;
    }
    if (!(tf_cache_time_ > 0.0)) {
      RCLCPP_WARN(get_logger(), "tf_cache_time must be positive; using 30.0");
      tf_cache_time_ = 30.0;
    }
    if (!(snapshot_publish_rate_ > 0.0 && snapshot_publish_rate_ <= 1000.0)) {
      RCLCPP_WARN(
        get_logger(), "snapshot_publish_rate must be in (0, 1000] Hz; using 2.0");
      snapshot_publish_rate_ = 2.0;
    }
    if (marker_topic_.empty()) {
      RCLCPP_WARN(get_logger(), "marker_topic is empty; using /sam2/stored_object_markers");
      marker_topic_ = "/sam2/stored_object_markers";
    }
    if (!(marker_scale_ > 0.0)) {
      RCLCPP_WARN(get_logger(), "marker_scale must be positive; using 0.12");
      marker_scale_ = 0.12;
    }
    if (!(marker_text_height_ > 0.0)) {
      RCLCPP_WARN(get_logger(), "marker_text_height must be positive; using 0.12");
      marker_text_height_ = 0.12;
    }
    if (!(marker_text_offset_ >= 0.0)) {
      RCLCPP_WARN(get_logger(), "marker_text_offset must be non-negative; using 0.15");
      marker_text_offset_ = 0.15;
    }
  }

  static double stamp_seconds(const builtin_interfaces::msg::Time & stamp)
  {
    return static_cast<double>(stamp.sec) +
           static_cast<double>(stamp.nanosec) * 1.0e-9;
  }

  void pointcloud_callback(const CloudPtr cloud)
  {
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      pending_clouds_.push_back(cloud);
      trim_queue(pending_clouds_);
    }
    match_pending_messages();
  }

  void detections_callback(const DetectionsPtr detections)
  {
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      pending_detections_.push_back(detections);
      trim_queue(pending_detections_);
    }
    match_pending_messages();
  }

  template<typename QueueT>
  void trim_queue(QueueT & queue)
  {
    while (queue.size() > static_cast<std::size_t>(sync_queue_size_)) {
      queue.pop_front();
    }
  }

  void match_pending_messages()
  {
    while (true) {
      CloudPtr cloud;
      DetectionsPtr detections;
      {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        if (pending_clouds_.empty() || pending_detections_.empty()) {
          return;
        }

        double best_offset = std::numeric_limits<double>::infinity();
        std::size_t best_cloud = 0U;
        std::size_t best_detections = 0U;
        for (std::size_t cloud_index = 0; cloud_index < pending_clouds_.size(); ++cloud_index) {
          const double cloud_stamp = stamp_seconds(pending_clouds_[cloud_index]->header.stamp);
          for (
            std::size_t detection_index = 0;
            detection_index < pending_detections_.size(); ++detection_index)
          {
            const double detection_stamp = stamp_seconds(
              pending_detections_[detection_index]->header.stamp);
            const double offset = std::abs(cloud_stamp - detection_stamp);
            if (offset < best_offset) {
              best_offset = offset;
              best_cloud = cloud_index;
              best_detections = detection_index;
            }
          }
        }
        if (best_offset > metadata_sync_tolerance_) {
          return;
        }
        cloud = pending_clouds_[best_cloud];
        detections = pending_detections_[best_detections];
        pending_clouds_.erase(pending_clouds_.begin() + static_cast<std::ptrdiff_t>(best_cloud));
        pending_detections_.erase(
          pending_detections_.begin() + static_cast<std::ptrdiff_t>(best_detections));
      }
      process_observation(*cloud, *detections);
    }
  }

  void process_observation(
    const PointCloud2 & cloud, const DetectedObjectArray & detections)
  {
    if (cloud.header.frame_id.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Ignoring instance cloud with an empty frame_id");
      return;
    }

    std::unordered_map<std::uint16_t, const DetectedObject *> metadata_by_id;
    for (const auto & detection : detections.objects) {
      if (std::isfinite(detection.confidence) &&
        detection.confidence >= yolo_confidence_)
      {
        metadata_by_id[detection.instance_id] = &detection;
      }
    }
    if (metadata_by_id.empty()) {
      return;
    }

    std::unordered_map<std::uint16_t, Accumulator> accumulators;
    try {
      sensor_msgs::PointCloud2ConstIterator<float> x_iterator(cloud, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y_iterator(cloud, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z_iterator(cloud, "z");
      sensor_msgs::PointCloud2ConstIterator<std::uint16_t> id_iterator(cloud, "instance_id");
      for (; x_iterator != x_iterator.end();
        ++x_iterator, ++y_iterator, ++z_iterator, ++id_iterator)
      {
        const auto metadata = metadata_by_id.find(*id_iterator);
        if (metadata == metadata_by_id.end() ||
          !std::isfinite(*x_iterator) || !std::isfinite(*y_iterator) ||
          !std::isfinite(*z_iterator))
        {
          continue;
        }
        auto & accumulator = accumulators[*id_iterator];
        accumulator.x += *x_iterator;
        accumulator.y += *y_iterator;
        accumulator.z += *z_iterator;
        ++accumulator.count;
      }
    } catch (const std::runtime_error & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Ignoring incompatible instance cloud: %s", error.what());
      return;
    }

    geometry_msgs::msg::TransformStamped transform;
    const bool transform_required = cloud.header.frame_id != target_frame_;
    if (transform_required) {
      try {
        transform = tf_buffer_->lookupTransform(
          target_frame_, cloud.header.frame_id,
          tf2_ros::fromMsg(cloud.header.stamp),
          tf2::durationFromSec(tf_timeout_));
      } catch (const tf2::TransformException & error) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Cannot transform object centroids from %s to %s: %s",
          cloud.header.frame_id.c_str(), target_frame_.c_str(), error.what());
        return;
      }
    }

    bool inserted = false;
    for (const auto & [instance_id, accumulator] : accumulators) {
      if (accumulator.count < static_cast<std::uint32_t>(min_points_)) {
        continue;
      }
      const auto metadata = metadata_by_id.find(instance_id);
      if (metadata == metadata_by_id.end()) {
        continue;
      }
      std::array<double, 3> position{
        accumulator.x / accumulator.count,
        accumulator.y / accumulator.count,
        accumulator.z / accumulator.count};
      if (transform_required) {
        geometry_msgs::msg::PointStamped source;
        geometry_msgs::msg::PointStamped target;
        source.header = cloud.header;
        source.point.x = position[0];
        source.point.y = position[1];
        source.point.z = position[2];
        tf2::doTransform(source, target, transform);
        position = {target.point.x, target.point.y, target.point.z};
      }

      const auto * detection = metadata->second;
      std::string label = detection->label;
      if (label.empty()) {
        label = "class_" + std::to_string(detection->class_id);
      }
      {
        std::lock_guard<std::mutex> lock(registry_mutex_);
        inserted = merge_observation_locked(
          label, detection->class_id, position, accumulator.count,
          detection->confidence, cloud.header.stamp) || inserted;
      }
    }

    // Existing-object updates remain immediately queryable in memory. The
    // timer coalesces their full snapshot and RViz marker serialization, while
    // a newly inserted object is made visible without waiting for that timer.
    if (inserted) {
      publish_snapshot();
    }
  }

  CellKey cell_for(const std::array<double, 3> & position) const
  {
    return CellKey{
      static_cast<std::int64_t>(std::floor(position[0] / duplicate_distance_threshold_)),
      static_cast<std::int64_t>(std::floor(position[1] / duplicate_distance_threshold_)),
      static_cast<std::int64_t>(std::floor(position[2] / duplicate_distance_threshold_))};
  }

  std::optional<std::uint64_t> find_nearest_locked(
    const std::string & label, const std::array<double, 3> & position) const
  {
    const auto label_cells = spatial_index_.find(label);
    if (label_cells == spatial_index_.end()) {
      return std::nullopt;
    }
    const CellKey center = cell_for(position);
    double nearest_distance = std::numeric_limits<double>::infinity();
    std::optional<std::uint64_t> nearest;
    for (std::int64_t dx = -1; dx <= 1; ++dx) {
      for (std::int64_t dy = -1; dy <= 1; ++dy) {
        for (std::int64_t dz = -1; dz <= 1; ++dz) {
          const CellKey candidate_cell{center.x + dx, center.y + dy, center.z + dz};
          const auto bucket = label_cells->second.find(candidate_cell);
          if (bucket == label_cells->second.end()) {
            continue;
          }
          for (const auto id : bucket->second) {
            const auto record = records_.find(id);
            if (record == records_.end()) {
              continue;
            }
            const auto & other = record->second.position;
            const double x = position[0] - other[0];
            const double y = position[1] - other[1];
            const double z = position[2] - other[2];
            const double distance = std::sqrt(x * x + y * y + z * z);
            if (distance <= duplicate_distance_threshold_ && distance < nearest_distance) {
              nearest_distance = distance;
              nearest = id;
            }
          }
        }
      }
    }
    return nearest;
  }

  bool merge_observation_locked(
    const std::string & label, std::int32_t class_id,
    const std::array<double, 3> & position, std::uint32_t point_count,
    float confidence, const builtin_interfaces::msg::Time & stamp)
  {
    const auto nearest = find_nearest_locked(label, position);
    if (!nearest.has_value()) {
      ObjectRecord record;
      record.id = next_id_++;
      record.name = allocate_name_locked(label);
      record.label = label;
      record.class_id = class_id;
      record.position = position;
      record.point_count = point_count;
      record.last_confidence = confidence;
      record.last_seen = stamp;
      record.cell = cell_for(position);
      const auto id = record.id;
      records_.emplace(id, record);
      name_index_[record.name] = id;
      label_index_[label].insert(id);
      spatial_index_[label][record.cell].insert(id);
      dirty_ = true;
      ++revision_;
      return true;
    }

    auto & record = records_.at(*nearest);
    const CellKey old_cell = record.cell;
    const double old_count = static_cast<double>(record.observation_count);
    for (std::size_t index = 0; index < position.size(); ++index) {
      record.position[index] =
        (record.position[index] * old_count + position[index]) / (old_count + 1.0);
    }
    ++record.observation_count;
    record.point_count = point_count;
    record.last_confidence = confidence;
    record.last_seen = stamp;
    if (record.class_id < 0 && class_id >= 0) {
      record.class_id = class_id;
    }
    record.cell = cell_for(record.position);
    if (!(record.cell == old_cell)) {
      auto & cells = spatial_index_[record.label];
      auto old_bucket = cells.find(old_cell);
      if (old_bucket != cells.end()) {
        old_bucket->second.erase(record.id);
        if (old_bucket->second.empty()) {
          cells.erase(old_bucket);
        }
      }
      cells[record.cell].insert(record.id);
    }
    dirty_ = true;
    ++revision_;
    return false;
  }

  std::string allocate_name_locked(const std::string & label)
  {
    const std::string base = label.empty() ? "object" : label;
    if (used_names_.insert(base).second) {
      next_suffix_[base] = 2U;
      return base;
    }
    auto & suffix = next_suffix_[base];
    suffix = std::max<std::uint32_t>(suffix, 2U);
    std::string candidate;
    do {
      candidate = base + "_" + std::to_string(suffix++);
    } while (!used_names_.insert(candidate).second);
    return candidate;
  }

  StoredObject to_message(const ObjectRecord & record) const
  {
    StoredObject result;
    result.name = record.name;
    result.label = record.label;
    result.class_id = record.class_id;
    result.position.x = record.position[0];
    result.position.y = record.position[1];
    result.position.z = record.position[2];
    result.observation_count = record.observation_count;
    result.point_count = record.point_count;
    result.last_confidence = record.last_confidence;
    result.last_seen = record.last_seen;
    return result;
  }

  StoredObjectArray snapshot_locked(
    const std::string & name, const std::string & label) const
  {
    StoredObjectArray result;
    result.header.frame_id = target_frame_;
    result.header.stamp = now();
    if (!name.empty()) {
      const auto id = name_index_.find(name);
      if (id != name_index_.end()) {
        const auto record = records_.find(id->second);
        if (record != records_.end() &&
          (label.empty() || record->second.label == label))
        {
          result.objects.push_back(to_message(record->second));
        }
      }
    } else if (label.empty()) {
      result.objects.reserve(records_.size());
      for (const auto & [id, record] : records_) {
        static_cast<void>(id);
        result.objects.push_back(to_message(record));
      }
    } else {
      const auto ids = label_index_.find(label);
      if (ids != label_index_.end()) {
        result.objects.reserve(ids->second.size());
        for (const auto id : ids->second) {
          const auto record = records_.find(id);
          if (record != records_.end()) {
            result.objects.push_back(to_message(record->second));
          }
        }
      }
    }
    std::sort(
      result.objects.begin(), result.objects.end(),
      [](const StoredObject & left, const StoredObject & right) {
        return left.name < right.name;
      });
    return result;
  }

  void publish_snapshot()
  {
    StoredObjectArray snapshot;
    std::uint64_t snapshot_revision = 0U;
    {
      std::lock_guard<std::mutex> lock(registry_mutex_);
      snapshot = snapshot_locked("", "");
      snapshot_revision = revision_;
    }
    objects_pub_->publish(snapshot);
    markers_pub_->publish(marker_snapshot(snapshot));
    {
      std::lock_guard<std::mutex> lock(registry_mutex_);
      published_revision_ = std::max(published_revision_, snapshot_revision);
    }
  }

  void publish_snapshot_if_changed()
  {
    {
      std::lock_guard<std::mutex> lock(registry_mutex_);
      if (revision_ == published_revision_) {
        return;
      }
    }
    publish_snapshot();
  }

  MarkerArray marker_snapshot(const StoredObjectArray & snapshot) const
  {
    MarkerArray result;

    Marker clear;
    clear.header = snapshot.header;
    clear.action = Marker::DELETEALL;
    result.markers.push_back(clear);

    Marker centroids;
    centroids.header = snapshot.header;
    centroids.ns = "stored_object_centroids";
    centroids.id = 0;
    centroids.type = Marker::SPHERE_LIST;
    centroids.action = Marker::ADD;
    centroids.pose.orientation.w = 1.0;
    centroids.scale.x = marker_scale_;
    centroids.scale.y = marker_scale_;
    centroids.scale.z = marker_scale_;
    centroids.color.r = 0.10F;
    centroids.color.g = 0.85F;
    centroids.color.b = 0.35F;
    centroids.color.a = 1.0F;
    centroids.points.reserve(snapshot.objects.size());
    for (const auto & object : snapshot.objects) {
      centroids.points.push_back(object.position);
    }
    result.markers.push_back(std::move(centroids));

    std::int32_t label_id = 0;
    for (const auto & object : snapshot.objects) {
      Marker label;
      label.header = snapshot.header;
      label.ns = "stored_object_labels";
      label.id = label_id++;
      label.type = Marker::TEXT_VIEW_FACING;
      label.action = Marker::ADD;
      label.pose.position = object.position;
      label.pose.position.z += marker_text_offset_;
      label.pose.orientation.w = 1.0;
      label.scale.z = marker_text_height_;
      label.color.r = 1.0F;
      label.color.g = 1.0F;
      label.color.b = 1.0F;
      label.color.a = 1.0F;
      label.text = object.name;
      result.markers.push_back(std::move(label));
    }
    return result;
  }

  void query_callback(
    const std::shared_ptr<GetStoredObjects::Request> request,
    std::shared_ptr<GetStoredObjects::Response> response)
  {
    std::lock_guard<std::mutex> lock(registry_mutex_);
    response->result = snapshot_locked(request->name, request->label);
  }

  void save_callback(
    const std::shared_ptr<Trigger::Request> request,
    std::shared_ptr<Trigger::Response> response)
  {
    static_cast<void>(request);
    try {
      save_database();
      response->success = true;
      response->message = "Object registry persisted to " + output_path_.string();
    } catch (const std::exception & error) {
      response->success = false;
      response->message = error.what();
      RCLCPP_ERROR(get_logger(), "Manual registry save failed: %s", error.what());
    }
  }

  void index_loaded_record_locked(ObjectRecord record)
  {
    if (!used_names_.insert(record.name).second) {
      throw std::runtime_error("duplicate object name in YAML: " + record.name);
    }
    record.id = next_id_++;
    record.cell = cell_for(record.position);
    const auto id = record.id;
    const auto label = record.label;
    records_.emplace(id, record);
    name_index_[record.name] = id;
    label_index_[label].insert(id);
    spatial_index_[label][record.cell].insert(id);
  }

  void load_database()
  {
    if (!fs::exists(output_path_)) {
      return;
    }
    try {
      const YAML::Node root = YAML::LoadFile(output_path_.string());
      const std::string frame = root["frame_id"] ?
        root["frame_id"].as<std::string>() : target_frame_;
      if (frame != target_frame_) {
        throw std::runtime_error(
                "YAML frame_id " + frame + " does not match " + target_frame_);
      }
      const YAML::Node objects = root["objects"];
      if (objects && !objects.IsSequence()) {
        throw std::runtime_error("YAML objects must be a sequence");
      }
      std::lock_guard<std::mutex> lock(registry_mutex_);
      if (objects) {
        for (const auto & node : objects) {
          ObjectRecord record;
          record.name = node["name"].as<std::string>();
          record.label = node["label"].as<std::string>();
          record.class_id = node["class_id"] ? node["class_id"].as<std::int32_t>() : -1;
          const auto position = node["position"];
          record.position = {
            position["x"].as<double>(), position["y"].as<double>(),
            position["z"].as<double>()};
          if (!std::isfinite(record.position[0]) ||
            !std::isfinite(record.position[1]) || !std::isfinite(record.position[2]))
          {
            throw std::runtime_error("non-finite position for " + record.name);
          }
          record.observation_count = node["observation_count"] ?
            std::max<std::uint64_t>(1U, node["observation_count"].as<std::uint64_t>()) : 1U;
          record.point_count = node["point_count"] ?
            node["point_count"].as<std::uint32_t>() : 0U;
          record.last_confidence = node["last_confidence"] ?
            node["last_confidence"].as<float>() : 0.0F;
          const auto last_seen = node["last_seen"];
          if (last_seen) {
            record.last_seen.sec = last_seen["sec"] ? last_seen["sec"].as<std::int32_t>() : 0;
            record.last_seen.nanosec = last_seen["nanosec"] ?
              last_seen["nanosec"].as<std::uint32_t>() : 0U;
          }
          if (record.name.empty() || record.label.empty()) {
            throw std::runtime_error("YAML objects require non-empty name and label");
          }
          index_loaded_record_locked(record);
        }
      }
      dirty_ = false;
      RCLCPP_INFO(
        get_logger(), "Loaded %zu objects from %s", records_.size(), output_path_.c_str());
    } catch (const std::exception & error) {
      {
        std::lock_guard<std::mutex> lock(registry_mutex_);
        records_.clear();
        name_index_.clear();
        label_index_.clear();
        spatial_index_.clear();
        used_names_.clear();
        next_suffix_.clear();
        next_id_ = 1U;
        revision_ = 0U;
        dirty_ = false;
      }
      persistence_enabled_ = false;
      persistence_error_ = error.what();
      RCLCPP_ERROR(
        get_logger(),
        "Cannot load %s: %s; in-memory registry remains available but shutdown save is disabled",
        output_path_.c_str(), error.what());
    }
  }

  static void append_yaml_object(YAML::Emitter & emitter, const ObjectRecord & record)
  {
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "name" << YAML::Value << record.name;
    emitter << YAML::Key << "label" << YAML::Value << record.label;
    emitter << YAML::Key << "class_id" << YAML::Value << record.class_id;
    emitter << YAML::Key << "position" << YAML::Value << YAML::BeginMap;
    emitter << YAML::Key << "x" << YAML::Value << record.position[0];
    emitter << YAML::Key << "y" << YAML::Value << record.position[1];
    emitter << YAML::Key << "z" << YAML::Value << record.position[2];
    emitter << YAML::EndMap;
    emitter << YAML::Key << "observation_count" << YAML::Value << record.observation_count;
    emitter << YAML::Key << "point_count" << YAML::Value << record.point_count;
    emitter << YAML::Key << "last_confidence" << YAML::Value << record.last_confidence;
    emitter << YAML::Key << "last_seen" << YAML::Value << YAML::BeginMap;
    emitter << YAML::Key << "sec" << YAML::Value << record.last_seen.sec;
    emitter << YAML::Key << "nanosec" << YAML::Value << record.last_seen.nanosec;
    emitter << YAML::EndMap;
    emitter << YAML::EndMap;
  }

  static void write_all(int descriptor, const std::string & content)
  {
    std::size_t written = 0U;
    while (written < content.size()) {
      const ssize_t result = ::write(
        descriptor, content.data() + written, content.size() - written);
      if (result < 0 && errno == EINTR) {
        continue;
      }
      if (result <= 0) {
        throw std::runtime_error("write failed: " + std::string(std::strerror(errno)));
      }
      written += static_cast<std::size_t>(result);
    }
  }

  void save_database()
  {
    std::lock_guard<std::mutex> save_lock(save_mutex_);
    if (!persistence_enabled_) {
      throw std::runtime_error("persistence disabled: " + persistence_error_);
    }

    std::vector<ObjectRecord> snapshot;
    std::uint64_t snapshot_revision = 0U;
    {
      std::lock_guard<std::mutex> lock(registry_mutex_);
      if (!dirty_) {
        return;
      }
      snapshot.reserve(records_.size());
      for (const auto & [id, record] : records_) {
        static_cast<void>(id);
        snapshot.push_back(record);
      }
      snapshot_revision = revision_;
    }
    std::sort(
      snapshot.begin(), snapshot.end(),
      [](const ObjectRecord & left, const ObjectRecord & right) {
        return left.name < right.name;
      });

    YAML::Emitter emitter;
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "frame_id" << YAML::Value << target_frame_;
    emitter << YAML::Key << "duplicate_distance_threshold" << YAML::Value <<
      duplicate_distance_threshold_;
    emitter << YAML::Key << "objects" << YAML::Value << YAML::BeginSeq;
    for (const auto & record : snapshot) {
      append_yaml_object(emitter, record);
    }
    emitter << YAML::EndSeq << YAML::EndMap;
    if (!emitter.good()) {
      throw std::runtime_error("YAML emission failed: " + emitter.GetLastError());
    }

    const fs::path parent = output_path_.parent_path().empty() ? fs::path(".") :
      output_path_.parent_path();
    fs::create_directories(parent);
    std::string pattern = (parent / ("." + output_path_.filename().string() + ".XXXXXX")).string();
    std::vector<char> temporary_name(pattern.begin(), pattern.end());
    temporary_name.push_back(0);
    int descriptor = ::mkstemp(temporary_name.data());
    if (descriptor < 0) {
      throw std::runtime_error("mkstemp failed: " + std::string(std::strerror(errno)));
    }
    const fs::path temporary_path(temporary_name.data());
    try {
      write_all(descriptor, std::string(emitter.c_str()) + "\n");
      if (::fsync(descriptor) != 0) {
        throw std::runtime_error("fsync failed: " + std::string(std::strerror(errno)));
      }
      const int close_result = ::close(descriptor);
      descriptor = -1;
      if (close_result != 0) {
        throw std::runtime_error("close failed: " + std::string(std::strerror(errno)));
      }
      if (::rename(temporary_path.c_str(), output_path_.c_str()) != 0) {
        throw std::runtime_error("rename failed: " + std::string(std::strerror(errno)));
      }
      const int directory_descriptor = ::open(parent.c_str(), O_RDONLY | O_DIRECTORY);
      if (directory_descriptor >= 0) {
        static_cast<void>(::fsync(directory_descriptor));
        static_cast<void>(::close(directory_descriptor));
      }
    } catch (...) {
      if (descriptor >= 0) {
        static_cast<void>(::close(descriptor));
      }
      static_cast<void>(::unlink(temporary_path.c_str()));
      throw;
    }

    {
      std::lock_guard<std::mutex> lock(registry_mutex_);
      if (revision_ == snapshot_revision) {
        dirty_ = false;
      }
    }
    RCLCPP_INFO(
      get_logger(), "Persisted %zu objects to %s", snapshot.size(), output_path_.c_str());
  }

  std::string pointcloud_topic_;
  std::string detections_topic_;
  std::string objects_topic_;
  std::string marker_topic_;
  std::string query_service_name_;
  std::string save_service_name_;
  fs::path output_path_;
  std::string target_frame_;
  double duplicate_distance_threshold_{};
  double metadata_sync_tolerance_{};
  int sync_queue_size_{};
  int min_points_{};
  double yolo_confidence_{};
  double tf_timeout_{};
  double tf_cache_time_{};
  double snapshot_publish_rate_{};
  double marker_scale_{};
  double marker_text_height_{};
  double marker_text_offset_{};
  bool save_on_shutdown_{};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Subscription<DetectedObjectArray>::SharedPtr detections_sub_;
  rclcpp::Publisher<StoredObjectArray>::SharedPtr objects_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr markers_pub_;
  rclcpp::Service<GetStoredObjects>::SharedPtr query_service_;
  rclcpp::Service<Trigger>::SharedPtr save_service_;
  rclcpp::TimerBase::SharedPtr snapshot_timer_;

  std::mutex queue_mutex_;
  std::deque<CloudPtr> pending_clouds_;
  std::deque<DetectionsPtr> pending_detections_;

  mutable std::mutex registry_mutex_;
  std::unordered_map<std::uint64_t, ObjectRecord> records_;
  std::unordered_map<std::string, std::uint64_t> name_index_;
  std::unordered_map<std::string, std::unordered_set<std::uint64_t>> label_index_;
  std::unordered_map<std::string, SpatialCells> spatial_index_;
  std::unordered_set<std::string> used_names_;
  std::unordered_map<std::string, std::uint32_t> next_suffix_;
  std::uint64_t next_id_{1U};
  std::uint64_t revision_{0U};
  std::uint64_t published_revision_{0U};
  bool dirty_{false};

  std::mutex save_mutex_;
  bool persistence_enabled_{true};
  std::string persistence_error_;
  std::atomic<bool> shutdown_save_attempted_{false};
};

}  // namespace sam2_object_registry

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<sam2_object_registry::ObjectRegistryNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2U);
  executor.add_node(node);
  executor.spin();
  node->flush_on_shutdown();
  executor.remove_node(node);
  node.reset();
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
