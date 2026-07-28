#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <exception>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "muto_command_layer/action/go_to_object.hpp"
#include "muto_command_layer/approach_geometry.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sam2_object_registry/srv/get_stored_objects.hpp"
#include "tf2/exceptions.hpp"
#include "tf2/time.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace muto_command_layer
{

using namespace std::chrono_literals;

class CommandLayerNode : public rclcpp::Node
{
public:
  using CommandAction = muto_command_layer::action::GoToObject;
  using CommandGoalHandle = rclcpp_action::ServerGoalHandle<CommandAction>;
  using NavigateAction = nav2_msgs::action::NavigateToPose;
  using NavigateGoalHandle = rclcpp_action::ClientGoalHandle<NavigateAction>;
  using RegistryService = sam2_object_registry::srv::GetStoredObjects;

  CommandLayerNode()
  : Node("command_layer")
  {
    declare_parameters();
    read_parameters();
    validate_parameters();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    // TF subscriptions run in the ROS executor while the command worker waits
    // for transforms. Tell BufferCore that those callbacks can make progress.
    tf_buffer_->setUsingDedicatedThread(true);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(
      *tf_buffer_, this, false);

    registry_client_ = create_client<RegistryService>(registry_service_);
    navigate_client_ = rclcpp_action::create_client<NavigateAction>(
      this, navigate_action_);
    target_pose_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      target_pose_topic_, rclcpp::QoS(1).reliable().transient_local());

    command_server_ = rclcpp_action::create_server<CommandAction>(
      this,
      action_name_,
      std::bind(
        &CommandLayerNode::handle_goal, this,
        std::placeholders::_1, std::placeholders::_2),
      std::bind(
        &CommandLayerNode::handle_cancel, this, std::placeholders::_1),
      std::bind(
        &CommandLayerNode::handle_accepted, this, std::placeholders::_1));

    worker_ = std::thread(&CommandLayerNode::worker_loop, this);

    RCLCPP_INFO(
      get_logger(),
      "Object command layer ready: action=%s registry=%s nav2=%s "
      "standoff=%.2f m frames=%s<-%s",
      action_name_.c_str(), registry_service_.c_str(), navigate_action_.c_str(),
      approach_distance_, global_frame_.c_str(), robot_base_frame_.c_str());
  }

  ~CommandLayerNode() override
  {
    stopping_.store(true);
    worker_condition_.notify_all();
    if (worker_.joinable()) {
      worker_.join();
    }
  }

private:
  enum class WaitStatus
  {
    ready,
    canceled,
    timeout,
    stopping,
  };

  static constexpr uint8_t kLookupPhase = 1;
  static constexpr uint8_t kPlanningPhase = 2;
  static constexpr uint8_t kNavigatingPhase = 3;

  void declare_parameters()
  {
    declare_parameter<std::string>("action_name", "/go_to_object");
    declare_parameter<std::string>(
      "registry_service", "/sam2/get_stored_objects");
    declare_parameter<std::string>(
      "navigate_to_pose_action", "/navigate_to_pose");
    declare_parameter<std::string>(
      "target_pose_topic", "/object_navigation/target_pose");
    declare_parameter<std::string>("global_frame", "map");
    declare_parameter<std::string>("robot_base_frame", "base_frame");
    declare_parameter<double>("approach_distance", 0.75);
    declare_parameter<double>("registry_timeout", 3.0);
    declare_parameter<double>("nav_server_timeout", 5.0);
    declare_parameter<double>("tf_timeout", 0.2);
    declare_parameter<double>("navigation_timeout", 0.0);
    declare_parameter<double>("cancel_timeout", 2.0);
    declare_parameter<std::string>("behavior_tree", "");
  }

  void read_parameters()
  {
    action_name_ = get_parameter("action_name").as_string();
    registry_service_ = get_parameter("registry_service").as_string();
    navigate_action_ = get_parameter("navigate_to_pose_action").as_string();
    target_pose_topic_ = get_parameter("target_pose_topic").as_string();
    global_frame_ = get_parameter("global_frame").as_string();
    robot_base_frame_ = get_parameter("robot_base_frame").as_string();
    approach_distance_ = get_parameter("approach_distance").as_double();
    registry_timeout_ = get_parameter("registry_timeout").as_double();
    nav_server_timeout_ = get_parameter("nav_server_timeout").as_double();
    tf_timeout_ = get_parameter("tf_timeout").as_double();
    navigation_timeout_ = get_parameter("navigation_timeout").as_double();
    cancel_timeout_ = get_parameter("cancel_timeout").as_double();
    behavior_tree_ = get_parameter("behavior_tree").as_string();
  }

  void validate_parameters() const
  {
    const auto require_name = [](const std::string & value, const char * name) {
        if (value.empty()) {
          throw std::invalid_argument(std::string(name) + " must not be empty");
        }
      };
    require_name(action_name_, "action_name");
    require_name(registry_service_, "registry_service");
    require_name(navigate_action_, "navigate_to_pose_action");
    require_name(target_pose_topic_, "target_pose_topic");
    require_name(global_frame_, "global_frame");
    require_name(robot_base_frame_, "robot_base_frame");

    if (!std::isfinite(approach_distance_) || approach_distance_ <= 0.0) {
      throw std::invalid_argument("approach_distance must be finite and positive");
    }
    if (!std::isfinite(registry_timeout_) || registry_timeout_ <= 0.0) {
      throw std::invalid_argument("registry_timeout must be finite and positive");
    }
    if (!std::isfinite(nav_server_timeout_) || nav_server_timeout_ <= 0.0) {
      throw std::invalid_argument("nav_server_timeout must be finite and positive");
    }
    if (!std::isfinite(tf_timeout_) || tf_timeout_ <= 0.0) {
      throw std::invalid_argument("tf_timeout must be finite and positive");
    }
    if (!std::isfinite(navigation_timeout_) || navigation_timeout_ < 0.0) {
      throw std::invalid_argument("navigation_timeout must be finite and nonnegative");
    }
    if (!std::isfinite(cancel_timeout_) || cancel_timeout_ <= 0.0) {
      throw std::invalid_argument("cancel_timeout must be finite and positive");
    }
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const CommandAction::Goal> goal)
  {
    if (goal->object_id.empty()) {
      RCLCPP_WARN(get_logger(), "Rejected go-to-object goal with an empty object_id");
      return rclcpp_action::GoalResponse::REJECT;
    }

    std::lock_guard<std::mutex> lock(worker_mutex_);
    if (busy_ || stopping_.load()) {
      RCLCPP_WARN(
        get_logger(), "Rejected go-to-object goal for '%s': command layer is busy",
        goal->object_id.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    busy_ = true;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<CommandGoalHandle>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<CommandGoalHandle> goal_handle)
  {
    {
      std::lock_guard<std::mutex> lock(worker_mutex_);
      pending_goal_ = goal_handle;
    }
    worker_condition_.notify_one();
  }

  void worker_loop()
  {
    while (!stopping_.load()) {
      std::shared_ptr<CommandGoalHandle> goal_handle;
      {
        std::unique_lock<std::mutex> lock(worker_mutex_);
        worker_condition_.wait(
          lock, [this]() {return stopping_.load() || pending_goal_ != nullptr;});
        if (stopping_.load()) {
          break;
        }
        goal_handle = std::move(pending_goal_);
      }

      try {
        execute(goal_handle);
      } catch (const std::exception & error) {
        RCLCPP_ERROR(
          get_logger(), "Unhandled go-to-object error: %s", error.what());
        abort_command(
          goal_handle, std::string("internal command-layer error: ") + error.what(),
          geometry_msgs::msg::PoseStamped{});
      }

      {
        std::lock_guard<std::mutex> lock(worker_mutex_);
        busy_ = false;
      }
    }
  }

  template<typename FutureT>
  WaitStatus wait_for_future(
    FutureT & future,
    const std::shared_ptr<CommandGoalHandle> & goal_handle,
    const double timeout_seconds)
  {
    const bool unlimited = timeout_seconds <= 0.0;
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(std::max(0.0, timeout_seconds));
    while (future.wait_for(50ms) != std::future_status::ready) {
      if (goal_handle->is_canceling()) {
        return WaitStatus::canceled;
      }
      if (stopping_.load() || !rclcpp::ok()) {
        return WaitStatus::stopping;
      }
      if (!unlimited && std::chrono::steady_clock::now() >= deadline) {
        return WaitStatus::timeout;
      }
    }
    return WaitStatus::ready;
  }

  template<typename ReadyFunction>
  WaitStatus wait_for_endpoint(
    ReadyFunction ready_function,
    const std::shared_ptr<CommandGoalHandle> & goal_handle,
    const double timeout_seconds)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(timeout_seconds);
    while (!ready_function()) {
      if (goal_handle->is_canceling()) {
        return WaitStatus::canceled;
      }
      if (stopping_.load() || !rclcpp::ok()) {
        return WaitStatus::stopping;
      }
      if (std::chrono::steady_clock::now() >= deadline) {
        return WaitStatus::timeout;
      }
      std::this_thread::sleep_for(50ms);
    }
    return WaitStatus::ready;
  }

  void publish_feedback(
    const std::shared_ptr<CommandGoalHandle> & goal_handle,
    const uint8_t phase,
    const std::string & status,
    const float distance_remaining,
    const geometry_msgs::msg::PoseStamped & target_pose)
  {
    if (!goal_handle->is_active()) {
      return;
    }
    auto feedback = std::make_shared<CommandAction::Feedback>();
    feedback->phase = phase;
    feedback->status = status;
    feedback->distance_remaining = distance_remaining;
    feedback->target_pose = target_pose;
    goal_handle->publish_feedback(feedback);
  }

  geometry_msgs::msg::PointStamped transform_object_to_global(
    const sam2_object_registry::msg::StoredObjectArray & registry_result,
    const sam2_object_registry::msg::StoredObject & object)
  {
    geometry_msgs::msg::PointStamped source_point;
    source_point.header.frame_id = registry_result.header.frame_id;
    source_point.header.stamp = rclcpp::Time(
      0, 0, get_clock()->get_clock_type());
    source_point.point = object.position;

    if (source_point.header.frame_id == global_frame_) {
      return source_point;
    }

    const auto transform = tf_buffer_->lookupTransform(
      global_frame_, source_point.header.frame_id, tf2::TimePointZero,
      tf2::durationFromSec(tf_timeout_));
    geometry_msgs::msg::PointStamped global_point;
    tf2::doTransform(source_point, global_point, transform);
    return global_point;
  }

  geometry_msgs::msg::PoseStamped make_target_pose(
    const geometry_msgs::msg::PointStamped & object_point)
  {
    const auto robot_transform = tf_buffer_->lookupTransform(
      global_frame_, robot_base_frame_, tf2::TimePointZero,
      tf2::durationFromSec(tf_timeout_));
    const double robot_yaw = tf2::getYaw(robot_transform.transform.rotation);
    const PlanarPose approach = compute_object_approach_pose(
      object_point.point.x, object_point.point.y,
      robot_transform.transform.translation.x,
      robot_transform.transform.translation.y,
      robot_yaw, approach_distance_);

    geometry_msgs::msg::PoseStamped target;
    target.header.frame_id = global_frame_;
    target.header.stamp = now();
    target.pose.position.x = approach.x;
    target.pose.position.y = approach.y;
    target.pose.position.z = 0.0;
    target.pose.orientation.z = std::sin(approach.yaw * 0.5);
    target.pose.orientation.w = std::cos(approach.yaw * 0.5);
    return target;
  }

  void cancel_nav_goal(const std::shared_ptr<NavigateGoalHandle> & nav_goal)
  {
    if (!nav_goal || !navigate_client_ || !rclcpp::ok()) {
      return;
    }
    auto cancel_future = navigate_client_->async_cancel_goal(nav_goal);
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(cancel_timeout_);
    while (cancel_future.wait_for(50ms) != std::future_status::ready &&
      rclcpp::ok() && !stopping_.load() &&
      std::chrono::steady_clock::now() < deadline)
    {
    }
  }

  template<typename FutureT>
  void cancel_nav_goal_when_available(FutureT & nav_goal_future)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(cancel_timeout_);
    while (nav_goal_future.wait_for(50ms) != std::future_status::ready &&
      rclcpp::ok() && !stopping_.load() &&
      std::chrono::steady_clock::now() < deadline)
    {
    }
    if (nav_goal_future.wait_for(0ms) == std::future_status::ready) {
      cancel_nav_goal(nav_goal_future.get());
    }
  }

  void cancel_command(
    const std::shared_ptr<CommandGoalHandle> & goal_handle,
    const std::string & message,
    const geometry_msgs::msg::PoseStamped & target_pose)
  {
    auto result = std::make_shared<CommandAction::Result>();
    result->success = false;
    result->message = message;
    result->target_pose = target_pose;
    if (goal_handle->is_canceling()) {
      goal_handle->canceled(result);
    }
  }

  void abort_command(
    const std::shared_ptr<CommandGoalHandle> & goal_handle,
    const std::string & message,
    const geometry_msgs::msg::PoseStamped & target_pose)
  {
    auto result = std::make_shared<CommandAction::Result>();
    result->success = false;
    result->message = message;
    result->target_pose = target_pose;
    if (goal_handle->is_canceling()) {
      goal_handle->canceled(result);
    } else if (goal_handle->is_active()) {
      goal_handle->abort(result);
    }
    RCLCPP_WARN(get_logger(), "Go-to-object failed: %s", message.c_str());
  }

  void execute(const std::shared_ptr<CommandGoalHandle> & goal_handle)
  {
    const std::string object_id = goal_handle->get_goal()->object_id;
    geometry_msgs::msg::PoseStamped target_pose;
    publish_feedback(
      goal_handle, kLookupPhase, "querying object registry", -1.0F,
      target_pose);

    auto endpoint_status = wait_for_endpoint(
      [this]() {return registry_client_->service_is_ready();},
      goal_handle, registry_timeout_);
    if (endpoint_status == WaitStatus::canceled) {
      cancel_command(goal_handle, "command canceled before registry lookup", target_pose);
      return;
    }
    if (endpoint_status != WaitStatus::ready) {
      abort_command(
        goal_handle, "object registry service is unavailable: " + registry_service_,
        target_pose);
      return;
    }

    auto request = std::make_shared<RegistryService::Request>();
    request->name = object_id;
    request->label = "";
    auto registry_future = registry_client_->async_send_request(request);
    const auto registry_status = wait_for_future(
      registry_future, goal_handle, registry_timeout_);
    if (registry_status == WaitStatus::canceled) {
      cancel_command(goal_handle, "command canceled during registry lookup", target_pose);
      return;
    }
    if (registry_status != WaitStatus::ready) {
      abort_command(
        goal_handle, "timed out querying object registry for '" + object_id + "'",
        target_pose);
      return;
    }

    const auto registry_response = registry_future.get();
    const auto & registry_result = registry_response->result;
    if (registry_result.objects.empty()) {
      abort_command(
        goal_handle, "object id '" + object_id + "' is not in the registry",
        target_pose);
      return;
    }
    if (registry_result.objects.size() != 1U) {
      abort_command(
        goal_handle, "object id '" + object_id + "' is not unique in the registry",
        target_pose);
      return;
    }
    if (registry_result.header.frame_id.empty()) {
      abort_command(
        goal_handle, "registry returned an object without a coordinate frame",
        target_pose);
      return;
    }

    const auto & object = registry_result.objects.front();
    if (!std::isfinite(object.position.x) || !std::isfinite(object.position.y) ||
      !std::isfinite(object.position.z))
    {
      abort_command(
        goal_handle, "registry returned a non-finite centroid for '" + object_id + "'",
        target_pose);
      return;
    }

    try {
      const auto object_point = transform_object_to_global(registry_result, object);
      target_pose = make_target_pose(object_point);
    } catch (const tf2::TransformException & error) {
      abort_command(
        goal_handle, std::string("TF could not place the object and robot in '") +
        global_frame_ + "': " + error.what(), target_pose);
      return;
    }

    target_pose_publisher_->publish(target_pose);
    publish_feedback(
      goal_handle, kPlanningPhase, "submitting standoff pose to Nav2", -1.0F,
      target_pose);

    endpoint_status = wait_for_endpoint(
      [this]() {return navigate_client_->action_server_is_ready();},
      goal_handle, nav_server_timeout_);
    if (endpoint_status == WaitStatus::canceled) {
      cancel_command(goal_handle, "command canceled before Nav2 dispatch", target_pose);
      return;
    }
    if (endpoint_status != WaitStatus::ready) {
      abort_command(
        goal_handle, "Nav2 action server is unavailable: " + navigate_action_,
        target_pose);
      return;
    }

    NavigateAction::Goal nav_goal;
    nav_goal.pose = target_pose;
    nav_goal.behavior_tree = behavior_tree_;
    rclcpp_action::Client<NavigateAction>::SendGoalOptions nav_options;
    std::weak_ptr<CommandGoalHandle> weak_command_goal = goal_handle;
    nav_options.feedback_callback =
      [this, weak_command_goal, target_pose](
      NavigateGoalHandle::SharedPtr,
      const std::shared_ptr<const NavigateAction::Feedback> feedback)
      {
        const auto command_goal = weak_command_goal.lock();
        if (!command_goal) {
          return;
        }
        publish_feedback(
          command_goal, kNavigatingPhase, "Nav2 is approaching the object",
          feedback->distance_remaining, target_pose);
      };

    auto nav_goal_future = navigate_client_->async_send_goal(nav_goal, nav_options);
    const auto nav_goal_status = wait_for_future(
      nav_goal_future, goal_handle, nav_server_timeout_);
    if (nav_goal_status == WaitStatus::canceled) {
      cancel_nav_goal_when_available(nav_goal_future);
      cancel_command(goal_handle, "command canceled during Nav2 dispatch", target_pose);
      return;
    }
    if (nav_goal_status == WaitStatus::stopping) {
      cancel_nav_goal_when_available(nav_goal_future);
      return;
    }
    if (nav_goal_status == WaitStatus::timeout) {
      cancel_nav_goal_when_available(nav_goal_future);
      abort_command(goal_handle, "Nav2 did not accept the goal in time", target_pose);
      return;
    }

    const auto accepted_nav_goal = nav_goal_future.get();
    if (!accepted_nav_goal) {
      abort_command(goal_handle, "Nav2 rejected the object approach pose", target_pose);
      return;
    }
    publish_feedback(
      goal_handle, kNavigatingPhase, "Nav2 accepted the object approach pose",
      -1.0F, target_pose);

    auto nav_result_future = navigate_client_->async_get_result(accepted_nav_goal);
    const auto nav_result_status = wait_for_future(
      nav_result_future, goal_handle, navigation_timeout_);
    if (nav_result_status == WaitStatus::canceled) {
      cancel_nav_goal(accepted_nav_goal);
      cancel_command(goal_handle, "go-to-object command canceled", target_pose);
      return;
    }
    if (nav_result_status == WaitStatus::stopping) {
      cancel_nav_goal(accepted_nav_goal);
      return;
    }
    if (nav_result_status == WaitStatus::timeout) {
      cancel_nav_goal(accepted_nav_goal);
      abort_command(goal_handle, "navigation timeout expired", target_pose);
      return;
    }

    const auto nav_result = nav_result_future.get();
    if (nav_result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      auto result = std::make_shared<CommandAction::Result>();
      result->success = true;
      result->message = "reached standoff pose facing object '" + object_id + "'";
      result->target_pose = target_pose;
      goal_handle->succeed(result);
      RCLCPP_INFO(
        get_logger(), "Reached object '%s' at standoff %.2f m",
        object_id.c_str(), approach_distance_);
      return;
    }

    if (goal_handle->is_canceling()) {
      cancel_command(goal_handle, "go-to-object command canceled", target_pose);
      return;
    }
    if (nav_result.code == rclcpp_action::ResultCode::CANCELED) {
      abort_command(goal_handle, "Nav2 goal was canceled externally", target_pose);
    } else if (nav_result.code == rclcpp_action::ResultCode::ABORTED) {
      abort_command(goal_handle, "Nav2 could not reach the object approach pose", target_pose);
    } else {
      abort_command(goal_handle, "Nav2 returned an unknown result", target_pose);
    }
  }

  std::string action_name_;
  std::string registry_service_;
  std::string navigate_action_;
  std::string target_pose_topic_;
  std::string global_frame_;
  std::string robot_base_frame_;
  std::string behavior_tree_;
  double approach_distance_{0.75};
  double registry_timeout_{3.0};
  double nav_server_timeout_{5.0};
  double tf_timeout_{0.2};
  double navigation_timeout_{0.0};
  double cancel_timeout_{2.0};

  rclcpp::Client<RegistryService>::SharedPtr registry_client_;
  rclcpp_action::Client<NavigateAction>::SharedPtr navigate_client_;
  rclcpp_action::Server<CommandAction>::SharedPtr command_server_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
    target_pose_publisher_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::atomic<bool> stopping_{false};
  std::mutex worker_mutex_;
  std::condition_variable worker_condition_;
  std::thread worker_;
  bool busy_{false};
  std::shared_ptr<CommandGoalHandle> pending_goal_;
};

}  // namespace muto_command_layer

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<muto_command_layer::CommandLayerNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
