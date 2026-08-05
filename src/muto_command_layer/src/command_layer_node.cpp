#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <functional>
#include <future>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
#include <utility>

#include "frontier_exploration_ros2/srv/control_exploration.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "muto_command_layer/action/explore_and_record.hpp"
#include "muto_command_layer/action/go_to_object.hpp"
#include "muto_command_layer/approach_geometry.hpp"
#include "muto_command_layer/visibility_viewpoint_planner.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/spin.hpp"
#include "nav2_msgs/srv/get_costmap.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sam2_object_registry/srv/get_stored_objects.hpp"
#include "slam_toolbox/srv/save_map.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2/exceptions.hpp"
#include "tf2/time.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace muto_command_layer
{

using namespace std::chrono_literals;

template<typename ResultT, typename = void>
struct HasSpinErrorDetails : std::false_type {};

template<typename ResultT>
struct HasSpinErrorDetails<ResultT, std::void_t<
    decltype(std::declval<const ResultT &>().error_code),
    decltype(std::declval<const ResultT &>().error_msg),
    decltype(ResultT::NONE)>>: std::true_type {};

template<typename ResultT>
bool spin_result_reports_failure(const ResultT & result)
{
  if constexpr (HasSpinErrorDetails<ResultT>::value) {
    return result.error_code != ResultT::NONE;
  }
  return false;
}

template<typename ResultT>
std::string spin_result_failure_message(const ResultT & result)
{
  if constexpr (HasSpinErrorDetails<ResultT>::value) {
    if (!result.error_msg.empty()) {
      return "Nav2 spin failed: " + result.error_msg;
    }
  }
  return "Nav2 spin failed";
}

class CommandLayerNode : public rclcpp::Node
{
public:
  using CommandAction = muto_command_layer::action::GoToObject;
  using CommandGoalHandle = rclcpp_action::ServerGoalHandle<CommandAction>;
  using ProgramAction = muto_command_layer::action::ExploreAndRecord;
  using ProgramGoalHandle = rclcpp_action::ServerGoalHandle<ProgramAction>;
  using NavigateAction = nav2_msgs::action::NavigateToPose;
  using NavigateGoalHandle = rclcpp_action::ClientGoalHandle<NavigateAction>;
  using SpinAction = nav2_msgs::action::Spin;
  using SpinGoalHandle = rclcpp_action::ClientGoalHandle<SpinAction>;
  using CostmapService = nav2_msgs::srv::GetCostmap;
  using RegistryService = sam2_object_registry::srv::GetStoredObjects;
  using RegistrySaveService = std_srvs::srv::Trigger;
  using ExploreService = std_srvs::srv::SetBool;
  using SaveMapService = slam_toolbox::srv::SaveMap;
  using FrontierControlService =
    frontier_exploration_ros2::srv::ControlExploration;

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
    registry_save_client_ = create_client<RegistrySaveService>(
      registry_save_service_);
    global_costmap_client_ = create_client<CostmapService>(
      global_costmap_service_);
    navigate_client_ = rclcpp_action::create_client<NavigateAction>(
      this, navigate_action_);
    spin_client_ = rclcpp_action::create_client<SpinAction>(
      this, spin_action_);
    frontier_client_group_ = create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);
    explore_service_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    save_map_client_group_ = create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);
    save_map_service_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    frontier_control_client_ = create_client<FrontierControlService>(
      frontier_control_service_, rmw_qos_profile_services_default,
      frontier_client_group_);
    slam_toolbox_save_map_client_ = create_client<SaveMapService>(
      slam_toolbox_save_map_service_, rmw_qos_profile_services_default,
      save_map_client_group_);
    explore_service_ = create_service<ExploreService>(
      explore_service_name_,
      [this](
        const std::shared_ptr<ExploreService::Request> request,
        std::shared_ptr<ExploreService::Response> response)
      {
        handle_explore_request(request, response);
      },
      rmw_qos_profile_services_default, explore_service_group_);
    save_map_service_ = create_service<SaveMapService>(
      save_map_service_name_,
      [this](
        const std::shared_ptr<SaveMapService::Request> request,
        std::shared_ptr<SaveMapService::Response> response)
      {
        handle_save_map_request(request, response);
      },
      rmw_qos_profile_services_default, save_map_service_group_);
    exploration_completion_sub_ = create_subscription<std_msgs::msg::Empty>(
      exploration_completion_topic_,
      rclcpp::QoS(1).reliable().durability_volatile(),
      [this](std_msgs::msg::Empty::ConstSharedPtr)
      {
        if (program_active_.load()) {
          exploration_completed_.store(true);
        }
      });
    visibility_map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      visibility_map_topic_,
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr map)
      {
        std::lock_guard<std::mutex> lock(visibility_map_mutex_);
        latest_visibility_map_ = std::move(map);
      });
    target_pose_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      target_pose_topic_, rclcpp::QoS(1).reliable().transient_local());
    visibility_target_publisher_ =
      create_publisher<geometry_msgs::msg::PoseStamped>(
      visibility_target_pose_topic_,
      rclcpp::QoS(1).reliable().transient_local());

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
    program_server_ = rclcpp_action::create_server<ProgramAction>(
      this,
      program_action_name_,
      std::bind(
        &CommandLayerNode::handle_program_goal, this,
        std::placeholders::_1, std::placeholders::_2),
      std::bind(
        &CommandLayerNode::handle_program_cancel, this,
        std::placeholders::_1),
      std::bind(
        &CommandLayerNode::handle_program_accepted, this,
        std::placeholders::_1));

    worker_ = std::thread(&CommandLayerNode::worker_loop, this);
    program_worker_ = std::thread(&CommandLayerNode::program_worker_loop, this);

    RCLCPP_INFO(
      get_logger(),
      "Object command layer ready: action=%s registry=%s nav2=%s "
      "explore=%s program=%s spin=%s frontier_control=%s "
      "save_map=%s->%s map_directory=%s "
      "coverage=%s visibility_map=%s global_costmap=%s "
      "minimum_approach_radius=%.2f m frames=%s<-%s",
      action_name_.c_str(), registry_service_.c_str(), navigate_action_.c_str(),
      explore_service_name_.c_str(), program_action_name_.c_str(),
      spin_action_.c_str(), frontier_control_service_.c_str(),
      save_map_service_name_.c_str(),
      slam_toolbox_save_map_service_.c_str(), map_save_directory_.c_str(),
      visibility_coverage_enabled_ ? "on" : "off",
      visibility_map_topic_.c_str(), global_costmap_service_.c_str(),
      approach_distance_, global_frame_.c_str(), robot_base_frame_.c_str());
  }

  ~CommandLayerNode() override
  {
    stopping_.store(true);
    worker_condition_.notify_all();
    program_condition_.notify_all();
    if (worker_.joinable()) {
      worker_.join();
    }
    if (program_worker_.joinable()) {
      program_worker_.join();
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

  enum class ProgramDelayStatus
  {
    elapsed,
    canceled,
    exploration_complete,
    stopping,
  };

  enum class CoverageStatus
  {
    complete,
    canceled,
    failed,
    stopping,
  };

  static constexpr uint8_t kLookupPhase = 1;
  static constexpr uint8_t kPlanningPhase = 2;
  static constexpr uint8_t kNavigatingPhase = 3;
  static constexpr uint8_t kProgramExploringPhase = 1;
  static constexpr uint8_t kProgramPausingPhase = 2;
  static constexpr uint8_t kProgramSpinningPhase = 3;
  static constexpr uint8_t kProgramObservingPhase = 4;
  static constexpr uint8_t kProgramSavingPhase = 5;
  static constexpr uint8_t kProgramCoveringPhase = 6;
  static constexpr double kPi = 3.14159265358979323846;

  static std::string trim_copy(const std::string & value)
  {
    const auto first = std::find_if_not(
      value.begin(), value.end(),
      [](const unsigned char character) {return std::isspace(character);});
    const auto last = std::find_if_not(
      value.rbegin(), value.rend(),
      [](const unsigned char character) {return std::isspace(character);})
      .base();
    return first < last ? std::string(first, last) : std::string();
  }

  static bool is_ascii_alphanumeric(const unsigned char character)
  {
    return (character >= 'a' && character <= 'z') ||
           (character >= 'A' && character <= 'Z') ||
           (character >= '0' && character <= '9');
  }

  static bool valid_map_name(const std::string & value)
  {
    if (value.empty() || value.size() > 128U ||
      value.find("..") != std::string::npos)
    {
      return false;
    }
    if (!is_ascii_alphanumeric(
        static_cast<unsigned char>(value.front())))
    {
      return false;
    }
    return std::all_of(
      value.begin(), value.end(),
      [](const unsigned char character) {
        return is_ascii_alphanumeric(character) || character == '_' ||
               character == '-' || character == '.';
      });
  }

  static bool valid_map_directory(const std::string & value)
  {
    // Humble's map saver inserts this prefix into a shell command.
    return std::all_of(
      value.begin(), value.end(),
      [](const unsigned char character) {
        return is_ascii_alphanumeric(character) || character == '_' ||
               character == '-' || character == '.' || character == '/';
      });
  }

  void declare_parameters()
  {
    declare_parameter<std::string>("action_name", "/go_to_object");
    declare_parameter<std::string>(
      "registry_service", "/sam2/get_stored_objects");
    declare_parameter<std::string>(
      "navigate_to_pose_action", "/navigate_to_pose");
    declare_parameter<std::string>(
      "global_costmap_service", "/global_costmap/get_costmap");
    declare_parameter<std::string>(
      "target_pose_topic", "/object_navigation/target_pose");
    declare_parameter<std::string>("explore_service", "/explore");
    declare_parameter<std::string>("save_map_service", "/save_map");
    declare_parameter<std::string>(
      "slam_toolbox_save_map_service", "/slam_toolbox/save_map");
    declare_parameter<std::string>("map_save_directory", "");
    declare_parameter<std::string>("default_map_name", "muto_map");
    declare_parameter<std::string>(
      "explore_and_record_action", "/explore_and_record");
    declare_parameter<std::string>(
      "frontier_control_service", "/control_exploration");
    declare_parameter<std::string>("spin_action", "/spin");
    declare_parameter<std::string>(
      "registry_save_service", "/sam2/save_stored_objects");
    declare_parameter<std::string>(
      "exploration_completion_topic", "/explore/exploration_complete");
    declare_parameter<bool>("visibility_coverage_enabled", true);
    declare_parameter<std::string>("visibility_map_topic", "/map");
    declare_parameter<std::string>(
      "visibility_target_pose_topic", "/explore/visibility_target_pose");
    declare_parameter<std::string>("global_frame", "map");
    declare_parameter<std::string>("robot_base_frame", "base_frame");
    declare_parameter<double>("approach_distance", 0.75);
    declare_parameter<double>("approach_robot_radius", 0.16);
    declare_parameter<double>("approach_start_snap_distance", 0.5);
    declare_parameter<int64_t>("approach_maximum_cost", 252);
    declare_parameter<double>("registry_timeout", 3.0);
    declare_parameter<double>("nav_server_timeout", 5.0);
    declare_parameter<double>("global_costmap_timeout", 5.0);
    declare_parameter<double>("tf_timeout", 0.2);
    declare_parameter<double>("navigation_timeout", 0.0);
    declare_parameter<double>("cancel_timeout", 2.0);
    declare_parameter<double>("exploration_service_timeout", 5.0);
    declare_parameter<double>("save_map_timeout", 10.0);
    declare_parameter<double>("program_endpoint_timeout", 5.0);
    declare_parameter<double>("exploration_cycle_duration", 10.0);
    declare_parameter<double>("observation_duration", 3.0);
    declare_parameter<int64_t>("scan_step_count", 8);
    declare_parameter<double>("spin_time_allowance", 15.0);
    declare_parameter<double>("navigation_settle_time", 1.0);
    declare_parameter<double>("visibility_map_timeout", 10.0);
    declare_parameter<int64_t>("visibility_free_threshold", 20);
    declare_parameter<int64_t>("visibility_occupied_threshold", 65);
    declare_parameter<int64_t>("visibility_maximum_cost", 252);
    declare_parameter<double>("visibility_robot_clearance", 0.22);
    declare_parameter<double>("visibility_candidate_spacing", 0.5);
    declare_parameter<double>("visibility_range", 2.5);
    declare_parameter<double>("visibility_boundary_weight", 2.0);
    declare_parameter<double>("visibility_nominal_linear_speed", 0.25);
    declare_parameter<double>("visibility_scan_time", 45.0);
    declare_parameter<double>("visibility_start_snap_distance", 0.5);
    declare_parameter<int64_t>("visibility_minimum_new_cells", 1);
    declare_parameter<double>("visibility_completion_ratio", 0.98);
    declare_parameter<int64_t>("visibility_max_viewpoints", 0);
    declare_parameter<int64_t>("visibility_max_navigation_failures", 3);
    declare_parameter<std::string>("behavior_tree", "");
  }

  void read_parameters()
  {
    action_name_ = get_parameter("action_name").as_string();
    registry_service_ = get_parameter("registry_service").as_string();
    navigate_action_ = get_parameter("navigate_to_pose_action").as_string();
    global_costmap_service_ =
      get_parameter("global_costmap_service").as_string();
    target_pose_topic_ = get_parameter("target_pose_topic").as_string();
    explore_service_name_ = get_parameter("explore_service").as_string();
    save_map_service_name_ = get_parameter("save_map_service").as_string();
    slam_toolbox_save_map_service_ =
      get_parameter("slam_toolbox_save_map_service").as_string();
    map_save_directory_ = get_parameter("map_save_directory").as_string();
    default_map_name_ = trim_copy(
      get_parameter("default_map_name").as_string());
    if (map_save_directory_.empty()) {
      const char * home = std::getenv("HOME");
      if (home == nullptr || *home == '\0') {
        throw std::invalid_argument(
                "map_save_directory is empty and HOME is unavailable");
      }
      map_save_directory_ =
        (std::filesystem::path(home) / ".ros" / "maps").string();
    }
    map_save_directory_ =
      std::filesystem::path(map_save_directory_).lexically_normal().string();
    program_action_name_ =
      get_parameter("explore_and_record_action").as_string();
    frontier_control_service_ =
      get_parameter("frontier_control_service").as_string();
    spin_action_ = get_parameter("spin_action").as_string();
    registry_save_service_ =
      get_parameter("registry_save_service").as_string();
    exploration_completion_topic_ =
      get_parameter("exploration_completion_topic").as_string();
    visibility_coverage_enabled_ =
      get_parameter("visibility_coverage_enabled").as_bool();
    visibility_map_topic_ = get_parameter("visibility_map_topic").as_string();
    visibility_target_pose_topic_ =
      get_parameter("visibility_target_pose_topic").as_string();
    global_frame_ = get_parameter("global_frame").as_string();
    robot_base_frame_ = get_parameter("robot_base_frame").as_string();
    approach_distance_ = get_parameter("approach_distance").as_double();
    approach_robot_radius_ =
      get_parameter("approach_robot_radius").as_double();
    approach_start_snap_distance_ =
      get_parameter("approach_start_snap_distance").as_double();
    approach_maximum_cost_ = get_parameter("approach_maximum_cost").as_int();
    registry_timeout_ = get_parameter("registry_timeout").as_double();
    nav_server_timeout_ = get_parameter("nav_server_timeout").as_double();
    global_costmap_timeout_ =
      get_parameter("global_costmap_timeout").as_double();
    tf_timeout_ = get_parameter("tf_timeout").as_double();
    navigation_timeout_ = get_parameter("navigation_timeout").as_double();
    cancel_timeout_ = get_parameter("cancel_timeout").as_double();
    exploration_service_timeout_ =
      get_parameter("exploration_service_timeout").as_double();
    save_map_timeout_ = get_parameter("save_map_timeout").as_double();
    program_endpoint_timeout_ =
      get_parameter("program_endpoint_timeout").as_double();
    exploration_cycle_duration_ =
      get_parameter("exploration_cycle_duration").as_double();
    observation_duration_ = get_parameter("observation_duration").as_double();
    scan_step_count_ = get_parameter("scan_step_count").as_int();
    spin_time_allowance_ = get_parameter("spin_time_allowance").as_double();
    navigation_settle_time_ =
      get_parameter("navigation_settle_time").as_double();
    visibility_map_timeout_ =
      get_parameter("visibility_map_timeout").as_double();
    visibility_planner_config_.free_threshold = static_cast<int>(
      get_parameter("visibility_free_threshold").as_int());
    visibility_planner_config_.occupied_threshold = static_cast<int>(
      get_parameter("visibility_occupied_threshold").as_int());
    visibility_planner_config_.maximum_traversable_cost = static_cast<int>(
      get_parameter("visibility_maximum_cost").as_int());
    visibility_planner_config_.robot_clearance =
      get_parameter("visibility_robot_clearance").as_double();
    visibility_planner_config_.candidate_spacing =
      get_parameter("visibility_candidate_spacing").as_double();
    visibility_planner_config_.visibility_range =
      get_parameter("visibility_range").as_double();
    visibility_planner_config_.boundary_weight =
      get_parameter("visibility_boundary_weight").as_double();
    visibility_planner_config_.nominal_linear_speed =
      get_parameter("visibility_nominal_linear_speed").as_double();
    visibility_planner_config_.scan_time =
      get_parameter("visibility_scan_time").as_double();
    visibility_planner_config_.start_snap_distance =
      get_parameter("visibility_start_snap_distance").as_double();
    visibility_planner_config_.minimum_new_target_cells =
      static_cast<std::size_t>(
      get_parameter("visibility_minimum_new_cells").as_int());
    visibility_completion_ratio_ =
      get_parameter("visibility_completion_ratio").as_double();
    visibility_max_viewpoints_ =
      get_parameter("visibility_max_viewpoints").as_int();
    visibility_max_navigation_failures_ =
      get_parameter("visibility_max_navigation_failures").as_int();
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
    require_name(global_costmap_service_, "global_costmap_service");
    require_name(target_pose_topic_, "target_pose_topic");
    require_name(explore_service_name_, "explore_service");
    require_name(save_map_service_name_, "save_map_service");
    require_name(
      slam_toolbox_save_map_service_, "slam_toolbox_save_map_service");
    require_name(map_save_directory_, "map_save_directory");
    require_name(program_action_name_, "explore_and_record_action");
    require_name(frontier_control_service_, "frontier_control_service");
    require_name(spin_action_, "spin_action");
    require_name(registry_save_service_, "registry_save_service");
    require_name(
      exploration_completion_topic_, "exploration_completion_topic");
    require_name(visibility_map_topic_, "visibility_map_topic");
    require_name(
      visibility_target_pose_topic_, "visibility_target_pose_topic");
    require_name(global_frame_, "global_frame");
    require_name(robot_base_frame_, "robot_base_frame");
    if (save_map_service_name_ == slam_toolbox_save_map_service_) {
      throw std::invalid_argument(
              "save_map_service and slam_toolbox_save_map_service must differ");
    }
    if (save_map_service_name_.front() != '/' ||
      slam_toolbox_save_map_service_.front() != '/')
    {
      throw std::invalid_argument(
              "save-map service names must be absolute");
    }
    if (!std::filesystem::path(map_save_directory_).is_absolute()) {
      throw std::invalid_argument("map_save_directory must be absolute");
    }
    if (!valid_map_directory(map_save_directory_)) {
      throw std::invalid_argument(
              "map_save_directory contains unsafe characters");
    }
    if (!valid_map_name(default_map_name_)) {
      throw std::invalid_argument(
              "default_map_name must be a safe basename of at most 128 characters");
    }

    if (!std::isfinite(approach_distance_) || approach_distance_ <= 0.0) {
      throw std::invalid_argument("approach_distance must be finite and positive");
    }
    if (!std::isfinite(approach_robot_radius_) ||
      approach_robot_radius_ <= 0.0)
    {
      throw std::invalid_argument(
              "approach_robot_radius must be finite and positive");
    }
    if (!std::isfinite(approach_start_snap_distance_) ||
      approach_start_snap_distance_ < 0.0)
    {
      throw std::invalid_argument(
              "approach_start_snap_distance must be finite and nonnegative");
    }
    if (approach_maximum_cost_ < 0 || approach_maximum_cost_ > 252) {
      throw std::invalid_argument("approach_maximum_cost must be in [0, 252]");
    }
    if (!std::isfinite(registry_timeout_) || registry_timeout_ <= 0.0) {
      throw std::invalid_argument("registry_timeout must be finite and positive");
    }
    if (!std::isfinite(nav_server_timeout_) || nav_server_timeout_ <= 0.0) {
      throw std::invalid_argument("nav_server_timeout must be finite and positive");
    }
    if (!std::isfinite(global_costmap_timeout_) ||
      global_costmap_timeout_ <= 0.0)
    {
      throw std::invalid_argument(
              "global_costmap_timeout must be finite and positive");
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
    if (!std::isfinite(exploration_service_timeout_) ||
      exploration_service_timeout_ <= 0.0)
    {
      throw std::invalid_argument(
              "exploration_service_timeout must be finite and positive");
    }
    if (!std::isfinite(save_map_timeout_) || save_map_timeout_ <= 0.0) {
      throw std::invalid_argument("save_map_timeout must be finite and positive");
    }
    if (!std::isfinite(program_endpoint_timeout_) ||
      program_endpoint_timeout_ <= 0.0)
    {
      throw std::invalid_argument(
              "program_endpoint_timeout must be finite and positive");
    }
    if (!std::isfinite(exploration_cycle_duration_) ||
      exploration_cycle_duration_ <= 0.0)
    {
      throw std::invalid_argument(
              "exploration_cycle_duration must be finite and positive");
    }
    if (!std::isfinite(observation_duration_) || observation_duration_ <= 0.0) {
      throw std::invalid_argument(
              "observation_duration must be finite and positive");
    }
    if (scan_step_count_ <= 0 || scan_step_count_ > 360) {
      throw std::invalid_argument("scan_step_count must be in [1, 360]");
    }
    if (!std::isfinite(spin_time_allowance_) || spin_time_allowance_ <= 0.0) {
      throw std::invalid_argument(
              "spin_time_allowance must be finite and positive");
    }
    if (!std::isfinite(navigation_settle_time_) ||
      navigation_settle_time_ < 0.0)
    {
      throw std::invalid_argument(
              "navigation_settle_time must be finite and nonnegative");
    }
    if (!std::isfinite(visibility_map_timeout_) ||
      visibility_map_timeout_ <= 0.0)
    {
      throw std::invalid_argument(
              "visibility_map_timeout must be finite and positive");
    }
    if (!std::isfinite(visibility_completion_ratio_) ||
      visibility_completion_ratio_ <= 0.0 ||
      visibility_completion_ratio_ > 1.0)
    {
      throw std::invalid_argument(
              "visibility_completion_ratio must be in (0, 1]");
    }
    if (visibility_max_viewpoints_ < 0) {
      throw std::invalid_argument("visibility_max_viewpoints must be nonnegative");
    }
    if (visibility_max_navigation_failures_ <= 0) {
      throw std::invalid_argument(
              "visibility_max_navigation_failures must be positive");
    }
    if (visibility_planner_config_.free_threshold < 0 ||
      visibility_planner_config_.free_threshold > 100 ||
      visibility_planner_config_.occupied_threshold < 0 ||
      visibility_planner_config_.occupied_threshold > 100 ||
      visibility_planner_config_.free_threshold >=
      visibility_planner_config_.occupied_threshold)
    {
      throw std::invalid_argument("visibility occupancy thresholds are invalid");
    }
    if (visibility_planner_config_.maximum_traversable_cost < 0 ||
      visibility_planner_config_.maximum_traversable_cost > 252)
    {
      throw std::invalid_argument(
              "visibility_maximum_cost must be in [0, 252]");
    }
    const auto require_positive_visibility = [](
      const double value, const char * name)
      {
        if (!std::isfinite(value) || value <= 0.0) {
          throw std::invalid_argument(
                  std::string(name) + " must be finite and positive");
        }
      };
    require_positive_visibility(
      visibility_planner_config_.robot_clearance,
      "visibility_robot_clearance");
    if (!std::isfinite(visibility_planner_config_.start_snap_distance) ||
      visibility_planner_config_.start_snap_distance < 0.0)
    {
      throw std::invalid_argument(
              "visibility_start_snap_distance must be finite and nonnegative");
    }
    require_positive_visibility(
      visibility_planner_config_.candidate_spacing,
      "visibility_candidate_spacing");
    require_positive_visibility(
      visibility_planner_config_.visibility_range, "visibility_range");
    require_positive_visibility(
      visibility_planner_config_.boundary_weight,
      "visibility_boundary_weight");
    require_positive_visibility(
      visibility_planner_config_.nominal_linear_speed,
      "visibility_nominal_linear_speed");
    require_positive_visibility(
      visibility_planner_config_.scan_time, "visibility_scan_time");
    if (visibility_planner_config_.minimum_new_target_cells == 0U ||
      visibility_planner_config_.minimum_new_target_cells >
      static_cast<std::size_t>(std::numeric_limits<int64_t>::max()))
    {
      throw std::invalid_argument("visibility_minimum_new_cells must be positive");
    }
  }

  static const char * frontier_state_name(const uint8_t state)
  {
    switch (state) {
      case FrontierControlService::Request::STATE_RUNNING:
        return "running";
      case FrontierControlService::Request::STATE_START_SCHEDULED:
        return "start scheduled";
      case FrontierControlService::Request::STATE_STOP_SCHEDULED:
        return "stop scheduled";
      case FrontierControlService::Request::STATE_STOPPING:
        return "stopping";
      case FrontierControlService::Request::STATE_SHUTDOWN_PENDING:
        return "shutdown pending";
      case FrontierControlService::Request::STATE_IDLE:
      default:
        return "idle";
    }
  }

  bool set_exploration_enabled(
    const bool enabled,
    const double timeout_seconds,
    std::string & message,
    uint8_t & state)
  {
    std::lock_guard<std::mutex> request_lock(frontier_request_mutex_);
    const char * operation = enabled ? "start" : "stop";
    const auto timeout = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(timeout_seconds));
    if (!frontier_control_client_->wait_for_service(timeout)) {
      message =
        "frontier exploration control service is unavailable: " +
        frontier_control_service_;
      return false;
    }

    auto frontier_request =
      std::make_shared<FrontierControlService::Request>();
    frontier_request->action = enabled ?
      FrontierControlService::Request::ACTION_START :
      FrontierControlService::Request::ACTION_STOP;
    frontier_request->delay_seconds = 0.0F;
    frontier_request->quit_after_stop = false;

    auto future = frontier_control_client_->async_send_request(frontier_request);
    if (future.wait_for(timeout) != std::future_status::ready) {
      frontier_control_client_->remove_pending_request(future);
      message =
        "timed out waiting for frontier exploration to " +
        std::string(operation);
      return false;
    }

    const auto frontier_response = future.get();
    state = frontier_response->state;
    message = frontier_response->message + " (state: " +
      frontier_state_name(frontier_response->state) + ")";
    return frontier_response->accepted;
  }

  void handle_explore_request(
    const std::shared_ptr<ExploreService::Request> request,
    std::shared_ptr<ExploreService::Response> response)
  {
    const char * operation = request->data ? "start" : "stop";
    if (program_active_.load()) {
      response->success = false;
      response->message =
        "explore-and-record owns exploration; cancel that action first";
      RCLCPP_WARN(
        get_logger(), "Explore %s rejected: %s", operation,
        response->message.c_str());
      return;
    }

    uint8_t state = FrontierControlService::Request::STATE_IDLE;
    response->success = set_exploration_enabled(
      request->data, exploration_service_timeout_, response->message, state);
    if (response->success) {
      RCLCPP_INFO(
        get_logger(), "Explore %s accepted: %s", operation,
        response->message.c_str());
    } else {
      RCLCPP_WARN(
        get_logger(), "Explore %s rejected: %s", operation,
        response->message.c_str());
    }
  }

  void handle_save_map_request(
    const std::shared_ptr<SaveMapService::Request> request,
    std::shared_ptr<SaveMapService::Response> response)
  {
    response->result = SaveMapService::Response::RESULT_UNDEFINED_FAILURE;
    std::string map_name = trim_copy(request->name.data);
    if (map_name.empty()) {
      map_name = default_map_name_;
    }
    if (!valid_map_name(map_name)) {
      RCLCPP_WARN(
        get_logger(),
        "Save-map request rejected: name must be a safe basename of at most "
        "128 characters");
      return;
    }

    std::error_code error;
    const std::filesystem::path directory(map_save_directory_);
    std::filesystem::create_directories(directory, error);
    if (error) {
      RCLCPP_ERROR(
        get_logger(), "Cannot use map-save directory '%s': %s",
        map_save_directory_.c_str(), error.message().c_str());
      return;
    }
    if (!std::filesystem::is_directory(directory, error)) {
      const std::string reason = error ? error.message() : "not a directory";
      RCLCPP_ERROR(
        get_logger(), "Cannot use map-save directory '%s': %s",
        map_save_directory_.c_str(), reason.c_str());
      return;
    }

    const auto timeout = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(save_map_timeout_));
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    if (!slam_toolbox_save_map_client_->wait_for_service(timeout)) {
      RCLCPP_WARN(
        get_logger(), "SLAM Toolbox save-map service is unavailable: %s",
        slam_toolbox_save_map_service_.c_str());
      return;
    }

    const std::filesystem::path map_prefix = directory / map_name;
    try {
      auto forwarded_request = std::make_shared<SaveMapService::Request>();
      forwarded_request->name.data = map_prefix.string();
      auto future =
        slam_toolbox_save_map_client_->async_send_request(forwarded_request);
      const auto remaining = deadline - std::chrono::steady_clock::now();
      if (remaining <= std::chrono::steady_clock::duration::zero() ||
        future.wait_for(remaining) != std::future_status::ready)
      {
        slam_toolbox_save_map_client_->remove_pending_request(future);
        RCLCPP_WARN(
          get_logger(), "Timed out saving map through %s",
          slam_toolbox_save_map_service_.c_str());
        return;
      }
      response->result = future.get()->result;
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        get_logger(), "SLAM Toolbox map-save request failed: %s",
        error.what());
      return;
    }
    if (response->result == SaveMapService::Response::RESULT_SUCCESS) {
      RCLCPP_INFO(
        get_logger(), "Saved occupancy map with prefix '%s'",
        map_prefix.c_str());
    } else if (
      response->result == SaveMapService::Response::RESULT_NO_MAP_RECEIEVD)
    {
      RCLCPP_WARN(get_logger(), "SLAM Toolbox has no map to save yet");
    } else {
      RCLCPP_WARN(
        get_logger(), "SLAM Toolbox map save failed with result code %u",
        static_cast<unsigned int>(response->result));
    }
  }

  rclcpp_action::GoalResponse handle_program_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const ProgramAction::Goal> goal)
  {
    const bool durations_valid =
      std::isfinite(goal->exploration_duration) &&
      goal->exploration_duration >= 0.0F &&
      std::isfinite(goal->observation_duration) &&
      goal->observation_duration >= 0.0F;
    const bool scan_step_count_valid = goal->scan_step_count <= 360U;
    if (!durations_valid || !scan_step_count_valid) {
      RCLCPP_WARN(
        get_logger(),
        "Rejected explore-and-record goal with invalid duration or scan step count");
      return rclcpp_action::GoalResponse::REJECT;
    }

    std::lock_guard<std::mutex> lock(worker_mutex_);
    if (busy_ || stopping_.load()) {
      RCLCPP_WARN(
        get_logger(),
        "Rejected explore-and-record goal: command layer is busy");
      return rclcpp_action::GoalResponse::REJECT;
    }
    busy_ = true;
    program_active_.store(true);
    exploration_completed_.store(false);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_program_cancel(
    const std::shared_ptr<ProgramGoalHandle>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_program_accepted(
    const std::shared_ptr<ProgramGoalHandle> goal_handle)
  {
    {
      std::lock_guard<std::mutex> lock(worker_mutex_);
      pending_program_goal_ = goal_handle;
    }
    program_condition_.notify_one();
  }

  void program_worker_loop()
  {
    while (!stopping_.load()) {
      std::shared_ptr<ProgramGoalHandle> goal_handle;
      {
        std::unique_lock<std::mutex> lock(worker_mutex_);
        program_condition_.wait(
          lock,
          [this]() {
            return stopping_.load() || pending_program_goal_ != nullptr;
          });
        if (stopping_.load()) {
          break;
        }
        goal_handle = std::move(pending_program_goal_);
      }

      try {
        execute_program(goal_handle);
      } catch (const std::exception & error) {
        RCLCPP_ERROR(
          get_logger(), "Unhandled explore-and-record error: %s", error.what());
        abort_program(
          goal_handle,
          std::string("internal explore-and-record error: ") + error.what(),
          0U, 0U, 0U);
      }

      program_active_.store(false);
      exploration_completed_.store(false);
      {
        std::lock_guard<std::mutex> lock(worker_mutex_);
        busy_ = false;
      }
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

  template<typename FutureT, typename GoalHandleT>
  WaitStatus wait_for_future(
    FutureT & future,
    const std::shared_ptr<GoalHandleT> & goal_handle,
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

  template<typename ReadyFunction, typename GoalHandleT>
  WaitStatus wait_for_endpoint(
    ReadyFunction ready_function,
    const std::shared_ptr<GoalHandleT> & goal_handle,
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

  void publish_program_feedback(
    const std::shared_ptr<ProgramGoalHandle> & goal_handle,
    const uint8_t phase,
    const std::string & status,
    const uint32_t completed_cycles,
    const uint32_t object_count)
  {
    if (!goal_handle->is_active()) {
      return;
    }
    auto feedback = std::make_shared<ProgramAction::Feedback>();
    feedback->phase = phase;
    feedback->status = status;
    feedback->completed_cycles = completed_cycles;
    feedback->object_count = object_count;
    goal_handle->publish_feedback(feedback);
  }

  ProgramDelayStatus wait_program_delay(
    const std::shared_ptr<ProgramGoalHandle> & goal_handle,
    const double duration_seconds,
    const bool watch_exploration_completion = true)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(duration_seconds);
    while (std::chrono::steady_clock::now() < deadline) {
      if (goal_handle->is_canceling()) {
        return ProgramDelayStatus::canceled;
      }
      if (stopping_.load() || !rclcpp::ok()) {
        return ProgramDelayStatus::stopping;
      }
      if (watch_exploration_completion && exploration_completed_.load()) {
        return ProgramDelayStatus::exploration_complete;
      }
      std::this_thread::sleep_for(50ms);
    }
    return ProgramDelayStatus::elapsed;
  }

  void best_effort_stop_exploration()
  {
    if (!frontier_control_client_ || !rclcpp::ok()) {
      return;
    }
    std::string message;
    uint8_t state = FrontierControlService::Request::STATE_IDLE;
    if (!set_exploration_enabled(
        false, program_endpoint_timeout_, message, state))
    {
      RCLCPP_WARN(
        get_logger(), "Could not stop exploration during program cleanup: %s",
        message.c_str());
    }
  }

  void best_effort_checkpoint_registry()
  {
    if (!registry_save_client_ || !rclcpp::ok()) {
      return;
    }
    const auto timeout = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(program_endpoint_timeout_));
    if (!registry_save_client_->wait_for_service(timeout)) {
      RCLCPP_WARN(
        get_logger(), "Registry checkpoint service is unavailable: %s",
        registry_save_service_.c_str());
      return;
    }
    auto future = registry_save_client_->async_send_request(
      std::make_shared<RegistrySaveService::Request>());
    if (future.wait_for(timeout) != std::future_status::ready) {
      registry_save_client_->remove_pending_request(future);
      RCLCPP_WARN(get_logger(), "Timed out checkpointing the object registry");
      return;
    }
    const auto response = future.get();
    if (!response->success) {
      RCLCPP_WARN(
        get_logger(), "Object registry checkpoint failed: %s",
        response->message.c_str());
    }
  }

  void cancel_program(
    const std::shared_ptr<ProgramGoalHandle> & goal_handle,
    const std::string & message,
    const uint32_t completed_cycles,
    const uint32_t objects_before,
    const uint32_t objects_after)
  {
    best_effort_stop_exploration();
    best_effort_checkpoint_registry();
    auto result = std::make_shared<ProgramAction::Result>();
    result->success = false;
    result->message = message;
    result->completed_cycles = completed_cycles;
    result->objects_before = objects_before;
    result->objects_after = objects_after;
    if (goal_handle->is_canceling()) {
      goal_handle->canceled(result);
    }
  }

  void abort_program(
    const std::shared_ptr<ProgramGoalHandle> & goal_handle,
    const std::string & message,
    const uint32_t completed_cycles,
    const uint32_t objects_before,
    const uint32_t objects_after)
  {
    best_effort_stop_exploration();
    best_effort_checkpoint_registry();
    auto result = std::make_shared<ProgramAction::Result>();
    result->success = false;
    result->message = message;
    result->completed_cycles = completed_cycles;
    result->objects_before = objects_before;
    result->objects_after = objects_after;
    if (goal_handle->is_canceling()) {
      goal_handle->canceled(result);
    } else if (goal_handle->is_active()) {
      goal_handle->abort(result);
    }
    RCLCPP_WARN(get_logger(), "Explore-and-record failed: %s", message.c_str());
  }

  void succeed_program(
    const std::shared_ptr<ProgramGoalHandle> & goal_handle,
    const std::string & message,
    const uint32_t completed_cycles,
    const uint32_t objects_before,
    const uint32_t objects_after)
  {
    auto result = std::make_shared<ProgramAction::Result>();
    result->success = true;
    result->message = message;
    result->completed_cycles = completed_cycles;
    result->objects_before = objects_before;
    result->objects_after = objects_after;
    if (goal_handle->is_active()) {
      goal_handle->succeed(result);
    }
    RCLCPP_INFO(
      get_logger(),
      "Explore-and-record completed: cycles=%u objects=%u->%u (%s)",
      completed_cycles, objects_before, objects_after, message.c_str());
  }

  WaitStatus query_program_object_count(
    const std::shared_ptr<ProgramGoalHandle> & goal_handle,
    uint32_t & object_count,
    std::string & error)
  {
    auto endpoint_status = wait_for_endpoint(
      [this]() {return registry_client_->service_is_ready();},
      goal_handle, program_endpoint_timeout_);
    if (endpoint_status != WaitStatus::ready) {
      error = endpoint_status == WaitStatus::timeout ?
        "object registry service is unavailable: " + registry_service_ :
        "object registry query interrupted";
      return endpoint_status;
    }

    auto request = std::make_shared<RegistryService::Request>();
    request->name = "";
    request->label = "";
    auto future = registry_client_->async_send_request(request);
    const auto status = wait_for_future(
      future, goal_handle, program_endpoint_timeout_);
    if (status != WaitStatus::ready) {
      if (status == WaitStatus::timeout) {
        registry_client_->remove_pending_request(future);
        error = "timed out querying the object registry";
      } else {
        error = "object registry query interrupted";
      }
      return status;
    }

    const auto count = future.get()->result.objects.size();
    object_count = static_cast<uint32_t>(std::min(
        count,
        static_cast<size_t>(std::numeric_limits<uint32_t>::max())));
    return WaitStatus::ready;
  }

  WaitStatus checkpoint_program_registry(
    const std::shared_ptr<ProgramGoalHandle> & goal_handle,
    std::string & error)
  {
    auto endpoint_status = wait_for_endpoint(
      [this]() {return registry_save_client_->service_is_ready();},
      goal_handle, program_endpoint_timeout_);
    if (endpoint_status != WaitStatus::ready) {
      error = endpoint_status == WaitStatus::timeout ?
        "registry checkpoint service is unavailable: " + registry_save_service_ :
        "registry checkpoint interrupted";
      return endpoint_status;
    }

    auto future = registry_save_client_->async_send_request(
      std::make_shared<RegistrySaveService::Request>());
    const auto status = wait_for_future(
      future, goal_handle, program_endpoint_timeout_);
    if (status != WaitStatus::ready) {
      if (status == WaitStatus::timeout) {
        registry_save_client_->remove_pending_request(future);
        error = "timed out checkpointing the object registry";
      } else {
        error = "registry checkpoint interrupted";
      }
      return status;
    }
    const auto response = future.get();
    if (!response->success) {
      error = "registry checkpoint failed: " + response->message;
      return WaitStatus::timeout;
    }
    return WaitStatus::ready;
  }

  void cancel_spin_goal(const std::shared_ptr<SpinGoalHandle> & spin_goal)
  {
    if (!spin_goal || !spin_client_ || !rclcpp::ok()) {
      return;
    }
    auto cancel_future = spin_client_->async_cancel_goal(spin_goal);
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(cancel_timeout_);
    while (cancel_future.wait_for(50ms) != std::future_status::ready &&
      rclcpp::ok() && !stopping_.load() &&
      std::chrono::steady_clock::now() < deadline)
    {
    }
  }

  template<typename FutureT>
  void cancel_spin_goal_when_available(FutureT & spin_goal_future)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(cancel_timeout_);
    while (spin_goal_future.wait_for(50ms) != std::future_status::ready &&
      rclcpp::ok() && !stopping_.load() &&
      std::chrono::steady_clock::now() < deadline)
    {
    }
    if (spin_goal_future.wait_for(0ms) == std::future_status::ready) {
      cancel_spin_goal(spin_goal_future.get());
    }
  }

  WaitStatus perform_program_spin(
    const std::shared_ptr<ProgramGoalHandle> & goal_handle,
    const double spin_angle_degrees,
    const uint32_t scan_step,
    const uint32_t scan_step_count,
    const uint32_t completed_cycles,
    const uint32_t object_count,
    std::string & error)
  {
    auto endpoint_status = wait_for_endpoint(
      [this]() {return spin_client_->action_server_is_ready();},
      goal_handle, program_endpoint_timeout_);
    if (endpoint_status != WaitStatus::ready) {
      error = endpoint_status == WaitStatus::timeout ?
        "Nav2 spin action is unavailable: " + spin_action_ :
        "spin dispatch interrupted";
      return endpoint_status;
    }

    SpinAction::Goal spin_goal;
    spin_goal.target_yaw = static_cast<float>(
      spin_angle_degrees * kPi / 180.0);
    spin_goal.time_allowance =
      rclcpp::Duration::from_seconds(spin_time_allowance_);
    rclcpp_action::Client<SpinAction>::SendGoalOptions options;
    std::weak_ptr<ProgramGoalHandle> weak_goal = goal_handle;
    options.feedback_callback =
      [this, weak_goal, scan_step, scan_step_count, completed_cycles,
        object_count](
      SpinGoalHandle::SharedPtr,
      const std::shared_ptr<const SpinAction::Feedback>)
      {
        const auto program_goal = weak_goal.lock();
        if (program_goal) {
          publish_program_feedback(
            program_goal, kProgramSpinningPhase,
            "Nav2 is rotating scan step " + std::to_string(scan_step) + "/" +
            std::to_string(scan_step_count),
            completed_cycles, object_count);
        }
      };

    auto goal_future = spin_client_->async_send_goal(spin_goal, options);
    const auto goal_status = wait_for_future(
      goal_future, goal_handle, program_endpoint_timeout_);
    if (goal_status != WaitStatus::ready) {
      cancel_spin_goal_when_available(goal_future);
      error = goal_status == WaitStatus::timeout ?
        "Nav2 did not accept the spin goal in time" :
        "spin dispatch interrupted";
      return goal_status;
    }

    const auto accepted_goal = goal_future.get();
    if (!accepted_goal) {
      error = "Nav2 rejected the spin goal";
      return WaitStatus::timeout;
    }

    auto result_future = spin_client_->async_get_result(accepted_goal);
    const auto result_status = wait_for_future(
      result_future, goal_handle,
      spin_time_allowance_ + cancel_timeout_);
    if (result_status != WaitStatus::ready) {
      cancel_spin_goal(accepted_goal);
      error = result_status == WaitStatus::timeout ?
        "Nav2 spin timed out" : "spin execution interrupted";
      return result_status;
    }

    const auto result = result_future.get();
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED ||
      !result.result ||
      (result.result && spin_result_reports_failure(*result.result)))
    {
      error = result.result ?
        spin_result_failure_message(*result.result) : "Nav2 spin failed";
      return WaitStatus::timeout;
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
    const geometry_msgs::msg::PointStamped & object_point,
    nav2_msgs::msg::Costmap costmap)
  {
    if (costmap.header.frame_id != global_frame_) {
      throw std::invalid_argument(
              "global costmap frame '" + costmap.header.frame_id +
              "' does not match global frame '" + global_frame_ + "'");
    }
    const auto robot_transform = tf_buffer_->lookupTransform(
      global_frame_, robot_base_frame_, tf2::TimePointZero,
      tf2::durationFromSec(tf_timeout_));
    ApproachPlannerConfig config;
    config.maximum_traversable_cost =
      static_cast<int>(approach_maximum_cost_);
    config.robot_radius = approach_robot_radius_;
    config.minimum_standoff = approach_distance_;
    config.start_snap_distance = approach_start_snap_distance_;
    const PlanarPose approach = compute_object_approach_pose(
      approach_grid_from_message(std::move(costmap)),
      object_point.point.x, object_point.point.y,
      robot_transform.transform.translation.x,
      robot_transform.transform.translation.y,
      config);

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

  template<typename GoalHandleT>
  WaitStatus wait_for_visibility_map(
    const std::shared_ptr<GoalHandleT> & goal_handle,
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr & map,
    std::string & error)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(visibility_map_timeout_);
    while (std::chrono::steady_clock::now() < deadline) {
      {
        std::lock_guard<std::mutex> lock(visibility_map_mutex_);
        map = latest_visibility_map_;
      }
      if (map) {
        return WaitStatus::ready;
      }
      if (goal_handle->is_canceling()) {
        error = "visibility-map wait interrupted";
        return WaitStatus::canceled;
      }
      if (stopping_.load() || !rclcpp::ok()) {
        error = "visibility-map wait interrupted";
        return WaitStatus::stopping;
      }
      std::this_thread::sleep_for(50ms);
    }
    error = "timed out waiting for visibility map: " + visibility_map_topic_;
    return WaitStatus::timeout;
  }

  template<typename GoalHandleT>
  WaitStatus request_global_costmap(
    const std::shared_ptr<GoalHandleT> & goal_handle,
    nav2_msgs::msg::Costmap & costmap,
    std::string & error)
  {
    const auto endpoint_status = wait_for_endpoint(
      [this]() {return global_costmap_client_->service_is_ready();},
      goal_handle, global_costmap_timeout_);
    if (endpoint_status != WaitStatus::ready) {
      error = endpoint_status == WaitStatus::timeout ?
        "Nav2 global costmap service is unavailable: " +
        global_costmap_service_ : "global costmap request interrupted";
      return endpoint_status;
    }

    auto future = global_costmap_client_->async_send_request(
      std::make_shared<CostmapService::Request>());
    const auto status = wait_for_future(
      future, goal_handle, global_costmap_timeout_);
    if (status != WaitStatus::ready) {
      global_costmap_client_->remove_pending_request(future);
      error = status == WaitStatus::timeout ?
        "timed out requesting the Nav2 global costmap" :
        "global costmap request interrupted";
      return status;
    }
    costmap = std::move(future.get()->map);
    return WaitStatus::ready;
  }

  static double occupancy_grid_origin_yaw(
    const nav_msgs::msg::OccupancyGrid & map)
  {
    const auto & orientation = map.info.origin.orientation;
    if (!std::isfinite(orientation.x) || !std::isfinite(orientation.y) ||
      !std::isfinite(orientation.z) || !std::isfinite(orientation.w))
    {
      throw std::invalid_argument("visibility map origin has invalid orientation");
    }
    const double norm = std::sqrt(
      orientation.x * orientation.x + orientation.y * orientation.y +
      orientation.z * orientation.z + orientation.w * orientation.w);
    if (norm < 1.0e-9) {
      throw std::invalid_argument("visibility map origin has a zero quaternion");
    }
    return tf2::getYaw(orientation);
  }

  static VisibilityGrid visibility_grid_from_message(
    const nav_msgs::msg::OccupancyGrid & map)
  {
    if (map.info.width == 0U || map.info.height == 0U ||
      map.info.width > static_cast<uint32_t>(std::numeric_limits<int>::max()) ||
      map.info.height > static_cast<uint32_t>(std::numeric_limits<int>::max()))
    {
      throw std::invalid_argument("visibility map dimensions are invalid");
    }
    const std::size_t expected_size =
      static_cast<std::size_t>(map.info.width) *
      static_cast<std::size_t>(map.info.height);
    if (map.data.size() != expected_size) {
      throw std::invalid_argument("visibility map data size is inconsistent");
    }
    if (!std::isfinite(map.info.resolution) || map.info.resolution <= 0.0F) {
      throw std::invalid_argument("visibility map resolution is invalid");
    }
    if (!std::isfinite(map.info.origin.position.x) ||
      !std::isfinite(map.info.origin.position.y))
    {
      throw std::invalid_argument("visibility map origin is invalid");
    }
    occupancy_grid_origin_yaw(map);

    VisibilityGrid grid;
    grid.width = static_cast<int>(map.info.width);
    grid.height = static_cast<int>(map.info.height);
    grid.resolution = static_cast<double>(map.info.resolution);
    grid.occupancy = map.data;
    return grid;
  }

  static double costmap_origin_yaw(const nav2_msgs::msg::Costmap & costmap)
  {
    const auto & orientation = costmap.metadata.origin.orientation;
    if (!std::isfinite(orientation.x) || !std::isfinite(orientation.y) ||
      !std::isfinite(orientation.z) || !std::isfinite(orientation.w))
    {
      throw std::invalid_argument(
              "global costmap origin has invalid orientation");
    }
    const double norm = std::sqrt(
      orientation.x * orientation.x + orientation.y * orientation.y +
      orientation.z * orientation.z + orientation.w * orientation.w);
    if (norm < 1.0e-9) {
      throw std::invalid_argument(
              "global costmap origin has a zero quaternion");
    }
    return tf2::getYaw(orientation);
  }

  static void validate_global_costmap_message(
    const nav2_msgs::msg::Costmap & costmap)
  {
    const auto & metadata = costmap.metadata;
    if (metadata.size_x == 0U || metadata.size_y == 0U ||
      metadata.size_x >
      static_cast<uint32_t>(std::numeric_limits<int>::max()) ||
      metadata.size_y >
      static_cast<uint32_t>(std::numeric_limits<int>::max()))
    {
      throw std::invalid_argument("global costmap dimensions are invalid");
    }
    const std::size_t expected_size =
      static_cast<std::size_t>(metadata.size_x) *
      static_cast<std::size_t>(metadata.size_y);
    if (costmap.data.size() != expected_size) {
      throw std::invalid_argument("global costmap data size is inconsistent");
    }
    if (!std::isfinite(metadata.resolution) || metadata.resolution <= 0.0F) {
      throw std::invalid_argument("global costmap resolution is invalid");
    }
    if (!std::isfinite(metadata.origin.position.x) ||
      !std::isfinite(metadata.origin.position.y))
    {
      throw std::invalid_argument("global costmap origin is invalid");
    }
    costmap_origin_yaw(costmap);
  }

  static VisibilityGrid visibility_grid_from_messages(
    const nav_msgs::msg::OccupancyGrid & map,
    const nav2_msgs::msg::Costmap & costmap)
  {
    VisibilityGrid grid = visibility_grid_from_message(map);
    validate_global_costmap_message(costmap);
    if (costmap.header.frame_id != map.header.frame_id) {
      throw std::invalid_argument(
              "visibility map and global costmap frames do not match");
    }

    grid.navigation_costs.assign(grid.occupancy.size(), uint8_t{255});
    const double map_yaw = occupancy_grid_origin_yaw(map);
    const double costmap_yaw = costmap_origin_yaw(costmap);
    for (int y = 0; y < grid.height; ++y) {
      for (int x = 0; x < grid.width; ++x) {
        const double map_local_x =
          (static_cast<double>(x) + 0.5) * grid.resolution;
        const double map_local_y =
          (static_cast<double>(y) + 0.5) * grid.resolution;
        const double world_x = map.info.origin.position.x +
          std::cos(map_yaw) * map_local_x -
          std::sin(map_yaw) * map_local_y;
        const double world_y = map.info.origin.position.y +
          std::sin(map_yaw) * map_local_x +
          std::cos(map_yaw) * map_local_y;
        const double dx = world_x - costmap.metadata.origin.position.x;
        const double dy = world_y - costmap.metadata.origin.position.y;
        const double costmap_local_x =
          std::cos(costmap_yaw) * dx + std::sin(costmap_yaw) * dy;
        const double costmap_local_y =
          -std::sin(costmap_yaw) * dx + std::cos(costmap_yaw) * dy;
        const int costmap_x = static_cast<int>(std::floor(
            costmap_local_x / costmap.metadata.resolution));
        const int costmap_y = static_cast<int>(std::floor(
            costmap_local_y / costmap.metadata.resolution));
        if (costmap_x < 0 || costmap_y < 0 ||
          costmap_x >= static_cast<int>(costmap.metadata.size_x) ||
          costmap_y >= static_cast<int>(costmap.metadata.size_y))
        {
          continue;
        }
        const std::size_t map_index = static_cast<std::size_t>(y) *
          static_cast<std::size_t>(grid.width) +
          static_cast<std::size_t>(x);
        const std::size_t costmap_index =
          static_cast<std::size_t>(costmap_y) *
          static_cast<std::size_t>(costmap.metadata.size_x) +
          static_cast<std::size_t>(costmap_x);
        grid.navigation_costs[map_index] = costmap.data[costmap_index];
      }
    }
    return grid;
  }

  static ApproachGrid approach_grid_from_message(
    nav2_msgs::msg::Costmap costmap)
  {
    validate_global_costmap_message(costmap);
    const auto & metadata = costmap.metadata;
    ApproachGrid grid;
    grid.width = static_cast<int>(metadata.size_x);
    grid.height = static_cast<int>(metadata.size_y);
    grid.resolution = static_cast<double>(metadata.resolution);
    grid.origin_x = metadata.origin.position.x;
    grid.origin_y = metadata.origin.position.y;
    grid.origin_yaw = costmap_origin_yaw(costmap);
    grid.costs = std::move(costmap.data);
    return grid;
  }

  static std::optional<GridCell> world_to_grid_cell(
    const nav_msgs::msg::OccupancyGrid & map,
    const double world_x,
    const double world_y)
  {
    const double yaw = occupancy_grid_origin_yaw(map);
    const double dx = world_x - map.info.origin.position.x;
    const double dy = world_y - map.info.origin.position.y;
    const double local_x = std::cos(yaw) * dx + std::sin(yaw) * dy;
    const double local_y = -std::sin(yaw) * dx + std::cos(yaw) * dy;
    const int cell_x = static_cast<int>(std::floor(
        local_x / static_cast<double>(map.info.resolution)));
    const int cell_y = static_cast<int>(std::floor(
        local_y / static_cast<double>(map.info.resolution)));
    if (cell_x < 0 || cell_y < 0 ||
      cell_x >= static_cast<int>(map.info.width) ||
      cell_y >= static_cast<int>(map.info.height))
    {
      return std::nullopt;
    }
    return GridCell{cell_x, cell_y};
  }

  geometry_msgs::msg::PoseStamped coverage_target_pose(
    const nav_msgs::msg::OccupancyGrid & map,
    const GridCell & cell,
    const double robot_x,
    const double robot_y,
    const double robot_yaw) const
  {
    const double map_yaw = occupancy_grid_origin_yaw(map);
    const double local_x =
      (static_cast<double>(cell.x) + 0.5) * map.info.resolution;
    const double local_y =
      (static_cast<double>(cell.y) + 0.5) * map.info.resolution;

    geometry_msgs::msg::PoseStamped target;
    target.header.frame_id = global_frame_;
    target.header.stamp = now();
    target.pose.position.x = map.info.origin.position.x +
      std::cos(map_yaw) * local_x - std::sin(map_yaw) * local_y;
    target.pose.position.y = map.info.origin.position.y +
      std::sin(map_yaw) * local_x + std::cos(map_yaw) * local_y;
    const double dx = target.pose.position.x - robot_x;
    const double dy = target.pose.position.y - robot_y;
    const double target_yaw = std::hypot(dx, dy) > 1.0e-6 ?
      std::atan2(dy, dx) : robot_yaw;
    target.pose.orientation.z = std::sin(target_yaw * 0.5);
    target.pose.orientation.w = std::cos(target_yaw * 0.5);
    return target;
  }

  WaitStatus perform_program_navigation(
    const std::shared_ptr<ProgramGoalHandle> & goal_handle,
    const geometry_msgs::msg::PoseStamped & target_pose,
    const std::size_t viewpoint_number,
    const std::size_t candidate_count,
    const uint32_t completed_cycles,
    const uint32_t object_count,
    std::string & error)
  {
    auto endpoint_status = wait_for_endpoint(
      [this]() {return navigate_client_->action_server_is_ready();},
      goal_handle, program_endpoint_timeout_);
    if (endpoint_status != WaitStatus::ready) {
      error = endpoint_status == WaitStatus::timeout ?
        "Nav2 navigation action is unavailable: " + navigate_action_ :
        "coverage navigation dispatch interrupted";
      return endpoint_status;
    }

    NavigateAction::Goal nav_goal;
    nav_goal.pose = target_pose;
    nav_goal.behavior_tree = behavior_tree_;
    rclcpp_action::Client<NavigateAction>::SendGoalOptions options;
    std::weak_ptr<ProgramGoalHandle> weak_goal = goal_handle;
    options.feedback_callback =
      [this, weak_goal, viewpoint_number, candidate_count, completed_cycles,
        object_count](
      NavigateGoalHandle::SharedPtr,
      const std::shared_ptr<const NavigateAction::Feedback> feedback)
      {
        const auto program_goal = weak_goal.lock();
        if (program_goal) {
          publish_program_feedback(
            program_goal, kProgramCoveringPhase,
            "Nav2 is approaching visibility viewpoint " +
            std::to_string(viewpoint_number) + "/" +
            std::to_string(candidate_count) + " (" +
            std::to_string(feedback->distance_remaining) + " m remaining)",
            completed_cycles, object_count);
        }
      };

    visibility_target_publisher_->publish(target_pose);
    auto goal_future = navigate_client_->async_send_goal(nav_goal, options);
    const auto goal_status = wait_for_future(
      goal_future, goal_handle, program_endpoint_timeout_);
    if (goal_status != WaitStatus::ready) {
      cancel_nav_goal_when_available(goal_future);
      error = goal_status == WaitStatus::timeout ?
        "Nav2 did not accept the coverage goal in time" :
        "coverage navigation dispatch interrupted";
      return goal_status;
    }

    const auto accepted_goal = goal_future.get();
    if (!accepted_goal) {
      error = "Nav2 rejected the visibility viewpoint";
      return WaitStatus::timeout;
    }
    auto result_future = navigate_client_->async_get_result(accepted_goal);
    const auto result_status = wait_for_future(
      result_future, goal_handle, navigation_timeout_);
    if (result_status != WaitStatus::ready) {
      cancel_nav_goal(accepted_goal);
      error = result_status == WaitStatus::timeout ?
        "coverage navigation timeout expired" :
        "coverage navigation interrupted";
      return result_status;
    }

    const auto result = result_future.get();
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      error = result.code == rclcpp_action::ResultCode::CANCELED ?
        "Nav2 coverage goal was canceled externally" :
        "Nav2 could not reach the visibility viewpoint";
      return WaitStatus::timeout;
    }
    return WaitStatus::ready;
  }

  static double combined_visibility_coverage(
    const VisibilityCoverageStats & stats)
  {
    return std::min(
      stats.observable_coverage_ratio(), stats.boundary_coverage_ratio());
  }

  CoverageStatus run_visibility_coverage(
    const std::shared_ptr<ProgramGoalHandle> & goal_handle,
    const double observation_duration,
    const uint32_t scan_step_count,
    const double spin_angle_degrees,
    const uint32_t completed_cycles,
    const uint32_t object_count,
    std::string & error)
  {
    publish_program_feedback(
      goal_handle, kProgramCoveringPhase,
      "waiting for the final occupancy map", completed_cycles, object_count);
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr map;
    const auto map_status = wait_for_visibility_map(goal_handle, map, error);
    if (map_status == WaitStatus::canceled) {
      return CoverageStatus::canceled;
    }
    if (map_status == WaitStatus::stopping) {
      return CoverageStatus::stopping;
    }
    if (map_status != WaitStatus::ready) {
      return CoverageStatus::failed;
    }
    if (map->header.frame_id != global_frame_) {
      error = "visibility map frame '" + map->header.frame_id +
        "' does not match global frame '" + global_frame_ + "'";
      return CoverageStatus::failed;
    }

    publish_program_feedback(
      goal_handle, kProgramCoveringPhase,
      "requesting the Nav2 global costmap", completed_cycles, object_count);
    nav2_msgs::msg::Costmap costmap;
    const auto costmap_status = request_global_costmap(
      goal_handle, costmap, error);
    if (costmap_status == WaitStatus::canceled) {
      return CoverageStatus::canceled;
    }
    if (costmap_status == WaitStatus::stopping) {
      return CoverageStatus::stopping;
    }
    if (costmap_status != WaitStatus::ready) {
      return CoverageStatus::failed;
    }

    geometry_msgs::msg::TransformStamped robot_transform;
    try {
      robot_transform = tf_buffer_->lookupTransform(
        global_frame_, robot_base_frame_, tf2::TimePointZero,
        tf2::durationFromSec(tf_timeout_));
    } catch (const tf2::TransformException & exception) {
      error = "TF could not place the robot for visibility coverage: " +
        std::string(exception.what());
      return CoverageStatus::failed;
    }

    std::optional<GridCell> requested_start;
    try {
      requested_start = world_to_grid_cell(
        *map, robot_transform.transform.translation.x,
        robot_transform.transform.translation.y);
    } catch (const std::invalid_argument & exception) {
      error = exception.what();
      return CoverageStatus::failed;
    }
    if (!requested_start) {
      error = "robot pose is outside the visibility occupancy map";
      return CoverageStatus::failed;
    }

    std::unique_ptr<VisibilityViewpointPlanner> planner;
    try {
      planner = std::make_unique<VisibilityViewpointPlanner>(
        visibility_grid_from_messages(*map, costmap), *requested_start,
        visibility_planner_config_);
    } catch (const std::invalid_argument & exception) {
      error = "could not construct visibility plan: " +
        std::string(exception.what());
      return CoverageStatus::failed;
    }

    GridCell current = planner->start_cell();
    double robot_x = robot_transform.transform.translation.x;
    double robot_y = robot_transform.transform.translation.y;
    double robot_yaw = tf2::getYaw(robot_transform.transform.rotation);
    std::size_t completed_viewpoints = 0U;
    int64_t navigation_failures = 0;
    RCLCPP_INFO(
      get_logger(),
      "Visibility coverage started: candidates=%zu map=%ux%u@%.3fm "
      "costmap=%ux%u@%.3fm target=%.1f%%",
      planner->candidates().size(), map->info.width, map->info.height,
      map->info.resolution, costmap.metadata.size_x,
      costmap.metadata.size_y, costmap.metadata.resolution,
      visibility_completion_ratio_ * 100.0);

    while (rclcpp::ok() && !stopping_.load()) {
      if (goal_handle->is_canceling()) {
        error = "explore-and-record canceled during visibility coverage";
        return CoverageStatus::canceled;
      }
      const auto stats_before = planner->coverage_stats();
      if (combined_visibility_coverage(stats_before) >=
        visibility_completion_ratio_)
      {
        return CoverageStatus::complete;
      }
      if (visibility_max_viewpoints_ > 0 &&
        completed_viewpoints >=
        static_cast<std::size_t>(visibility_max_viewpoints_))
      {
        error = "visibility coverage stopped at the configured viewpoint limit";
        return CoverageStatus::failed;
      }

      const auto selection = planner->select_next(current);
      if (!selection.valid()) {
        error = "visibility planner exhausted candidates at " +
          std::to_string(
          combined_visibility_coverage(stats_before) * 100.0) +
          "% observable coverage";
        return CoverageStatus::failed;
      }

      const std::size_t viewpoint_number = completed_viewpoints + 1U;
      const auto target_pose = coverage_target_pose(
        *map, selection.cell, robot_x, robot_y, robot_yaw);
      publish_program_feedback(
        goal_handle, kProgramCoveringPhase,
        "sending visibility viewpoint " + std::to_string(viewpoint_number) +
        " with " + std::to_string(selection.new_boundary_cells) +
        " new boundary cells", completed_cycles, object_count);
      const auto navigation_status = perform_program_navigation(
        goal_handle, target_pose, viewpoint_number,
        planner->candidates().size(), completed_cycles, object_count, error);
      if (navigation_status == WaitStatus::canceled) {
        return CoverageStatus::canceled;
      }
      if (navigation_status == WaitStatus::stopping) {
        return CoverageStatus::stopping;
      }
      if (navigation_status != WaitStatus::ready) {
        planner->discard(selection.candidate_index);
        ++navigation_failures;
        RCLCPP_WARN(
          get_logger(), "Discarding unreachable visibility viewpoint: %s",
          error.c_str());
        if (navigation_failures >= visibility_max_navigation_failures_) {
          error = "visibility coverage exceeded the navigation failure limit";
          return CoverageStatus::failed;
        }
        continue;
      }

      current = selection.cell;
      robot_x = target_pose.pose.position.x;
      robot_y = target_pose.pose.position.y;
      robot_yaw = tf2::getYaw(target_pose.pose.orientation);
      const auto settle_status = wait_program_delay(
        goal_handle, navigation_settle_time_, false);
      if (settle_status == ProgramDelayStatus::canceled) {
        error = "explore-and-record canceled before visibility scan";
        return CoverageStatus::canceled;
      }
      if (settle_status == ProgramDelayStatus::stopping) {
        return CoverageStatus::stopping;
      }

      for (uint32_t scan_step = 1U; scan_step <= scan_step_count; ++scan_step) {
        const std::string scan_progress = std::to_string(scan_step) + "/" +
          std::to_string(scan_step_count);
        publish_program_feedback(
          goal_handle, kProgramSpinningPhase,
          "visibility viewpoint " + std::to_string(viewpoint_number) +
          ", scan step " + scan_progress,
          completed_cycles, object_count);
        const auto spin_status = perform_program_spin(
          goal_handle, spin_angle_degrees, scan_step, scan_step_count,
          completed_cycles, object_count, error);
        if (spin_status == WaitStatus::canceled) {
          return CoverageStatus::canceled;
        }
        if (spin_status == WaitStatus::stopping) {
          return CoverageStatus::stopping;
        }
        if (spin_status != WaitStatus::ready) {
          return CoverageStatus::failed;
        }

        publish_program_feedback(
          goal_handle, kProgramObservingPhase,
          "observing static objects at visibility viewpoint " +
          std::to_string(viewpoint_number) + ", scan step " + scan_progress,
          completed_cycles, object_count);
        const auto observation_status = wait_program_delay(
          goal_handle, observation_duration, false);
        if (observation_status == ProgramDelayStatus::canceled) {
          error = "explore-and-record canceled during visibility observation";
          return CoverageStatus::canceled;
        }
        if (observation_status == ProgramDelayStatus::stopping) {
          return CoverageStatus::stopping;
        }
      }

      planner->observe(selection.candidate_index);
      ++completed_viewpoints;
      const auto stats = planner->coverage_stats();
      const double coverage = combined_visibility_coverage(stats);
      publish_program_feedback(
        goal_handle, kProgramSavingPhase,
        "checkpointing static objects after visibility viewpoint " +
        std::to_string(completed_viewpoints), completed_cycles, object_count);
      const auto checkpoint_status = checkpoint_program_registry(
        goal_handle, error);
      if (checkpoint_status == WaitStatus::canceled) {
        return CoverageStatus::canceled;
      }
      if (checkpoint_status == WaitStatus::stopping) {
        return CoverageStatus::stopping;
      }
      if (checkpoint_status != WaitStatus::ready) {
        return CoverageStatus::failed;
      }
      RCLCPP_INFO(
        get_logger(),
        "Visibility viewpoint %zu complete: observable=%.1f%% "
        "boundary=%.1f%% map=%.1f%%",
        completed_viewpoints, stats.observable_coverage_ratio() * 100.0,
        stats.boundary_coverage_ratio() * 100.0,
        stats.map_coverage_ratio() * 100.0);
      if (coverage >= visibility_completion_ratio_) {
        return CoverageStatus::complete;
      }
    }
    return CoverageStatus::stopping;
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

  void execute_program(const std::shared_ptr<ProgramGoalHandle> & goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    const double exploration_duration = goal->exploration_duration > 0.0F ?
      static_cast<double>(goal->exploration_duration) :
      exploration_cycle_duration_;
    const double observation_duration = goal->observation_duration > 0.0F ?
      static_cast<double>(goal->observation_duration) : observation_duration_;
    const uint32_t scan_step_count = goal->scan_step_count > 0U ?
      goal->scan_step_count : static_cast<uint32_t>(scan_step_count_);
    const double spin_angle_degrees = 360.0 /
      static_cast<double>(scan_step_count);
    const uint32_t max_cycles = goal->max_cycles;

    uint32_t completed_cycles = 0U;
    uint32_t objects_before = 0U;
    uint32_t objects_after = 0U;
    bool exploration_running = false;
    std::string error;

    auto count_status = query_program_object_count(
      goal_handle, objects_before, error);
    objects_after = objects_before;
    if (count_status == WaitStatus::canceled) {
      cancel_program(
        goal_handle, "explore-and-record canceled before startup",
        completed_cycles, objects_before, objects_after);
      return;
    }
    if (count_status == WaitStatus::stopping) {
      return;
    }
    if (count_status != WaitStatus::ready) {
      abort_program(
        goal_handle, error, completed_cycles, objects_before, objects_after);
      return;
    }

    const auto finish_successfully =
      [this, &goal_handle, &completed_cycles, &objects_before, &objects_after,
        &exploration_running](
      const std::string & message)
      {
        if (exploration_running) {
          best_effort_stop_exploration();
          exploration_running = false;
        }
        publish_program_feedback(
          goal_handle, kProgramSavingPhase,
          "checkpointing recorded objects", completed_cycles, objects_after);
        std::string finish_error;
        const auto checkpoint_status = checkpoint_program_registry(
          goal_handle, finish_error);
        if (checkpoint_status == WaitStatus::canceled) {
          cancel_program(
            goal_handle, "explore-and-record canceled while checkpointing",
            completed_cycles, objects_before, objects_after);
          return;
        }
        if (checkpoint_status == WaitStatus::stopping) {
          return;
        }
        if (checkpoint_status != WaitStatus::ready) {
          abort_program(
            goal_handle, finish_error, completed_cycles,
            objects_before, objects_after);
          return;
        }
        const auto final_count_status = query_program_object_count(
          goal_handle, objects_after, finish_error);
        if (final_count_status == WaitStatus::canceled) {
          cancel_program(
            goal_handle, "explore-and-record canceled during final registry query",
            completed_cycles, objects_before, objects_after);
          return;
        }
        if (final_count_status == WaitStatus::stopping) {
          return;
        }
        if (final_count_status != WaitStatus::ready) {
          abort_program(
            goal_handle, finish_error, completed_cycles,
            objects_before, objects_after);
          return;
        }
        succeed_program(
          goal_handle, message, completed_cycles,
          objects_before, objects_after);
      };

    const auto finish_after_frontier =
      [this, &goal_handle, &finish_successfully, &exploration_running,
        &observation_duration, &scan_step_count, &spin_angle_degrees,
        &completed_cycles, &objects_before, &objects_after, &error]()
      {
        if (exploration_running) {
          best_effort_stop_exploration();
          exploration_running = false;
        }
        exploration_completed_.store(false);
        if (!visibility_coverage_enabled_) {
          finish_successfully("frontier exploration completed");
          return;
        }

        const auto coverage_status = run_visibility_coverage(
          goal_handle, observation_duration, scan_step_count,
          spin_angle_degrees, completed_cycles, objects_after, error);
        if (coverage_status == CoverageStatus::canceled) {
          cancel_program(
            goal_handle,
            error.empty() ?
            "explore-and-record canceled during visibility coverage" : error,
            completed_cycles, objects_before, objects_after);
          return;
        }
        if (coverage_status == CoverageStatus::stopping) {
          return;
        }
        if (coverage_status == CoverageStatus::failed) {
          abort_program(
            goal_handle, error, completed_cycles,
            objects_before, objects_after);
          return;
        }
        finish_successfully(
          "frontier exploration and visibility coverage completed");
      };

    if (goal_handle->is_canceling()) {
      cancel_program(
        goal_handle, "explore-and-record canceled before exploration",
        completed_cycles, objects_before, objects_after);
      return;
    }

    exploration_completed_.store(false);
    std::string control_message;
    uint8_t frontier_state = FrontierControlService::Request::STATE_IDLE;
    if (!set_exploration_enabled(
        true, program_endpoint_timeout_, control_message, frontier_state))
    {
      abort_program(
        goal_handle, "could not start exploration: " + control_message,
        completed_cycles, objects_before, objects_after);
      return;
    }
    exploration_running = true;

    RCLCPP_INFO(
      get_logger(),
      "Explore-and-record started: explore=%.2fs scan=%ux%.1fdeg "
      "observe=%.2fs/step max_cycles=%u objects=%u",
      exploration_duration, scan_step_count, spin_angle_degrees,
      observation_duration, max_cycles, objects_before);

    while (rclcpp::ok() && !stopping_.load()) {
      publish_program_feedback(
        goal_handle, kProgramExploringPhase,
        "frontier exploration is mapping", completed_cycles, objects_after);
      const auto exploration_wait = wait_program_delay(
        goal_handle, exploration_duration);
      if (exploration_wait == ProgramDelayStatus::canceled) {
        cancel_program(
          goal_handle, "explore-and-record canceled during exploration",
          completed_cycles, objects_before, objects_after);
        return;
      }
      if (exploration_wait == ProgramDelayStatus::stopping) {
        best_effort_stop_exploration();
        return;
      }
      if (exploration_wait == ProgramDelayStatus::exploration_complete) {
        finish_after_frontier();
        return;
      }

      publish_program_feedback(
        goal_handle, kProgramPausingPhase,
        "stopping frontier exploration", completed_cycles, objects_after);
      if (!set_exploration_enabled(
          false, program_endpoint_timeout_, control_message, frontier_state))
      {
        abort_program(
          goal_handle, "could not pause exploration: " + control_message,
          completed_cycles, objects_before, objects_after);
        return;
      }
      exploration_running = false;
      if (goal_handle->is_canceling()) {
        cancel_program(
          goal_handle, "explore-and-record canceled while pausing",
          completed_cycles, objects_before, objects_after);
        return;
      }

      const auto settle_wait = wait_program_delay(
        goal_handle, navigation_settle_time_);
      if (settle_wait == ProgramDelayStatus::canceled) {
        cancel_program(
          goal_handle, "explore-and-record canceled before spin",
          completed_cycles, objects_before, objects_after);
        return;
      }
      if (settle_wait == ProgramDelayStatus::stopping) {
        return;
      }
      if (settle_wait == ProgramDelayStatus::exploration_complete) {
        finish_after_frontier();
        return;
      }

      for (uint32_t scan_step = 1U; scan_step <= scan_step_count; ++scan_step) {
        const std::string scan_progress = std::to_string(scan_step) + "/" +
          std::to_string(scan_step_count);
        publish_program_feedback(
          goal_handle, kProgramSpinningPhase,
          "starting Nav2 scan step " + scan_progress,
          completed_cycles, objects_after);
        const auto spin_status = perform_program_spin(
          goal_handle, spin_angle_degrees, scan_step, scan_step_count,
          completed_cycles, objects_after, error);
        if (spin_status == WaitStatus::canceled) {
          cancel_program(
            goal_handle, "explore-and-record canceled during scan step " +
            scan_progress, completed_cycles, objects_before, objects_after);
          return;
        }
        if (spin_status == WaitStatus::stopping) {
          return;
        }
        if (spin_status != WaitStatus::ready) {
          abort_program(
            goal_handle, error, completed_cycles, objects_before, objects_after);
          return;
        }

        publish_program_feedback(
          goal_handle, kProgramObservingPhase,
          "observing objects at scan step " + scan_progress,
          completed_cycles, objects_after);
        const auto observation_wait = wait_program_delay(
          goal_handle, observation_duration);
        if (observation_wait == ProgramDelayStatus::canceled) {
          cancel_program(
            goal_handle, "explore-and-record canceled while observing scan step " +
            scan_progress, completed_cycles, objects_before, objects_after);
          return;
        }
        if (observation_wait == ProgramDelayStatus::stopping) {
          return;
        }
        if (observation_wait == ProgramDelayStatus::exploration_complete) {
          finish_after_frontier();
          return;
        }
      }

      publish_program_feedback(
        goal_handle, kProgramSavingPhase,
        "checkpointing recorded objects", completed_cycles, objects_after);
      const auto checkpoint_status = checkpoint_program_registry(
        goal_handle, error);
      if (checkpoint_status == WaitStatus::canceled) {
        cancel_program(
          goal_handle, "explore-and-record canceled while checkpointing",
          completed_cycles, objects_before, objects_after);
        return;
      }
      if (checkpoint_status == WaitStatus::stopping) {
        return;
      }
      if (checkpoint_status != WaitStatus::ready) {
        abort_program(
          goal_handle, error, completed_cycles, objects_before, objects_after);
        return;
      }

      count_status = query_program_object_count(
        goal_handle, objects_after, error);
      if (count_status == WaitStatus::canceled) {
        cancel_program(
          goal_handle, "explore-and-record canceled during registry query",
          completed_cycles, objects_before, objects_after);
        return;
      }
      if (count_status == WaitStatus::stopping) {
        return;
      }
      if (count_status != WaitStatus::ready) {
        abort_program(
          goal_handle, error, completed_cycles, objects_before, objects_after);
        return;
      }

      ++completed_cycles;
      if (max_cycles > 0U && completed_cycles >= max_cycles) {
        succeed_program(
          goal_handle, "requested observation cycles completed",
          completed_cycles, objects_before, objects_after);
        return;
      }
      if (exploration_completed_.load()) {
        finish_after_frontier();
        return;
      }

      if (!set_exploration_enabled(
          true, program_endpoint_timeout_, control_message, frontier_state))
      {
        abort_program(
          goal_handle, "could not resume exploration: " + control_message,
          completed_cycles, objects_before, objects_after);
        return;
      }
      exploration_running = true;
    }

    best_effort_stop_exploration();
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

    publish_feedback(
      goal_handle, kPlanningPhase, "requesting Nav2 global costmap", -1.0F,
      target_pose);

    nav2_msgs::msg::Costmap costmap;
    std::string costmap_error;
    const auto costmap_status = request_global_costmap(
      goal_handle, costmap, costmap_error);
    if (costmap_status == WaitStatus::canceled) {
      cancel_command(
        goal_handle, "command canceled during the global costmap request",
        target_pose);
      return;
    }
    if (costmap_status == WaitStatus::stopping) {
      return;
    }
    if (costmap_status != WaitStatus::ready) {
      abort_command(goal_handle, costmap_error, target_pose);
      return;
    }

    try {
      const auto object_point = transform_object_to_global(registry_result, object);
      target_pose = make_target_pose(object_point, std::move(costmap));
    } catch (const tf2::TransformException & error) {
      abort_command(
        goal_handle, std::string("TF could not place the object and robot in '") +
        global_frame_ + "': " + error.what(), target_pose);
      return;
    } catch (const std::exception & error) {
      abort_command(
        goal_handle,
        std::string("could not select a reachable object approach cell: ") +
        error.what(), target_pose);
      return;
    }

    target_pose_publisher_->publish(target_pose);
    publish_feedback(
      goal_handle, kPlanningPhase,
      "submitting reachable global-costmap pose to Nav2", -1.0F, target_pose);

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
      result->message =
        "reached global-costmap pose facing object '" + object_id + "'";
      result->target_pose = target_pose;
      goal_handle->succeed(result);
      RCLCPP_INFO(
        get_logger(), "Reached object '%s' outside minimum radius %.2f m",
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
  std::string global_costmap_service_;
  std::string target_pose_topic_;
  std::string explore_service_name_;
  std::string save_map_service_name_;
  std::string slam_toolbox_save_map_service_;
  std::string map_save_directory_;
  std::string default_map_name_;
  std::string program_action_name_;
  std::string frontier_control_service_;
  std::string spin_action_;
  std::string registry_save_service_;
  std::string exploration_completion_topic_;
  std::string visibility_map_topic_;
  std::string visibility_target_pose_topic_;
  std::string global_frame_;
  std::string robot_base_frame_;
  std::string behavior_tree_;
  double approach_distance_{0.75};
  double approach_robot_radius_{0.16};
  double approach_start_snap_distance_{0.5};
  int64_t approach_maximum_cost_{252};
  double registry_timeout_{3.0};
  double nav_server_timeout_{5.0};
  double global_costmap_timeout_{5.0};
  double tf_timeout_{0.2};
  double navigation_timeout_{0.0};
  double cancel_timeout_{2.0};
  double exploration_service_timeout_{5.0};
  double save_map_timeout_{10.0};
  double program_endpoint_timeout_{5.0};
  double exploration_cycle_duration_{10.0};
  double observation_duration_{3.0};
  int64_t scan_step_count_{8};
  double spin_time_allowance_{15.0};
  double navigation_settle_time_{1.0};
  bool visibility_coverage_enabled_{true};
  double visibility_map_timeout_{10.0};
  VisibilityPlannerConfig visibility_planner_config_;
  double visibility_completion_ratio_{0.98};
  int64_t visibility_max_viewpoints_{0};
  int64_t visibility_max_navigation_failures_{3};

  rclcpp::Client<RegistryService>::SharedPtr registry_client_;
  rclcpp::Client<RegistrySaveService>::SharedPtr registry_save_client_;
  rclcpp::Client<CostmapService>::SharedPtr global_costmap_client_;
  rclcpp_action::Client<NavigateAction>::SharedPtr navigate_client_;
  rclcpp_action::Client<SpinAction>::SharedPtr spin_client_;
  rclcpp::Client<FrontierControlService>::SharedPtr frontier_control_client_;
  rclcpp::Client<SaveMapService>::SharedPtr slam_toolbox_save_map_client_;
  rclcpp_action::Server<CommandAction>::SharedPtr command_server_;
  rclcpp_action::Server<ProgramAction>::SharedPtr program_server_;
  rclcpp::Service<ExploreService>::SharedPtr explore_service_;
  rclcpp::Service<SaveMapService>::SharedPtr save_map_service_;
  rclcpp::CallbackGroup::SharedPtr frontier_client_group_;
  rclcpp::CallbackGroup::SharedPtr explore_service_group_;
  rclcpp::CallbackGroup::SharedPtr save_map_client_group_;
  rclcpp::CallbackGroup::SharedPtr save_map_service_group_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
    target_pose_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
    visibility_target_publisher_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr
    exploration_completion_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr
    visibility_map_sub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::atomic<bool> stopping_{false};
  std::atomic<bool> program_active_{false};
  std::atomic<bool> exploration_completed_{false};
  std::mutex frontier_request_mutex_;
  std::mutex visibility_map_mutex_;
  std::mutex worker_mutex_;
  std::condition_variable worker_condition_;
  std::condition_variable program_condition_;
  std::thread worker_;
  std::thread program_worker_;
  bool busy_{false};
  std::shared_ptr<CommandGoalHandle> pending_goal_;
  std::shared_ptr<ProgramGoalHandle> pending_program_goal_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr latest_visibility_map_;
};

}  // namespace muto_command_layer

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<muto_command_layer::CommandLayerNode>();
  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
