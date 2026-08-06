#ifndef MUTO_EXPLORATION_BAG__EXPLORATION_BAG_TRANSPORT_HPP_
#define MUTO_EXPLORATION_BAG__EXPLORATION_BAG_TRANSPORT_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "rclcpp/context.hpp"
#include "rclcpp/executor.hpp"
#include "rosbag2_transport/recorder.hpp"

namespace muto_exploration_bag
{

struct ExplorationBagOptions
{
  std::string uri;
  std::string storage_id{"mcap"};
  std::string storage_preset_profile{"none"};
  std::vector<std::string> topics;
  std::string topics_regex;
  std::string exclude_regex;
  uint64_t max_cache_size{104857600U};
  bool include_hidden_topics{true};
  bool record_all_services{true};
  bool use_sim_time{false};
  std::unordered_map<std::string, std::string> custom_data;
};

class ExplorationBagTransport
{
public:
  explicit ExplorationBagTransport(rclcpp::Context::SharedPtr context);
  ~ExplorationBagTransport();

  ExplorationBagTransport(const ExplorationBagTransport &) = delete;
  ExplorationBagTransport & operator=(const ExplorationBagTransport &) = delete;

  void start(const ExplorationBagOptions & options);
  void stop();

  [[nodiscard]] bool active() const noexcept;
  [[nodiscard]] const std::string & path() const noexcept;

private:
  rclcpp::Context::SharedPtr context_;
  std::shared_ptr<rosbag2_transport::Recorder> recorder_;
  std::shared_ptr<rclcpp::Executor> executor_;
  std::thread executor_thread_;
  std::string path_;
};

}  // namespace muto_exploration_bag

#endif  // MUTO_EXPLORATION_BAG__EXPLORATION_BAG_TRANSPORT_HPP_
