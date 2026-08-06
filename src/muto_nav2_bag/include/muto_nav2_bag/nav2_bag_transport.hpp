#ifndef MUTO_NAV2_BAG__NAV2_BAG_TRANSPORT_HPP_
#define MUTO_NAV2_BAG__NAV2_BAG_TRANSPORT_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "rclcpp/context.hpp"
#include "rclcpp/executor.hpp"
#include "rosbag2_transport/recorder.hpp"

namespace muto_nav2_bag
{

struct Nav2BagOptions
{
  std::string uri;
  std::string storage_id{"mcap"};
  std::string storage_preset_profile{"zstd_fast"};
  std::vector<std::string> topics;
  uint64_t max_cache_size{52428800U};
  bool include_hidden_topics{true};
  bool record_all_services{false};
  bool use_sim_time{false};
  std::unordered_map<std::string, std::string> custom_data;
};

class Nav2BagTransport
{
public:
  explicit Nav2BagTransport(rclcpp::Context::SharedPtr context);
  ~Nav2BagTransport();

  Nav2BagTransport(const Nav2BagTransport &) = delete;
  Nav2BagTransport & operator=(const Nav2BagTransport &) = delete;

  void start(const Nav2BagOptions & options);
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

}  // namespace muto_nav2_bag

#endif  // MUTO_NAV2_BAG__NAV2_BAG_TRANSPORT_HPP_
