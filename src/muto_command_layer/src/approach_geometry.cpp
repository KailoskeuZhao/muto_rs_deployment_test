#include "muto_command_layer/approach_geometry.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
#include <queue>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace muto_command_layer
{
namespace
{

constexpr double kEpsilon = 1.0e-9;

struct Cell
{
  int x{0};
  int y{0};
};

constexpr std::array<Cell, 8> kNeighbors{{
  {-1, -1}, {0, -1}, {1, -1},
  {-1, 0}, {1, 0},
  {-1, 1}, {0, 1}, {1, 1},
}};

class ApproachPlanner
{
public:
  ApproachPlanner(const ApproachGrid & grid, ApproachPlannerConfig config)
  : grid_(grid), config_(config)
  {
    validate();
  }

  PlanarPose plan(
    const double object_x,
    const double object_y,
    const double robot_x,
    const double robot_y) const
  {
    require_finite(object_x, "object_x");
    require_finite(object_y, "object_y");
    require_finite(robot_x, "robot_x");
    require_finite(robot_y, "robot_y");

    if (!in_bounds(world_to_cell(object_x, object_y))) {
      throw std::runtime_error("object centroid is outside the global costmap");
    }
    const Cell start = snap_start(robot_x, robot_y);
    const auto path_distance = reachable_path_distances(start);
    const double minimum_radius = std::max(
      config_.minimum_standoff, config_.robot_radius);

    std::vector<std::vector<std::size_t>> candidate_rings;
    for (int y = 0; y < grid_.height; ++y) {
      for (int x = 0; x < grid_.width; ++x) {
        const Cell candidate{x, y};
        const double candidate_path = path_distance[index(candidate)];
        if (!std::isfinite(candidate_path)) {
          continue;
        }
        const auto world = cell_center(candidate);
        const double object_distance = std::hypot(
          world.first - object_x, world.second - object_y);
        if (object_distance + kEpsilon < minimum_radius) {
          continue;
        }
        const double excess = std::max(0.0, object_distance - minimum_radius);
        const auto ring = static_cast<std::size_t>(std::floor(
            excess / grid_.resolution + kEpsilon));
        if (candidate_rings.size() <= ring) {
          candidate_rings.resize(ring + 1U);
        }
        candidate_rings[ring].push_back(index(candidate));
      }
    }

    bool found = false;
    Cell best;
    for (const auto & ring : candidate_rings) {
      double best_path = std::numeric_limits<double>::infinity();
      double best_excess = std::numeric_limits<double>::infinity();
      for (const std::size_t candidate_index : ring) {
        const Cell candidate{
          static_cast<int>(
            candidate_index % static_cast<std::size_t>(grid_.width)),
          static_cast<int>(
            candidate_index / static_cast<std::size_t>(grid_.width)),
        };
        const auto world = cell_center(candidate);
        const double excess = std::hypot(
          world.first - object_x, world.second - object_y) - minimum_radius;
        const double candidate_path = path_distance[candidate_index];
        if (!found || candidate_path + kEpsilon < best_path ||
          (std::abs(candidate_path - best_path) <= kEpsilon &&
          excess + kEpsilon < best_excess))
        {
          found = true;
          best = candidate;
          best_path = candidate_path;
          best_excess = excess;
        }
      }
      if (found) {
        break;
      }
    }

    if (!found) {
      throw std::runtime_error(
              "no reachable global-costmap cell satisfies the approach radius");
    }

    const auto target = cell_center(best);
    PlanarPose result;
    result.x = target.first;
    result.y = target.second;
    result.yaw = std::atan2(object_y - result.y, object_x - result.x);
    return result;
  }

private:
  static void require_finite(const double value, const char * name)
  {
    if (!std::isfinite(value)) {
      throw std::invalid_argument(std::string(name) + " must be finite");
    }
  }

  void validate() const
  {
    if (grid_.width <= 0 || grid_.height <= 0) {
      throw std::invalid_argument(
              "approach costmap dimensions must be positive");
    }
    const auto expected_size = static_cast<std::size_t>(grid_.width) *
      static_cast<std::size_t>(grid_.height);
    if (grid_.costs.size() != expected_size) {
      throw std::invalid_argument("approach costmap data size is inconsistent");
    }
    if (!std::isfinite(grid_.resolution) || grid_.resolution <= 0.0 ||
      !std::isfinite(grid_.origin_x) || !std::isfinite(grid_.origin_y) ||
      !std::isfinite(grid_.origin_yaw))
    {
      throw std::invalid_argument("approach costmap geometry is invalid");
    }
    if (config_.maximum_traversable_cost < 0 ||
      config_.maximum_traversable_cost > 252)
    {
      throw std::invalid_argument(
              "maximum traversable cost must be in [0, 252]");
    }
    if (!std::isfinite(config_.robot_radius) || config_.robot_radius <= 0.0 ||
      !std::isfinite(config_.minimum_standoff) ||
      config_.minimum_standoff <= 0.0 ||
      !std::isfinite(config_.start_snap_distance) ||
      config_.start_snap_distance < 0.0)
    {
      throw std::invalid_argument("approach planner distances are invalid");
    }
  }

  std::size_t index(const Cell & cell) const
  {
    return static_cast<std::size_t>(cell.y) *
           static_cast<std::size_t>(grid_.width) +
           static_cast<std::size_t>(cell.x);
  }

  bool in_bounds(const Cell & cell) const
  {
    return cell.x >= 0 && cell.y >= 0 &&
           cell.x < grid_.width && cell.y < grid_.height;
  }

  Cell world_to_cell(const double world_x, const double world_y) const
  {
    const double dx = world_x - grid_.origin_x;
    const double dy = world_y - grid_.origin_y;
    const double local_x =
      std::cos(grid_.origin_yaw) * dx + std::sin(grid_.origin_yaw) * dy;
    const double local_y =
      -std::sin(grid_.origin_yaw) * dx + std::cos(grid_.origin_yaw) * dy;
    return Cell{
      static_cast<int>(std::floor(local_x / grid_.resolution)),
      static_cast<int>(std::floor(local_y / grid_.resolution)),
    };
  }

  std::pair<double, double> cell_center(const Cell & cell) const
  {
    const double local_x = (static_cast<double>(cell.x) + 0.5) *
      grid_.resolution;
    const double local_y = (static_cast<double>(cell.y) + 0.5) *
      grid_.resolution;
    return {
      grid_.origin_x + std::cos(grid_.origin_yaw) * local_x -
      std::sin(grid_.origin_yaw) * local_y,
      grid_.origin_y + std::sin(grid_.origin_yaw) * local_x +
      std::cos(grid_.origin_yaw) * local_y,
    };
  }

  bool is_traversable(const Cell & cell) const
  {
    return in_bounds(cell) &&
           static_cast<int>(grid_.costs[index(cell)]) <=
           config_.maximum_traversable_cost;
  }

  bool can_step(const Cell & source, const Cell & target) const
  {
    if (!is_traversable(target)) {
      return false;
    }
    const int dx = target.x - source.x;
    const int dy = target.y - source.y;
    if (dx != 0 && dy != 0) {
      if (!is_traversable(Cell{source.x + dx, source.y}) ||
        !is_traversable(Cell{source.x, source.y + dy}))
      {
        return false;
      }
    }
    return true;
  }

  Cell snap_start(const double robot_x, const double robot_y) const
  {
    const Cell requested = world_to_cell(robot_x, robot_y);
    if (is_traversable(requested)) {
      return requested;
    }
    const int radius_cells = static_cast<int>(std::ceil(
        config_.start_snap_distance / grid_.resolution + 1.0));
    Cell best;
    double best_distance = std::numeric_limits<double>::infinity();
    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
      for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
        const Cell candidate{requested.x + dx, requested.y + dy};
        if (!is_traversable(candidate)) {
          continue;
        }
        const auto world = cell_center(candidate);
        const double distance = std::hypot(
          world.first - robot_x, world.second - robot_y);
        if (distance <= config_.start_snap_distance + kEpsilon &&
          distance < best_distance)
        {
          best = candidate;
          best_distance = distance;
        }
      }
    }
    if (!std::isfinite(best_distance)) {
      throw std::runtime_error(
              "robot pose has no nearby traversable global-costmap cell");
    }
    return best;
  }

  std::vector<double> reachable_path_distances(const Cell & start) const
  {
    using QueueEntry = std::pair<double, std::size_t>;
    std::priority_queue<
      QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> queue;
    std::vector<double> distances(
      grid_.costs.size(), std::numeric_limits<double>::infinity());
    distances[index(start)] = 0.0;
    queue.emplace(0.0, index(start));

    while (!queue.empty()) {
      const auto [distance, current_index] = queue.top();
      queue.pop();
      if (distance > distances[current_index] + kEpsilon) {
        continue;
      }
      const Cell current{
        static_cast<int>(current_index % static_cast<std::size_t>(grid_.width)),
        static_cast<int>(current_index / static_cast<std::size_t>(grid_.width)),
      };
      for (const Cell & offset : kNeighbors) {
        const Cell next{current.x + offset.x, current.y + offset.y};
        if (!can_step(current, next)) {
          continue;
        }
        const bool diagonal = offset.x != 0 && offset.y != 0;
        const double next_distance = distance + grid_.resolution *
          (diagonal ? std::sqrt(2.0) : 1.0);
        if (next_distance + kEpsilon < distances[index(next)]) {
          distances[index(next)] = next_distance;
          queue.emplace(next_distance, index(next));
        }
      }
    }
    return distances;
  }

  const ApproachGrid & grid_;
  ApproachPlannerConfig config_;
};

}  // namespace

PlanarPose compute_object_approach_pose(
  const ApproachGrid & grid,
  const double object_x,
  const double object_y,
  const double robot_x,
  const double robot_y,
  const ApproachPlannerConfig & config)
{
  return ApproachPlanner(grid, config).plan(
    object_x, object_y, robot_x, robot_y);
}

}  // namespace muto_command_layer
