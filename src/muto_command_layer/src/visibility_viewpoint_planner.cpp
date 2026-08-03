#include "muto_command_layer/visibility_viewpoint_planner.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>

namespace muto_command_layer
{

namespace
{

constexpr std::array<GridCell, 8> kNeighbors{{
  {-1, -1}, {0, -1}, {1, -1},
  {-1, 0}, {1, 0},
  {-1, 1}, {0, 1}, {1, 1}
}};

constexpr std::array<GridCell, 4> kCardinalNeighbors{{
  {0, -1}, {-1, 0}, {1, 0}, {0, 1}
}};

double ratio(const std::size_t numerator, const std::size_t denominator)
{
  return denominator == 0U ? 1.0 :
         static_cast<double>(numerator) / static_cast<double>(denominator);
}

}  // namespace

double VisibilityCoverageStats::map_coverage_ratio() const
{
  return ratio(covered_free_cells, target_free_cells);
}

double VisibilityCoverageStats::observable_coverage_ratio() const
{
  return ratio(covered_free_cells, coverable_free_cells);
}

double VisibilityCoverageStats::boundary_coverage_ratio() const
{
  return ratio(covered_boundary_cells, coverable_boundary_cells);
}

VisibilityViewpointPlanner::VisibilityViewpointPlanner(
  VisibilityGrid grid,
  const GridCell start,
  VisibilityPlannerConfig config)
: grid_(std::move(grid)), config_(std::move(config))
{
  validate();
  word_count_ = (grid_.occupancy.size() + 63U) / 64U;
  build_clearance_mask();
  start_cell_ = snap_start(start);
  build_reachable_masks(start_cell_);
  build_target_masks();
  build_candidates(start_cell_);
  build_candidate_visibility();
  discarded_candidates_.assign(candidate_cells_.size(), 0U);
}

const std::vector<GridCell> & VisibilityViewpointPlanner::candidates() const
{
  return candidate_cells_;
}

const GridCell & VisibilityViewpointPlanner::start_cell() const
{
  return start_cell_;
}

void VisibilityViewpointPlanner::validate() const
{
  if (grid_.width <= 0 || grid_.height <= 0) {
    throw std::invalid_argument("visibility grid dimensions must be positive");
  }
  const auto expected_size = static_cast<std::size_t>(grid_.width) *
    static_cast<std::size_t>(grid_.height);
  if (grid_.occupancy.size() != expected_size) {
    throw std::invalid_argument("visibility grid occupancy size does not match dimensions");
  }
  if (!grid_.navigation_costs.empty() &&
    grid_.navigation_costs.size() != expected_size)
  {
    throw std::invalid_argument(
            "visibility navigation-cost size does not match dimensions");
  }
  if (!std::isfinite(grid_.resolution) || grid_.resolution <= 0.0) {
    throw std::invalid_argument("visibility grid resolution must be finite and positive");
  }
  if (config_.free_threshold < 0 || config_.free_threshold > 100 ||
    config_.occupied_threshold < 0 || config_.occupied_threshold > 100 ||
    config_.free_threshold >= config_.occupied_threshold)
  {
    throw std::invalid_argument("occupancy thresholds are invalid");
  }
  if (config_.maximum_traversable_cost < 0 ||
    config_.maximum_traversable_cost > 252)
  {
    throw std::invalid_argument(
            "maximum_traversable_cost must be in [0, 252]");
  }
  const auto require_positive = [](const double value, const char * name) {
      if (!std::isfinite(value) || value <= 0.0) {
        throw std::invalid_argument(std::string(name) + " must be finite and positive");
      }
    };
  if (!std::isfinite(config_.robot_clearance) || config_.robot_clearance < 0.0) {
    throw std::invalid_argument("robot_clearance must be finite and nonnegative");
  }
  require_positive(config_.candidate_spacing, "candidate_spacing");
  require_positive(config_.visibility_range, "visibility_range");
  require_positive(config_.boundary_weight, "boundary_weight");
  require_positive(config_.nominal_linear_speed, "nominal_linear_speed");
  require_positive(config_.scan_time, "scan_time");
  if (!std::isfinite(config_.start_snap_distance) ||
    config_.start_snap_distance < 0.0)
  {
    throw std::invalid_argument(
            "start_snap_distance must be finite and nonnegative");
  }
  if (config_.minimum_new_target_cells == 0U) {
    throw std::invalid_argument("minimum_new_target_cells must be positive");
  }
}

std::size_t VisibilityViewpointPlanner::index(const GridCell & cell) const
{
  return static_cast<std::size_t>(cell.y) * static_cast<std::size_t>(grid_.width) +
         static_cast<std::size_t>(cell.x);
}

bool VisibilityViewpointPlanner::in_bounds(const GridCell & cell) const
{
  return cell.x >= 0 && cell.y >= 0 &&
         cell.x < grid_.width && cell.y < grid_.height;
}

bool VisibilityViewpointPlanner::is_free(const GridCell & cell) const
{
  if (!in_bounds(cell)) {
    return false;
  }
  const int value = static_cast<int>(grid_.occupancy[index(cell)]);
  return value >= 0 && value <= config_.free_threshold;
}

bool VisibilityViewpointPlanner::is_traversable(const GridCell & cell) const
{
  return in_bounds(cell) && !traversable_.empty() && traversable_[index(cell)] != 0U;
}

int VisibilityViewpointPlanner::navigation_cost(const GridCell & cell) const
{
  if (!in_bounds(cell) || grid_.navigation_costs.empty()) {
    return 0;
  }
  return static_cast<int>(grid_.navigation_costs[index(cell)]);
}

bool VisibilityViewpointPlanner::is_reachable(const GridCell & cell) const
{
  return in_bounds(cell) && !reachable_.empty() && reachable_[index(cell)] != 0U;
}

bool VisibilityViewpointPlanner::is_free_cell_covered(const GridCell & cell) const
{
  return in_bounds(cell) && bit_is_set(covered_free_bits_, index(cell));
}

void VisibilityViewpointPlanner::build_clearance_mask()
{
  const int cell_count = grid_.width * grid_.height;
  const int infinity = std::numeric_limits<int>::max() / 4;
  clearance_cells_.assign(static_cast<std::size_t>(cell_count), infinity);
  std::queue<GridCell> queue;

  for (int y = 0; y < grid_.height; ++y) {
    for (int x = 0; x < grid_.width; ++x) {
      const GridCell cell{x, y};
      const auto cell_index = index(cell);
      if (!is_free(cell)) {
        clearance_cells_[cell_index] = 0;
        queue.push(cell);
      } else if (x == 0 || y == 0 || x == grid_.width - 1 || y == grid_.height - 1) {
        clearance_cells_[cell_index] = 1;
        queue.push(cell);
      }
    }
  }

  while (!queue.empty()) {
    const GridCell current = queue.front();
    queue.pop();
    const int next_distance = clearance_cells_[index(current)] + 1;
    for (const auto & offset : kNeighbors) {
      const GridCell next{current.x + offset.x, current.y + offset.y};
      if (!in_bounds(next) || clearance_cells_[index(next)] <= next_distance) {
        continue;
      }
      clearance_cells_[index(next)] = next_distance;
      queue.push(next);
    }
  }

  traversable_.assign(grid_.occupancy.size(), 0U);
  for (int y = 0; y < grid_.height; ++y) {
    for (int x = 0; x < grid_.width; ++x) {
      const GridCell cell{x, y};
      if (!is_free(cell)) {
        continue;
      }
      if (!grid_.navigation_costs.empty()) {
        if (navigation_cost(cell) <= config_.maximum_traversable_cost) {
          traversable_[index(cell)] = 1U;
        }
        continue;
      }
      const double conservative_clearance =
        (static_cast<double>(clearance_cells_[index(cell)]) - 0.5) *
        grid_.resolution;
      if (conservative_clearance + 1.0e-9 >= config_.robot_clearance) {
        traversable_[index(cell)] = 1U;
      }
    }
  }
}

GridCell VisibilityViewpointPlanner::snap_start(
  const GridCell & requested_start) const
{
  if (is_traversable(requested_start)) {
    return requested_start;
  }

  const int radius_cells = static_cast<int>(std::ceil(
      config_.start_snap_distance / grid_.resolution));
  GridCell best;
  double best_distance = std::numeric_limits<double>::infinity();
  for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
      const GridCell candidate{requested_start.x + dx, requested_start.y + dy};
      if (!is_traversable(candidate)) {
        continue;
      }
      const double distance = std::hypot(
        static_cast<double>(dx), static_cast<double>(dy)) * grid_.resolution;
      if (distance <= config_.start_snap_distance + 1.0e-9 &&
        distance < best_distance)
      {
        best = candidate;
        best_distance = distance;
      }
    }
  }
  if (!std::isfinite(best_distance)) {
    throw std::invalid_argument(
            "no traversable start cell is within start_snap_distance");
  }
  return best;
}

bool VisibilityViewpointPlanner::can_step(
  const GridCell & source,
  const GridCell & target,
  const std::vector<uint8_t> & mask) const
{
  if (!in_bounds(target) || mask[index(target)] == 0U) {
    return false;
  }
  const int dx = target.x - source.x;
  const int dy = target.y - source.y;
  if (dx != 0 && dy != 0) {
    const GridCell side_x{source.x + dx, source.y};
    const GridCell side_y{source.x, source.y + dy};
    if (!in_bounds(side_x) || !in_bounds(side_y) ||
      mask[index(side_x)] == 0U || mask[index(side_y)] == 0U)
    {
      return false;
    }
  }
  return true;
}

void VisibilityViewpointPlanner::build_reachable_masks(const GridCell & start)
{
  reachable_.assign(grid_.occupancy.size(), 0U);
  std::queue<GridCell> queue;
  reachable_[index(start)] = 1U;
  queue.push(start);
  while (!queue.empty()) {
    const GridCell current = queue.front();
    queue.pop();
    for (const auto & offset : kNeighbors) {
      const GridCell next{current.x + offset.x, current.y + offset.y};
      if (!can_step(current, next, traversable_) || reachable_[index(next)] != 0U) {
        continue;
      }
      reachable_[index(next)] = 1U;
      queue.push(next);
    }
  }

  target_free_.assign(grid_.occupancy.size(), 0U);
  target_free_[index(start)] = 1U;
  queue.push(start);
  while (!queue.empty()) {
    const GridCell current = queue.front();
    queue.pop();
    for (const auto & offset : kCardinalNeighbors) {
      const GridCell next{current.x + offset.x, current.y + offset.y};
      if (!in_bounds(next) || !is_free(next) || target_free_[index(next)] != 0U) {
        continue;
      }
      target_free_[index(next)] = 1U;
      queue.push(next);
    }
  }
}

void VisibilityViewpointPlanner::build_target_masks()
{
  target_boundary_.assign(grid_.occupancy.size(), 0U);
  target_boundary_bits_.assign(word_count_, 0U);
  for (int y = 0; y < grid_.height; ++y) {
    for (int x = 0; x < grid_.width; ++x) {
      const GridCell cell{x, y};
      if (target_free_[index(cell)] == 0U) {
        continue;
      }
      for (const auto & offset : kCardinalNeighbors) {
        const GridCell neighbor{cell.x + offset.x, cell.y + offset.y};
        if (!in_bounds(neighbor)) {
          continue;
        }
        const int value = static_cast<int>(grid_.occupancy[index(neighbor)]);
        if (value >= config_.occupied_threshold) {
          target_boundary_[index(cell)] = 1U;
          set_bit(target_boundary_bits_, index(cell));
          break;
        }
      }
    }
  }
}

void VisibilityViewpointPlanner::build_candidates(const GridCell & start)
{
  const int spacing_cells = std::max(
    1, static_cast<int>(std::lround(config_.candidate_spacing / grid_.resolution)));
  std::vector<uint8_t> visited(grid_.occupancy.size(), 0U);

  for (int block_y = 0; block_y < grid_.height; block_y += spacing_cells) {
    const int block_end_y = std::min(grid_.height, block_y + spacing_cells);
    for (int block_x = 0; block_x < grid_.width; block_x += spacing_cells) {
      const int block_end_x = std::min(grid_.width, block_x + spacing_cells);
      for (int seed_y = block_y; seed_y < block_end_y; ++seed_y) {
        for (int seed_x = block_x; seed_x < block_end_x; ++seed_x) {
          const GridCell seed{seed_x, seed_y};
          if (reachable_[index(seed)] == 0U || visited[index(seed)] != 0U) {
            continue;
          }

          GridCell best = seed;
          int best_navigation_cost = navigation_cost(seed);
          int best_clearance = clearance_cells_[index(seed)];
          double best_center_distance = std::numeric_limits<double>::infinity();
          std::queue<GridCell> queue;
          visited[index(seed)] = 1U;
          queue.push(seed);
          while (!queue.empty()) {
            const GridCell current = queue.front();
            queue.pop();
            const double center_x = 0.5 * static_cast<double>(block_x + block_end_x - 1);
            const double center_y = 0.5 * static_cast<double>(block_y + block_end_y - 1);
            const double center_distance = std::hypot(
              static_cast<double>(current.x) - center_x,
              static_cast<double>(current.y) - center_y);
            const int clearance = clearance_cells_[index(current)];
            const int current_navigation_cost = navigation_cost(current);
            if (current_navigation_cost < best_navigation_cost ||
              (current_navigation_cost == best_navigation_cost &&
              (clearance > best_clearance ||
              (clearance == best_clearance &&
              center_distance < best_center_distance))))
            {
              best = current;
              best_navigation_cost = current_navigation_cost;
              best_clearance = clearance;
              best_center_distance = center_distance;
            }

            for (const auto & offset : kNeighbors) {
              const GridCell next{current.x + offset.x, current.y + offset.y};
              if (!in_bounds(next) || next.x < block_x || next.x >= block_end_x ||
                next.y < block_y || next.y >= block_end_y ||
                visited[index(next)] != 0U || !can_step(current, next, reachable_))
              {
                continue;
              }
              visited[index(next)] = 1U;
              queue.push(next);
            }
          }
          candidate_cells_.push_back(best);
        }
      }
    }
  }

  const auto add_candidate = [this](const GridCell cell) {
      if (std::find(candidate_cells_.begin(), candidate_cells_.end(), cell) ==
        candidate_cells_.end())
      {
        candidate_cells_.push_back(cell);
      }
    };

  // Coarse tiles can contain a bent narrow corridor. Add its visibility-critical
  // turn and dead-end cells because a camera ray cannot see around the bend.
  for (int y = 0; y < grid_.height; ++y) {
    for (int x = 0; x < grid_.width; ++x) {
      const GridCell cell{x, y};
      if (reachable_[index(cell)] == 0U) {
        continue;
      }
      const bool left = x > 0 && reachable_[index(GridCell{x - 1, y})] != 0U;
      const bool right = x + 1 < grid_.width &&
        reachable_[index(GridCell{x + 1, y})] != 0U;
      const bool up = y > 0 && reachable_[index(GridCell{x, y - 1})] != 0U;
      const bool down = y + 1 < grid_.height &&
        reachable_[index(GridCell{x, y + 1})] != 0U;
      const int degree = static_cast<int>(left) + static_cast<int>(right) +
        static_cast<int>(up) + static_cast<int>(down);
      const bool orthogonal_turn = degree == 2 && (left || right) && (up || down);
      if (degree <= 1 || orthogonal_turn) {
        add_candidate(cell);
      }
    }
  }

  add_candidate(start);
}

void VisibilityViewpointPlanner::set_bit(
  std::vector<uint64_t> & bits, const std::size_t bit)
{
  bits[bit / 64U] |= uint64_t{1} << (bit % 64U);
}

bool VisibilityViewpointPlanner::bit_is_set(
  const std::vector<uint64_t> & bits, const std::size_t bit)
{
  return !bits.empty() && (bits[bit / 64U] & (uint64_t{1} << (bit % 64U))) != 0U;
}

std::size_t VisibilityViewpointPlanner::count_bits(
  const std::vector<uint64_t> & bits)
{
  std::size_t count = 0U;
  for (const uint64_t word : bits) {
    count += static_cast<std::size_t>(__builtin_popcountll(word));
  }
  return count;
}

std::size_t VisibilityViewpointPlanner::count_new_bits(
  const std::vector<uint64_t> & visible,
  const std::vector<uint64_t> & covered)
{
  std::size_t count = 0U;
  for (std::size_t i = 0; i < visible.size(); ++i) {
    count += static_cast<std::size_t>(
      __builtin_popcountll(visible[i] & ~covered[i]));
  }
  return count;
}

std::size_t VisibilityViewpointPlanner::count_new_masked_bits(
  const std::vector<uint64_t> & visible,
  const std::vector<uint64_t> & target,
  const std::vector<uint64_t> & covered)
{
  std::size_t count = 0U;
  for (std::size_t i = 0; i < visible.size(); ++i) {
    count += static_cast<std::size_t>(
      __builtin_popcountll(visible[i] & target[i] & ~covered[i]));
  }
  return count;
}

void VisibilityViewpointPlanner::cast_visibility_ray(
  const GridCell & source,
  const GridCell & endpoint,
  std::vector<uint64_t> & visible_bits) const
{
  int x = source.x;
  int y = source.y;
  const int dx = std::abs(endpoint.x - source.x);
  const int dy = std::abs(endpoint.y - source.y);
  const int step_x = source.x < endpoint.x ? 1 : -1;
  const int step_y = source.y < endpoint.y ? 1 : -1;
  int error = dx - dy;

  while (true) {
    const GridCell current{x, y};
    if (!in_bounds(current)) {
      return;
    }
    const double range = std::hypot(
      static_cast<double>(current.x - source.x),
      static_cast<double>(current.y - source.y)) * grid_.resolution;
    if (range > config_.visibility_range + 1.0e-9) {
      return;
    }
    if (!is_free(current)) {
      return;
    }
    if (target_free_[index(current)] != 0U) {
      set_bit(visible_bits, index(current));
    }
    if (x == endpoint.x && y == endpoint.y) {
      return;
    }

    const int doubled_error = 2 * error;
    const int previous_x = x;
    const int previous_y = y;
    if (doubled_error > -dy) {
      error -= dy;
      x += step_x;
    }
    if (doubled_error < dx) {
      error += dx;
      y += step_y;
    }
    if (x != previous_x && y != previous_y &&
      !is_free(GridCell{x, previous_y}) &&
      !is_free(GridCell{previous_x, y}))
    {
      return;
    }
  }
}

void VisibilityViewpointPlanner::build_candidate_visibility()
{
  coverable_free_bits_.assign(word_count_, 0U);
  coverable_boundary_bits_.assign(word_count_, 0U);
  covered_free_bits_.assign(word_count_, 0U);
  covered_boundary_bits_.assign(word_count_, 0U);
  const int radius_cells = std::max(
    1, static_cast<int>(std::ceil(config_.visibility_range / grid_.resolution)));

  for (const GridCell & candidate_cell : candidate_cells_) {
    CandidateVisibility candidate;
    candidate.cell = candidate_cell;
    candidate.free_bits.assign(word_count_, 0U);

    for (int x = candidate_cell.x - radius_cells;
      x <= candidate_cell.x + radius_cells; ++x)
    {
      cast_visibility_ray(
        candidate_cell, GridCell{x, candidate_cell.y - radius_cells},
        candidate.free_bits);
      cast_visibility_ray(
        candidate_cell, GridCell{x, candidate_cell.y + radius_cells},
        candidate.free_bits);
    }
    for (int y = candidate_cell.y - radius_cells + 1;
      y < candidate_cell.y + radius_cells; ++y)
    {
      cast_visibility_ray(
        candidate_cell, GridCell{candidate_cell.x - radius_cells, y},
        candidate.free_bits);
      cast_visibility_ray(
        candidate_cell, GridCell{candidate_cell.x + radius_cells, y},
        candidate.free_bits);
    }

    for (std::size_t word = 0; word < word_count_; ++word) {
      coverable_free_bits_[word] |= candidate.free_bits[word];
      coverable_boundary_bits_[word] |=
        candidate.free_bits[word] & target_boundary_bits_[word];
    }
    candidate_visibility_.push_back(std::move(candidate));
  }
}

bool VisibilityViewpointPlanner::line_of_sight(
  const GridCell & source, const GridCell & target) const
{
  if (!in_bounds(source) || !in_bounds(target) ||
    !is_free(source) || !is_free(target))
  {
    return false;
  }
  const double range = std::hypot(
    static_cast<double>(target.x - source.x),
    static_cast<double>(target.y - source.y)) * grid_.resolution;
  if (range > config_.visibility_range + 1.0e-9) {
    return false;
  }

  int x = source.x;
  int y = source.y;
  const int dx = std::abs(target.x - source.x);
  const int dy = std::abs(target.y - source.y);
  const int step_x = source.x < target.x ? 1 : -1;
  const int step_y = source.y < target.y ? 1 : -1;
  int error = dx - dy;
  while (x != target.x || y != target.y) {
    const int doubled_error = 2 * error;
    const int previous_x = x;
    const int previous_y = y;
    if (doubled_error > -dy) {
      error -= dy;
      x += step_x;
    }
    if (doubled_error < dx) {
      error += dx;
      y += step_y;
    }
    if (x != previous_x && y != previous_y &&
      !is_free(GridCell{x, previous_y}) &&
      !is_free(GridCell{previous_x, y}))
    {
      return false;
    }
    if (!is_free(GridCell{x, y})) {
      return false;
    }
  }
  return true;
}

std::vector<double> VisibilityViewpointPlanner::path_distances(
  const GridCell & source) const
{
  const double infinity = std::numeric_limits<double>::infinity();
  std::vector<double> distances(grid_.occupancy.size(), infinity);
  if (!is_reachable(source)) {
    return distances;
  }

  using QueueEntry = std::pair<double, GridCell>;
  const auto compare = [](const QueueEntry & left, const QueueEntry & right) {
      return left.first > right.first;
    };
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, decltype(compare)> queue(compare);
  distances[index(source)] = 0.0;
  queue.push({0.0, source});
  while (!queue.empty()) {
    const auto [distance, current] = queue.top();
    queue.pop();
    if (distance > distances[index(current)] + 1.0e-12) {
      continue;
    }
    for (const auto & offset : kNeighbors) {
      const GridCell next{current.x + offset.x, current.y + offset.y};
      if (!can_step(current, next, reachable_)) {
        continue;
      }
      const double step_cost = offset.x != 0 && offset.y != 0 ?
        std::sqrt(2.0) * grid_.resolution : grid_.resolution;
      const double next_distance = distance + step_cost;
      if (next_distance + 1.0e-12 >= distances[index(next)]) {
        continue;
      }
      distances[index(next)] = next_distance;
      queue.push({next_distance, next});
    }
  }
  return distances;
}

ViewpointSelection VisibilityViewpointPlanner::select_next(
  const GridCell & current) const
{
  ViewpointSelection best;
  const auto distances = path_distances(current);
  double best_weighted_gain = 0.0;

  for (std::size_t i = 0; i < candidate_visibility_.size(); ++i) {
    if (discarded_candidates_[i] != 0U) {
      continue;
    }
    const auto & candidate = candidate_visibility_[i];
    const double path_length = distances[index(candidate.cell)];
    if (!std::isfinite(path_length)) {
      continue;
    }
    const std::size_t free_gain = count_new_bits(
      candidate.free_bits, covered_free_bits_);
    const std::size_t boundary_gain = count_new_masked_bits(
      candidate.free_bits, target_boundary_bits_, covered_boundary_bits_);
    if (free_gain + boundary_gain < config_.minimum_new_target_cells) {
      continue;
    }
    const double weighted_gain = static_cast<double>(free_gain) +
      config_.boundary_weight * static_cast<double>(boundary_gain);
    const double action_time = config_.scan_time +
      path_length / config_.nominal_linear_speed;
    const double score = weighted_gain / action_time;
    const bool better_score = score > best.score + 1.0e-12;
    const bool equal_score_more_gain =
      std::abs(score - best.score) <= 1.0e-12 &&
      weighted_gain > best_weighted_gain + 1.0e-12;
    const bool equal_gain_shorter_path =
      std::abs(score - best.score) <= 1.0e-12 &&
      std::abs(weighted_gain - best_weighted_gain) <= 1.0e-12 &&
      path_length < best.path_length;
    if (!best.valid() || better_score || equal_score_more_gain ||
      equal_gain_shorter_path)
    {
      best.candidate_index = i;
      best.cell = candidate.cell;
      best.new_free_cells = free_gain;
      best.new_boundary_cells = boundary_gain;
      best.path_length = path_length;
      best.score = score;
      best_weighted_gain = weighted_gain;
    }
  }
  return best;
}

void VisibilityViewpointPlanner::observe(const std::size_t candidate_index)
{
  if (candidate_index >= candidate_visibility_.size()) {
    throw std::out_of_range("viewpoint candidate index is out of range");
  }
  const auto & candidate = candidate_visibility_[candidate_index];
  for (std::size_t word = 0; word < word_count_; ++word) {
    covered_free_bits_[word] |= candidate.free_bits[word];
    covered_boundary_bits_[word] |=
      candidate.free_bits[word] & target_boundary_bits_[word];
  }
}

void VisibilityViewpointPlanner::discard(const std::size_t candidate_index)
{
  if (candidate_index >= discarded_candidates_.size()) {
    throw std::out_of_range("viewpoint candidate index is out of range");
  }
  discarded_candidates_[candidate_index] = 1U;
}

VisibilityCoverageStats VisibilityViewpointPlanner::coverage_stats() const
{
  VisibilityCoverageStats stats;
  stats.candidate_count = candidate_cells_.size();
  stats.target_free_cells = static_cast<std::size_t>(std::count(
      target_free_.begin(), target_free_.end(), uint8_t{1}));
  stats.coverable_free_cells = count_bits(coverable_free_bits_);
  stats.covered_free_cells = count_bits(covered_free_bits_);
  stats.target_boundary_cells = static_cast<std::size_t>(std::count(
      target_boundary_.begin(), target_boundary_.end(), uint8_t{1}));
  stats.coverable_boundary_cells = count_bits(coverable_boundary_bits_);
  stats.covered_boundary_cells = count_bits(covered_boundary_bits_);
  return stats;
}

}  // namespace muto_command_layer
