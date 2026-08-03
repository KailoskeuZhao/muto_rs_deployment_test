#ifndef MUTO_COMMAND_LAYER__VISIBILITY_VIEWPOINT_PLANNER_HPP_
#define MUTO_COMMAND_LAYER__VISIBILITY_VIEWPOINT_PLANNER_HPP_

#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace muto_command_layer
{

struct GridCell
{
  int x{0};
  int y{0};

  bool operator==(const GridCell & other) const
  {
    return x == other.x && y == other.y;
  }
};

struct VisibilityGrid
{
  int width{0};
  int height{0};
  double resolution{0.0};
  std::vector<int8_t> occupancy;
  std::vector<uint8_t> navigation_costs;
};

struct VisibilityPlannerConfig
{
  int free_threshold{20};
  int occupied_threshold{65};
  int maximum_traversable_cost{252};
  double robot_clearance{0.22};
  double candidate_spacing{0.75};
  double visibility_range{2.5};
  double boundary_weight{2.0};
  double nominal_linear_speed{0.25};
  double scan_time{24.0};
  double start_snap_distance{0.5};
  std::size_t minimum_new_target_cells{1U};
};

struct ViewpointSelection
{
  static constexpr std::size_t kInvalidIndex =
    std::numeric_limits<std::size_t>::max();

  std::size_t candidate_index{kInvalidIndex};
  GridCell cell;
  std::size_t new_free_cells{0U};
  std::size_t new_boundary_cells{0U};
  double path_length{0.0};
  double score{0.0};

  bool valid() const
  {
    return candidate_index != kInvalidIndex;
  }
};

struct VisibilityCoverageStats
{
  std::size_t candidate_count{0U};
  std::size_t target_free_cells{0U};
  std::size_t coverable_free_cells{0U};
  std::size_t covered_free_cells{0U};
  std::size_t target_boundary_cells{0U};
  std::size_t coverable_boundary_cells{0U};
  std::size_t covered_boundary_cells{0U};

  double map_coverage_ratio() const;
  double observable_coverage_ratio() const;
  double boundary_coverage_ratio() const;
};

class VisibilityViewpointPlanner
{
public:
  VisibilityViewpointPlanner(
    VisibilityGrid grid,
    GridCell start,
    VisibilityPlannerConfig config = VisibilityPlannerConfig{});

  const std::vector<GridCell> & candidates() const;
  const GridCell & start_cell() const;
  ViewpointSelection select_next(const GridCell & current) const;
  void observe(std::size_t candidate_index);
  void discard(std::size_t candidate_index);

  VisibilityCoverageStats coverage_stats() const;
  bool is_traversable(const GridCell & cell) const;
  bool is_reachable(const GridCell & cell) const;
  bool is_free_cell_covered(const GridCell & cell) const;
  bool line_of_sight(const GridCell & source, const GridCell & target) const;

private:
  struct CandidateVisibility
  {
    GridCell cell;
    std::vector<uint64_t> free_bits;
  };

  std::size_t index(const GridCell & cell) const;
  bool in_bounds(const GridCell & cell) const;
  bool is_free(const GridCell & cell) const;
  int navigation_cost(const GridCell & cell) const;
  bool can_step(
    const GridCell & source,
    const GridCell & target,
    const std::vector<uint8_t> & mask) const;

  void validate() const;
  void build_clearance_mask();
  GridCell snap_start(const GridCell & requested_start) const;
  void build_reachable_masks(const GridCell & start);
  void build_target_masks();
  void build_candidates(const GridCell & start);
  void build_candidate_visibility();
  void cast_visibility_ray(
    const GridCell & source,
    const GridCell & endpoint,
    std::vector<uint64_t> & visible_bits) const;
  std::vector<double> path_distances(const GridCell & source) const;

  static void set_bit(std::vector<uint64_t> & bits, std::size_t bit);
  static bool bit_is_set(const std::vector<uint64_t> & bits, std::size_t bit);
  static std::size_t count_bits(const std::vector<uint64_t> & bits);
  static std::size_t count_new_bits(
    const std::vector<uint64_t> & visible,
    const std::vector<uint64_t> & covered);
  static std::size_t count_new_masked_bits(
    const std::vector<uint64_t> & visible,
    const std::vector<uint64_t> & target,
    const std::vector<uint64_t> & covered);

  VisibilityGrid grid_;
  VisibilityPlannerConfig config_;
  std::size_t word_count_{0U};
  std::vector<int> clearance_cells_;
  std::vector<uint8_t> traversable_;
  std::vector<uint8_t> reachable_;
  std::vector<uint8_t> target_free_;
  std::vector<uint8_t> target_boundary_;
  std::vector<GridCell> candidate_cells_;
  std::vector<CandidateVisibility> candidate_visibility_;
  std::vector<uint8_t> discarded_candidates_;
  std::vector<uint64_t> target_boundary_bits_;
  std::vector<uint64_t> coverable_free_bits_;
  std::vector<uint64_t> coverable_boundary_bits_;
  std::vector<uint64_t> covered_free_bits_;
  std::vector<uint64_t> covered_boundary_bits_;
  GridCell start_cell_;
};

}  // namespace muto_command_layer

#endif  // MUTO_COMMAND_LAYER__VISIBILITY_VIEWPOINT_PLANNER_HPP_
