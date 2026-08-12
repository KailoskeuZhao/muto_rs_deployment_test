#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <limits>
#include <numeric>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "muto_command_layer/visibility_viewpoint_planner.hpp"

namespace
{

using muto_command_layer::GridCell;
using muto_command_layer::ViewpointSelection;
using muto_command_layer::VisibilityCoverageStats;
using muto_command_layer::VisibilityGrid;
using muto_command_layer::VisibilityPlannerConfig;
using muto_command_layer::VisibilityViewpointPlanner;

struct ParsedGrid
{
  VisibilityGrid grid;
  GridCell start;
};

ParsedGrid parse_grid(
  const std::vector<std::string> & rows,
  const double resolution = 0.25)
{
  if (rows.empty() || rows.front().empty()) {
    throw std::invalid_argument("test grid must not be empty");
  }
  ParsedGrid parsed;
  parsed.grid.width = static_cast<int>(rows.front().size());
  parsed.grid.height = static_cast<int>(rows.size());
  parsed.grid.resolution = resolution;
  parsed.grid.occupancy.reserve(
    static_cast<std::size_t>(parsed.grid.width * parsed.grid.height));
  bool found_start = false;
  for (int y = 0; y < parsed.grid.height; ++y) {
    if (static_cast<int>(rows[static_cast<std::size_t>(y)].size()) !=
      parsed.grid.width)
    {
      throw std::invalid_argument("test grid rows must have equal length");
    }
    for (int x = 0; x < parsed.grid.width; ++x) {
      const char value = rows[static_cast<std::size_t>(y)][static_cast<std::size_t>(x)];
      if (value == '#') {
        parsed.grid.occupancy.push_back(100);
      } else if (value == '?') {
        parsed.grid.occupancy.push_back(-1);
      } else if (value == '.' || value == 'S') {
        parsed.grid.occupancy.push_back(0);
        if (value == 'S') {
          if (found_start) {
            throw std::invalid_argument("test grid contains multiple starts");
          }
          parsed.start = GridCell{x, y};
          found_start = true;
        }
      } else {
        throw std::invalid_argument("test grid contains an unsupported character");
      }
    }
  }
  if (!found_start) {
    throw std::invalid_argument("test grid does not contain a start");
  }
  return parsed;
}

ParsedGrid metric_floorplan()
{
  ParsedGrid parsed;
  parsed.grid.width = 300;
  parsed.grid.height = 200;
  parsed.grid.resolution = 0.04;
  parsed.grid.occupancy.assign(
    static_cast<std::size_t>(parsed.grid.width * parsed.grid.height), 0);
  const auto set_occupied = [&parsed](const int x, const int y) {
      parsed.grid.occupancy[static_cast<std::size_t>(
          y * parsed.grid.width + x)] = 100;
    };
  for (int x = 0; x < parsed.grid.width; ++x) {
    set_occupied(x, 0);
    set_occupied(x, parsed.grid.height - 1);
  }
  for (int y = 0; y < parsed.grid.height; ++y) {
    set_occupied(0, y);
    set_occupied(parsed.grid.width - 1, y);
  }
  for (int y = 1; y < 160; ++y) {
    if (y < 72 || y > 96) {
      set_occupied(100, y);
    }
  }
  for (int y = 40; y < parsed.grid.height - 1; ++y) {
    if (y < 118 || y > 142) {
      set_occupied(205, y);
    }
  }
  for (int x = 100; x < 205; ++x) {
    if (x < 142 || x > 166) {
      set_occupied(x, 118);
    }
  }
  for (int y = 25; y < 55; ++y) {
    for (int x = 235; x < 260; ++x) {
      set_occupied(x, y);
    }
  }
  for (int y = 150; y < 178; ++y) {
    for (int x = 35; x < 60; ++x) {
      set_occupied(x, y);
    }
  }
  parsed.start = GridCell{25, 25};
  return parsed;
}

VisibilityPlannerConfig test_config()
{
  VisibilityPlannerConfig config;
  config.robot_clearance = 0.1;
  config.candidate_spacing = 1.0;
  config.visibility_range = 2.0;
  config.boundary_weight = 2.0;
  config.nominal_linear_speed = 0.25;
  config.scan_time = 24.0;
  return config;
}

std::size_t closest_candidate(
  const VisibilityViewpointPlanner & planner,
  const GridCell target)
{
  std::size_t best_index = 0U;
  double best_distance = std::numeric_limits<double>::infinity();
  const auto & candidates = planner.candidates();
  for (std::size_t i = 0; i < candidates.size(); ++i) {
    const double distance = std::hypot(
      static_cast<double>(candidates[i].x - target.x),
      static_cast<double>(candidates[i].y - target.y));
    if (distance < best_distance) {
      best_distance = distance;
      best_index = i;
    }
  }
  return best_index;
}

struct PlanMetrics
{
  std::size_t scans{0U};
  double travel{0.0};
  VisibilityCoverageStats coverage;
};

PlanMetrics run_adaptive_plan(
  VisibilityViewpointPlanner & planner,
  GridCell current,
  const std::size_t maximum_scans = 100U)
{
  PlanMetrics metrics;
  std::set<std::pair<int, int>> selected_cells;
  while (metrics.scans < maximum_scans) {
    const auto selection = planner.select_next(current);
    if (!selection.valid()) {
      break;
    }
    EXPECT_TRUE(selected_cells.emplace(selection.cell.x, selection.cell.y).second)
      << "planner selected the same static viewpoint twice";
    planner.observe(selection.candidate_index);
    metrics.travel += selection.path_length;
    current = selection.cell;
    ++metrics.scans;
  }
  metrics.coverage = planner.coverage_stats();
  return metrics;
}

TEST(VisibilityViewpointPlanner, ExcludesDisconnectedFreeSpace)
{
  const auto parsed = parse_grid({
      "###############",
      "#S.....#......#",
      "#......#......#",
      "#......#......#",
      "#......#......#",
      "###############",
    });
  auto config = test_config();
  config.visibility_range = 10.0;
  VisibilityViewpointPlanner planner(parsed.grid, parsed.start, config);

  EXPECT_TRUE(planner.is_reachable(GridCell{4, 3}));
  EXPECT_FALSE(planner.is_reachable(GridCell{10, 3}));
  EXPECT_TRUE(std::all_of(
      planner.candidates().begin(), planner.candidates().end(),
      [](const GridCell cell) {return cell.x < 7;}));

  const auto metrics = run_adaptive_plan(planner, parsed.start);
  EXPECT_DOUBLE_EQ(metrics.coverage.observable_coverage_ratio(), 1.0);
  EXPECT_DOUBLE_EQ(metrics.coverage.map_coverage_ratio(), 1.0);
}

TEST(VisibilityViewpointPlanner, PoseObservationReducesRemainingGain)
{
  const auto parsed = parse_grid({
      "###############",
      "#.............#",
      "#......S......#",
      "#.............#",
      "###############",
    });
  auto config = test_config();
  config.visibility_range = 4.0;
  VisibilityViewpointPlanner planner(parsed.grid, parsed.start, config);
  const auto before = planner.coverage_report(parsed.start, 0U);
  const std::size_t before_gain = std::accumulate(
    before.points_of_interest.begin(), before.points_of_interest.end(),
    std::size_t{0}, [](const std::size_t total, const auto & point) {
      return total + point.new_free_cells + point.new_boundary_cells;
    });

  EXPECT_TRUE(planner.observe_from(parsed.start, 0.0, std::acos(-1.0)));

  const auto after = planner.coverage_report(parsed.start, 0U);
  const std::size_t after_gain = std::accumulate(
    after.points_of_interest.begin(), after.points_of_interest.end(),
    std::size_t{0}, [](const std::size_t total, const auto & point) {
      return total + point.new_free_cells + point.new_boundary_cells;
    });
  EXPECT_GT(after.stats.covered_free_cells, 0U);
  EXPECT_LT(after_gain, before_gain);
}

TEST(VisibilityViewpointPlanner, RejectsInvalidMissionObservation)
{
  const auto parsed = parse_grid({
      "#######",
      "#..S..#",
      "#######",
    });
  VisibilityViewpointPlanner planner(
    parsed.grid, parsed.start, test_config());

  EXPECT_FALSE(planner.observe_from(GridCell{-1, 1}, 0.0, 1.0));
  EXPECT_FALSE(planner.observe_from(parsed.start, 0.0, 0.0));
  EXPECT_EQ(planner.coverage_stats().covered_free_cells, 0U);
}

TEST(VisibilityViewpointPlanner, UsesNav2CostsForReachability)
{
  auto parsed = parse_grid({
      "############",
      "#S.........#",
      "#..........#",
      "#..........#",
      "############",
    });
  parsed.grid.navigation_costs.assign(
    parsed.grid.occupancy.size(), uint8_t{0});
  for (int y = 1; y < parsed.grid.height - 1; ++y) {
    parsed.grid.navigation_costs[static_cast<std::size_t>(
        y * parsed.grid.width + 5)] = uint8_t{253};
  }

  VisibilityViewpointPlanner planner(
    std::move(parsed.grid), parsed.start, test_config());

  EXPECT_TRUE(planner.is_reachable(GridCell{3, 2}));
  EXPECT_FALSE(planner.is_reachable(GridCell{8, 2}));
  EXPECT_TRUE(std::all_of(
      planner.candidates().begin(), planner.candidates().end(),
      [](const GridCell cell) {return cell.x < 5;}));
}

TEST(VisibilityViewpointPlanner, Nav2CostsAvoidDuplicateRadiusInflation)
{
  auto parsed = parse_grid({
      "#######",
      "#S....#",
      "#######",
    });
  parsed.grid.navigation_costs.assign(
    parsed.grid.occupancy.size(), uint8_t{0});
  auto config = test_config();
  config.robot_clearance = 10.0;

  VisibilityViewpointPlanner planner(
    std::move(parsed.grid), parsed.start, config);

  EXPECT_TRUE(planner.is_traversable(planner.start_cell()));
  EXPECT_TRUE(planner.is_reachable(GridCell{5, 1}));
}

TEST(VisibilityViewpointPlanner, RejectsMisalignedNavigationCostData)
{
  auto parsed = parse_grid({
      "#####",
      "#S..#",
      "#####",
    });
  parsed.grid.navigation_costs.assign(1U, uint8_t{0});

  EXPECT_THROW(
    VisibilityViewpointPlanner(
      std::move(parsed.grid), parsed.start, test_config()),
    std::invalid_argument);
}

TEST(VisibilityViewpointPlanner, WallsBlockVisibilityButDoorwaysDoNot)
{
  const auto parsed = parse_grid({
      "#################",
      "#S......#.......#",
      "#.......#.......#",
      "#.......#.......#",
      "#...............#",
      "#.......#.......#",
      "#.......#.......#",
      "#################",
    });
  auto config = test_config();
  config.visibility_range = 10.0;
  VisibilityViewpointPlanner planner(parsed.grid, parsed.start, config);

  EXPECT_FALSE(planner.line_of_sight(GridCell{3, 2}, GridCell{12, 2}));
  EXPECT_TRUE(planner.line_of_sight(GridCell{3, 4}, GridCell{12, 4}));

  const auto left_candidate = closest_candidate(planner, GridCell{3, 2});
  planner.observe(left_candidate);
  EXPECT_FALSE(planner.is_free_cell_covered(GridCell{12, 2}));
}

TEST(VisibilityViewpointPlanner, DiagonallyTouchingObstaclesBlockVisibility)
{
  const auto parsed = parse_grid({
      "######",
      "#S#..#",
      "##...#",
      "#....#",
      "######",
    });
  auto config = test_config();
  config.robot_clearance = 0.0;
  config.visibility_range = 10.0;
  VisibilityViewpointPlanner planner(parsed.grid, parsed.start, config);

  EXPECT_FALSE(planner.line_of_sight(GridCell{1, 1}, GridCell{2, 2}));
}

TEST(VisibilityViewpointPlanner, SnapsRobotPoseToNearbyTraversableCell)
{
  auto parsed = parse_grid({
      "#########",
      "#.......#",
      "#S......#",
      "#.......#",
      "#########",
  }, 0.1);
  auto config = test_config();
  config.robot_clearance = 0.11;
  config.start_snap_distance = 0.25;

  VisibilityViewpointPlanner planner(
    std::move(parsed.grid), parsed.start, config);

  EXPECT_EQ(planner.start_cell(), (GridCell{2, 2}));
  EXPECT_TRUE(planner.is_reachable(planner.start_cell()));
}

TEST(VisibilityViewpointPlanner, DiscardsRejectedCandidateWithoutCoveringIt)
{
  auto parsed = parse_grid({
      "############",
      "#S.........#",
      "#..........#",
      "#..........#",
      "############",
  });
  auto config = test_config();
  config.visibility_range = 1.5;
  VisibilityViewpointPlanner planner(
    std::move(parsed.grid), parsed.start, config);

  const auto rejected = planner.select_next(planner.start_cell());
  ASSERT_TRUE(rejected.valid());
  planner.discard(rejected.candidate_index);
  const auto replacement = planner.select_next(planner.start_cell());

  ASSERT_TRUE(replacement.valid());
  EXPECT_NE(replacement.candidate_index, rejected.candidate_index);
  EXPECT_EQ(planner.coverage_stats().covered_free_cells, 0U);
}

TEST(VisibilityViewpointPlanner, ReportsRankedObservationPointsWithoutMutatingCoverage)
{
  const auto parsed = parse_grid({
      "###############",
      "#S............#",
      "#.............#",
      "#.............#",
      "#.............#",
      "###############",
    });
  auto config = test_config();
  config.visibility_range = 1.25;
  config.minimum_new_target_cells = 1U;
  VisibilityViewpointPlanner planner(parsed.grid, parsed.start, config);

  const auto before = planner.coverage_stats();
  const auto report = planner.coverage_report(parsed.start, 3U);

  EXPECT_EQ(report.current_cell, parsed.start);
  EXPECT_EQ(report.stats.covered_free_cells, before.covered_free_cells);
  ASSERT_GE(report.points_of_interest.size(), 2U);
  EXPECT_LE(report.points_of_interest.size(), 3U);
  EXPECT_EQ(
    report.points_of_interest.front().candidate_index,
    planner.select_next(parsed.start).candidate_index);
  EXPECT_EQ(planner.coverage_stats().covered_free_cells, before.covered_free_cells);

  for (const auto & point : report.points_of_interest) {
    EXPECT_TRUE(point.valid());
    EXPECT_GT(point.new_free_cells + point.new_boundary_cells, 0U);
    EXPECT_GT(point.weighted_gain, 0.0);
    EXPECT_GT(point.score, 0.0);
  }
  for (std::size_t i = 1U; i < report.points_of_interest.size(); ++i) {
    const ViewpointSelection & previous = report.points_of_interest[i - 1U];
    const ViewpointSelection & current = report.points_of_interest[i];
    EXPECT_GE(previous.score + 1.0e-12, current.score);
  }
}

TEST(VisibilityViewpointPlanner, ObservationPointReportReflectsNewCoverage)
{
  const auto parsed = parse_grid({
      "###############",
      "#S............#",
      "#.............#",
      "#.............#",
      "#.............#",
      "###############",
    });
  auto config = test_config();
  config.visibility_range = 1.25;
  config.minimum_new_target_cells = 1U;
  VisibilityViewpointPlanner planner(parsed.grid, parsed.start, config);

  const auto initial = planner.coverage_report(parsed.start, 5U);
  ASSERT_FALSE(initial.points_of_interest.empty());
  const auto first = initial.points_of_interest.front();
  planner.observe(first.candidate_index);

  const auto after = planner.coverage_report(first.cell, 5U);
  EXPECT_GT(after.stats.covered_free_cells, initial.stats.covered_free_cells);
  ASSERT_FALSE(after.points_of_interest.empty());
  EXPECT_NE(after.points_of_interest.front().candidate_index, first.candidate_index);
  EXPECT_LT(after.points_of_interest.front().new_free_cells, initial.stats.target_free_cells);
}

TEST(VisibilityViewpointPlanner, OpenRoomNeedsOneCentralScan)
{
  const auto parsed = parse_grid({
      "#############",
      "#...........#",
      "#...........#",
      "#...........#",
      "#...........#",
      "#...........#",
      "#.....S.....#",
      "#...........#",
      "#...........#",
      "#...........#",
      "#...........#",
      "#...........#",
      "#############",
    });
  auto config = test_config();
  config.candidate_spacing = 10.0;
  config.visibility_range = 10.0;
  VisibilityViewpointPlanner planner(parsed.grid, parsed.start, config);

  const auto metrics = run_adaptive_plan(planner, parsed.start);
  EXPECT_EQ(metrics.scans, 1U);
  EXPECT_NEAR(metrics.travel, 0.0, 1.0e-9);
  EXPECT_DOUBLE_EQ(metrics.coverage.map_coverage_ratio(), 1.0);
  EXPECT_DOUBLE_EQ(metrics.coverage.boundary_coverage_ratio(), 1.0);
}

TEST(VisibilityViewpointPlanner, CoarseSamplingStillCoversNarrowWindingCorridor)
{
  const auto parsed = parse_grid({
      "#################",
      "#S..............#",
      "###############.#",
      "#...............#",
      "#.###############",
      "#...............#",
      "#################",
    });
  auto config = test_config();
  config.candidate_spacing = 1.5;
  config.visibility_range = 1.0;
  VisibilityViewpointPlanner planner(parsed.grid, parsed.start, config);

  EXPECT_TRUE(planner.is_reachable(GridCell{2, 5}));
  const auto metrics = run_adaptive_plan(planner, parsed.start);
  if (metrics.coverage.map_coverage_ratio() < 0.98) {
    std::cout << "narrow corridor candidates:";
    for (const auto & candidate : planner.candidates()) {
      std::cout << " (" << candidate.x << "," << candidate.y << ")";
    }
    std::cout << " uncovered:";
    for (int y = 0; y < parsed.grid.height; ++y) {
      for (int x = 0; x < parsed.grid.width; ++x) {
        const GridCell cell{x, y};
        const auto cell_index = static_cast<std::size_t>(
          y * parsed.grid.width + x);
        if (parsed.grid.occupancy[cell_index] == 0 &&
          !planner.is_free_cell_covered(cell))
        {
          std::cout << " (" << x << "," << y << ")";
        }
      }
    }
    std::cout << std::endl;
  }
  EXPECT_LT(metrics.scans, planner.candidates().size());
  EXPECT_GE(metrics.coverage.map_coverage_ratio(), 0.98);
  EXPECT_DOUBLE_EQ(metrics.coverage.observable_coverage_ratio(), 1.0);
}

TEST(VisibilityViewpointPlanner, AdaptiveSelectionCoversIndoorFloorplan)
{
  const auto parsed = parse_grid({
      "#########################",
      "#S........#.............#",
      "#.........#..####.......#",
      "#............#..........#",
      "#.........#..#..........#",
      "#####.#####..#..........#",
      "#............#####.######",
      "#.......................#",
      "#..#####................#",
      "#.......................#",
      "#########################",
    });
  auto config = test_config();
  config.candidate_spacing = 1.0;
  config.visibility_range = 1.75;
  VisibilityViewpointPlanner planner(parsed.grid, parsed.start, config);

  const auto metrics = run_adaptive_plan(planner, parsed.start);
  std::cout << "visibility benchmark: candidates="
            << metrics.coverage.candidate_count
            << " scans=" << metrics.scans
            << " travel_m=" << metrics.travel
            << " map_coverage=" << metrics.coverage.map_coverage_ratio()
            << " boundary_coverage=" << metrics.coverage.boundary_coverage_ratio()
            << std::endl;

  EXPECT_LT(metrics.scans, metrics.coverage.candidate_count);
  EXPECT_LE(metrics.scans, 16U);
  EXPECT_GE(metrics.coverage.map_coverage_ratio(), 0.95);
  EXPECT_DOUBLE_EQ(metrics.coverage.observable_coverage_ratio(), 1.0);
  EXPECT_DOUBLE_EQ(metrics.coverage.boundary_coverage_ratio(), 1.0);
}

TEST(VisibilityViewpointPlanner, MetricResolutionFloorplanBenchmark)
{
  const auto parsed = metric_floorplan();
  VisibilityPlannerConfig config;
  const auto construction_start = std::chrono::steady_clock::now();
  VisibilityViewpointPlanner planner(parsed.grid, parsed.start, config);
  const auto construction_end = std::chrono::steady_clock::now();
  const auto metrics = run_adaptive_plan(planner, parsed.start, 100U);
  const auto planning_end = std::chrono::steady_clock::now();
  const double construction_ms = std::chrono::duration<double, std::milli>(
    construction_end - construction_start).count();
  const double selection_ms = std::chrono::duration<double, std::milli>(
    planning_end - construction_end).count();

  std::cout << "metric visibility benchmark: candidates="
            << metrics.coverage.candidate_count
            << " scans=" << metrics.scans
            << " travel_m=" << metrics.travel
            << " map_coverage=" << metrics.coverage.map_coverage_ratio()
            << " construction_ms=" << construction_ms
            << " selection_ms=" << selection_ms
            << std::endl;

  EXPECT_LT(metrics.scans, metrics.coverage.candidate_count);
  EXPECT_LT(metrics.scans, 40U);
  EXPECT_GE(metrics.coverage.map_coverage_ratio(), 0.98);
  EXPECT_DOUBLE_EQ(metrics.coverage.observable_coverage_ratio(), 1.0);
  EXPECT_DOUBLE_EQ(metrics.coverage.boundary_coverage_ratio(), 1.0);
}

}  // namespace
