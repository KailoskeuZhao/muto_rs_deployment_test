from muto_command_layer_v2.poi_grid import PoiGridPlanner
from muto_command_layer_v2.reachability import OccupancyGrid, ReachabilityConfig, ReachabilityPlanner


def _grid(data, *, revision=1):
    return OccupancyGrid(
        width=5,
        height=3,
        resolution=1.0,
        origin_x=0.0,
        origin_y=0.0,
        data=tuple(data),
        revision=revision,
    )


def test_poi_grid_selects_only_reachable_known_free_cells():
    # The occupied middle cell blocks the direct route; the lower row remains
    # connected and is the deterministic farthest candidate.
    grid = _grid((0, 0, 0, 0, 0, 0, 100, 100, 100, 0, 0, 0, 0, 0, 0))
    planner = PoiGridPlanner(
        ReachabilityPlanner(ReachabilityConfig(footprint_radius_m=0.0)),
        spacing_m=1.0,
        minimum_progress_m=0.5,
    )
    result = planner.select(grid, (0.5, 0.5, 0.0))

    assert result.reason_code == "poi_goal_selected"
    assert result.selection is not None
    assert result.selection.poi_id == "poi:4,2"
    assert result.selection.grid_revision == 1


def test_poi_grid_never_uses_unknown_cells_and_reports_exhaustion():
    grid = _grid((-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0))
    planner = PoiGridPlanner(
        ReachabilityPlanner(ReachabilityConfig(footprint_radius_m=0.0)),
        spacing_m=1.0,
        minimum_progress_m=0.5,
    )
    first = planner.select(grid, (0.5, 2.5, 0.0))
    assert first.selection is not None
    visited = [first.selection.poi_id]
    exhausted = first
    for _ in range(20):
        exhausted = planner.select(grid, exhausted.selection.pose, visited)
        if exhausted.selection is None:
            break
        visited.append(exhausted.selection.poi_id)
    assert exhausted.reason_code == "poi_exhausted"


def test_poi_grid_reports_no_reachable_goal_for_disconnected_start():
    grid = _grid((100, 100, 100, 100, 100, 100, 0, 0, 0, 100, 100, 100, 100, 100, 100))
    planner = PoiGridPlanner(
        ReachabilityPlanner(ReachabilityConfig(footprint_radius_m=0.0)),
        minimum_progress_m=0.5,
    )
    result = planner.select(grid, (10.5, 10.5, 0.0))
    assert result.reason_code == "poi_no_reachable_goal"
