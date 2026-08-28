"""Deterministic point-of-interest grid search planning.

The POI grid is the v2 search authority.  It plans observation viewpoints
from the known, reachable part of the current map; it never sends navigation
goals and it never treats unknown cells as endpoints.  Nav2 remains the sole
navigation and obstacle-avoidance authority.
"""

from dataclasses import dataclass
from typing import Iterable, Optional, Set, Tuple

from .reachability import OccupancyGrid, ReachabilityPlanner


@dataclass(frozen=True)
class PoiSelection:
    """One deterministic observation point selected from a grid snapshot."""

    poi_id: str
    cell: Tuple[int, int]
    pose: Tuple[float, float, float]
    grid_revision: int
    path_length_m: float
    estimated_time_s: float


@dataclass(frozen=True)
class PoiGridDecision:
    """Planner result before Nav2 is invoked."""

    outcome: str
    reason_code: str
    grid_revision: int = 0
    selection: Optional[PoiSelection] = None
    visited_count: int = 0


class PoiGridPlanner:
    """Select a stable, reachable set of map-backed observation viewpoints.

    The planner deliberately works on known-free cells only.  A grid cell is
    identified by its map coordinates rather than by the current map revision,
    so an unchanged viewpoint is not revisited merely because the map was
    republished.  A changed map can still make a previously visited cell
    useful to Nav2, but it does not make the search loop spin on it.
    """

    def __init__(
        self,
        reachability: Optional[ReachabilityPlanner] = None,
        *,
        spacing_m: float = 1.0,
        nominal_speed_mps: float = 0.25,
        minimum_progress_m: float = 0.25,
    ) -> None:
        if spacing_m <= 0.0 or nominal_speed_mps <= 0.0 or minimum_progress_m < 0.0:
            raise ValueError("POI grid parameters must be positive")
        self._reachability = reachability or ReachabilityPlanner()
        self.spacing_m = float(spacing_m)
        self.nominal_speed_mps = float(nominal_speed_mps)
        self.minimum_progress_m = float(minimum_progress_m)

    def select(
        self,
        grid: Optional[OccupancyGrid],
        start_pose: Optional[Tuple[float, float, float]],
        visited_ids: Iterable[str] = (),
    ) -> PoiGridDecision:
        visited: Set[str] = {str(item) for item in visited_ids}
        revision = int(getattr(grid, "revision", 0) or 0)
        if grid is None:
            return PoiGridDecision(
                "unavailable", "poi_grid_unavailable", revision, visited_count=len(visited)
            )
        if grid.freshness != "fresh":
            return PoiGridDecision(
                "stale", "poi_grid_stale", revision, visited_count=len(visited)
            )
        if start_pose is None:
            return PoiGridDecision(
                "unavailable", "map_pose_unavailable", revision, visited_count=len(visited)
            )

        distances = self._reachability.reachable_cells(
            grid, (float(start_pose[0]), float(start_pose[1]))
        )
        if not distances:
            return PoiGridDecision(
                "unreachable", "poi_no_reachable_goal", revision, visited_count=len(visited)
            )

        spacing_cells = max(1, int(round(self.spacing_m / grid.resolution)))
        candidates = [
            cell for cell in distances
            if self._poi_id(cell) not in visited
            and self._is_sample_cell(cell, spacing_cells)
            and distances[cell] * grid.resolution >= self.minimum_progress_m
        ]
        # Small maps can contain no cell on the preferred sampling lattice.
        # Fall back to every unvisited reachable cell before declaring
        # exhaustion; this keeps fixtures and tight rooms useful.
        if not candidates:
            candidates = [
                cell for cell in distances
                if self._poi_id(cell) not in visited
                and distances[cell] * grid.resolution >= self.minimum_progress_m
            ]
        if not candidates:
            return PoiGridDecision(
                "exhausted", "poi_exhausted", revision, visited_count=len(visited)
            )

        cell = max(
            candidates,
            key=lambda item: (
                distances[item],
                -item[1],
                -item[0],
            ),
        )
        x, y = self._reachability._cell_to_world(grid, cell)
        # Nav2MotionAuthority evaluates and dispatches this point using the
        # current robot heading.  Keep the published selected pose honest;
        # a separate rotate_to_heading tool remains available when a skill
        # explicitly needs a viewpoint orientation.
        heading = float(start_pose[2])
        path_length_m = float(distances[cell] * grid.resolution)
        selection = PoiSelection(
            poi_id=self._poi_id(cell),
            cell=cell,
            pose=(x, y, heading),
            grid_revision=revision,
            path_length_m=path_length_m,
            estimated_time_s=path_length_m / self.nominal_speed_mps,
        )
        return PoiGridDecision(
            "selected",
            "poi_goal_selected",
            revision,
            selection,
            visited_count=len(visited),
        )

    @staticmethod
    def _poi_id(cell: Tuple[int, int]) -> str:
        return "poi:{},{}".format(int(cell[0]), int(cell[1]))

    @staticmethod
    def _is_sample_cell(cell: Tuple[int, int], spacing_cells: int) -> bool:
        return cell[0] % spacing_cells == 0 and cell[1] % spacing_cells == 0
