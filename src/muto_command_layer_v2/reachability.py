"""Conservative grid reachability preflight used by motion backends.

This is intentionally a small deterministic helper, not a replacement for
Nav2.  It answers whether a snapshot admits a safe endpoint and returns a
projection when explicitly requested.  Nav2 remains authoritative at
dispatch and execution time.
"""

from dataclasses import dataclass
from math import floor, hypot
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

from .contracts import ReachabilityReport, ReachabilityState


@dataclass(frozen=True)
class OccupancyGrid:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    data: Tuple[int, ...]
    revision: int = 0
    freshness: str = "fresh"

    def __post_init__(self) -> None:
        if self.width <= 0 or self.height <= 0 or self.resolution <= 0.0:
            raise ValueError("grid dimensions and resolution must be positive")
        if len(self.data) != self.width * self.height:
            raise ValueError("grid data length does not match dimensions")


@dataclass(frozen=True)
class ReachabilityConfig:
    free_cost: int = 0
    unknown_cost: int = -1
    footprint_radius_m: float = 0.0
    nominal_speed_mps: float = 0.25
    snap_radius_cells: int = 2


class ReachabilityPlanner:
    """Evaluate connected, footprint-safe cells in a costmap snapshot."""

    _MOVES = (
        (-1, 0, 1.0),
        (1, 0, 1.0),
        (0, -1, 1.0),
        (0, 1, 1.0),
        (-1, -1, 2 ** 0.5),
        (-1, 1, 2 ** 0.5),
        (1, -1, 2 ** 0.5),
        (1, 1, 2 ** 0.5),
    )

    def __init__(self, config: ReachabilityConfig = ReachabilityConfig()) -> None:
        self.config = config

    def evaluate(
        self,
        grid: Optional[OccupancyGrid],
        start_xy: Tuple[float, float],
        goal_xy: Tuple[float, float],
        *,
        projection_policy: str = "reject",
        heading: float = 0.0,
    ) -> ReachabilityReport:
        if projection_policy not in {"reject", "allow"}:
            raise ValueError("projection_policy must be reject or allow")
        if grid is None:
            return ReachabilityReport(
                state=ReachabilityState.UNKNOWN,
                reason_code="costmap_unavailable",
                freshness="unknown",
            )
        if grid.freshness != "fresh":
            return ReachabilityReport(
                state=ReachabilityState.UNKNOWN,
                reason_code="costmap_stale",
                costmap_revision=grid.revision,
                freshness=grid.freshness,
            )
        start = self._world_to_cell(grid, start_xy)
        goal = self._world_to_cell(grid, goal_xy)
        if start is None:
            return self._failure(grid, "preflight_invalid_start")
        if goal is None:
            return self._failure(grid, "preflight_out_of_bounds")
        safe = self._safe_cells(grid)
        start = self._nearest_safe(start, safe, self.config.snap_radius_cells)
        if start is None:
            return self._failure(grid, "preflight_invalid_start")
        if goal in safe:
            selected = goal
            projected = False
        elif projection_policy == "allow":
            selected = self._nearest_reachable_projection(grid, start, goal, safe)
            if selected is None:
                return self._failure(grid, "preflight_disconnected")
            projected = selected != goal
        else:
            if not self._in_bounds(grid, goal):
                return self._failure(grid, "preflight_out_of_bounds")
            cell_cost = grid.data[self._index(grid, goal)]
            reason = (
                "preflight_unknown_space"
                if cell_cost == self.config.unknown_cost
                else "preflight_high_cost"
            )
            return self._failure(grid, reason)
        distances = self._flood_fill(grid, start, safe)
        if selected not in distances:
            return self._failure(grid, "preflight_disconnected")
        distance_m = distances[selected] * grid.resolution
        return ReachabilityReport(
            state=ReachabilityState.REACHABLE,
            reason_code="preflight_goal_projected" if projected else "preflight_reachable",
            path_length_m=distance_m,
            estimated_time_s=distance_m / self.config.nominal_speed_mps,
            costmap_revision=grid.revision,
            freshness=grid.freshness,
            selected_pose=(*self._cell_to_world(grid, selected), heading),
            projected=projected,
        )

    def _failure(self, grid: OccupancyGrid, reason: str) -> ReachabilityReport:
        return ReachabilityReport(
            state=ReachabilityState.UNREACHABLE,
            reason_code=reason,
            costmap_revision=grid.revision,
            freshness=grid.freshness,
        )

    def _safe_cells(self, grid: OccupancyGrid) -> set:
        radius = int(self.config.footprint_radius_m / grid.resolution + 0.999999)
        safe = set()
        for y in range(grid.height):
            for x in range(grid.width):
                cost = grid.data[self._index(grid, (x, y))]
                if cost == self.config.unknown_cost or cost > self.config.free_cost:
                    continue
                if self._footprint_safe(grid, (x, y), radius):
                    safe.add((x, y))
        return safe

    def _footprint_safe(self, grid: OccupancyGrid, cell: Tuple[int, int], radius: int) -> bool:
        cx, cy = cell
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                if dx * dx + dy * dy > radius * radius:
                    continue
                neighbor = (cx + dx, cy + dy)
                if not self._in_bounds(grid, neighbor):
                    return False
                cost = grid.data[self._index(grid, neighbor)]
                if cost == self.config.unknown_cost or cost > self.config.free_cost:
                    return False
        return True

    def _flood_fill(
        self, grid: OccupancyGrid, start: Tuple[int, int], safe: set
    ) -> Dict[Tuple[int, int], float]:
        distances = {start: 0.0}
        queue = [start]
        while queue:
            current = queue.pop(0)
            cx, cy = current
            for dx, dy, cost in self._MOVES:
                nxt = (cx + dx, cy + dy)
                if nxt not in safe or nxt in distances:
                    continue
                if dx and dy and (
                    (cx + dx, cy) not in safe or (cx, cy + dy) not in safe
                ):
                    continue
                distances[nxt] = distances[current] + cost
                queue.append(nxt)
        return distances

    def _nearest_reachable_projection(self, grid, start, goal, safe):
        distances = self._flood_fill(grid, start, safe)
        candidates = [cell for cell in distances if cell in safe]
        if not candidates:
            return None
        return min(
            candidates,
            key=lambda cell: (
                hypot(cell[0] - goal[0], cell[1] - goal[1]),
                distances[cell],
                cell[1],
                cell[0],
            ),
        )

    @staticmethod
    def _index(grid: OccupancyGrid, cell: Tuple[int, int]) -> int:
        return cell[1] * grid.width + cell[0]

    @staticmethod
    def _in_bounds(grid: OccupancyGrid, cell: Tuple[int, int]) -> bool:
        return 0 <= cell[0] < grid.width and 0 <= cell[1] < grid.height

    @staticmethod
    def _world_to_cell(
        grid: OccupancyGrid, point: Tuple[float, float]
    ) -> Optional[Tuple[int, int]]:
        cell = (
            floor((point[0] - grid.origin_x) / grid.resolution),
            floor((point[1] - grid.origin_y) / grid.resolution),
        )
        return cell if ReachabilityPlanner._in_bounds(grid, cell) else None

    @staticmethod
    def _cell_to_world(grid: OccupancyGrid, cell: Tuple[int, int]) -> Tuple[float, float]:
        return (
            grid.origin_x + (cell[0] + 0.5) * grid.resolution,
            grid.origin_y + (cell[1] + 0.5) * grid.resolution,
        )

    @staticmethod
    def _nearest_safe(start, safe, radius):
        if start in safe:
            return start
        candidates = [
            cell for cell in safe
            if max(abs(cell[0] - start[0]), abs(cell[1] - start[1])) <= radius
        ]
        return (
            min(
                candidates,
                key=lambda cell: (
                    abs(cell[0] - start[0]) + abs(cell[1] - start[1]),
                    cell[1],
                    cell[0],
                ),
            )
            if candidates
            else None
        )
