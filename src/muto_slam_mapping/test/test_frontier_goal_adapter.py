import importlib.machinery
import importlib.util
import math
from pathlib import Path
import sys

from nav_msgs.msg import OccupancyGrid


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SCRIPT_PATH = PACKAGE_ROOT / 'scripts' / 'frontier_goal_adapter'


def _load_adapter_module():
    loader = importlib.machinery.SourceFileLoader(
        'frontier_goal_adapter_under_test', str(SCRIPT_PATH))
    specification = importlib.util.spec_from_loader(loader.name, loader)
    module = importlib.util.module_from_spec(specification)
    sys.modules[loader.name] = module
    loader.exec_module(module)
    return module


adapter = _load_adapter_module()


def _grid(width=100, height=80, resolution=0.04, fill=-1):
    message = OccupancyGrid()
    message.header.frame_id = 'map'
    message.info.width = width
    message.info.height = height
    message.info.resolution = resolution
    message.info.origin.orientation.w = 1.0
    message.data = [fill] * (width * height)
    return message


def _set_rect(message, min_x, min_y, max_x, max_y, value):
    for cell_y in range(min_y, max_y + 1):
        for cell_x in range(min_x, max_x + 1):
            message.data[cell_y * message.info.width + cell_x] = value


def _cell_center(message, cell_x, cell_y):
    return (
        message.info.origin.position.x
        + (cell_x + 0.5) * message.info.resolution,
        message.info.origin.position.y
        + (cell_y + 0.5) * message.info.resolution,
    )


def test_raw_unknown_frontier_goal_is_projected_inside_known_space():
    message = _grid()
    _set_rect(message, 25, 20, 74, 59, 0)
    target_x, target_y = _cell_center(message, 50, 60)

    result = adapter.project_goal_to_known_free(
        message, target_x, target_y, 0.27, 0.80)

    assert result is not None
    assert result.displaced is True
    assert result.x == target_x
    assert result.y < target_y
    assert math.isclose(result.displacement, 0.32, abs_tol=1.0e-9)


def test_already_safe_known_free_goal_stays_at_its_cell_center():
    message = _grid(fill=0)
    target_x, target_y = _cell_center(message, 50, 40)

    result = adapter.project_goal_to_known_free(
        message, target_x, target_y, 0.27, 0.80)

    assert result is not None
    assert result.displaced is False
    assert result.displacement == 0.0
    assert result.x == target_x
    assert result.y == target_y


def test_goal_is_rejected_when_no_footprint_safe_cell_exists_nearby():
    message = _grid()
    _set_rect(message, 45, 35, 54, 44, 0)
    target_x, target_y = _cell_center(message, 50, 40)

    result = adapter.project_goal_to_known_free(
        message, target_x, target_y, 0.27, 0.80)

    assert result is None


def test_sparse_free_scan_chain_is_not_a_robot_sized_endpoint():
    message = _grid(width=120, height=120)
    # This resembles the recorded failure: point-free cells make a diagonal
    # chain through unknown space, but no 0.27 m circular footprint fits.
    for cell in range(35, 76, 2):
        message.data[cell * message.info.width + cell] = 0
    target_x, target_y = _cell_center(message, 75, 75)

    result = adapter.project_goal_to_known_free(
        message, target_x, target_y, 0.27, 0.80)

    assert message.data[75 * message.info.width + 75] == 0
    assert result is None


def test_unsafe_distant_frontier_becomes_reachable_staged_advance():
    message = _grid(width=120, height=100)
    _set_rect(message, 10, 20, 55, 80, 0)
    # A disconnected free island is closer to the requested target, but the
    # robot cannot reach it through footprint-safe known-free space.
    _set_rect(message, 88, 35, 108, 65, 0)
    robot_x, robot_y = _cell_center(message, 30, 50)
    target_x, target_y = _cell_center(message, 98, 50)

    capped = adapter.project_goal_to_reachable_known_free(
        message, target_x, target_y, robot_x, robot_y,
        0.27, 0.75, 0.80)
    staged = adapter.project_goal_to_reachable_known_free(
        message, target_x, target_y, robot_x, robot_y,
        0.27, 0.75, 0.0)

    assert capped is None
    assert staged is not None
    assert staged.displaced
    assert staged.displacement > 0.80
    assert staged.x < target_x
    staged_cell = adapter._world_to_map_continuous(
        message, staged.x, staged.y)
    assert adapter._footprint_is_known_free(
        message,
        int(math.floor(staged_cell[0])),
        int(math.floor(staged_cell[1])),
        0.27,
        0,
    )


def test_projection_uses_lateral_staging_cell_when_nearest_is_too_close():
    message = _grid(width=100, height=80)
    _set_rect(message, 20, 33, 80, 49, 0)
    robot_x, robot_y = _cell_center(message, 50, 40)
    target_x, target_y = _cell_center(message, 50, 60)

    nearest = adapter.project_goal_to_reachable_known_free(
        message, target_x, target_y, robot_x, robot_y,
        0.27, 0.75, 0.0, 0, 0.0)
    staged = adapter.project_goal_to_reachable_known_free(
        message, target_x, target_y, robot_x, robot_y,
        0.27, 0.75, 0.0, 0, 0.20)

    assert nearest is not None
    assert staged is not None
    assert math.hypot(nearest.x - robot_x, nearest.y - robot_y) < 0.20
    assert math.hypot(staged.x - robot_x, staged.y - robot_y) >= 0.20
    assert staged.displaced


def test_projection_respects_a_rotated_map_origin():
    message = _grid(fill=0)
    message.info.origin.position.x = 2.0
    message.info.origin.position.y = -1.0
    yaw = math.pi / 2.0
    message.info.origin.orientation.z = math.sin(yaw / 2.0)
    message.info.origin.orientation.w = math.cos(yaw / 2.0)
    target_x, target_y = adapter._map_to_world(message, 50, 40)

    result = adapter.project_goal_to_known_free(
        message, target_x, target_y, 0.27, 0.80)

    assert result is not None
    assert result.displaced is False
    assert math.isclose(result.x, target_x, abs_tol=1.0e-9)
    assert math.isclose(result.y, target_y, abs_tol=1.0e-9)
