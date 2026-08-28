"""Transport-level smoke checks for the v2 POI-grid authority."""

import sys

import pytest

pytest.importorskip("rclpy")

# launch_testing/ament_pytest may prepend the source workspace while this file
# is collected.  The ROSIDL message package must come from the installed
# Humble prefix; otherwise ``muto_command_layer_v2.msg`` becomes a namespace
# package without the generated ``PoiGridResult`` type.  Remove only source
# entries and modules, preserving the installed type-support modules.
for _path in list(sys.path):
    if _path.rstrip("/").endswith("/src") or "/src/muto_command_layer_v2" in _path:
        sys.path.remove(_path)
for _name, _module in list(sys.modules.items()):
    _module_file = getattr(_module, "__file__", "") or ""
    if (
        (_name == "muto_command_layer_v2" or _name.startswith("muto_command_layer_v2."))
        and "/src/muto_command_layer_v2" in _module_file
    ):
        del sys.modules[_name]

from builtin_interfaces.msg import Time
from muto_command_layer_v2.contracts import MissionBoard
from muto_command_layer_v2.reachability import OccupancyGrid
from muto_command_layer_v2.ros_authorities import RosPoiGridAuthority
from muto_command_layer_v2.backend_adapters import MotionResult


class _Clock:
    def now(self):
        return self

    def to_msg(self):
        return Time(sec=1, nanosec=2)


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _Node:
    def __init__(self):
        self.publishers = {}

    def create_publisher(self, message_type, topic, _qos):
        publisher = _Publisher()
        self.publishers[topic] = publisher
        return publisher

    def get_clock(self):
        return _Clock()


class _Reachability:
    def __init__(self):
        self.grid = OccupancyGrid(
            width=3,
            height=1,
            resolution=1.0,
            origin_x=0.0,
            origin_y=0.0,
            data=(0, 0, 0),
            revision=7,
        )

    def snapshot(self, _board):
        return self.grid, (0.5, 0.5, 0.0)


class _Motion:
    def __init__(self):
        self.calls = []

    def go_to_point(self, point, projection_policy, _board):
        self.calls.append((point, projection_policy))
        return MotionResult(True, reason_code="motion_completed", progress_delta=2.0)


def test_poi_authority_publishes_selection_then_terminal_result_and_delegates_to_nav2():
    node = _Node()
    reachability = _Reachability()
    motion = _Motion()
    authority = RosPoiGridAuthority(
        node,
        reachability=reachability,
        motion=motion,
        spacing_m=1.0,
        minimum_progress_m=0.25,
    )

    result = authority.observe(MissionBoard(mission_id="mission-0001"))

    assert result.success
    assert result.reason_code == "poi_goal_succeeded"
    assert motion.calls == [((2.5, 0.5), "reject")]
    selected = node.publishers["/muto/poi_grid/result"].messages[0]
    terminal = node.publishers["/muto/poi_grid/result"].messages[1]
    assert selected.outcome == "poi_goal_selected"
    assert selected.reason_code == "poi_goal_selected"
    assert selected.poi_id == "poi:2,0"
    assert terminal.outcome == "poi_goal_succeeded"
    assert terminal.reason_code == "poi_goal_succeeded"
    assert terminal.poi_id == "poi:2,0"
    assert node.publishers["/muto/poi_grid/selected_pose"].messages[0].pose.position.x == 2.5
