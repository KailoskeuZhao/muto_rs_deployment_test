import math

import pytest

from muto_command_layer_v2.contracts import ReachabilityReport, ReachabilityState
from muto_command_layer_v2.ros_authorities import (
    Nav2MotionAuthority,
    RosMapReachability,
    _shortest_angle,
    _snapshot_revision,
)


class _Stamp:
    sec = 4
    nanosec = 9


class _Point:
    x = 1.23456
    y = -0.25


class _Object:
    name = "chair_1"
    label = "chair"
    class_id = 7
    position = _Point()
    image_path = "/tmp/chair.jpg"
    observation_count = 3
    last_seen = _Stamp()


def test_registry_revision_is_deterministic_and_content_scoped():
    first = _snapshot_revision((_Object(),))
    second = _snapshot_revision((_Object(),))
    assert first == second
    assert len(first) == 16

    # Detector freshness updates are not a new shortlist.  They must not
    # invalidate a same-revision visual rejection/confirmation or interrupt a
    # bounded frontier search.
    refreshed = _Object()
    refreshed.last_seen = type("_LaterStamp", (), {"sec": 99, "nanosec": 7})()
    refreshed.observation_count = 99
    refreshed.position = type("_JitteredPoint", (), {"x": 1.2349, "y": -0.2498})()
    assert _snapshot_revision((refreshed,)) == first

    moved = _Object()
    moved.position = type("_MovedPoint", (), {"x": 1.31, "y": -0.25})()
    assert _snapshot_revision((moved,)) != first


def test_heading_wrap_chooses_positive_pi_for_exact_half_turn():
    assert _shortest_angle(math.pi) == math.pi
    assert _shortest_angle(-math.pi) == math.pi
    assert -math.pi < _shortest_angle(3.0 * math.pi / 2.0) <= math.pi


def test_nav2_authority_rejects_projection_when_tool_does_not_allow_it():
    authority = object.__new__(Nav2MotionAuthority)
    authority._reachability_fn = lambda _point, _board, _policy: ReachabilityReport(
        state=ReachabilityState.REACHABLE,
        reason_code="preflight_goal_projected",
        selected_pose=(0.5, 0.5, 0.0),
        projected=True,
    )
    result = authority.go_to_point((1.0, 1.0), "reject", None)
    assert not result.success
    assert result.reason_code == "projection_required"


def test_map_revision_tracks_grid_changes_not_repeated_timestamps():
    pytest.importorskip("nav_msgs")
    from nav_msgs.msg import OccupancyGrid as RosOccupancyGrid

    authority = object.__new__(RosMapReachability)
    authority._revision = 0
    authority._map_fingerprint = None
    authority._grid = None

    message = RosOccupancyGrid()
    message.info.width = 2
    message.info.height = 1
    message.info.resolution = 0.1
    message.data = [0, 0]
    authority._map_callback(message)
    assert authority._revision == 1

    repeated = RosOccupancyGrid()
    repeated.info = message.info
    repeated.data = [0, 0]
    authority._map_callback(repeated)
    assert authority._revision == 1

    changed = RosOccupancyGrid()
    changed.info = message.info
    changed.data = [0, 100]
    authority._map_callback(changed)
    assert authority._revision == 2
