import pytest

pytest.importorskip("rclpy")

import importlib
import sys

for _path in list(sys.path):
    if _path.rstrip("/").endswith("/src") or "/src/muto_command_layer_v2" in _path:
        sys.path.remove(_path)
for _name in list(sys.modules):
    if _name == "muto_command_layer_v2" or _name.startswith("muto_command_layer_v2."):
        del sys.modules[_name]
importlib.invalidate_caches()

from muto_command_layer_v2.high_level_recorder_node import HIGH_LEVEL_TOPICS


def test_recorder_allowlist_contains_only_bounded_high_level_topics():
    topics = {topic for topic, _type_name, _message_type in HIGH_LEVEL_TOPICS}
    assert {
        "muto/mission_board",
        "muto/mission_event",
        "muto/mission_recorder_status",
        "muto/mission_recorder_manifest",
    }.issubset(topics)
    assert {
        "/explore/selected_frontier",
        "/frontier_goal_adapter/original_goal",
        "/frontier_goal_adapter/projected_goal",
        "/frontier_goal_adapter/status",
    }.issubset(topics)
    assert all(
        sensor not in topic
        for topic in topics
        for sensor in ("camera", "lidar", "imu", "scan", "pointcloud")
    )
