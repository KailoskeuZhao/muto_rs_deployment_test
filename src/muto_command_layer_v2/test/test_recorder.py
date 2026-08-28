import sys

import pytest

pytest.importorskip("rclpy")

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
        "/muto/poi_grid/selected_pose",
        "/muto/poi_grid/result",
    }.issubset(topics)
    assert all(
        sensor not in topic
        for topic in topics
        for sensor in ("camera", "lidar", "imu", "scan", "pointcloud")
    )
