"""ROS graph smoke test for the independent v2 authority adapters."""

import sys
import threading
import time

import pytest

pytest.importorskip("rclpy")

# launch_testing can preload the source namespace package.  Use the installed
# generated interfaces for this graph test, just like test_ros_transport.py.
for _path in list(sys.path):
    if _path.rstrip("/").endswith("/src") or "/src/muto_command_layer_v2" in _path:
        sys.path.remove(_path)
for _name in list(sys.modules):
    if _name == "muto_command_layer_v2" or _name.startswith("muto_command_layer_v2."):
        del sys.modules[_name]

import rclpy
from frontier_exploration_ros2.srv import ControlExploration
from geometry_msgs.msg import Point
from nav2_msgs.action import NavigateToPose, Spin
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sam2_object_registry.msg import StoredObject
from sam2_object_registry.srv import GetStoredObjects

from muto_command_layer_v2.backend_adapters import CandidateDecision
from muto_command_layer_v2.contracts import MissionBoard, ReachabilityReport, ReachabilityState
from muto_command_layer_v2.ros_authorities import (
    Nav2MotionAuthority,
    RosFrontierAuthority,
    RosRegistryAuthority,
)


class _AuthorityServer(Node):
    def __init__(self):
        super().__init__("v2_authority_server")
        self.create_service(GetStoredObjects, "/v2/test_registry", self._query)
        self._navigate = ActionServer(self, NavigateToPose, "/v2/test_navigate", self._navigate_cb)
        self._spin = ActionServer(self, Spin, "/v2/test_spin", self._spin_cb)
        self._frontier_actions = []
        self._registry_queries = []
        self.create_service(ControlExploration, "/v2/test_frontier", self._frontier_cb)

    def _query(self, request, response):
        self._registry_queries.append((request.name, request.label))
        if request.name and request.name != "chair_1":
            return response
        if request.label and request.label != "chair":
            return response
        item = StoredObject()
        item.name = "chair_1"
        item.label = "chair"
        item.class_id = 7
        item.position = Point(x=1.0, y=2.0, z=0.0)
        item.image_path = "/tmp/chair_1.jpg"
        item.observation_count = 3
        response.result.objects = [item]
        return response

    def _navigate_cb(self, goal_handle):
        goal_handle.succeed()
        return NavigateToPose.Result()

    def _spin_cb(self, goal_handle):
        goal_handle.succeed()
        return Spin.Result()

    def _frontier_cb(self, request, response):
        self._frontier_actions.append(int(request.action))
        response.accepted = True
        response.scheduled = False
        response.state = (
            ControlExploration.Request.STATE_RUNNING
            if request.action == ControlExploration.Request.ACTION_START
            else ControlExploration.Request.STATE_IDLE
        )
        response.message = "test frontier control accepted"
        return response


def test_real_ros_service_and_nav2_action_adapters_round_trip():
    rclpy.init()
    server = _AuthorityServer()
    client_node = Node("v2_authority_client")
    board = MissionBoard(object_request="chair")
    registry = RosRegistryAuthority(
        client_node,
        query_service="/v2/test_registry",
        visual_selector=lambda _request, candidates, _board: tuple(
            CandidateDecision(candidate.candidate_id, candidate.candidate_id == "chair_1")
            for candidate in candidates
        ),
    )
    motion = Nav2MotionAuthority(
        client_node,
        navigate_action="/v2/test_navigate",
        spin_action="/v2/test_spin",
        reachability_fn=lambda _point, _board: ReachabilityReport(
            state=ReachabilityState.REACHABLE,
            reason_code="test_preflight",
            path_length_m=1.0,
        ),
        pose_fn=lambda _board: (0.0, 0.0, 0.0),
    )
    frontier = RosFrontierAuthority(
        client_node,
        control_service="/v2/test_frontier",
        completion_topic="/v2/test_completion",
        service_timeout_s=1.0,
        observe_duration_s=0.05,
    )
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(server)
    executor.add_node(client_node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        deadline = time.monotonic() + 5.0
        snapshot = registry.query("chair", board)
        while not snapshot.checked and time.monotonic() < deadline:
            snapshot = registry.query("chair", board)
        assert snapshot.checked
        assert [candidate.candidate_id for candidate in snapshot.candidates] == ["chair_1"]
        natural_snapshot = registry.query("the purple chair", board)
        assert [candidate.candidate_id for candidate in natural_snapshot.candidates] == [
            "chair_1"
        ]
        assert ("", "chair") in server._registry_queries
        decisions = registry.inspect("chair", snapshot, ("chair_1",), board)
        assert decisions[0].confirmed
        rotated = motion.rotate_to_heading(3.141592653589793, board)
        assert rotated.success
        navigated = motion.go_to_point((1.0, 2.0), "allow", board)
        assert navigated.success
        observed = frontier.observe(board)
        assert observed.success
        assert observed.reason_code == "frontier_cycle_completed"
        assert server._frontier_actions == [
            ControlExploration.Request.ACTION_START,
            ControlExploration.Request.ACTION_STOP,
        ]
    finally:
        executor.shutdown(timeout_sec=2.0)
        client_node.destroy_node()
        server.destroy_node()
        rclpy.shutdown()
        thread.join(timeout=2.0)
