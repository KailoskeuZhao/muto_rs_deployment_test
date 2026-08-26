"""End-to-end ROS graph test for the independent v2 composition.

This deliberately uses the real v2 composition and authority transports.  The
external authorities are deterministic ROS test servers, so the test is not a
model-quality or Nav2-world test; it verifies that the production wiring keeps
the natural-language -> planner -> registry shortlist -> visual confirmation
-> executive completion chain intact.
"""

import json
import os
import sys
import threading
import time

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

import rclpy
from geometry_msgs.msg import Point
from muto_vlm_socket.action import GenerateVlm
from nav2_msgs.action import NavigateToPose, Spin
from rclpy.action import ActionClient, ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sam2_object_registry.msg import StoredObject
from sam2_object_registry.srv import GetStoredObjects

from muto_command_layer_v2.action import Mission
from muto_command_layer_v2.composition import create_v2_node
from muto_command_layer_v2.contracts import EventType, LifecycleState


class _FullGraphAuthorities(Node):
    """Deterministic registry and VLM authorities for one complete mission."""

    def __init__(self, image_path):
        super().__init__("v2_full_graph_authorities")
        self.registry_queries = []
        self.planner_calls = 0
        self.inspection_calls = 0
        self._image_path = image_path
        self._vlm = ActionServer(
            self, GenerateVlm, "/v2/full_graph_vlm", self._vlm_callback
        )
        self.create_service(GetStoredObjects, "/v2/full_graph_registry", self._registry)
        # These are not used by the report-confirmed scenario, but exposing the
        # action names makes sure composition can construct all Nav2 clients.
        self._navigate = ActionServer(
            self, NavigateToPose, "/v2/full_graph_nav", self._nav_callback
        )
        self._spin = ActionServer(
            self, Spin, "/v2/full_graph_spin", self._spin_callback
        )

    def _registry(self, request, response):
        self.registry_queries.append((request.name, request.label))
        if request.name and request.name != "chair-1":
            return response
        if request.label and request.label != "chair":
            return response
        item = StoredObject()
        item.name = "chair-1"
        item.label = "chair"
        item.class_id = 7
        item.position = Point(x=1.0, y=2.0, z=0.0)
        item.image_path = self._image_path
        item.observation_count = 3
        response.result.objects = [item]
        return response

    def _vlm_callback(self, goal_handle):
        schema = goal_handle.request.response_json_schema
        result = GenerateVlm.Result()
        result.success = True
        result.error_message = ""
        if "candidate_decisions" in schema:
            self.inspection_calls += 1
            result.response_text = json.dumps({
                "schema_version": "muto_command_layer_v2",
                "candidate_decisions": [{
                    "candidate_id": "chair-1",
                    "confirmed": True,
                    "confidence": 0.91,
                    "reason_code": "stored_image_matches_request",
                }],
            })
        else:
            self.planner_calls += 1
            state = {}
            if goal_handle.request.content:
                prompt = goal_handle.request.content[0].text
                marker = "STATE_JSON="
                if marker in prompt:
                    state = json.loads(prompt.split(marker, 1)[1])
            if self.planner_calls == 1:
                tool = {
                    "name": "query_registry",
                    "object_request": "purple chair",
                    "candidate_ids": [],
                    "registry_revision": "",
                    "candidate_id": "",
                    "point": None,
                    "heading": None,
                    "frame_id": "map",
                    "projection_policy": "reject",
                }
                proposal = None
            elif self.planner_calls == 2:
                tool = {
                    "name": "inspect_candidates",
                    "object_request": "purple chair",
                    "candidate_ids": ["chair-1"],
                    "registry_revision": state.get("registry_revision", ""),
                    "candidate_id": "",
                    "point": None,
                    "heading": None,
                    "frame_id": "map",
                    "projection_policy": "reject",
                }
                proposal = None
            else:
                tool = None
                proposal = "report_confirmed"
            result.response_text = json.dumps({
                "schema_version": "muto_command_layer_v2",
                "skill": "search_for_object",
                "tool": tool,
                "completion_proposal": proposal,
                "rationale": "deterministic full-graph test decision",
            })
        goal_handle.succeed()
        return result

    def _nav_callback(self, goal_handle):
        goal_handle.succeed()
        return NavigateToPose.Result()

    def _spin_callback(self, goal_handle):
        goal_handle.succeed()
        return Spin.Result()


def _wait_future(future, timeout=10.0):
    done = threading.Event()
    holder = []

    def _complete(completed):
        holder.append(completed)
        done.set()

    future.add_done_callback(_complete)
    assert done.wait(timeout), "ROS future timed out"
    return holder[0].result()


def test_real_v2_composition_runs_natural_language_registry_confirmation_chain(tmp_path):
    image_path = str(tmp_path / "chair.jpg")
    # The inspector transport only requires bytes; a tiny JPEG marker keeps the
    # test independent of image libraries while exercising the image payload.
    with open(image_path, "wb") as stream:
        stream.write(b"\xff\xd8\xff\xd9")

    rclpy.init()
    authorities = _FullGraphAuthorities(image_path)
    v2 = create_v2_node(
        action_name="/v2/full_graph_mission",
        vlm_action="/v2/full_graph_vlm",
        registry_query_service="/v2/full_graph_registry",
        nav_action="/v2/full_graph_nav",
        spin_action="/v2/full_graph_spin",
        vlm_timeout_s=2.0,
    )
    client_node = Node("v2_full_graph_client")
    client = ActionClient(client_node, Mission, "/v2/full_graph_mission")
    executor = MultiThreadedExecutor(num_threads=8)
    for node in (authorities, v2, client_node):
        executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        assert client.wait_for_server(timeout_sec=5.0)
        goal = Mission.Goal()
        goal.request_id = "v2-full-graph-request"
        goal.objective = "please find the purple chair"
        goal.object_request = ""
        goal.completion_policy = ""
        goal.schema_version = "muto_command_layer_v2"
        goal_handle = _wait_future(client.send_goal_async(goal))
        assert goal_handle.accepted
        wrapped = _wait_future(goal_handle.get_result_async(), timeout=15.0)
        assert wrapped.result.outcome == Mission.Result.OUTCOME_SUCCEEDED
        assert wrapped.result.confirmed_target_id == "chair-1"
        assert authorities.planner_calls == 3
        assert authorities.inspection_calls == 1
        assert ("", "chair") in authorities.registry_queries
        assert v2.executive.board.lifecycle_state is LifecycleState.SUCCEEDED
        event_types = [event.event_type for event in v2.executive.events]
        assert EventType.CANDIDATE_CONFIRMED in event_types
        assert event_types[-1] is EventType.MISSION_SUCCEEDED
    finally:
        executor.shutdown(timeout_sec=3.0)
        client_node.destroy_node()
        v2.destroy_node()
        authorities.destroy_node()
        thread.join(timeout=3.0)
        rclpy.shutdown()
