"""ROS 2 adapters for independent v2 registry and navigation authorities.

These adapters are intentionally thin transport boundaries.  They do not
import the legacy command layer or copy its lifecycle logic: registry data is
converted to a revision-scoped snapshot, while Nav2 remains the authority for
actual navigation and obstacle avoidance.
"""

from __future__ import annotations

import hashlib
import json
import math
import re
import threading
import time
from typing import Callable, Optional, Sequence, Tuple

from .backend_adapters import (
    CandidateDecision,
    MotionResult,
    RegistryCandidate,
    RegistrySnapshot,
)
from .contracts import MissionBoard, ReachabilityReport, SCHEMA_VERSION
from .poi_grid import PoiGridPlanner


def _wait_future(future, timeout_s: float, operation: str):
    done = threading.Event()
    holder = []

    def _complete(completed):
        holder.append(completed)
        done.set()

    future.add_done_callback(_complete)
    if not done.wait(timeout_s):
        raise RuntimeError("{} timed out".format(operation))
    try:
        return holder[0].result()
    except Exception as exc:
        raise RuntimeError("{} failed: {}".format(operation, exc)) from exc


def _snapshot_revision(objects, frame_id: str = "map") -> str:
    """Fingerprint the semantic contents of one registry response.

    Registry observations also carry volatile fields such as ``last_seen`` and
    ``observation_count``.  Those fields describe freshness, not a new object
    identity or a changed shortlist.  Including them in the revision would
    invalidate a visual rejection/confirmation every time the detector
    refreshed the same object and would recreate the search-cancellation
    race this boundary is meant to prevent.  Identity, class/label, evidence
    path, and a centimetre-quantized pose are the revision-bearing fields;
    freshness remains available on the registry response itself.
    """

    rows = []
    for item in objects:
        rows.append({
            "name": str(getattr(item, "name", "")),
            "label": str(getattr(item, "label", "")),
            "class_id": int(getattr(item, "class_id", 0)),
            "frame_id": str(frame_id or "map"),
            # Millimetre-scale detector jitter must not manufacture a new
            # identity revision; a centimetre-scale move is meaningful for a
            # later approach and therefore remains revision-bearing.
            "x": round(float(getattr(getattr(item, "position", None), "x", 0.0)), 2),
            "y": round(float(getattr(getattr(item, "position", None), "y", 0.0)), 2),
            "image_path": str(getattr(item, "image_path", "")),
        })
    # Service response order is not semantic.  Sort by the complete semantic
    # row so detector/container iteration order cannot manufacture a new
    # registry revision and invalidate an otherwise current confirmation.
    encoded = json.dumps(
        sorted(rows, key=lambda row: json.dumps(row, sort_keys=True)),
        sort_keys=True,
    )
    return hashlib.sha256(encoded.encode("utf-8")).hexdigest()[:16]


class RosRegistryAuthority:
    """Adapt ``sam2_object_registry/GetStoredObjects`` to the v2 protocol."""

    def __init__(
        self,
        node,
        *,
        query_service: str = "/sam2/get_stored_objects",
        timeout_s: float = 3.0,
        visual_selector: Optional[
            Callable[[str, Sequence[RegistryCandidate], MissionBoard], Sequence[CandidateDecision]]
        ] = None,
    ) -> None:
        if timeout_s <= 0.0:
            raise ValueError("timeout_s must be positive")
        try:
            from sam2_object_registry.srv import GetStoredObjects
        except ImportError as exc:  # pragma: no cover - non-ROS host
            raise RuntimeError("sam2_object_registry ROS interfaces are unavailable") from exc
        self._node = node
        self._timeout_s = float(timeout_s)
        self._visual_selector = visual_selector
        self._service_type = GetStoredObjects
        self._client = node.create_client(GetStoredObjects, query_service)

    def query(self, object_request: str, board: MissionBoard) -> RegistrySnapshot:
        if not self._client.wait_for_service(timeout_sec=self._timeout_s):
            return RegistrySnapshot(revision="", checked=False)
        request_text = object_request.strip()
        objects, frame_id = self._name_and_label_shortlist(request_text)
        # A mixed-frame token shortlist is represented by an empty frame ID.
        # Do not publish a checked snapshot with no candidates: that would
        # make an impossible registry result look like a valid not-found
        # lookup and could incorrectly clear confirmation state.
        if frame_id == "":
            return RegistrySnapshot(revision="", checked=False)
        revision = _snapshot_revision(objects, frame_id)
        candidates = tuple(
            RegistryCandidate(
                candidate_id=str(item.name),
                label=str(getattr(item, "label", "")),
                registry_revision=revision,
                evidence_id=str(getattr(item, "image_path", "")),
                metadata={
                    "class_id": str(getattr(item, "class_id", 0)),
                    "x": str(getattr(getattr(item, "position", None), "x", 0.0)),
                    "y": str(getattr(getattr(item, "position", None), "y", 0.0)),
                    "frame_id": frame_id,
                    "image_path": str(getattr(item, "image_path", "")),
                },
            )
            for item in objects
            if str(getattr(item, "name", ""))
        )
        return RegistrySnapshot(revision=revision, candidates=candidates, checked=True)

    def _query_response(self, label: str = "", name: str = ""):
        request = self._service_type.Request()
        request.name = name
        request.label = label
        return _wait_future(
            self._client.call_async(request), self._timeout_s, "registry query"
        )

    def _name_and_label_shortlist(self, request_text: str):
        """Resolve names/labels before visual confirmation.

        The registry service intentionally supports exact filters only. A
        request such as ``"the purple chair"`` therefore tries an exact name,
        an exact label, and then meaningful label tokens (``chair``). It never
        falls back to returning every object: color/appearance remains the
        separate commander visual-confirmation step.
        """

        if not request_text:
            response = self._query_response()
            return self._response_objects_and_frame(response)

        for response in (
            self._query_response(name=request_text),
            self._query_response(label=request_text),
        ):
            objects, frame_id = self._response_objects_and_frame(response)
            if objects:
                return objects, frame_id

        stop_words = {
            "a", "an", "approach", "confirmed", "find", "for", "go",
            "identify", "look", "locate", "object", "please", "search",
            "the", "to",
        }
        tokens = [
            token for token in re.findall(r"[a-z0-9_]+", request_text.lower())
            if token not in stop_words
        ]
        by_name = {}
        frame_id = "map"
        for token in tokens:
            response = self._query_response(label=token)
            objects, response_frame = self._response_objects_and_frame(response)
            if objects and frame_id == "map":
                frame_id = response_frame
            if objects and response_frame != frame_id:
                # A single shortlist must have one coordinate frame.  Fail
                # closed instead of mixing positions from incompatible maps.
                return (), ""
            for item in objects:
                name = str(getattr(item, "name", ""))
                if name:
                    by_name[name] = item
        return tuple(by_name.values()), frame_id

    @staticmethod
    def _response_objects_and_frame(response):
        result = getattr(response, "result", None)
        objects = tuple(getattr(result, "objects", ()))
        frame_id = str(getattr(getattr(result, "header", None), "frame_id", "map") or "map")
        return objects, frame_id

    def inspect(
        self,
        object_request: str,
        snapshot: RegistrySnapshot,
        candidate_ids: Sequence[str],
        board: MissionBoard,
    ) -> Sequence[CandidateDecision]:
        # Stored-image selection is an injected authority.  Without one, the
        # adapter fails closed instead of silently promoting a name shortlist.
        if self._visual_selector is None:
            return tuple(
                CandidateDecision(candidate_id=candidate_id, confirmed=False,
                                  reason_code="visual_selector_unavailable")
                for candidate_id in candidate_ids
            )
        selected = self._visual_selector(
            object_request,
            tuple(
                candidate for candidate in snapshot.candidates
                if candidate.candidate_id in set(candidate_ids)
            ),
            board,
        )
        return tuple(selected)

    def cancel_active(self) -> None:
        cancel = getattr(self._visual_selector, "cancel", None)
        if callable(cancel):
            cancel()


class Nav2MotionAuthority:
    """Adapt Nav2 ``NavigateToPose`` and ``Spin`` actions to motion tools."""

    def __init__(
        self,
        node,
        *,
        navigate_action: str = "/navigate_to_pose",
        spin_action: str = "/spin",
        timeout_s: float = 5.0,
        motion_timeout_s: float = 120.0,
        observe_fn: Optional[Callable[[MissionBoard], MotionResult]] = None,
        reachability_fn: Optional[
            Callable[[Tuple[float, float], MissionBoard], ReachabilityReport]
        ] = None,
        pose_fn: Optional[Callable[[MissionBoard], Tuple[float, float, float]]] = None,
        reachability_revision_fn: Optional[Callable[[], int]] = None,
        lifecycle_state_service: Optional[str] = None,
        lifecycle_timeout_s: Optional[float] = None,
    ) -> None:
        if timeout_s <= 0.0 or motion_timeout_s <= 0.0:
            raise ValueError("action timeouts must be positive")
        try:
            from rclpy.action import ActionClient
            from rclpy.callback_groups import ReentrantCallbackGroup
            from nav2_msgs.action import NavigateToPose, Spin
        except ImportError as exc:  # pragma: no cover - non-ROS host
            raise RuntimeError("Nav2 ROS interfaces are unavailable") from exc
        self._node = node
        self._timeout_s = float(timeout_s)
        self._motion_timeout_s = float(motion_timeout_s)
        self._observe_fn = observe_fn
        self._reachability_fn = reachability_fn
        self._pose_fn = pose_fn
        self._reachability_revision_fn = reachability_revision_fn
        self._lifecycle_timeout_s = float(
            timeout_s if lifecycle_timeout_s is None else lifecycle_timeout_s
        )
        if self._lifecycle_timeout_s <= 0.0:
            raise ValueError("lifecycle_timeout_s must be positive")
        self._active_handle = None
        self._active_lock = threading.RLock()
        group = ReentrantCallbackGroup()
        self._navigate_action = NavigateToPose
        self._spin_action = Spin
        self._navigate = ActionClient(node, NavigateToPose, navigate_action, callback_group=group)
        self._spin = ActionClient(node, Spin, spin_action, callback_group=group)
        self._lifecycle_state_type = None
        self._lifecycle_client = None
        if lifecycle_state_service:
            try:
                from lifecycle_msgs.srv import GetState
            except ImportError as exc:  # pragma: no cover - non-ROS host
                raise RuntimeError("lifecycle ROS interfaces are unavailable") from exc
            self._lifecycle_state_type = GetState
            self._lifecycle_client = node.create_client(
                GetState, lifecycle_state_service, callback_group=group
            )

    def observe(self, board: MissionBoard) -> MotionResult:
        if self._observe_fn is None:
            return MotionResult(False, reason_code="observation_authority_unavailable")
        return self._observe_fn(board)

    def set_observe_authority(self, observe_fn: Callable[[MissionBoard], MotionResult]) -> None:
        """Attach the independent search authority after Nav2 construction."""

        if not callable(observe_fn):
            raise ValueError("observe_fn must be callable")
        self._observe_fn = observe_fn

    def rotate_to_heading(self, heading: float, board: MissionBoard) -> MotionResult:
        if not math.isfinite(float(heading)):
            return MotionResult(False, reason_code="invalid_heading")
        try:
            current = self._pose_fn(board) if self._pose_fn is not None else None
            if current is None:
                return MotionResult(False, reason_code="map_pose_unavailable")
            delta = _shortest_angle(float(heading) - float(current[2]))
            return self._run_spin(delta)
        except RuntimeError as exc:
            return MotionResult(False, reason_code="nav2_spin_failed", detail=str(exc))

    def go_to_point(
        self,
        point: Tuple[float, float],
        projection_policy: str,
        board: MissionBoard,
    ) -> MotionResult:
        if len(point) != 2 or not all(math.isfinite(float(value)) for value in point):
            return MotionResult(False, reason_code="invalid_point")
        if self._reachability_fn is None:
            report = ReachabilityReport()
        else:
            try:
                report = self._reachability_fn(point, board, projection_policy)
            except TypeError:
                # Keep the small callback seam source-compatible with early
                # test doubles that accepted only point and board.
                report = self._reachability_fn(point, board)
        if report.state.value == "unknown":
            return MotionResult(False, reason_code="reachability_unknown", reachability=report)
        if report.state.value == "unreachable":
            return MotionResult(False, reason_code="unreachable", reachability=report)
        if report.projected and projection_policy == "reject":
            return MotionResult(
                False,
                reason_code="projection_required",
                reachability=report,
            )
        if (
            self._reachability_revision_fn is not None
            and report.costmap_revision is not None
            and int(self._reachability_revision_fn()) != int(report.costmap_revision)
        ):
            return MotionResult(
                False,
                reason_code="costmap_changed",
                reachability=ReachabilityReport(
                    state="unknown",
                    reason_code="costmap_changed",
                    costmap_revision=int(self._reachability_revision_fn()),
                    freshness="changed_after_preflight",
                ),
            )
        target = report.selected_pose if report.projected and report.selected_pose else (
            float(point[0]), float(point[1]), 0.0
        )
        try:
            return self._run_navigation(target, report)
        except RuntimeError as exc:
            return MotionResult(False, reason_code="nav2_navigation_failed",
                                detail=str(exc), reachability=report)

    def cancel_active(self) -> None:
        with self._active_lock:
            handle = self._active_handle
        if handle is not None:
            try:
                handle.cancel_goal_async()
            except Exception:
                pass

    def _run_spin(self, delta: float) -> MotionResult:
        if not self._wait_for_active_navigator():
            return MotionResult(False, reason_code="nav2_not_active")
        if not self._spin.wait_for_server(timeout_sec=self._timeout_s):
            return MotionResult(False, reason_code="nav2_spin_unavailable")
        goal = self._spin_action.Goal()
        goal.target_yaw = float(delta)
        future = self._spin.send_goal_async(goal)
        handle = _wait_future(future, self._timeout_s, "Nav2 spin goal")
        if not handle.accepted:
            return MotionResult(False, reason_code="nav2_spin_rejected")
        with self._active_lock:
            self._active_handle = handle
        try:
            wrapped = _wait_future(
                handle.get_result_async(), self._motion_timeout_s, "Nav2 spin result"
            )
        finally:
            with self._active_lock:
                self._active_handle = None
        success = _nav2_succeeded(wrapped)
        canceled = _nav2_canceled(wrapped)
        return MotionResult(
            success,
            reason_code=(
                "motion_completed" if success
                else "motion_canceled" if canceled
                else "nav2_spin_aborted"
            ),
            progress_delta=abs(float(delta)) if success else 0.0,
        )

    def _run_navigation(self, target, report: ReachabilityReport) -> MotionResult:
        if not self._wait_for_active_navigator():
            return MotionResult(
                False, reason_code="nav2_not_active", reachability=report
            )
        if not self._navigate.wait_for_server(timeout_sec=self._timeout_s):
            return MotionResult(False, reason_code="nav2_navigation_unavailable",
                                reachability=report)
        goal = self._navigate_action.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self._node.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(target[0])
        goal.pose.pose.position.y = float(target[1])
        goal.pose.pose.orientation.z = math.sin(float(target[2]) / 2.0)
        goal.pose.pose.orientation.w = math.cos(float(target[2]) / 2.0)
        handle = _wait_future(
            self._navigate.send_goal_async(goal), self._timeout_s,
            "Nav2 navigation goal"
        )
        if not handle.accepted:
            return MotionResult(False, reason_code="nav2_navigation_rejected",
                                reachability=report)
        with self._active_lock:
            self._active_handle = handle
        try:
            wrapped = _wait_future(
                handle.get_result_async(), self._motion_timeout_s, "Nav2 navigation result"
            )
        finally:
            with self._active_lock:
                self._active_handle = None
        success = _nav2_succeeded(wrapped)
        canceled = _nav2_canceled(wrapped)
        return MotionResult(
            success,
            reason_code=(
                "motion_completed" if success
                else "motion_canceled" if canceled
                else "nav2_navigation_aborted"
            ),
            progress_delta=report.path_length_m or 0.0 if success else 0.0,
            reachability=report,
        )

    def _wait_for_active_navigator(self) -> bool:
        """Optionally gate dispatch on the Nav2 lifecycle state.

        Action servers can be discoverable while their lifecycle node is still
        configuring.  The v2 production composition enables this gate; test
        transports and deployments that provide their own readiness gate leave
        ``lifecycle_state_service`` unset.
        """

        client = self._lifecycle_client
        state_type = self._lifecycle_state_type
        if client is None or state_type is None:
            return True
        deadline = time.monotonic() + self._lifecycle_timeout_s
        while time.monotonic() < deadline:
            remaining = max(0.0, deadline - time.monotonic())
            if not client.wait_for_service(timeout_sec=min(0.1, remaining)):
                continue
            try:
                response = _wait_future(
                    client.call_async(state_type.Request()),
                    min(self._timeout_s, max(0.1, remaining)),
                    "Nav2 lifecycle state",
                )
                if int(response.current_state.id) == 3:  # lifecycle ACTIVE
                    return True
            except RuntimeError:
                # A lifecycle transition can invalidate a concurrent query;
                # keep waiting until the bounded readiness window expires.
                continue
        return False


class RosPoiGridAuthority:
    """Adapt the deterministic POI planner to the Nav2 motion authority.

    The planner selects one known-free reachable viewpoint and then waits for
    the Nav2 result.  There is no independent search process or wall
    clock watchdog: a command ends on a POI result, exhaustion, no reachable
    goal, Nav2 cancellation, or the Nav2 authority's own bounded action
    timeout.  Commander regains control only after that terminal result.
    """

    def __init__(
        self,
        node,
        *,
        reachability: "RosMapReachability",
        motion: Nav2MotionAuthority,
        spacing_m: float = 1.0,
        nominal_speed_mps: float = 0.25,
        minimum_progress_m: float = 0.25,
        result_topic: str = "/muto/poi_grid/result",
        selected_pose_topic: str = "/muto/poi_grid/selected_pose",
    ) -> None:
        try:
            from geometry_msgs.msg import PoseStamped
            from muto_command_layer_v2.msg import PoiGridResult
            from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        except ImportError as exc:  # pragma: no cover - non-ROS host
            raise RuntimeError("POI-grid ROS interfaces are unavailable") from exc
        self._node = node
        self._reachability = reachability
        self._motion = motion
        self._planner = PoiGridPlanner(
            spacing_m=spacing_m,
            nominal_speed_mps=nominal_speed_mps,
            minimum_progress_m=minimum_progress_m,
        )
        self._result_type = PoiGridResult
        self._pose_type = PoseStamped
        self._visited = set()
        self._attempted = set()
        self._mission_id = ""
        self._sequence = 0
        qos = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._result_pub = node.create_publisher(PoiGridResult, result_topic, qos)
        self._selected_pub = node.create_publisher(PoseStamped, selected_pose_topic, qos)

    def observe(self, board: MissionBoard) -> MotionResult:
        if board.mission_id != self._mission_id:
            self._mission_id = board.mission_id
            self._visited.clear()
            self._attempted.clear()
        grid, pose = self._reachability.snapshot(board)
        decision = self._planner.select(grid, pose, self._attempted)
        if decision.selection is None:
            detail = self._summary(decision.reason_code, decision.grid_revision)
            self._publish(board, decision, outcome=decision.reason_code, detail=detail)
            # Exhaustion is a successful search-authority result.  It is not a
            # confirmed object and the commander decides whether the scenario
            # policy permits mission completion.
            return MotionResult(
                decision.reason_code == "poi_exhausted",
                reason_code=decision.reason_code,
                detail=detail,
                progress_delta=0.0,
            )

        selection = decision.selection
        self._publish(
            board,
            decision,
            outcome="poi_goal_selected",
            detail="",
            selection=selection,
        )
        try:
            motion = self._motion.go_to_point(
                (selection.pose[0], selection.pose[1]), "reject", board
            )
        except Exception as exc:
            motion = MotionResult(False, reason_code="poi_goal_failed", detail=str(exc))
        # A failed point is marked attempted so the commander cannot ask the
        # deterministic planner to retry the same bad cell forever.  A
        # cancellation is left available for a later mission retry.
        if motion.reason_code == "motion_canceled":
            outcome = "poi_goal_canceled"
        elif motion.success:
            self._visited.add(selection.poi_id)
            self._attempted.add(selection.poi_id)
            outcome = "poi_goal_succeeded"
        else:
            self._attempted.add(selection.poi_id)
            outcome = "poi_goal_failed"
        detail = self._summary(
            outcome,
            selection.grid_revision,
            selection.poi_id,
            len(self._visited),
        )
        if motion.detail:
            detail = motion.detail + ";" + detail
        self._publish(
            board,
            decision,
            outcome=outcome,
            detail=detail,
            reason_code=(
                "poi_goal_succeeded"
                if motion.success
                else motion.reason_code or outcome
            ),
            selection=selection,
        )
        return MotionResult(
            bool(motion.success),
            reason_code=("poi_goal_succeeded" if motion.success else motion.reason_code or outcome),
            detail=detail,
            progress_delta=selection.path_length_m if motion.success else 0.0,
            reachability=motion.reachability,
        )

    def _summary(self, reason, revision, poi_id="", visited=None) -> str:
        return "poi_reason={};grid_revision={};poi_id={};visited={}".format(
            reason,
            int(revision),
            poi_id,
            len(self._visited) if visited is None else int(visited),
        )

    def _publish(
        self,
        board: MissionBoard,
        decision,
        *,
        outcome: str,
        detail: str,
        reason_code: Optional[str] = None,
        selection=None,
    ) -> None:
        self._sequence += 1
        message = self._result_type()
        message.header.stamp = self._node.get_clock().now().to_msg()
        message.schema_version = SCHEMA_VERSION
        message.mission_id = board.mission_id
        message.sequence = self._sequence
        message.grid_revision = int(decision.grid_revision)
        message.poi_id = selection.poi_id if selection is not None else ""
        message.outcome = outcome
        message.reason_code = reason_code or outcome
        message.detail = detail
        if selection is not None:
            pose = self._pose_type()
            pose.header.stamp = message.header.stamp
            pose.header.frame_id = "map"
            pose.pose.position.x = float(selection.pose[0])
            pose.pose.position.y = float(selection.pose[1])
            pose.pose.orientation.z = math.sin(float(selection.pose[2]) / 2.0)
            pose.pose.orientation.w = math.cos(float(selection.pose[2]) / 2.0)
            message.selected_pose = pose
            message.path_length_m = float(selection.path_length_m)
            message.estimated_time_s = float(selection.estimated_time_s)
            message.visited_count = len(self._visited)
            self._selected_pub.publish(pose)
        else:
            message.visited_count = len(self._visited)
        self._result_pub.publish(message)


def _shortest_angle(angle: float) -> float:
    """Wrap to [-pi, pi], choosing +pi for the exact 180-degree case."""

    wrapped = (angle + math.pi) % (2.0 * math.pi) - math.pi
    if math.isclose(wrapped, -math.pi, abs_tol=1e-12):
        return math.pi
    return wrapped


def _nav2_succeeded(result) -> bool:
    """Use the standard action status while tolerating test doubles."""

    status = getattr(result, "status", None)
    if status is not None:
        return int(status) == 4  # action_msgs/GoalStatus.STATUS_SUCCEEDED
    payload = getattr(result, "result", None)
    error_code = getattr(payload, "error_code", None)
    return payload is not None and error_code in (None, 0)


def _nav2_canceled(result) -> bool:
    status = getattr(result, "status", None)
    return status is not None and int(status) == 5  # action_msgs/GoalStatus.STATUS_CANCELED


class RosVisualInput:
    """Latest camera observation encoded as JPEG for CommanderAgent.

    The deployed Muto camera publishes raw ``sensor_msgs/Image``.  A
    compressed topic remains supported for simulator/replay fixtures.
    """

    def __init__(self, node, *, topic: str = "/camera/color/image_raw") -> None:
        try:
            from sensor_msgs.msg import CompressedImage, Image
        except ImportError as exc:  # pragma: no cover - non-ROS host
            raise RuntimeError("sensor_msgs CompressedImage is unavailable") from exc
        self._lock = threading.RLock()
        self._jpeg = b""
        self._sequence = 0
        self._node = node
        self._bridge = None
        if topic.endswith("/compressed"):
            self._subscription = node.create_subscription(
                CompressedImage, topic, self._compressed_callback, 10
            )
        else:
            self._subscription = node.create_subscription(
                Image, topic, self._raw_callback, 10
            )

    def _store(self, data: bytes) -> None:
        if not data:
            return
        with self._lock:
            self._jpeg = data
            self._sequence += 1

    def _compressed_callback(self, message) -> None:
        self._store(bytes(message.data))

    def _raw_callback(self, message) -> None:
        try:
            if self._bridge is None:
                from cv_bridge import CvBridge
                self._bridge = CvBridge()
            import cv2
            image = self._bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            ok, encoded = cv2.imencode(".jpg", image)
            data = encoded.tobytes() if ok else b""
        except Exception as exc:
            data = b""
            try:
                self._node.get_logger().debug("raw visual input unavailable: %s", exc)
            except Exception:
                pass
        self._store(data)

    def _callback(self, message) -> None:
        # Kept as a compatibility alias for older injected test doubles.
        data = bytes(message.data)
        self._store(data)

    def jpeg(self) -> bytes:
        with self._lock:
            return bytes(self._jpeg)

    @property
    def sequence(self) -> int:
        with self._lock:
            return self._sequence


class RosMapReachability:
    """Build conservative preflight reports from the live map and TF.

    This is only a snapshot helper.  It never sends a navigation goal and it
    never overrides Nav2's final planning, recovery, or obstacle decisions.
    """

    def __init__(
        self,
        node,
        *,
        map_topic: str = "/map",
        map_frame: str = "map",
        base_frame: str = "base_frame",
        stale_after_s: float = 2.0,
        tf_stale_after_s: Optional[float] = None,
        # Keep the preflight footprint aligned with the Nav2 costmap and the
        # simulation plant.  A smaller radius can incorrectly bless a gap
        # that Nav2 (or the physical robot) cannot traverse.
        footprint_radius_m: float = 0.26,
    ) -> None:
        if stale_after_s <= 0.0:
            raise ValueError("stale_after_s must be positive")
        try:
            from nav_msgs.msg import OccupancyGrid
            from tf2_ros import Buffer, TransformListener
        except ImportError as exc:  # pragma: no cover - non-ROS host
            raise RuntimeError("map/TF ROS interfaces are unavailable") from exc
        from .reachability import ReachabilityConfig, ReachabilityPlanner

        self._node = node
        self._map_frame = map_frame
        self._base_frame = base_frame
        self._stale_after_s = float(stale_after_s)
        self._tf_stale_after_s = float(tf_stale_after_s or stale_after_s)
        self._planner = ReachabilityPlanner(
            ReachabilityConfig(footprint_radius_m=float(footprint_radius_m))
        )
        self._grid = None
        self._revision = 0
        self._map_fingerprint = None
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, node)
        self._subscription = node.create_subscription(
            OccupancyGrid, map_topic, self._map_callback, 1
        )

    def _map_callback(self, message) -> None:
        from .reachability import OccupancyGrid

        data = tuple(int(value) for value in message.data)
        fingerprint = (
            int(message.info.width),
            int(message.info.height),
            float(message.info.resolution),
            float(message.info.origin.position.x),
            float(message.info.origin.position.y),
            data,
        )
        # A live map/costmap may publish its unchanged contents at sensor
        # rate.  Treating every timestamp as a new revision would make the
        # immediate preflight-to-Nav2 recheck reject almost every goal.  A
        # revision represents a semantic grid change; the message timestamp
        # still refreshes staleness below.
        if fingerprint != self._map_fingerprint:
            self._revision += 1
            self._map_fingerprint = fingerprint
        self._grid = (message, OccupancyGrid(
            width=int(message.info.width),
            height=int(message.info.height),
            resolution=float(message.info.resolution),
            origin_x=float(message.info.origin.position.x),
            origin_y=float(message.info.origin.position.y),
            data=data,
            revision=self._revision,
            freshness="fresh",
        ))

    def current_pose(self, _board: MissionBoard):
        try:
            from rclpy.time import Time
            transform = self._tf_buffer.lookup_transform(
                self._map_frame, self._base_frame, Time()
            )
        except Exception:
            return None
        try:
            now = self._node.get_clock().now().nanoseconds / 1e9
            stamp = transform.header.stamp
            transform_time = float(stamp.sec) + float(stamp.nanosec) / 1e9
            if now >= transform_time and now - transform_time > self._tf_stale_after_s:
                return None
        except (AttributeError, TypeError, ValueError):
            return None
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
        )
        return float(translation.x), float(translation.y), float(yaw)

    def revision(self) -> int:
        return int(self._revision)

    def snapshot(self, board: Optional[MissionBoard] = None):
        """Return the current map snapshot and map-frame robot pose.

        The POI planner consumes the same semantic grid and freshness rules as
        motion preflight.  Returning a copied stale grid lets it emit a typed
        ``poi_grid_stale`` result instead of silently treating missing data as
        search exhaustion.
        """

        if self._grid is None:
            return None, None
        message, grid = self._grid
        try:
            now = self._node.get_clock().now().nanoseconds / 1e9
            stamp = getattr(message.header, "stamp", None)
            map_time = float(stamp.sec) + float(stamp.nanosec) / 1e9
            freshness = (
                "stale"
                if now >= map_time and now - map_time > self._stale_after_s
                else "fresh"
            )
        except (AttributeError, TypeError, ValueError):
            freshness = grid.freshness
        if freshness != grid.freshness:
            grid = type(grid)(
                width=grid.width,
                height=grid.height,
                resolution=grid.resolution,
                origin_x=grid.origin_x,
                origin_y=grid.origin_y,
                data=grid.data,
                revision=grid.revision,
                freshness=freshness,
            )
        return grid, self.current_pose(board)

    def evaluate_point(
        self,
        point: Tuple[float, float],
        board: MissionBoard,
        projection_policy: str = "reject",
    ) -> ReachabilityReport:
        grid, pose = self.snapshot(board)
        if grid is None:
            return ReachabilityReport(reason_code="costmap_unavailable")
        if pose is None:
            return ReachabilityReport(reason_code="map_pose_unavailable")
        report = self._planner.evaluate(
            grid,
            (pose[0], pose[1]),
            (float(point[0]), float(point[1])),
            projection_policy=projection_policy,
            heading=pose[2],
        )
        # The map can be updated while the planner is traversing the grid.
        # Never let that snapshot authorize a goal on a newer costmap.
        if self._revision != grid.revision:
            return ReachabilityReport(
                state="unknown",
                reason_code="costmap_changed",
                costmap_revision=self._revision,
                freshness="changed_during_preflight",
            )
        return report
