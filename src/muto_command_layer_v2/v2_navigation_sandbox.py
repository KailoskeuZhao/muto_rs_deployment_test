#!/usr/bin/env python3
"""Probe live reachability and the v2 Nav2 motion authority together.

This diagnostic is deliberately below the commander and registry.  A target
is treated as a stored object's map position, then evaluated against the live
map/TF snapshot and, unless ``--preflight-only`` is selected, passed through
the real ``Nav2MotionAuthority``.  The output makes the distinction between
"the map rejected this object" and "Nav2 accepted but could not execute"
explicit.
"""

from __future__ import annotations

import argparse
import json
import math
import threading
import time
from typing import Any, Optional

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from lifecycle_msgs.srv import GetState

from muto_command_layer_v2.contracts import MissionBoard
from muto_command_layer_v2.ros_authorities import Nav2MotionAuthority, RosMapReachability


def _report_dict(report: Any) -> dict[str, Any]:
    selected = getattr(report, "selected_pose", None)
    return {
        "state": getattr(getattr(report, "state", None), "value", str(report.state)),
        "reason_code": str(getattr(report, "reason_code", "")),
        "path_length_m": getattr(report, "path_length_m", None),
        "estimated_time_s": getattr(report, "estimated_time_s", None),
        "costmap_revision": getattr(report, "costmap_revision", None),
        "freshness": str(getattr(report, "freshness", "")),
        "selected_pose": list(selected) if selected is not None else None,
        "projected": bool(getattr(report, "projected", False)),
    }


def _motion_dict(result: Any) -> dict[str, Any]:
    return {
        "success": bool(result.success),
        "reason_code": str(result.reason_code),
        "detail": str(result.detail),
        "progress_delta": float(result.progress_delta),
        "reachability": _report_dict(result.reachability),
    }


class NavigationSandboxNode(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__(
            "muto_command_layer_v2_navigation_sandbox",
            parameter_overrides=[
                Parameter("use_sim_time", Parameter.Type.BOOL, bool(args.use_sim_time))
            ],
        )
        self._args = args
        self.board = MissionBoard(
            objective="approach stored object {}".format(args.object_id),
            object_request=args.object_id,
        )
        self.reachability = RosMapReachability(
            self,
            map_topic=args.map_topic,
            map_frame=args.map_frame,
            base_frame=args.base_frame,
            stale_after_s=args.stale_after_s,
            footprint_radius_m=args.footprint_radius_m,
        )
        self._lifecycle = self.create_client(GetState, args.lifecycle_state_service)
        self.motion = Nav2MotionAuthority(
            self,
            navigate_action=args.navigate_action,
            spin_action=args.spin_action,
            timeout_s=args.action_server_timeout_s,
            motion_timeout_s=args.motion_timeout_s,
            reachability_fn=self.reachability.evaluate_point,
            pose_fn=self.reachability.current_pose,
            reachability_revision_fn=self.reachability.revision,
            lifecycle_state_service=args.lifecycle_state_service,
            lifecycle_timeout_s=args.server_timeout_s,
        )

    @staticmethod
    def _remaining(deadline: float) -> float:
        return max(0.0, deadline - time.monotonic())

    def wait_for_snapshot(self, timeout_s: float) -> bool:
        """Wait for both a map message and a usable map-to-base pose."""

        deadline = time.monotonic() + timeout_s
        while self._remaining(deadline) > 0.0:
            if self.reachability._grid is not None and self.reachability.current_pose(self.board):
                return True
            # The executor is already spinning in the companion thread.  Do
            # not call spin_once here: concurrent spins on one executor race
            # its callback generator.
            time.sleep(min(0.05, self._remaining(deadline)))
        return False

    def wait_for_active_navigator(self, timeout_s: float) -> tuple[bool, str]:
        """Keep action dispatch behind Nav2 lifecycle activation."""

        if self._args.skip_lifecycle_gate:
            return True, "skipped"
        deadline = time.monotonic() + timeout_s
        while self._remaining(deadline) > 0.0:
            if not self._lifecycle.wait_for_service(timeout_sec=0.0):
                time.sleep(min(0.05, self._remaining(deadline)))
                continue
            future = self._lifecycle.call_async(GetState.Request())
            while not future.done() and self._remaining(deadline) > 0.0:
                time.sleep(min(0.05, self._remaining(deadline)))
            if not future.done():
                break
            try:
                state = future.result().current_state
                state_id = int(state.id)
                state_label = str(state.label)
            except Exception as exc:  # pragma: no cover - middleware failure
                state_id = -1
                state_label = "query_error:{}".format(exc)
            if state_id == 3:  # lifecycle_msgs/msg/State.ACTIVE
                return True, state_label or "active"
            time.sleep(min(0.05, self._remaining(deadline)))
        return False, "lifecycle_timeout"


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--target-x", type=float, required=True)
    parser.add_argument("--target-y", type=float, required=True)
    parser.add_argument("--object-id", default="sandbox_object")
    parser.add_argument("--projection-policy", choices=("reject", "allow"), default="reject")
    parser.add_argument("--preflight-only", action="store_true")
    parser.add_argument("--map-topic", default="/map")
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--base-frame", default="base_frame")
    parser.add_argument("--navigate-action", default="/navigate_to_pose")
    parser.add_argument("--spin-action", default="/spin")
    parser.add_argument("--lifecycle-state-service", default="/bt_navigator/get_state")
    parser.add_argument("--stale-after-s", type=float, default=2.0)
    parser.add_argument(
        "--footprint-radius-m",
        type=float,
        default=0.26,
        help="robot footprint radius; keep this equal to Nav2/plant robot_radius_m",
    )
    parser.add_argument("--action-server-timeout-s", type=float, default=5.0)
    parser.add_argument("--motion-timeout-s", type=float, default=120.0)
    parser.add_argument("--server-timeout-s", type=float, default=45.0)
    parser.add_argument(
        "--use-sim-time",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="follow /clock (default: true; use --no-use-sim-time for hardware)",
    )
    parser.add_argument(
        "--skip-lifecycle-gate",
        action="store_true",
        help="send as soon as the Nav2 action server is visible",
    )
    return parser


def main(argv: Optional[list[str]] = None) -> int:
    args = _parser().parse_args(argv)
    if not all(math.isfinite(value) for value in (args.target_x, args.target_y)):
        raise SystemExit("target coordinates must be finite")
    rclpy.init(args=None)
    node = NavigationSandboxNode(args)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    result: dict[str, Any]
    try:
        if not node.wait_for_snapshot(args.server_timeout_s):
            result = {
                "success": False,
                "reason": "map_or_pose_unavailable",
                "target": [args.target_x, args.target_y],
            }
        else:
            active, lifecycle_state = node.wait_for_active_navigator(args.server_timeout_s)
            if not active:
                result = {
                    "success": False,
                    "reason": lifecycle_state,
                    "target": [args.target_x, args.target_y],
                }
                print("V2_NAVIGATION_SANDBOX_RESULT=" + json.dumps(result, sort_keys=True))
                return 1
            report = node.reachability.evaluate_point(
                (args.target_x, args.target_y),
                node.board,
                args.projection_policy,
            )
            result = {
                "target": [args.target_x, args.target_y],
                "object_id": args.object_id,
                "projection_policy": args.projection_policy,
                "lifecycle_state": lifecycle_state,
                "preflight": _report_dict(report),
            }
            if args.preflight_only:
                result["success"] = report.state.value == "reachable"
                result["motion"] = None
            else:
                motion = node.motion.go_to_point(
                    (args.target_x, args.target_y),
                    args.projection_policy,
                    node.board,
                )
                result["success"] = bool(motion.success)
                result["motion"] = _motion_dict(motion)
    finally:
        node.motion.cancel_active()
        executor.shutdown(timeout_sec=2.0)
        executor_thread.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print("V2_NAVIGATION_SANDBOX_RESULT=" + json.dumps(result, sort_keys=True))
    return 0 if result.get("success") else 1


if __name__ == "__main__":
    raise SystemExit(main())
