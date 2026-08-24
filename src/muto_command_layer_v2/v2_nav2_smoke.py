#!/usr/bin/env python3
"""Small, sim-time-aware Nav2 live-smoke client for the v2 fixture.

``ros2 action send_goal`` is convenient for a wall-clock robot, but its CLI
node does not consistently follow the simulated clock used by the v2 Nav2
fixture.  That can make a valid goal appear to time out before the goal
response is received.  This client owns a ROS node with ``use_sim_time`` set
explicitly, waits for the BT navigator to become active, and reports the
actual Nav2 action status.

This is intentionally a diagnostic driver, not another motion authority.  It
only sends a goal to Nav2 and never publishes ``cmd_vel`` or implements path
following.
"""

import argparse
import json
import math
import time
from typing import Any, Optional

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose, Spin
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter


def _quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def _status_name(status: int) -> str:
    names = {
        GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
        GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
        GoalStatus.STATUS_EXECUTING: "EXECUTING",
        GoalStatus.STATUS_CANCELING: "CANCELING",
        GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
        GoalStatus.STATUS_CANCELED: "CANCELED",
        GoalStatus.STATUS_ABORTED: "ABORTED",
    }
    return names.get(int(status), f"STATUS_{int(status)}")


class Nav2SmokeClient(Node):
    """Action client that advances with wall time while ROS uses sim time."""

    def __init__(self, args: argparse.Namespace):
        super().__init__(
            "muto_command_layer_v2_nav2_smoke",
            parameter_overrides=[
                Parameter(
                    "use_sim_time",
                    Parameter.Type.BOOL,
                    bool(args.use_sim_time),
                )
            ],
        )
        self._args = args
        self._navigate = ActionClient(self, NavigateToPose, args.navigate_action)
        self._spin = ActionClient(self, Spin, args.spin_action)
        self._lifecycle = self.create_client(GetState, args.lifecycle_state_service)

    @staticmethod
    def _remaining(deadline: float) -> float:
        return max(0.0, deadline - time.monotonic())

    def _spin_until(
        self, executor: SingleThreadedExecutor, future: Any, deadline: float
    ) -> bool:
        while not future.done() and self._remaining(deadline) > 0.0:
            executor.spin_once(timeout_sec=min(0.1, self._remaining(deadline)))
        return future.done()

    def wait_for_clock(self, executor: SingleThreadedExecutor, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while self._remaining(deadline) > 0.0:
            if not self._args.use_sim_time or self.get_clock().now().nanoseconds > 0:
                return True
            executor.spin_once(timeout_sec=min(0.1, self._remaining(deadline)))
        return not self._args.use_sim_time

    def wait_for_active_navigator(
        self, executor: SingleThreadedExecutor, timeout_s: float
    ) -> tuple[bool, str]:
        if self._args.skip_lifecycle_gate:
            return True, "skipped"
        deadline = time.monotonic() + timeout_s
        while self._remaining(deadline) > 0.0:
            if not self._lifecycle.wait_for_service(timeout_sec=0.0):
                executor.spin_once(timeout_sec=min(0.1, self._remaining(deadline)))
                continue
            future = self._lifecycle.call_async(GetState.Request())
            if not self._spin_until(executor, future, deadline):
                break
            try:
                state = future.result().current_state
                state_id = int(state.id)
                state_label = str(state.label)
            except Exception as exc:  # pragma: no cover - middleware failure
                state_id = -1
                state_label = f"query_error:{exc}"
            if state_id == 3:  # lifecycle_msgs/msg/State.ACTIVE
                return True, state_label or "active"
            executor.spin_once(timeout_sec=min(0.1, self._remaining(deadline)))
        return False, "lifecycle_timeout"

    def _feedback(self, message: Any) -> None:
        # Keep smoke output bounded.  The final action status is authoritative;
        # feedback is only useful when debugging with --ros-args --log-level.
        if hasattr(message, "distance_remaining"):
            self.get_logger().debug(
                "distance_remaining=%.3f", float(message.distance_remaining)
            )
        elif hasattr(message, "angular_distance_traveled"):
            self.get_logger().debug(
                "angular_distance_traveled=%.3f",
                float(message.angular_distance_traveled),
            )

    def _wait_for_server(
        self, executor: SingleThreadedExecutor, client: ActionClient, deadline: float
    ) -> bool:
        while self._remaining(deadline) > 0.0:
            if client.server_is_ready():
                return True
            executor.spin_once(timeout_sec=min(0.1, self._remaining(deadline)))
        return client.server_is_ready()

    def _send_navigate(
        self, executor: SingleThreadedExecutor, deadline: float
    ) -> dict[str, Any]:
        if not self._wait_for_server(executor, self._navigate, deadline):
            return {"success": False, "reason": "navigate_action_unavailable"}

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self._args.frame_id
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(self._args.goal_x)
        goal.pose.pose.position.y = float(self._args.goal_y)
        _, _, z, w = _quaternion_from_yaw(float(self._args.goal_yaw))
        goal.pose.pose.orientation.z = z
        goal.pose.pose.orientation.w = w
        sent = self._navigate.send_goal_async(goal, feedback_callback=self._feedback)
        if not self._spin_until(executor, sent, deadline):
            return {"success": False, "reason": "navigate_goal_response_timeout"}
        handle = sent.result()
        if handle is None or not handle.accepted:
            return {"success": False, "reason": "navigate_goal_rejected"}
        result_future = handle.get_result_async()
        if not self._spin_until(executor, result_future, deadline):
            return {"success": False, "reason": "navigate_result_timeout"}
        result = result_future.result()
        status = int(result.status)
        return {
            "success": status == GoalStatus.STATUS_SUCCEEDED,
            "status": status,
            "status_name": _status_name(status),
            "reason": "nav2_succeeded"
            if status == GoalStatus.STATUS_SUCCEEDED
            else "nav2_terminal_failure",
            "error_code": int(getattr(result.result, "error_code", 0)),
            "error_msg": str(getattr(result.result, "error_msg", "")),
        }

    def _send_spin(self, executor: SingleThreadedExecutor, deadline: float) -> dict[str, Any]:
        if not self._wait_for_server(executor, self._spin, deadline):
            return {"success": False, "reason": "spin_action_unavailable"}

        goal = Spin.Goal()
        goal.target_yaw = float(self._args.spin_yaw)
        goal.time_allowance = Duration(
            sec=int(max(1.0, self._args.timeout_s)),
            nanosec=0,
        )
        sent = self._spin.send_goal_async(goal, feedback_callback=self._feedback)
        if not self._spin_until(executor, sent, deadline):
            return {"success": False, "reason": "spin_goal_response_timeout"}
        handle = sent.result()
        if handle is None or not handle.accepted:
            return {"success": False, "reason": "spin_goal_rejected"}
        result_future = handle.get_result_async()
        if not self._spin_until(executor, result_future, deadline):
            return {"success": False, "reason": "spin_result_timeout"}
        result = result_future.result()
        status = int(result.status)
        return {
            "success": status == GoalStatus.STATUS_SUCCEEDED,
            "status": status,
            "status_name": _status_name(status),
            "reason": "nav2_succeeded"
            if status == GoalStatus.STATUS_SUCCEEDED
            else "nav2_terminal_failure",
            "error_code": int(getattr(result.result, "error_code", 0)),
            "error_msg": str(getattr(result.result, "error_msg", "")),
        }

    def run(self, executor: SingleThreadedExecutor) -> dict[str, Any]:
        started = time.monotonic()
        if not self.wait_for_clock(executor, self._args.server_timeout_s):
            return {"success": False, "reason": "sim_clock_timeout"}
        active, lifecycle_state = self.wait_for_active_navigator(
            executor, self._args.server_timeout_s
        )
        if not active:
            return {"success": False, "reason": lifecycle_state}
        deadline = time.monotonic() + self._args.timeout_s
        if self._args.spin_yaw is not None:
            result = self._send_spin(executor, deadline)
            command = "spin"
        else:
            result = self._send_navigate(executor, deadline)
            command = "navigate_to_pose"
        result.update(
            {
                "command": command,
                "lifecycle_state": lifecycle_state,
                "elapsed_wall_s": round(time.monotonic() - started, 3),
            }
        )
        return result


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--goal-x", type=float)
    mode.add_argument("--spin-yaw", type=float)
    parser.add_argument("--goal-y", type=float, default=0.0)
    parser.add_argument("--goal-yaw", type=float, default=0.0)
    parser.add_argument("--frame-id", default="map")
    parser.add_argument("--navigate-action", default="/navigate_to_pose")
    parser.add_argument("--spin-action", default="/spin")
    parser.add_argument("--lifecycle-state-service", default="/bt_navigator/get_state")
    parser.add_argument("--server-timeout-s", type=float, default=45.0)
    parser.add_argument("--timeout-s", type=float, default=90.0)
    parser.add_argument(
        "--use-sim-time",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="follow /clock (default: true; use --no-use-sim-time for hardware)",
    )
    parser.add_argument(
        "--skip-lifecycle-gate",
        action="store_true",
        help="send as soon as the action server is visible",
    )
    return parser


def main(argv: Optional[list[str]] = None) -> int:
    args = _parser().parse_args(argv)
    if args.goal_x is not None and (
        not math.isfinite(args.goal_x) or not math.isfinite(args.goal_y)
    ):
        raise SystemExit("goal coordinates must be finite")
    if args.spin_yaw is not None and not math.isfinite(args.spin_yaw):
        raise SystemExit("--spin-yaw must be finite")
    rclpy.init(args=None)
    node = Nav2SmokeClient(args)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        result = node.run(executor)
    finally:
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print("V2_NAV2_SMOKE_RESULT=" + json.dumps(result, sort_keys=True))
    return 0 if result.get("success") else 1


if __name__ == "__main__":
    raise SystemExit(main())
