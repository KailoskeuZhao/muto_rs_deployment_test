#!/usr/bin/env python3
"""Lifecycle-managed rosbag2 recorder for v2 mission-level data only."""

from __future__ import annotations

import os
import secrets
from datetime import datetime, timezone

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import serialize_message

from geometry_msgs.msg import PoseStamped

from muto_command_layer_v2.msg import (
    MissionBoard,
    MissionEvent,
    MissionRecorderManifest,
    MissionRejection,
    RecorderStatus,
)
from std_msgs.msg import String


HIGH_LEVEL_TOPICS = (
    ("muto/mission_board", "muto_command_layer_v2/msg/MissionBoard", MissionBoard),
    ("muto/mission_event", "muto_command_layer_v2/msg/MissionEvent", MissionEvent),
    (
        "muto/mission_recorder_status",
        "muto_command_layer_v2/msg/RecorderStatus",
        RecorderStatus,
    ),
    (
        "muto/mission_recorder_manifest",
        "muto_command_layer_v2/msg/MissionRecorderManifest",
        MissionRecorderManifest,
    ),
    (
        "muto/mission_rejection",
        "muto_command_layer_v2/msg/MissionRejection",
        MissionRejection,
    ),
    (
        "/explore/selected_frontier",
        "geometry_msgs/msg/PoseStamped",
        PoseStamped,
    ),
    (
        "/frontier_goal_adapter/original_goal",
        "geometry_msgs/msg/PoseStamped",
        PoseStamped,
    ),
    (
        "/frontier_goal_adapter/projected_goal",
        "geometry_msgs/msg/PoseStamped",
        PoseStamped,
    ),
    (
        "/frontier_goal_adapter/status",
        "std_msgs/msg/String",
        String,
    ),
)

_MISSION_TOPIC_NAMES = {
    "muto/mission_board",
    "muto/mission_event",
}


class HighLevelRecorderNode(Node):
    """Record only canonical mission projections, manifest, and status."""

    def __init__(self) -> None:
        super().__init__("muto_command_layer_v2_recorder")
        # rclpy's TimeSource owns this ROS-wide parameter and applies the
        # launch override during Node construction.  Redeclaring it here
        # raises ParameterAlreadyDeclaredException on Humble.
        self.declare_parameter("output_uri", "")
        self.declare_parameter("run_id", "")
        self.declare_parameter("storage_id", "mcap")
        self.declare_parameter("serialization_format", "cdr")
        self.declare_parameter("qos_depth", 100)
        self.declare_parameter("build_revision", "unknown")
        self.declare_parameter("commander_model", "unknown")
        configured_output_uri = str(self.get_parameter("output_uri").value).strip()
        configured_run_id = str(self.get_parameter("run_id").value).strip()
        self._run_id = configured_run_id or self._generate_run_id()
        if configured_output_uri:
            self._output_uri_base = configured_output_uri
        else:
            # Keep hardware captures beside the other Muto bags and make a
            # process restart produce a new directory instead of colliding
            # with mission-0001 from the previous process.
            self._output_uri_base = (
                "/opt/muto_rs_ws/bags/muto_command_v2_"
                "{run_id}_{mission_id}"
            )
        self.output_uri = ""
        self._writer = None
        self._active_mission_id = ""
        try:
            import rosbag2_py
        except ImportError as exc:  # pragma: no cover - non-ROS host
            raise RuntimeError("rosbag2_py is unavailable") from exc
        self._rosbag2_py = rosbag2_py
        self._converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format=str(
                self.get_parameter("serialization_format").value
            ),
            output_serialization_format=str(
                self.get_parameter("serialization_format").value
            ),
        )
        qos = QoSProfile(
            depth=max(1, int(self.get_parameter("qos_depth").value)),
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        event_qos = QoSProfile(
            depth=max(1, int(self.get_parameter("qos_depth").value)),
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._subscriptions = [
            self.create_subscription(
                MissionBoard, "muto/mission_board", self._board_callback, qos
            ),
            self.create_subscription(
                MissionEvent, "muto/mission_event", self._event_callback, event_qos
            ),
        ]
        diagnostic_qos = QoSProfile(
            depth=max(1, int(self.get_parameter("qos_depth").value)),
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        for topic, _type_name, message_type in HIGH_LEVEL_TOPICS:
            if topic in _MISSION_TOPIC_NAMES or topic.startswith("muto/mission_recorder_"):
                continue
            self._subscriptions.append(
                self.create_subscription(
                    message_type,
                    topic,
                    lambda message, topic=topic: self._external_callback(
                        topic, message
                    ),
                    diagnostic_qos,
                )
            )
        self._status_pub = self.create_publisher(
            RecorderStatus, "muto/mission_recorder_status", qos
        )
        self.get_logger().info(
            "v2 high-level recorder waiting for mission lifecycle; board/event/status/manifest only"
        )

    def _board_callback(self, message):
        mission_id = str(message.mission_id)
        if not mission_id:
            return
        if message.lifecycle_state in (MissionBoard.ACCEPTED, MissionBoard.RUNNING):
            if self._writer is None or mission_id != self._active_mission_id:
                self._start_writer(mission_id, message.request_id, message)
            self._write("muto/mission_board", message)
            return
        if message.lifecycle_state in (
            MissionBoard.SUCCEEDED, MissionBoard.CANCELED, MissionBoard.FAILED
        ):
            if self._writer is None and mission_id:
                self._start_writer(mission_id, message.request_id, message)
            self._write("muto/mission_board", message)
            self._close_writer(mission_id, terminal=True)

    def _event_callback(self, message):
        if self._writer is not None and str(message.mission_id) == self._active_mission_id:
            self._write("muto/mission_event", message)

    def _external_callback(self, topic, message):
        """Record bounded, typed navigation/frontier diagnostics in-flight.

        These topics carry no mission id.  The executive permits only one
        mission at a time, so the active writer is the ownership boundary;
        diagnostics received while no mission is active are intentionally
        discarded instead of being assigned to an unrelated bag.
        """

        if self._writer is not None and self._active_mission_id:
            self._write(topic, message)

    def _start_writer(self, mission_id, request_id, board=None):
        if self._writer is not None:
            self._close_writer(self._active_mission_id, terminal=False)
        uri = self._output_uri_base
        if "{mission_id}" in uri or "{run_id}" in uri:
            uri = uri.format(mission_id=mission_id, run_id=self._run_id)
        else:
            uri = os.path.join(uri, mission_id)
        try:
            os.makedirs(os.path.dirname(uri) or ".", exist_ok=True)
            storage_options = self._rosbag2_py.StorageOptions(
                uri=uri,
                storage_id=str(self.get_parameter("storage_id").value),
                max_bagfile_size=0,
                max_bagfile_duration=0,
            )
            self._writer = self._rosbag2_py.SequentialWriter()
            self._writer.open(storage_options, self._converter_options)
            for topic, type_name, _message_type in HIGH_LEVEL_TOPICS:
                self._writer.create_topic(
                    self._rosbag2_py.TopicMetadata(
                        name=topic,
                        type=type_name,
                        serialization_format=self._converter_options.output_serialization_format,
                    )
                )
            self._active_mission_id = mission_id
            self.output_uri = uri
            manifest = MissionRecorderManifest()
            manifest.header.stamp = self.get_clock().now().to_msg()
            manifest.schema_version = "muto_command_layer_v2"
            manifest.mission_id = mission_id
            manifest.request_id = request_id
            manifest.scenario_id = str(getattr(board, "scenario_id", ""))
            manifest.completion_policy = str(getattr(board, "completion_policy", ""))
            manifest.build_revision = str(self.get_parameter("build_revision").value)
            manifest.commander_model = str(self.get_parameter("commander_model").value)
            manifest.profile = "high_level"
            self._writer.write(
                "muto/mission_recorder_manifest",
                serialize_message(manifest),
                self.get_clock().now().nanoseconds,
            )
            self._publish_status(mission_id, True, uri, "recorder_started", False)
        except Exception as exc:
            self._writer = None
            self._active_mission_id = mission_id
            self.output_uri = ""
            self._publish_status(mission_id, False, "", "recorder_open_failed", False)
            self.get_logger().error("v2 high-level recorder unavailable: %s", exc)

    @staticmethod
    def _generate_run_id() -> str:
        timestamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
        return "{}_{}".format(timestamp, secrets.token_hex(3))

    def _write(self, topic, message):
        if self._writer is None:
            return
        stamp = self.get_clock().now().nanoseconds
        self._writer.write(topic, serialize_message(message), stamp)

    def _close_writer(self, mission_id, *, terminal):
        uri = self.output_uri
        if self._writer is not None:
            self._publish_status(mission_id, bool(uri), uri, "recorder_closed", terminal)
        if self._writer is not None:
            try:
                self._writer.close()
            except Exception:
                pass
        self._writer = None
        self._active_mission_id = ""

    def _publish_status(self, mission_id, available, uri, reason_code, terminal):
        status = RecorderStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.schema_version = "muto_command_layer_v2"
        status.mission_id = mission_id
        status.available = bool(available)
        status.terminal = bool(terminal)
        status.uri = uri
        status.reason_code = reason_code
        self._status_pub.publish(status)
        if self._writer is not None and mission_id == self._active_mission_id:
            self._write("muto/mission_recorder_status", status)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HighLevelRecorderNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node._close_writer(node._active_mission_id, terminal=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
