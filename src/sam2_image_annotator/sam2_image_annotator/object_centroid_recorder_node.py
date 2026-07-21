#!/usr/bin/env python3

from collections import deque
import json
import os
from pathlib import Path
import tempfile

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
import yaml


class ObjectCentroidRecorderNode(Node):
    def __init__(self):
        super().__init__("object_centroid_recorder")

        self.pointcloud_topic = self.declare_parameter(
            "pointcloud_topic", "/sam2/instance_pointcloud").value
        self.segments_topic = self.declare_parameter(
            "segments_topic", "/sam2/segments").value
        self.save_service_name = self.declare_parameter(
            "save_service", "/sam2/save_object_centroids").value
        self.output_yaml = str(self.declare_parameter(
            "output_yaml", "~/.ros/sam2_objects.yaml").value)
        self.target_frame = str(self.declare_parameter(
            "target_frame", "map").value).strip()
        self.duplicate_distance_threshold = float(self.declare_parameter(
            "duplicate_distance_threshold", 0.25).value)
        self.metadata_sync_tolerance = float(self.declare_parameter(
            "metadata_sync_tolerance", 0.2).value)
        self.sync_queue_size = int(self.declare_parameter(
            "sync_queue_size", 10).value)
        self.min_points = int(self.declare_parameter(
            "min_points", 20).value)
        self.tf_timeout = float(self.declare_parameter(
            "tf_timeout", 0.1).value)
        self.tf_cache_time = float(self.declare_parameter(
            "tf_cache_time", 30.0).value)

        if not self.target_frame:
            self.get_logger().warn("target_frame is empty; using map")
            self.target_frame = "map"
        if self.duplicate_distance_threshold <= 0.0:
            self.get_logger().warn(
                "duplicate_distance_threshold must be positive; using 0.25")
            self.duplicate_distance_threshold = 0.25
        if self.metadata_sync_tolerance < 0.0:
            self.get_logger().warn(
                "metadata_sync_tolerance must be non-negative; using 0.2")
            self.metadata_sync_tolerance = 0.2
        if self.sync_queue_size < 1:
            self.get_logger().warn("sync_queue_size must be positive; using 10")
            self.sync_queue_size = 10
        if self.min_points < 1:
            self.get_logger().warn("min_points must be positive; using 20")
            self.min_points = 20
        if self.tf_timeout < 0.0:
            self.get_logger().warn("tf_timeout must be non-negative; using 0.1")
            self.tf_timeout = 0.1
        if self.tf_cache_time <= 0.0:
            self.get_logger().warn(
                "tf_cache_time must be positive; using 30.0")
            self.tf_cache_time = 30.0

        self.output_path = Path(self.output_yaml).expanduser().resolve()
        self.objects = []
        self.database_error = ""
        self.pending_clouds = deque(maxlen=self.sync_queue_size)
        self.pending_segments = deque(maxlen=self.sync_queue_size)
        self.tf_buffer = Buffer(
            cache_time=Duration(seconds=self.tf_cache_time), node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.load_database()

        input_qos = QoSProfile(
            depth=self.sync_queue_size,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            self.pointcloud_topic,
            self.pointcloud_callback,
            input_qos,
        )
        self.segments_sub = self.create_subscription(
            String,
            self.segments_topic,
            self.segments_callback,
            input_qos,
        )
        self.save_service = self.create_service(
            Trigger, self.save_service_name, self.save_callback)

        self.get_logger().info(
            f"Recording centroids from {self.pointcloud_topic} in "
            f"{self.target_frame}; save with {self.save_service_name} to "
            f"{self.output_path}")

    def pointcloud_callback(self, msg):
        stamp = self.stamp_seconds(msg.header.stamp)
        self.pending_clouds.append((stamp, msg))
        self.match_pending_messages()

    def segments_callback(self, msg):
        try:
            payload = json.loads(msg.data)
            header = payload.get("header", {})
            stamp_data = header.get("stamp", {})
            stamp = (
                float(stamp_data.get("sec", 0))
                + float(stamp_data.get("nanosec", 0)) * 1e-9
            )
            objects = payload.get("objects", [])
            if not isinstance(objects, list):
                raise ValueError("objects must be a list")
        except (TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().warn(
                f"Ignoring invalid segmentation metadata: {exc}",
                throttle_duration_sec=5.0,
            )
            return

        self.pending_segments.append((stamp, objects))
        self.match_pending_messages()

    def match_pending_messages(self):
        while self.pending_clouds and self.pending_segments:
            best = None
            for cloud_index, (cloud_stamp, _) in enumerate(self.pending_clouds):
                for segment_index, (segment_stamp, _) in enumerate(
                        self.pending_segments):
                    offset = abs(cloud_stamp - segment_stamp)
                    if best is None or offset < best[0]:
                        best = (offset, cloud_index, segment_index)

            if best is None or best[0] > self.metadata_sync_tolerance:
                return

            _, cloud_index, segment_index = best
            _, cloud_msg = self.pending_clouds[cloud_index]
            _, metadata_objects = self.pending_segments[segment_index]
            del self.pending_clouds[cloud_index]
            del self.pending_segments[segment_index]
            self.process_observation(cloud_msg, metadata_objects)

    def process_observation(self, cloud_msg, metadata_objects):
        if self.database_error:
            return
        if not cloud_msg.header.frame_id:
            self.get_logger().warn(
                "Ignoring point cloud with an empty frame_id",
                throttle_duration_sec=5.0,
            )
            return

        required_fields = {"x", "y", "z", "instance_id"}
        available_fields = {field.name for field in cloud_msg.fields}
        missing_fields = required_fields - available_fields
        if missing_fields:
            self.get_logger().warn(
                "Ignoring point cloud missing fields: "
                + ", ".join(sorted(missing_fields)),
                throttle_duration_sec=5.0,
            )
            return

        try:
            points = point_cloud2.read_points(
                cloud_msg,
                field_names=("x", "y", "z", "instance_id"),
                skip_nans=True,
            )
            xyz, instance_ids = self.points_to_arrays(points)
        except Exception as exc:
            self.get_logger().warn(
                f"Failed to read instance point cloud: {exc}",
                throttle_duration_sec=5.0,
            )
            return

        if xyz.shape[0] == 0:
            return

        metadata_by_id = {}
        for metadata in metadata_objects:
            try:
                instance_id = int(metadata["instance_id"])
            except (KeyError, TypeError, ValueError):
                continue
            metadata_by_id[instance_id] = metadata

        source_centroids = []
        centroid_metadata = []
        for instance_id in np.unique(instance_ids):
            metadata = metadata_by_id.get(int(instance_id))
            if metadata is None:
                continue
            selected = xyz[instance_ids == instance_id]
            if selected.shape[0] < self.min_points:
                continue
            source_centroids.append(np.mean(selected, axis=0))
            centroid_metadata.append((metadata, int(selected.shape[0])))

        if not source_centroids:
            return

        source_centroids = np.asarray(source_centroids, dtype=np.float64)
        try:
            target_centroids = self.transform_centroids(
                source_centroids,
                cloud_msg.header.frame_id,
                cloud_msg.header.stamp,
            )
        except TransformException as exc:
            self.get_logger().warn(
                f"Cannot transform centroids from {cloud_msg.header.frame_id} "
                f"to {self.target_frame}: {exc}",
                throttle_duration_sec=5.0,
            )
            return

        stamp = {
            "sec": int(cloud_msg.header.stamp.sec),
            "nanosec": int(cloud_msg.header.stamp.nanosec),
        }
        for position, (metadata, point_count) in zip(
                target_centroids, centroid_metadata):
            try:
                class_id = int(metadata.get("class_id", -1))
            except (TypeError, ValueError):
                class_id = -1
            label = str(metadata.get("label", "")).strip()
            if not label:
                label = f"class_{class_id}"
            try:
                confidence = float(metadata.get("confidence", 0.0))
            except (TypeError, ValueError):
                confidence = 0.0
            self.merge_observation(
                label,
                class_id,
                position,
                point_count,
                confidence,
                stamp,
            )

    @staticmethod
    def points_to_arrays(points):
        if isinstance(points, np.ndarray) and points.dtype.names:
            xyz = np.column_stack((points["x"], points["y"], points["z"]))
            instance_ids = np.asarray(points["instance_id"], dtype=np.uint16)
        else:
            rows = list(points)
            if not rows:
                return np.empty((0, 3), np.float64), np.empty(0, np.uint16)
            values = np.asarray(rows)
            xyz = values[:, :3]
            instance_ids = values[:, 3].astype(np.uint16)
        return np.asarray(xyz, dtype=np.float64), instance_ids

    def transform_centroids(self, centroids, source_frame, stamp):
        if source_frame == self.target_frame:
            return centroids

        transform = self.lookup_transform(source_frame, stamp).transform
        rotation = self.quaternion_matrix(transform.rotation)
        translation = np.array([
            transform.translation.x,
            transform.translation.y,
            transform.translation.z,
        ], dtype=np.float64)
        return centroids @ rotation.T + translation

    def lookup_transform(self, source_frame, stamp):
        return self.tf_buffer.lookup_transform(
            self.target_frame,
            source_frame,
            Time.from_msg(stamp),
            timeout=Duration(seconds=self.tf_timeout),
        )

    def merge_observation(
            self, label, class_id, position, point_count, confidence, stamp):
        position = np.asarray(position, dtype=np.float64)
        nearest = None
        for entry in self.objects:
            if entry["label"] != label:
                continue
            entry_position = self.position_array(entry)
            distance = float(np.linalg.norm(position - entry_position))
            if distance <= self.duplicate_distance_threshold:
                if nearest is None or distance < nearest[0]:
                    nearest = (distance, entry)

        if nearest is None:
            self.objects.append({
                "name": self.unique_name(label),
                "label": label,
                "class_id": int(class_id),
                "position": self.position_dict(position),
                "observation_count": 1,
                "point_count": int(point_count),
                "last_confidence": float(confidence),
                "last_seen": dict(stamp),
            })
            return True

        entry = nearest[1]
        old_count = max(1, int(entry.get("observation_count", 1)))
        averaged = (
            self.position_array(entry) * old_count + position
        ) / float(old_count + 1)
        entry["position"] = self.position_dict(averaged)
        entry["observation_count"] = old_count + 1
        entry["point_count"] = int(point_count)
        entry["last_confidence"] = float(confidence)
        entry["last_seen"] = dict(stamp)
        if int(entry.get("class_id", -1)) < 0 and class_id >= 0:
            entry["class_id"] = int(class_id)
        return False

    def unique_name(self, label):
        base_name = label or "object"
        existing_names = {entry["name"] for entry in self.objects}
        if base_name not in existing_names:
            return base_name
        suffix = 2
        while f"{base_name}_{suffix}" in existing_names:
            suffix += 1
        return f"{base_name}_{suffix}"

    def load_database(self):
        if not self.output_path.exists():
            return
        try:
            with self.output_path.open("r", encoding="utf-8") as stream:
                payload = yaml.safe_load(stream) or {}
            if not isinstance(payload, dict):
                raise ValueError("YAML root must be a mapping")
            frame_id = str(payload.get("frame_id", self.target_frame))
            if frame_id != self.target_frame:
                raise ValueError(
                    f"YAML frame_id {frame_id!r} does not match target_frame "
                    f"{self.target_frame!r}")
            entries = payload.get("objects", [])
            if not isinstance(entries, list):
                raise ValueError("objects must be a list")

            loaded = []
            names = set()
            for raw_entry in entries:
                entry = self.normalize_entry(raw_entry)
                if entry["name"] in names:
                    duplicate_name = entry["name"]
                    raise ValueError(
                        f"duplicate object name {duplicate_name!r}")
                names.add(entry["name"])
                loaded.append(entry)
            self.objects = loaded
            self.get_logger().info(
                f"Loaded {len(self.objects)} objects from {self.output_path}")
        except Exception as exc:
            self.database_error = str(exc)
            self.get_logger().error(
                f"Cannot load centroid YAML {self.output_path}: {exc}; "
                "save service is disabled to protect the existing file")

    @classmethod
    def normalize_entry(cls, raw_entry):
        if not isinstance(raw_entry, dict):
            raise ValueError("each object must be a mapping")
        name = str(raw_entry.get("name", "")).strip()
        label = str(raw_entry.get("label", "")).strip()
        if not name or not label:
            raise ValueError("each object requires non-empty name and label")
        position = cls.position_array(raw_entry)
        if position.shape != (3,) or not np.all(np.isfinite(position)):
            raise ValueError(f"object {name!r} has an invalid position")
        last_seen = raw_entry.get("last_seen", {})
        if not isinstance(last_seen, dict):
            last_seen = {}
        return {
            "name": name,
            "label": label,
            "class_id": int(raw_entry.get("class_id", -1)),
            "position": cls.position_dict(position),
            "observation_count": max(
                1, int(raw_entry.get("observation_count", 1))),
            "point_count": max(0, int(raw_entry.get("point_count", 0))),
            "last_confidence": float(raw_entry.get("last_confidence", 0.0)),
            "last_seen": {
                "sec": int(last_seen.get("sec", 0)),
                "nanosec": int(last_seen.get("nanosec", 0)),
            },
        }

    @staticmethod
    def position_array(entry):
        position = entry.get("position", {})
        if not isinstance(position, dict):
            raise ValueError("position must be a mapping")
        return np.array([
            position.get("x", np.nan),
            position.get("y", np.nan),
            position.get("z", np.nan),
        ], dtype=np.float64)

    @staticmethod
    def position_dict(position):
        return {
            "x": float(position[0]),
            "y": float(position[1]),
            "z": float(position[2]),
        }

    def save_callback(self, request, response):
        del request
        if self.database_error:
            response.success = False
            response.message = (
                f"Not saved: existing YAML could not be loaded: "
                f"{self.database_error}")
            return response
        try:
            self.write_database()
        except Exception as exc:
            response.success = False
            response.message = f"Failed to save centroid YAML: {exc}"
            self.get_logger().error(response.message)
            return response

        response.success = True
        response.message = (
            f"Saved {len(self.objects)} objects in {self.target_frame} to "
            f"{self.output_path}")
        self.get_logger().info(response.message)
        return response

    def write_database(self):
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "frame_id": self.target_frame,
            "duplicate_distance_threshold": float(
                self.duplicate_distance_threshold),
            "objects": self.objects,
        }
        file_descriptor, temporary_name = tempfile.mkstemp(
            prefix=f".{self.output_path.name}.",
            suffix=".tmp",
            dir=str(self.output_path.parent),
        )
        try:
            with os.fdopen(file_descriptor, "w", encoding="utf-8") as stream:
                yaml.safe_dump(payload, stream, sort_keys=False)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary_name, self.output_path)
        except Exception:
            try:
                os.unlink(temporary_name)
            except FileNotFoundError:
                pass
            raise

    @staticmethod
    def quaternion_matrix(quaternion):
        x = float(quaternion.x)
        y = float(quaternion.y)
        z = float(quaternion.z)
        w = float(quaternion.w)
        norm = x * x + y * y + z * z + w * w
        if norm < np.finfo(float).eps:
            return np.eye(3, dtype=np.float64)
        scale = 2.0 / norm
        return np.array([
            [1.0 - scale * (y * y + z * z),
             scale * (x * y - z * w),
             scale * (x * z + y * w)],
            [scale * (x * y + z * w),
             1.0 - scale * (x * x + z * z),
             scale * (y * z - x * w)],
            [scale * (x * z - y * w),
             scale * (y * z + x * w),
             1.0 - scale * (x * x + y * y)],
        ], dtype=np.float64)

    @staticmethod
    def stamp_seconds(stamp):
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = ObjectCentroidRecorderNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
