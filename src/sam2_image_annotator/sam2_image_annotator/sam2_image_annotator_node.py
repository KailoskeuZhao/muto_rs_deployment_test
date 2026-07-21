#!/usr/bin/env python3

from contextlib import ExitStack, nullcontext
import json
import time

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String

from sam2_image_annotator.paths import resolve_checkpoint_path


class Sam2ImageAnnotatorNode(Node):
    def __init__(self):
        super().__init__("sam2_image_annotator")

        self.image_topic = self.declare_parameter(
            "image_topic", "/camera/color/image_raw").value
        self.annotated_topic = self.declare_parameter(
            "annotated_topic", "/sam2/annotated_image").value
        self.mask_topic = self.declare_parameter("mask_topic", "/sam2/mask").value
        self.instance_mask_topic = self.declare_parameter(
            "instance_mask_topic", "/sam2/instance_mask").value
        self.segments_topic = self.declare_parameter(
            "segments_topic", "/sam2/segments").value

        self.checkpoint = self.declare_parameter(
            "checkpoint", "checkpoints/sam2.1_hiera_base_plus.pt").value
        self.model_cfg = self.declare_parameter(
            "model_cfg", "configs/sam2.1/sam2.1_hiera_b+.yaml").value
        self.device = self.declare_parameter("device", "cuda").value
        self.use_autocast = bool(self.declare_parameter("use_autocast", True).value)
        self.autocast_dtype = self.declare_parameter("autocast_dtype", "bfloat16").value

        self.prompt_mode = str(
            self.declare_parameter("prompt_mode", "yolo").value).strip().lower()
        self.yolo_model_path = self.declare_parameter(
            "yolo_model", "yolo11m.pt").value
        self.yolo_device = str(
            self.declare_parameter("yolo_device", "0").value).strip()
        self.yolo_confidence = float(
            self.declare_parameter("yolo_confidence", 0.25).value)
        self.yolo_iou = float(self.declare_parameter("yolo_iou", 0.7).value)
        self.yolo_imgsz = int(self.declare_parameter("yolo_imgsz", 640).value)
        self.yolo_max_detections = int(
            self.declare_parameter("yolo_max_detections", 20).value)
        self.yolo_classes_text = str(
            self.declare_parameter("yolo_classes", "").value)
        self.yolo_quantize = str(
            self.declare_parameter("yolo_quantize", "fp16").value).strip()

        self.default_prompt = self.declare_parameter("default_prompt", "center_point").value
        self.point_coords_text = self.declare_parameter("point_coords", "").value
        self.point_labels_text = self.declare_parameter("point_labels", "").value
        self.box_text = self.declare_parameter("box", "").value
        self.multimask_output = bool(
            self.declare_parameter("multimask_output", False).value)

        self.overlay_alpha = float(self.declare_parameter("overlay_alpha", 0.45).value)
        self.mask_color_bgr = self.parse_color(
            self.declare_parameter("mask_color_bgr", "0,255,0").value,
            (0, 255, 0),
            "mask_color_bgr",
        )
        self.contour_color_bgr = self.parse_color(
            self.declare_parameter("contour_color_bgr", "0,255,255").value,
            (0, 255, 255),
            "contour_color_bgr",
        )
        self.draw_prompts = bool(self.declare_parameter("draw_prompts", True).value)
        self.publish_passthrough_on_error = bool(
            self.declare_parameter("publish_passthrough_on_error", True).value)
        self.max_publish_rate = float(self.declare_parameter("max_publish_rate", 1.0).value)
        self.queue_size = int(self.declare_parameter("queue_size", 2).value)

        if self.overlay_alpha < 0.0 or self.overlay_alpha > 1.0:
            self.get_logger().warn("overlay_alpha must be in [0.0, 1.0]; using 0.45")
            self.overlay_alpha = 0.45
        if self.max_publish_rate < 0.0:
            self.get_logger().warn("max_publish_rate must be non-negative; using 0.0")
            self.max_publish_rate = 0.0
        if self.queue_size < 1:
            self.get_logger().warn("queue_size must be positive; using 1")
            self.queue_size = 1
        if self.prompt_mode not in ("yolo", "manual"):
            self.get_logger().warn(
                f"Unknown prompt_mode '{self.prompt_mode}'; using yolo")
            self.prompt_mode = "yolo"
        if self.yolo_max_detections < 1:
            self.get_logger().warn(
                "yolo_max_detections must be positive; using 20")
            self.yolo_max_detections = 20
        if self.yolo_imgsz < 32:
            self.get_logger().warn("yolo_imgsz must be at least 32; using 640")
            self.yolo_imgsz = 640
        if not 0.0 <= self.yolo_confidence <= 1.0:
            self.get_logger().warn(
                "yolo_confidence must be in [0.0, 1.0]; using 0.25")
            self.yolo_confidence = 0.25
        if not 0.0 <= self.yolo_iou <= 1.0:
            self.get_logger().warn(
                "yolo_iou must be in [0.0, 1.0]; using 0.7")
            self.yolo_iou = 0.7

        self.bridge = CvBridge()
        self.predictor = None
        self.detector = None
        self.torch = None
        self.predictor_error = ""
        self.detector_error = ""
        self.processing = False
        self.last_process_wall_time = 0.0

        self.load_predictor()
        if self.prompt_mode == "yolo":
            self.load_detector()

        input_qos = QoSProfile(
            depth=self.queue_size,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        output_qos = QoSProfile(depth=self.queue_size)

        self.annotated_pub = self.create_publisher(Image, self.annotated_topic, output_qos)
        self.mask_pub = self.create_publisher(Image, self.mask_topic, output_qos)
        self.instance_mask_pub = self.create_publisher(
            Image, self.instance_mask_topic, output_qos)
        self.segments_pub = self.create_publisher(
            String, self.segments_topic, output_qos)
        self.image_sub = self.create_subscription(
            Image, self.image_topic, self.image_callback, input_qos)

        self.get_logger().info(
            f"Subscribing to {self.image_topic} in {self.prompt_mode} mode; "
            f"publishing annotations on {self.annotated_topic}, masks on "
            f"{self.mask_topic}, and object results on {self.segments_topic}")

    def load_predictor(self):
        try:
            import sam2
            import torch
            from sam2.build_sam import build_sam2
            from sam2.sam2_image_predictor import SAM2ImagePredictor
        except Exception as exc:
            self.predictor_error = f"SAM2 runtime imports failed: {exc}"
            self.get_logger().warn(
                self.predictor_error + "; node will spin without segmentation")
            return

        try:
            self.checkpoint = resolve_checkpoint_path(
                self.checkpoint,
                getattr(sam2, "__file__", None),
            )
            try:
                sam_model = build_sam2(
                    self.model_cfg,
                    self.checkpoint,
                    device=self.device,
                )
            except TypeError:
                sam_model = build_sam2(self.model_cfg, self.checkpoint)
                if self.device and hasattr(sam_model, "to"):
                    sam_model = sam_model.to(self.device)

            self.torch = torch
            self.predictor = SAM2ImagePredictor(sam_model)
            self.get_logger().info(
                f"Loaded SAM2 predictor with config '{self.model_cfg}' and "
                f"checkpoint '{self.checkpoint}' on device '{self.device}'")
        except Exception as exc:
            self.predictor = None
            self.torch = torch
            self.predictor_error = f"SAM2 predictor initialization failed: {exc}"
            self.get_logger().warn(
                self.predictor_error + "; node will spin without segmentation")

    def load_detector(self):
        try:
            from ultralytics import YOLO
        except Exception as exc:
            self.detector_error = f"Ultralytics runtime import failed: {exc}"
            self.get_logger().warn(
                self.detector_error + "; node will spin without detection")
            return

        try:
            self.detector = YOLO(self.yolo_model_path)
            self.get_logger().info(
                f"Loaded YOLO detector from '{self.yolo_model_path}'")
        except Exception as exc:
            self.detector = None
            self.detector_error = f"YOLO detector initialization failed: {exc}"
            self.get_logger().warn(
                self.detector_error + "; node will spin without detection")

    def image_callback(self, msg):
        if self.processing:
            return
        if self.should_throttle():
            return

        self.processing = True
        try:
            bgr_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            if self.predictor is None:
                self.publish_unavailable_image(msg, bgr_image)
                return
            if self.prompt_mode == "yolo" and self.detector is None:
                self.publish_unavailable_image(msg, bgr_image)
                return

            rgb_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)
            if self.prompt_mode == "yolo":
                detections = self.detect_objects(bgr_image)
                segments = self.segment_detections(rgb_image, detections)
                annotated, mask, instance_mask, objects = (
                    self.compose_detection_outputs(bgr_image, segments)
                )
                self.publish_images(
                    msg, annotated, mask, instance_mask, objects)
                return

            point_coords, point_labels, box = self.build_prompts(bgr_image.shape)
            if point_coords is None and box is None:
                self.get_logger().warn(
                    "No SAM2 prompt configured; set point_coords, box, or "
                    "default_prompt:=center_point",
                    throttle_duration_sec=5.0,
                )
                self.publish_unavailable_image(msg, bgr_image)
                return

            mask = self.predict_mask(rgb_image, point_coords, point_labels, box)
            annotated = self.annotate_image(bgr_image, mask, point_coords, point_labels, box)
            self.publish_images(msg, annotated, mask)
        except Exception as exc:
            self.get_logger().error(f"Failed to process image with SAM2: {exc}")
            if self.publish_passthrough_on_error:
                try:
                    bgr_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                    self.publish_unavailable_image(msg, bgr_image)
                except Exception:
                    pass
        finally:
            self.last_process_wall_time = time.monotonic()
            self.processing = False

    def should_throttle(self):
        if self.max_publish_rate <= 0.0:
            return False
        if self.last_process_wall_time <= 0.0:
            return False

        return time.monotonic() - self.last_process_wall_time < 1.0 / self.max_publish_rate

    def detect_objects(self, bgr_image):
        class_ids = self.parse_class_ids(self.yolo_classes_text)
        quantize = self.yolo_quantize or None
        if self.yolo_device.lower() == "cpu" and quantize in ("16", "fp16"):
            quantize = None
        results = self.detector.predict(
            source=bgr_image,
            conf=self.yolo_confidence,
            iou=self.yolo_iou,
            imgsz=self.yolo_imgsz,
            max_det=self.yolo_max_detections,
            classes=class_ids,
            device=self.yolo_device or None,
            quantize=quantize,
            verbose=False,
        )
        if not results or results[0].boxes is None:
            return []

        result = results[0]
        boxes = self.to_numpy(result.boxes.xyxy)
        confidences = self.to_numpy(result.boxes.conf)
        class_indices = self.to_numpy(result.boxes.cls)
        names = result.names if result.names is not None else self.detector.names
        height, width = bgr_image.shape[:2]
        detections = []

        for box, confidence, class_index in zip(
                boxes, confidences, class_indices):
            box = np.asarray(box, dtype=np.float32).reshape(4)
            box[0::2] = np.clip(box[0::2], 0, max(width - 1, 0))
            box[1::2] = np.clip(box[1::2], 0, max(height - 1, 0))
            if box[2] <= box[0] or box[3] <= box[1]:
                continue

            class_id = int(class_index)
            if isinstance(names, dict):
                label = names.get(class_id, str(class_id))
            elif class_id < len(names):
                label = names[class_id]
            else:
                label = str(class_id)
            detections.append({
                "class_id": class_id,
                "label": str(label),
                "confidence": float(confidence),
                "box": box,
            })

        return detections

    def segment_detections(self, rgb_image, detections):
        if not detections:
            return []

        segments = []
        with self.torch_context():
            self.predictor.set_image(rgb_image)
            for detection in detections:
                masks, scores, _ = self.predictor.predict(
                    point_coords=None,
                    point_labels=None,
                    box=detection["box"],
                    multimask_output=self.multimask_output,
                )
                mask, sam_score = self.select_best_mask(masks, scores)
                segment = detection.copy()
                segment["mask"] = mask
                segment["sam_score"] = sam_score
                segments.append(segment)

        return segments

    def predict_mask(self, rgb_image, point_coords, point_labels, box):
        with self.torch_context():
            self.predictor.set_image(rgb_image)
            masks, scores, _ = self.predictor.predict(
                point_coords=point_coords,
                point_labels=point_labels,
                box=box,
                multimask_output=self.multimask_output,
            )

        selected_mask, _ = self.select_best_mask(masks, scores)
        return selected_mask

    def select_best_mask(self, masks, scores):
        masks = self.to_numpy(masks)
        scores = self.to_numpy(scores)
        selected_index = 0
        if masks.ndim == 2:
            selected_mask = masks
        else:
            masks = np.squeeze(masks)
            if masks.ndim == 2:
                selected_mask = masks
            else:
                if scores is not None and np.size(scores) > 0:
                    selected_index = int(np.argmax(np.asarray(scores).reshape(-1)))
                    selected_index = min(selected_index, masks.shape[0] - 1)
                selected_mask = masks[selected_index]

        selected_score = 0.0
        if scores is not None and np.size(scores) > 0:
            flattened_scores = np.asarray(scores).reshape(-1)
            selected_index = min(selected_index if masks.ndim > 2 else 0,
                                 flattened_scores.shape[0] - 1)
            selected_score = float(flattened_scores[selected_index])

        return np.asarray(selected_mask) > 0, selected_score

    def torch_context(self):
        if self.torch is None:
            return nullcontext()

        stack = ExitStack()
        stack.enter_context(self.torch.inference_mode())
        if self.use_autocast and self.device.startswith("cuda"):
            dtype = getattr(self.torch, self.autocast_dtype, None)
            if dtype is None:
                self.get_logger().warn(
                    f"Unknown torch dtype '{self.autocast_dtype}'; disabling autocast",
                    throttle_duration_sec=10.0,
                )
            else:
                stack.enter_context(self.torch.autocast("cuda", dtype=dtype))
        return stack

    def build_prompts(self, image_shape):
        height, width = image_shape[:2]
        point_coords = self.parse_points(self.point_coords_text)
        point_labels = self.parse_labels(self.point_labels_text)
        box = self.parse_box(self.box_text)

        if point_coords is not None:
            if point_labels is None:
                point_labels = np.ones((point_coords.shape[0],), dtype=np.int32)
            elif point_labels.shape[0] != point_coords.shape[0]:
                self.get_logger().warn(
                    "point_labels length does not match point_coords; using positive labels")
                point_labels = np.ones((point_coords.shape[0],), dtype=np.int32)
        elif box is None and self.default_prompt == "center_point":
            point_coords = np.array([[width * 0.5, height * 0.5]], dtype=np.float32)
            point_labels = np.array([1], dtype=np.int32)

        return point_coords, point_labels, box

    def compose_detection_outputs(self, bgr_image, segments):
        height, width = bgr_image.shape[:2]
        annotated = bgr_image.copy()
        union_mask = np.zeros((height, width), dtype=bool)
        instance_mask = np.zeros((height, width), dtype=np.uint16)
        objects = []

        for segment in segments:
            mask = np.asarray(segment["mask"], dtype=bool)
            if mask.shape != (height, width):
                self.get_logger().warn(
                    f"Ignoring {segment['label']} mask with unexpected shape "
                    f"{mask.shape}; expected {(height, width)}")
                continue

            instance_id = len(objects) + 1
            union_mask |= mask
            instance_mask[np.logical_and(mask, instance_mask == 0)] = instance_id
            color = self.instance_color(instance_id)
            self.overlay_mask(annotated, mask, color)

            box = segment["box"]
            box_points = (
                (int(round(box[0])), int(round(box[1]))),
                (int(round(box[2])), int(round(box[3]))),
            )
            cv2.rectangle(annotated, box_points[0], box_points[1], color, 2)
            tag = (
                f"{segment['label']} {segment['confidence']:.2f} "
                f"SAM {segment['sam_score']:.2f}"
            )
            self.draw_label(annotated, tag, box_points[0], color)
            objects.append({
                "instance_id": instance_id,
                "class_id": segment["class_id"],
                "label": segment["label"],
                "confidence": segment["confidence"],
                "box_xyxy": [float(value) for value in box],
                "sam_score": segment["sam_score"],
                "mask_area": int(np.count_nonzero(mask)),
            })

        return annotated, union_mask, instance_mask, objects

    def overlay_mask(self, image, mask, color, contour_color=None):
        if not np.any(mask):
            return

        color_array = np.array(color, dtype=np.float32)
        masked_pixels = image[mask].astype(np.float32)
        image[mask] = np.clip(
            masked_pixels * (1.0 - self.overlay_alpha)
            + color_array * self.overlay_alpha,
            0,
            255,
        ).astype(np.uint8)
        contours, _ = cv2.findContours(
            mask.astype(np.uint8) * 255,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )
        cv2.drawContours(
            image, contours, -1, contour_color or color, 2)

    @staticmethod
    def draw_label(image, text, origin, color):
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.5
        thickness = 1
        (text_width, text_height), baseline = cv2.getTextSize(
            text, font, font_scale, thickness)
        x = max(0, min(origin[0], image.shape[1] - text_width - 4))
        y = max(text_height + baseline + 4, origin[1])
        cv2.rectangle(
            image,
            (x, y - text_height - baseline - 4),
            (x + text_width + 4, y),
            color,
            -1,
        )
        cv2.putText(
            image,
            text,
            (x + 2, y - baseline - 2),
            font,
            font_scale,
            (255, 255, 255),
            thickness,
            cv2.LINE_AA,
        )

    @staticmethod
    def instance_color(instance_id):
        palette = (
            (0, 200, 255),
            (0, 180, 0),
            (255, 120, 0),
            (220, 0, 180),
            (0, 80, 255),
            (255, 200, 0),
            (180, 80, 255),
            (255, 80, 80),
        )
        return palette[(instance_id - 1) % len(palette)]

    def annotate_image(self, bgr_image, mask, point_coords, point_labels, box):
        annotated = bgr_image.copy()
        if mask is not None and np.any(mask):
            self.overlay_mask(
                annotated,
                mask,
                self.mask_color_bgr,
                self.contour_color_bgr,
            )

        if self.draw_prompts:
            if point_coords is not None:
                labels = point_labels if point_labels is not None else []
                for index, point in enumerate(point_coords):
                    label = int(labels[index]) if index < len(labels) else 1
                    color = (0, 255, 0) if label > 0 else (0, 0, 255)
                    cv2.circle(
                        annotated,
                        (int(round(point[0])), int(round(point[1]))),
                        5,
                        color,
                        -1,
                    )
            if box is not None:
                cv2.rectangle(
                    annotated,
                    (int(round(box[0])), int(round(box[1]))),
                    (int(round(box[2])), int(round(box[3]))),
                    (255, 0, 0),
                    2,
                )

        return annotated

    def publish_images(
            self, input_msg, annotated, mask, instance_mask=None, objects=None):
        annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        annotated_msg.header = input_msg.header
        self.annotated_pub.publish(annotated_msg)

        mask_msg = self.bridge.cv2_to_imgmsg((mask.astype(np.uint8)) * 255, encoding="mono8")
        mask_msg.header = input_msg.header
        self.mask_pub.publish(mask_msg)

        if instance_mask is None:
            instance_mask = mask.astype(np.uint16)
        instance_mask_msg = self.bridge.cv2_to_imgmsg(
            instance_mask, encoding="16UC1")
        instance_mask_msg.header = input_msg.header
        self.instance_mask_pub.publish(instance_mask_msg)

        payload = {
            "header": {
                "stamp": {
                    "sec": input_msg.header.stamp.sec,
                    "nanosec": input_msg.header.stamp.nanosec,
                },
                "frame_id": input_msg.header.frame_id,
            },
            "image_width": int(annotated.shape[1]),
            "image_height": int(annotated.shape[0]),
            "objects": objects if objects is not None else [],
        }
        self.segments_pub.publish(String(data=json.dumps(payload)))

    def publish_unavailable_image(self, input_msg, bgr_image):
        if not self.publish_passthrough_on_error:
            return

        annotated = bgr_image.copy()
        text = "SAM2 unavailable"
        error = self.predictor_error
        if self.prompt_mode == "yolo" and self.detector is None:
            text = "YOLO unavailable"
            error = self.detector_error
        if error:
            text = text + ": " + error.split(":")[0]
        cv2.putText(
            annotated,
            text,
            (12, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 0, 255),
            2,
            cv2.LINE_AA,
        )
        mask = np.zeros(annotated.shape[:2], dtype=bool)
        self.publish_images(input_msg, annotated, mask)

    def parse_class_ids(self, text):
        text = str(text).strip()
        if not text:
            return None

        try:
            class_ids = [
                int(value.strip())
                for value in text.split(",")
                if value.strip()
            ]
        except ValueError:
            self.get_logger().warn(
                f"Could not parse yolo_classes '{text}'; expected '0,2,5'",
                throttle_duration_sec=10.0,
            )
            return None

        return class_ids or None

    def parse_points(self, text):
        text = str(text).strip()
        if not text:
            return None

        points = []
        try:
            for pair in text.split(";"):
                x_text, y_text = pair.split(",", 1)
                points.append([float(x_text), float(y_text)])
        except ValueError:
            self.get_logger().warn(
                f"Could not parse point_coords '{text}'; expected 'x,y;x,y'")
            return None

        if not points:
            return None
        return np.asarray(points, dtype=np.float32)

    def parse_labels(self, text):
        text = str(text).strip()
        if not text:
            return None

        try:
            labels = [int(value.strip()) for value in text.split(",") if value.strip()]
        except ValueError:
            self.get_logger().warn(
                f"Could not parse point_labels '{text}'; expected '1,0'")
            return None

        if not labels:
            return None
        return np.asarray(labels, dtype=np.int32)

    def parse_box(self, text):
        text = str(text).strip()
        if not text:
            return None

        try:
            values = [float(value.strip()) for value in text.split(",")]
        except ValueError:
            self.get_logger().warn(f"Could not parse box '{text}'; expected 'x1,y1,x2,y2'")
            return None

        if len(values) != 4:
            self.get_logger().warn(f"Could not parse box '{text}'; expected four values")
            return None
        return np.asarray(values, dtype=np.float32)

    def parse_color(self, text, fallback, parameter_name):
        try:
            values = [int(value.strip()) for value in str(text).split(",")]
        except ValueError:
            self.get_logger().warn(
                f"Could not parse {parameter_name}; expected 'b,g,r'. Using {fallback}")
            return fallback

        if len(values) != 3 or any(value < 0 or value > 255 for value in values):
            self.get_logger().warn(
                f"Could not parse {parameter_name}; expected three values in [0, 255]. "
                f"Using {fallback}")
            return fallback
        return tuple(values)

    @staticmethod
    def to_numpy(value):
        if value is None:
            return None
        if hasattr(value, "detach"):
            return value.detach().cpu().numpy()
        return np.asarray(value)


def main(args=None):
    rclpy.init(args=args)
    node = Sam2ImageAnnotatorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
