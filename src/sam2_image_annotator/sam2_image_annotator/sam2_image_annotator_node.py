#!/usr/bin/env python3

from contextlib import ExitStack, nullcontext
import time

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from sam2_image_annotator.paths import resolve_checkpoint_path


class Sam2ImageAnnotatorNode(Node):
    def __init__(self):
        super().__init__("sam2_image_annotator")

        self.image_topic = self.declare_parameter(
            "image_topic", "/camera/color/image_raw").value
        self.annotated_topic = self.declare_parameter(
            "annotated_topic", "/sam2/annotated_image").value
        self.mask_topic = self.declare_parameter("mask_topic", "/sam2/mask").value

        self.checkpoint = self.declare_parameter(
            "checkpoint", "checkpoints/sam2.1_hiera_base_plus.pt").value
        self.model_cfg = self.declare_parameter(
            "model_cfg", "configs/sam2.1/sam2.1_hiera_b+.yaml").value
        self.device = self.declare_parameter("device", "cuda").value
        self.use_autocast = bool(self.declare_parameter("use_autocast", True).value)
        self.autocast_dtype = self.declare_parameter("autocast_dtype", "bfloat16").value

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

        self.bridge = CvBridge()
        self.predictor = None
        self.torch = None
        self.predictor_error = ""
        self.processing = False
        self.last_process_wall_time = 0.0

        self.load_predictor()

        input_qos = QoSProfile(
            depth=self.queue_size,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        output_qos = QoSProfile(depth=self.queue_size)

        self.annotated_pub = self.create_publisher(Image, self.annotated_topic, output_qos)
        self.mask_pub = self.create_publisher(Image, self.mask_topic, output_qos)
        self.image_sub = self.create_subscription(
            Image, self.image_topic, self.image_callback, input_qos)

        self.get_logger().info(
            f"Subscribing to {self.image_topic}; publishing annotated images on "
            f"{self.annotated_topic} and masks on {self.mask_topic}")

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

            rgb_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)
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

    def predict_mask(self, rgb_image, point_coords, point_labels, box):
        with self.torch_context():
            self.predictor.set_image(rgb_image)
            masks, scores, _ = self.predictor.predict(
                point_coords=point_coords,
                point_labels=point_labels,
                box=box,
                multimask_output=self.multimask_output,
            )

        masks = self.to_numpy(masks)
        scores = self.to_numpy(scores)
        if masks.ndim == 2:
            selected_mask = masks
        else:
            masks = np.squeeze(masks)
            if masks.ndim == 2:
                selected_mask = masks
            else:
                selected_index = 0
                if scores is not None and np.size(scores) > 0:
                    selected_index = int(np.argmax(np.asarray(scores).reshape(-1)))
                    selected_index = min(selected_index, masks.shape[0] - 1)
                selected_mask = masks[selected_index]

        return np.asarray(selected_mask) > 0

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

    def annotate_image(self, bgr_image, mask, point_coords, point_labels, box):
        annotated = bgr_image.copy()
        if mask is not None and np.any(mask):
            color = np.array(self.mask_color_bgr, dtype=np.float32)
            masked_pixels = annotated[mask].astype(np.float32)
            annotated[mask] = np.clip(
                masked_pixels * (1.0 - self.overlay_alpha) + color * self.overlay_alpha,
                0,
                255,
            ).astype(np.uint8)

            mask_uint8 = (mask.astype(np.uint8)) * 255
            contours, _ = cv2.findContours(
                mask_uint8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(annotated, contours, -1, self.contour_color_bgr, 2)

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

    def publish_images(self, input_msg, annotated, mask):
        annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        annotated_msg.header = input_msg.header
        self.annotated_pub.publish(annotated_msg)

        mask_msg = self.bridge.cv2_to_imgmsg((mask.astype(np.uint8)) * 255, encoding="mono8")
        mask_msg.header = input_msg.header
        self.mask_pub.publish(mask_msg)

    def publish_unavailable_image(self, input_msg, bgr_image):
        if not self.publish_passthrough_on_error:
            return

        annotated = bgr_image.copy()
        text = "SAM2 unavailable"
        if self.predictor_error:
            text = text + ": " + self.predictor_error.split(":")[0]
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
