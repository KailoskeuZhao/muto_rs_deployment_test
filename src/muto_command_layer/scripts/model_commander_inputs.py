"""Bounded transient subscriptions and camera input normalization."""

from dataclasses import dataclass
import math
import queue
import threading
import time

import cv2
from model_commander_errors import InputFlowFailure, PlannerFailure
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node


class TransientSubscriptionRequest:
    """Thread-safe state for one bounded subscription collection window."""

    def __init__(self, message_type, topic, qos, maximum_messages):
        """Initialize a request that retains only its latest message."""
        self.message_type = message_type
        self.topic = topic
        self.qos = qos
        self.maximum_messages = maximum_messages
        self.cancel_requested = threading.Event()
        self.done = threading.Event()
        self.started = threading.Event()
        self._lock = threading.Lock()
        self._message_count = 0
        self._latest_message = None
        self._latest_receipt_time = None
        self._error = None

    def record(self, message):
        """Record a newly received message without blocking the worker."""
        with self._lock:
            if self.cancel_requested.is_set():
                return
            self._message_count += 1
            self._latest_message = message
            self._latest_receipt_time = time.monotonic()

    def target_reached(self):
        """Return whether the bounded message target has been received."""
        with self._lock:
            return self.maximum_messages is not None and \
                self._message_count >= self.maximum_messages

    def set_error(self, error):
        """Retain an input or teardown failure for the requesting thread."""
        with self._lock:
            self._error = error

    def snapshot(self):
        """Return an atomic view of the request's current state."""
        with self._lock:
            return (
                self._message_count,
                self._latest_message,
                self._latest_receipt_time,
                self._error,
            )


class TransientSubscriptionWorker:
    """
    Own dynamic subscriptions on an isolated single-threaded executor.

    The commander never destroys a subscription still referenced by its main
    executor. Admission is single-flight and shutdown is bounded, avoiding the
    wait-set race that previously crashed Humble's rclpy executor.
    """

    _STOP = object()

    def __init__(self, context, node_name, poll_period):
        """Start the isolated executor worker and wait for readiness."""
        self._context = context
        self._node_name = node_name
        self._poll_period = poll_period
        self._queue = queue.Queue(maxsize=1)
        self._admission_lock = threading.Lock()
        self._active_request = None
        self._closed = False
        self._startup_error = None
        self._ready = threading.Event()
        self._thread = threading.Thread(
            target=self._run,
            name=f'{node_name}_thread',
            daemon=True,
        )
        self._thread.start()
        if not self._ready.wait(timeout=5.0):
            raise InputFlowFailure(
                f'{node_name} worker did not initialize within 5 seconds')
        if self._startup_error is not None:
            raise InputFlowFailure(
                f'{node_name} worker could not initialize') from \
                self._startup_error

    def start(self, message_type, topic, qos, maximum_messages=None):
        """Admit one bounded subscription request."""
        with self._admission_lock:
            if self._closed:
                raise InputFlowFailure(
                    f'{self._node_name} worker is closed')
            if self._active_request is not None:
                raise InputFlowFailure(
                    f'{self._node_name} worker already has an active request')
            request = TransientSubscriptionRequest(
                message_type, topic, qos, maximum_messages)
            self._active_request = request
            try:
                self._queue.put_nowait(request)
            except queue.Full as error:
                self._active_request = None
                raise InputFlowFailure(
                    f'{self._node_name} worker request queue is full') from \
                    error
            return request

    def cancel_and_wait(self, request, timeout):
        """Request collection shutdown and wait for worker-owned teardown."""
        request.cancel_requested.set()
        return request.done.wait(timeout=timeout)

    def close(self, timeout):
        """Stop any active request and then terminate the worker thread."""
        with self._admission_lock:
            if self._closed:
                return not self._thread.is_alive()
            self._closed = True
            active_request = self._active_request
        if active_request is not None:
            active_request.cancel_requested.set()
            active_request.done.wait(timeout=timeout)
        try:
            self._queue.put(self._STOP, timeout=timeout)
        except queue.Full:
            return False
        self._thread.join(timeout=timeout)
        return not self._thread.is_alive()

    def _run(self):
        node = None
        executor = None
        try:
            node = Node(
                self._node_name,
                context=self._context,
                use_global_arguments=False,
            )
            executor = SingleThreadedExecutor(context=self._context)
            executor.add_node(node)
        except Exception as error:
            self._startup_error = error
            self._ready.set()
            return
        self._ready.set()
        try:
            while True:
                request = self._queue.get()
                if request is self._STOP:
                    break
                self._serve_request(node, executor, request)
        finally:
            try:
                executor.remove_node(node)
                executor.shutdown()
            finally:
                node.destroy_node()

    def _serve_request(self, node, executor, request):
        subscription = None
        try:
            subscription = node.create_subscription(
                request.message_type,
                request.topic,
                request.record,
                request.qos,
            )
            request.started.set()
            while not request.cancel_requested.is_set() and \
                    not request.target_reached():
                if not rclpy.ok(context=self._context):
                    raise InputFlowFailure(
                        f'{self._node_name} ROS context is shutting down')
                executor.spin_once(timeout_sec=self._poll_period)
        except Exception as error:
            request.set_error(error)
        finally:
            if subscription is not None:
                try:
                    node.destroy_subscription(subscription)
                except Exception as error:
                    request.set_error(error)
            with self._admission_lock:
                if self._active_request is request:
                    self._active_request = None
            request.done.set()


@dataclass(frozen=True)
class VisualObservation:
    """One bounded camera snapshot supplied to a VLM planning request."""

    sequence: int
    jpeg_data: bytes
    receipt_monotonic: float
    frame_id: str
    stamp_seconds: float
    receipt_age_seconds: float
    source_width: int
    source_height: int
    encoded_width: int
    encoded_height: int


@dataclass(frozen=True)
class VisualCodecLimits:
    """Resource limits applied before and during camera-frame conversion."""

    max_width: int
    max_height: int
    jpeg_quality: int
    max_jpeg_bytes: int
    max_source_width: int
    max_source_height: int
    max_source_bytes: int


class VisualObservationCodec:
    """Convert a bounded ROS Image into the commander's JPEG observation."""

    def __init__(self, limits):
        """Initialize the codec with immutable resource limits."""
        self._limits = limits

    def encode(self, message, sequence, receipt_time):
        """Validate, resize, and encode one ROS Image as a bounded JPEG."""
        limits = self._limits
        if message.width <= 0 or message.height <= 0 or \
                message.width > limits.max_source_width or \
                message.height > limits.max_source_height:
            raise PlannerFailure('camera frame source dimensions are invalid')
        if len(message.data) <= 0 or \
                len(message.data) > limits.max_source_bytes:
            raise PlannerFailure('camera frame source payload is invalid')
        image = self._decode_to_bgr(message)
        if image is None or image.ndim != 3 or image.shape[2] != 3:
            raise PlannerFailure('camera frame did not convert to BGR image')
        source_height, source_width = image.shape[:2]
        if source_width <= 0 or source_height <= 0:
            raise PlannerFailure('camera frame has invalid dimensions')

        scale = min(
            1.0,
            limits.max_width / float(source_width),
            limits.max_height / float(source_height),
        )
        try:
            if scale < 1.0:
                image = cv2.resize(
                    image,
                    (
                        max(1, int(round(source_width * scale))),
                        max(1, int(round(source_height * scale))),
                    ),
                    interpolation=cv2.INTER_AREA,
                )

            jpeg_data = b''
            for attempt in range(7):
                encoded_ok, encoded = cv2.imencode(
                    '.jpg',
                    image,
                    [cv2.IMWRITE_JPEG_QUALITY, limits.jpeg_quality],
                )
                if not encoded_ok:
                    raise PlannerFailure('camera frame JPEG encoding failed')
                jpeg_data = encoded.tobytes()
                if len(jpeg_data) <= limits.max_jpeg_bytes:
                    break
                height, width = image.shape[:2]
                if attempt == 6 or width <= 64 or height <= 64:
                    break
                reduction = min(
                    0.85,
                    math.sqrt(
                        limits.max_jpeg_bytes / float(len(jpeg_data))) * 0.9,
                )
                image = cv2.resize(
                    image,
                    (
                        max(64, int(round(width * reduction))),
                        max(64, int(round(height * reduction))),
                    ),
                    interpolation=cv2.INTER_AREA,
                )
        except cv2.error as error:
            raise PlannerFailure(
                'camera frame OpenCV processing failed') from error
        if len(jpeg_data) > limits.max_jpeg_bytes:
            raise PlannerFailure(
                'camera observation exceeds the JPEG byte limit')
        if len(jpeg_data) < 4 or not jpeg_data.startswith(b'\xff\xd8') or \
                not jpeg_data.endswith(b'\xff\xd9'):
            raise PlannerFailure(
                'camera observation is not a complete JPEG stream')

        stamp = message.header.stamp
        return VisualObservation(
            sequence=sequence,
            jpeg_data=jpeg_data,
            receipt_monotonic=receipt_time,
            frame_id=message.header.frame_id.strip(),
            stamp_seconds=float(stamp.sec) + float(stamp.nanosec) * 1e-9,
            receipt_age_seconds=max(0.0, time.monotonic() - receipt_time),
            source_width=source_width,
            source_height=source_height,
            encoded_width=int(image.shape[1]),
            encoded_height=int(image.shape[0]),
        )

    @staticmethod
    def _decode_to_bgr(message):
        encoding = message.encoding.strip().lower()
        channel_counts = {
            'bgr8': 3,
            'rgb8': 3,
            'mono8': 1,
            'bgra8': 4,
            'rgba8': 4,
        }
        channels = channel_counts.get(encoding)
        if channels is None:
            raise PlannerFailure(
                f'unsupported camera image encoding: {message.encoding}')
        minimum_step = message.width * channels
        if message.step < minimum_step:
            raise PlannerFailure('camera frame row stride is invalid')
        required_bytes = message.step * (message.height - 1) + minimum_step
        if len(message.data) < required_bytes:
            raise PlannerFailure('camera frame payload is shorter than layout')
        try:
            flat = np.frombuffer(message.data, dtype=np.uint8)
            rows = flat[:message.step * message.height].reshape(
                (message.height, message.step))
            image = rows[:, :minimum_step].reshape(
                (message.height, message.width, channels))
            if encoding == 'bgr8':
                return image.copy()
            if encoding == 'rgb8':
                return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            if encoding == 'mono8':
                return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
            if encoding == 'bgra8':
                return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
            if encoding == 'rgba8':
                return cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
        except (AttributeError, ImportError, RuntimeError, TypeError,
                ValueError, cv2.error) as error:
            raise PlannerFailure(
                'camera frame could not be converted to bgr8') from error
        raise PlannerFailure(
            f'unsupported camera image encoding: {message.encoding}')
