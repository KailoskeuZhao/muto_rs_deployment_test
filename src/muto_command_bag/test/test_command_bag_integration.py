"""Integration test for one complete command-mission recording."""

import json
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String


def wait_until(predicate, timeout=8.0):
    deadline = time.monotonic() + timeout
    while not predicate() and time.monotonic() < deadline:
        time.sleep(0.02)
    assert predicate(), 'condition did not become true before timeout'


def test_records_parent_lifecycle_decision_context_and_inspected_jpeg(
        tmp_path):
    domain_id = int(os.environ.get('ROS_DOMAIN_ID', '185'))
    rclpy.init(domain_id=domain_id)
    node = Node('fake_command_mission')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    status_messages = []
    transient_qos = QoSProfile(
        depth=10,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE,
    )
    trace_qos = QoSProfile(
        depth=100,
        durability=DurabilityPolicy.VOLATILE,
        reliability=ReliabilityPolicy.RELIABLE,
    )
    lifecycle_publisher = node.create_publisher(
        String, '/model_commander/recording_event', transient_qos)
    decision_publisher = node.create_publisher(
        String, '/model_commander/decision_event', trace_qos)
    image_publisher = node.create_publisher(
        CompressedImage, '/model_commander/inspected_image', trace_qos)
    status_subscription = node.create_subscription(
        String,
        '/model_commander/bag_status',
        lambda message: status_messages.append(json.loads(message.data)),
        transient_qos,
    )
    assert status_subscription is not None

    process_environment = os.environ.copy()
    process_environment['ROS_DOMAIN_ID'] = str(domain_id)
    process = subprocess.Popen(
        [
            'ros2', 'run', 'muto_exploration_bag',
            'exploration_bag_recorder',
            '--ros-args',
            '-r', '__node:=command_bag_recorder',
            '-p', f'output_directory:={tmp_path}',
            '-p', 'storage_id:=mcap',
            '-p', 'post_terminal_delay:=0.1',
            '-p',
            'lifecycle_event_topic:=/model_commander/recording_event',
            '-p', 'status_topic:=/model_commander/bag_status',
            '-p', 'path_topic:=/model_commander/last_bag_path',
            '-p', 'operator_event_topic:=/model_commander/operator_event',
            '-p', 'bag_prefix:=muto_command',
            '-p', 'manifest_schema:=command_mission_v1',
            '-p', 'status_schema:=muto_command_bag_status_v1',
            '-p', 'recording_label:=command_mission',
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
        env=process_environment,
    )
    try:
        wait_until(
            lambda: node.count_subscribers(
                '/model_commander/recording_event') >= 1)
        start = String()
        start.data = json.dumps({
            'schema': 'muto_command_lifecycle_v1',
            'event': 'mission_started',
            'goal_id': 'abcdef0123456789',
            'objective': 'green office chair',
            'model': 'test-model',
        }, separators=(',', ':'))
        lifecycle_publisher.publish(start)

        def recording_ready():
            executor.spin_once(timeout_sec=0.05)
            return any(
                item.get('event') == 'recording_ready'
                for item in status_messages)

        wait_until(recording_ready)
        ready = next(
            item for item in status_messages
            if item.get('event') == 'recording_ready')
        bag_path = Path(ready['bag_path'])

        trace = String()
        trace.data = json.dumps({
            'schema': 'muto_command_decision_trace_v1',
            'event': 'planning_decision',
            'mission_id': 'abcdef0123456789',
            'planning_step': 1,
            'state': {'confirmed_object_count': 2},
            'decision': 'observe',
            'reason': 'inspect the current room',
        }, separators=(',', ':'))
        image = CompressedImage()
        image.header.stamp = node.get_clock().now().to_msg()
        image.header.frame_id = 'camera_color_optical_frame'
        image.format = 'jpeg'
        image.data = b'\xff\xd8synthetic-jpeg\xff\xd9'
        for _ in range(3):
            decision_publisher.publish(trace)
            image_publisher.publish(image)
            executor.spin_once(timeout_sec=0.05)

        terminal = String()
        terminal.data = json.dumps({
            'schema': 'muto_command_lifecycle_v1',
            'event': 'succeeded',
            'goal_id': 'abcdef0123456789',
            'outcome': 1,
        }, separators=(',', ':'))
        lifecycle_publisher.publish(terminal)

        def recording_finalized():
            executor.spin_once(timeout_sec=0.05)
            return any(
                item.get('event') == 'recording_finalized'
                for item in status_messages)

        wait_until(recording_finalized)
    finally:
        os.killpg(process.pid, signal.SIGINT)
        try:
            output, _ = process.communicate(timeout=8.0)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            output, _ = process.communicate(timeout=5.0)
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        assert process.returncode == 0, output

    assert bag_path.name.startswith('muto_command_')
    manifest = json.loads(
        (bag_path / 'muto_recording_manifest.json').read_text(
            encoding='utf-8'))
    assert manifest['muto_schema'] == 'command_mission_v1'
    assert manifest['recording_label'] == 'command_mission'
    assert 'green office chair' in manifest['start_event']

    info = subprocess.run(
        ['ros2', 'bag', 'info', str(bag_path)],
        check=True,
        capture_output=True,
        text=True,
        env=process_environment,
    ).stdout
    assert '/model_commander/recording_event' in info
    assert '/model_commander/decision_event' in info
    assert '/model_commander/inspected_image' in info
