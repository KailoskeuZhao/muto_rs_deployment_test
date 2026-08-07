# Copyright 2026 kailoskeuzhao
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from builtin_interfaces.msg import Time

from yahboomcar_imu import imu_node


class FakeLogger:
    def __init__(self):
        self.warnings = []

    def warn(self, message, **_kwargs):
        self.warnings.append(message)


class FakeNow:
    def to_msg(self):
        return Time(sec=12, nanosec=34)


class FakeClock:
    def now(self):
        return FakeNow()


class FakeNode:
    def __init__(self):
        self.logger = FakeLogger()

    def get_logger(self):
        return self.logger

    def get_clock(self):
        return FakeClock()


class RecordingPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class SequenceMuto:
    last_response_type = imu_node.CONTROLLER_DATA_RETURN_TYPE

    def __init__(self, samples):
        self.samples = iter(samples)
        self.timeouts = []

    def read_IMU(self, response_timeout=None):
        self.timeouts.append(response_timeout)
        return next(self.samples)


def make_publisher(samples):
    publisher = imu_node.ImuPublisher.__new__(imu_node.ImuPublisher)
    publisher.node = FakeNode()
    publisher.muto = SequenceMuto(samples)
    publisher.controller_attitude_publisher = RecordingPublisher()
    publisher.response_timeout_sec = imu_node.DEFAULT_RESPONSE_TIMEOUT_SEC
    publisher.attitude_poll_count = 0
    publisher.successful_attitude_read_count = 0
    publisher.attitude_skipped_for_locomotion_count = 0
    return publisher


def test_controller_attitude_preserves_vendor_euler_and_temperature():
    publisher = make_publisher(((-12.34, 0.25, 179.99, 42),))

    assert publisher.publish_controller_attitude(
        response_timeout_sec=0.006
    ) is True

    message = publisher.controller_attitude_publisher.messages[0]
    assert message.header.stamp == Time(sec=12, nanosec=34)
    assert message.header.frame_id == ''
    assert message.roll_deg == -12.34
    assert message.pitch_deg == 0.25
    assert message.yaw_deg == 179.99
    assert message.temperature_raw == 42
    assert publisher.muto.timeouts == [0.006]
    assert publisher.attitude_poll_count == 1
    assert publisher.successful_attitude_read_count == 1


def test_controller_attitude_retains_identical_successful_polls():
    sample = (1.0, 2.0, 3.0, 20)
    publisher = make_publisher((sample, sample))

    assert publisher.publish_controller_attitude() is True
    assert publisher.publish_controller_attitude() is True

    assert len(publisher.controller_attitude_publisher.messages) == 2
    assert publisher.attitude_poll_count == 2
    assert publisher.successful_attitude_read_count == 2


def test_controller_attitude_rejects_short_and_nonfinite_payloads():
    publisher = make_publisher(((1.0, 2.0), (1.0, 2.0, float('nan'), 20)))

    assert publisher.publish_controller_attitude() is False
    assert publisher.publish_controller_attitude() is False

    assert publisher.controller_attitude_publisher.messages == []
    assert publisher.attitude_poll_count == 2
    assert publisher.successful_attitude_read_count == 0
    assert len(publisher.node.logger.warnings) == 2
