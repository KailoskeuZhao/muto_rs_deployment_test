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

from yahboomcar_imu import imu_node


class FakeLogger:
    def __init__(self):
        self.warnings = []

    def info(self, message):
        del message

    def warn(self, message, **_kwargs):
        self.warnings.append(message)


class FakeNode:
    def __init__(self):
        self.logger = FakeLogger()

    def get_logger(self):
        return self.logger


class UnresponsiveMuto:
    def __init__(self):
        self.calls = 0
        self.last_read_diagnostic = 'no controller bytes'

    def read_IMU_Raw(self):
        self.calls += 1
        return None


class AdvancingClock:
    def __init__(self):
        self.now = 0.0

    def monotonic(self):
        current = self.now
        self.now += 1.0
        return current

    def sleep(self, duration):
        self.now += duration


def test_calibration_defaults_are_bounded():
    assert imu_node.DEFAULT_CALIBRATION_SAMPLE_COUNT == 300
    assert imu_node.DEFAULT_CALIBRATION_MAX_READS == 600
    assert imu_node.DEFAULT_CALIBRATION_TIMEOUT_SEC == 30.0


def test_calibration_stops_at_wall_clock_limit(monkeypatch):
    publisher = imu_node.ImuPublisher.__new__(imu_node.ImuPublisher)
    publisher.node = FakeNode()
    publisher.muto = UnresponsiveMuto()
    publisher.calibration_sample_count = 300
    publisher.calibration_max_reads = 600
    publisher.calibration_timeout_sec = 3.0
    publisher.calibration_read_interval = 0.005
    clock = AdvancingClock()
    monkeypatch.setattr(imu_node.time, 'monotonic', clock.monotonic)
    monkeypatch.setattr(imu_node.time, 'sleep', clock.sleep)

    publisher.calibrate_from_startup_samples()

    assert publisher.muto.calls == 1
    assert any(
        'wall-clock timeout reached' in warning
        for warning in publisher.node.logger.warnings
    )
