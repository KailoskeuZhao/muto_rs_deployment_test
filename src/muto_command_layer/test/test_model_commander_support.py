"""Unit tests for the model commander's extracted support modules."""

from pathlib import Path
import re
import sys
from types import SimpleNamespace

import pytest


SCRIPTS = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(SCRIPTS))

from model_commander_config import (  # noqa: E402
    PARAMETER_DEFAULTS,
    read_parameters,
    validate_parameters,
)
from model_commander_memory import (  # noqa: E402
    angle_delta,
    pose_delta_context,
    primitive_memory_entry,
)


class FakeParameterNode:
    """Minimal parameter reader used without creating a ROS context."""

    def __init__(self, overrides=None):
        """Build a default configuration with optional test overrides."""
        self._values = dict(PARAMETER_DEFAULTS)
        self._values.update(overrides or {})

    def get_parameter(self, name):
        """Return one value using the rclpy parameter result shape."""
        return SimpleNamespace(value=self._values[name])


def test_parameter_defaults_have_unique_names_and_validate():
    """The centralized default table must be unambiguous and valid."""
    names = [name for name, _ in PARAMETER_DEFAULTS]
    assert len(names) == len(set(names))

    config = FakeParameterNode()
    read_parameters(config)
    validate_parameters(config)


@pytest.mark.parametrize(
    ('override', 'message'),
    [
        ({'action_name': ''}, 'action_name must not be empty'),
        ({'input_worker_poll_period': 0.0}, 'must be finite and positive'),
        ({'visual_observation_jpeg_quality': 101}, 'must be in [1, 100]'),
        (
            {'command_bag_enabled': False, 'command_bag_required': True},
            'cannot be true',
        ),
    ],
)
def test_parameter_contract_rejects_unsafe_values(override, message):
    """Invalid resource bounds and cross-field choices fail at startup."""
    config = FakeParameterNode(override)
    read_parameters(config)
    with pytest.raises(ValueError, match=re.escape(message)):
        validate_parameters(config)


def test_pose_delta_wraps_yaw_and_measures_translation():
    """Mission memory reports metric travel and wrapped yaw changes."""
    start = {'x': 1.0, 'y': 2.0, 'z': 0.0, 'yaw_rad': 3.10}
    end = {'x': 1.3, 'y': 2.4, 'z': 0.1, 'yaw_rad': -3.10}

    delta = pose_delta_context(start, end)

    assert delta['distance_xy'] == pytest.approx(0.5)
    assert delta['dyaw_rad'] == pytest.approx(
        angle_delta(-3.10, 3.10), abs=1e-6)
    assert abs(delta['dyaw_rad']) < 0.1


def test_primitive_memory_is_bounded_and_omits_empty_optional_fields():
    """Primitive records stay prompt-safe and omit irrelevant fields."""
    entry = primitive_memory_entry(
        8,
        'rotate',
        'completed',
        '0123456789',
        4,
        {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw_rad': 0.0},
        {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw_rad': 1.0},
        requested_rotation_radians=1.0,
        unused=None,
    )

    assert entry['message'] == '01234567'
    assert entry['delta_pose']['dyaw_rad'] == pytest.approx(1.0)
    assert entry['requested_rotation_radians'] == pytest.approx(1.0)
    assert 'unused' not in entry
