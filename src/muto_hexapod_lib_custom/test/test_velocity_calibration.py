from dataclasses import FrozenInstanceError
import math

from muto_hexapod_lib_custom.movement.velocity_calibration import (
    MotionLevels,
    VelocityCalibrationMapper,
    VelocityCalibrationProfile,
)
import pytest


def profile_mapping():
    return {
        'schema_version': 1,
        'profile_id': 'test-bench-v1',
        'provenance': 'measured on the test bench',
        'reference_phase_rate_hz': 50.0,
        'x': {
            'positive': {1: 0.01, 30: 0.30},
            'negative': {1: 0.008, 30: 0.24},
        },
        'y': {
            'positive': {1: 0.005, 30: 0.15},
            'negative': {1: 0.004, 30: 0.12},
        },
        'yaw': {
            'positive': {10: 0.20, 20: 0.60},
            'negative': {10: 0.15, 20: 0.45},
        },
    }


def mapper(phase_rate_hz=None):
    profile = VelocityCalibrationProfile.from_mapping(profile_mapping())
    return VelocityCalibrationMapper(profile, phase_rate_hz=phase_rate_hz)


def test_profile_is_immutable_and_preserves_identity_metadata():
    profile = VelocityCalibrationProfile.from_mapping(profile_mapping())

    assert profile.schema_version == 1
    assert profile.profile_id == 'test-bench-v1'
    assert profile.provenance == 'measured on the test bench'
    assert profile.reference_phase_rate_hz == 50.0
    with pytest.raises(FrozenInstanceError):
        profile.profile_id = 'modified'


@pytest.mark.parametrize(
    'update, message',
    (
        ({'profile_id': ''}, 'profile_id'),
        ({'provenance': ''}, 'provenance'),
        ({'reference_phase_rate_hz': math.nan}, 'finite'),
        ({'reference_phase_rate_hz': 0.0}, 'positive'),
        ({'schema_version': 2}, 'schema_version'),
        ({'schema_version': '1'}, 'schema_version'),
        ({'schema_version': True}, 'schema_version'),
    ),
)
def test_profile_rejects_invalid_metadata(update, message):
    value = profile_mapping()
    value.update(update)

    with pytest.raises(ValueError, match=message):
        VelocityCalibrationProfile.from_mapping(value)


def test_profile_requires_schema_version():
    value = profile_mapping()
    del value['schema_version']

    with pytest.raises(ValueError, match='missing schema_version'):
        VelocityCalibrationProfile.from_mapping(value)


def test_profile_rejects_non_finite_non_monotone_and_out_of_range_tables():
    non_finite = profile_mapping()
    non_finite['x']['positive'][10] = math.inf
    with pytest.raises(ValueError, match='finite'):
        VelocityCalibrationProfile.from_mapping(non_finite)

    non_monotone = profile_mapping()
    non_monotone['x']['positive'] = {1: 0.02, 10: 0.01}
    with pytest.raises(ValueError, match='monotone'):
        VelocityCalibrationProfile.from_mapping(non_monotone)

    bad_linear_level = profile_mapping()
    bad_linear_level['y']['negative'] = {1: 0.01, 31: 0.20}
    with pytest.raises(ValueError, match=r'inside \[1, 30\]'):
        VelocityCalibrationProfile.from_mapping(bad_linear_level)

    bad_yaw_level = profile_mapping()
    bad_yaw_level['yaw']['positive'] = {9: 0.10, 20: 0.60}
    with pytest.raises(ValueError, match=r'inside \[10, 20\]'):
        VelocityCalibrationProfile.from_mapping(bad_yaw_level)


def test_inverse_selection_interpolates_only_integer_levels():
    result = mapper().select(linear_x_m_s=0.146)

    # x calibration is 0.01 m/s per integer level.
    assert result.levels == MotionLevels(x_level=15)
    assert result.mode == 'move_x'
    assert math.isclose(result.predicted.linear_x_m_s, 0.15)
    assert not result.saturated
    assert result.projected
    assert result.quantized
    assert result.projection.x
    assert not result.projection.x_to_zero
    assert 'nearest calibrated integer level' in result.detail
    assert result.profile_id == 'test-bench-v1'


def test_positive_and_negative_directions_use_different_curves():
    positive = mapper().select(linear_x_m_s=0.10)
    negative = mapper().select(linear_x_m_s=-0.10)

    assert positive.levels.x_level == 10
    # Negative x is calibrated at 0.008 m/s per level.
    assert negative.levels.x_level == -13
    assert math.isclose(negative.predicted.linear_x_m_s, -0.104)


def test_zero_is_an_implicit_candidate_below_the_minimum_level():
    value = profile_mapping()
    value['x']['positive'] = {5: 0.05, 30: 0.30}
    local_mapper = VelocityCalibrationMapper(
        VelocityCalibrationProfile.from_mapping(value))

    below_half = local_mapper.select(linear_x_m_s=0.024)
    above_half = local_mapper.select(linear_x_m_s=0.026)
    exact_tie = local_mapper.select(linear_x_m_s=0.025)

    assert below_half.levels.x_level == 0
    assert below_half.mode == 'standby'
    assert below_half.projected
    assert below_half.quantized
    assert below_half.projection.x
    assert below_half.projection.x_to_zero
    assert not below_half.saturated
    assert 'projected to zero' in below_half.detail
    assert above_half.levels.x_level == 5
    assert above_half.projection.x
    assert not above_half.projection.x_to_zero
    # Prefer less actuation when two executable levels are equally close.
    assert exact_tie.levels.x_level == 0
    assert exact_tie.projection.x_to_zero


def test_request_above_curve_clamps_without_extrapolation_and_reports_axis():
    result = mapper().select(
        linear_x_m_s=0.80,
        angular_z_rad_s=-1.00,
    )

    assert result.levels == MotionLevels(x_level=30, z_level=-20)
    assert result.predicted.linear_x_m_s == 0.30
    assert result.predicted.angular_z_rad_s == -0.45
    assert result.saturation.x
    assert result.saturation.yaw
    assert result.saturated
    assert result.projected
    assert result.projection.x
    assert result.projection.yaw
    assert not result.projection.any_to_zero
    assert 'x/yaw' in result.detail


def test_x_and_yaw_are_selected_independently_for_composed_mixed_gait():
    result = mapper().select(
        linear_x_m_s=-0.10,
        angular_z_rad_s=0.36,
    )

    assert result.levels == MotionLevels(x_level=-13, z_level=14)
    assert result.mode == 'move_xz'
    assert math.isclose(result.predicted.linear_x_m_s, -0.104)
    assert math.isclose(result.predicted.angular_z_rad_s, 0.36)
    assert result.supported
    assert not result.saturated
    assert result.quantized
    assert result.projection.x
    assert not result.projection.yaw


def test_lateral_combination_returns_safe_unsupported_selection():
    result = mapper().select(
        linear_x_m_s=0.10,
        linear_y_m_s=0.05,
        angular_z_rad_s=0.20,
    )

    assert not result.supported
    assert result.mode == 'standby'
    assert result.levels == MotionLevels()
    assert result.predicted.linear_x_m_s == 0.0
    assert result.saturation.unsupported_combination
    assert not result.saturated
    assert result.projected
    assert not result.quantized
    assert result.projection.x
    assert result.projection.y
    assert result.projection.yaw
    assert not result.projection.any_to_zero
    assert result.unsupported_reason
    assert 'lateral' in result.detail


def test_reference_phase_rate_is_a_required_calibration_condition():
    with pytest.raises(ValueError, match='does not match profile'):
        mapper(phase_rate_hz=25.0)

    result = mapper(phase_rate_hz=50.0).select(linear_x_m_s=0.10)
    assert result.levels.x_level == 10
    assert math.isclose(result.predicted.linear_x_m_s, 0.10)
    assert result.phase_rate_hz == 50.0
    assert not result.projected


def test_raw_prediction_is_strict_and_never_extrapolates():
    local_mapper = mapper()

    predicted = local_mapper.predict_levels(x_level=12, z_level=-15)
    assert math.isclose(predicted.linear_x_m_s, 0.12)
    assert math.isclose(predicted.angular_z_rad_s, -0.30)

    with pytest.raises(ValueError, match='outside calibrated interval'):
        local_mapper.predict_levels(z_level=9)
    with pytest.raises(ValueError, match='cannot be combined'):
        local_mapper.predict_levels(x_level=10, y_level=10)


@pytest.mark.parametrize(
    'velocity_request',
    (
        {'linear_x_m_s': math.nan},
        {'linear_y_m_s': math.inf},
        {'angular_z_rad_s': -math.inf},
    ),
)
def test_non_finite_velocity_request_is_rejected(velocity_request):
    with pytest.raises(ValueError, match='finite'):
        mapper().select(**velocity_request)
