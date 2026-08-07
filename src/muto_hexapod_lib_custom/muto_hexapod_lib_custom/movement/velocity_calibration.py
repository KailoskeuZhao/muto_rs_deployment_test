"""Map planar SI velocity requests to calibrated integer gait levels.

The low-level Muto gait API intentionally continues to accept raw integer
levels.  This module is a ROS-independent boundary in front of that API: a
calibration profile describes the physical speed measured at selected levels,
and :class:`VelocityCalibrationMapper` selects the closest executable level.

Calibration speeds are magnitudes.  Positive and negative motion have separate
curves so asymmetric hardware can be represented without hiding the sign in a
table value.  Zero is always an implicit candidate.  Values between calibration
knots are linearly interpolated at integer levels; values outside the
calibrated level interval are never extrapolated.

``reference_phase_rate_hz`` names the configured gait cadence under which the
profile was obtained.  It is a calibration condition, not a linear scale
factor: using the profile at a different configured cadence is rejected.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Mapping, Optional, Tuple


LINEAR_LEVEL_MIN = 1
LINEAR_LEVEL_MAX = 30
YAW_LEVEL_MIN = 10
YAW_LEVEL_MAX = 20
SUPPORTED_SCHEMA_VERSION = 1


def _finite_float(value, field_name):
    if isinstance(value, bool):
        raise ValueError('%s must be a finite number' % field_name)
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError('%s must be a finite number' % field_name) from error
    if not math.isfinite(result):
        raise ValueError('%s must be a finite number' % field_name)
    return result


def _positive_level(value, field_name):
    if isinstance(value, bool):
        raise ValueError('%s must be a positive integer' % field_name)
    if isinstance(value, int):
        result = value
    elif isinstance(value, str):
        try:
            result = int(value)
        except ValueError as error:
            raise ValueError(
                '%s must be a positive integer' % field_name) from error
        if str(result) != value.strip():
            raise ValueError('%s must be a positive integer' % field_name)
    else:
        raise ValueError('%s must be a positive integer' % field_name)
    if result <= 0:
        raise ValueError('%s must be a positive integer' % field_name)
    return result


def _raw_level(value, field_name):
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError('%s must be an integer' % field_name)
    return value


@dataclass(frozen=True)
class CalibrationPoint:
    """Physical speed magnitude at one positive raw level magnitude."""

    level: int
    speed: float

    def __post_init__(self):
        if isinstance(self.level, bool) or not isinstance(self.level, int):
            raise ValueError('calibration level must be a positive integer')
        if self.level <= 0:
            raise ValueError('calibration level must be a positive integer')
        speed = _finite_float(self.speed, 'calibration speed')
        if speed <= 0.0:
            raise ValueError('calibration speed must be positive')
        object.__setattr__(self, 'speed', speed)


@dataclass(frozen=True)
class CalibrationCurve:
    """Monotone calibration knots for one direction of one gait axis."""

    points: Tuple[CalibrationPoint, ...]

    def __post_init__(self):
        points = tuple(self.points)
        if not points:
            raise ValueError(
                'calibration curve must contain at least one point')
        if not all(isinstance(point, CalibrationPoint) for point in points):
            raise ValueError('calibration curve contains an invalid point')
        for previous, current in zip(points, points[1:]):
            if current.level <= previous.level:
                raise ValueError(
                    'calibration levels must be strictly increasing')
            if current.speed < previous.speed:
                raise ValueError(
                    'calibration speeds must be monotone non-decreasing')
        object.__setattr__(self, 'points', points)

    @property
    def minimum_level(self):
        return self.points[0].level

    @property
    def maximum_level(self):
        return self.points[-1].level

    def speed_at_level(self, level):
        """Interpolate within the calibrated interval, never beyond it."""
        level = _raw_level(level, 'level')
        if level < self.minimum_level or level > self.maximum_level:
            raise ValueError(
                'level %d is outside calibrated interval [%d, %d]'
                % (level, self.minimum_level, self.maximum_level))

        for point in self.points:
            if level == point.level:
                return point.speed

        for lower, upper in zip(self.points, self.points[1:]):
            if lower.level < level < upper.level:
                fraction = (
                    float(level - lower.level)
                    / float(upper.level - lower.level)
                )
                return lower.speed + fraction * (upper.speed - lower.speed)

        raise RuntimeError('calibration interval lookup failed')


@dataclass(frozen=True)
class AxisCalibration:
    """Separate positive and negative physical-speed curves for one axis."""

    positive: CalibrationCurve
    negative: CalibrationCurve

    def __post_init__(self):
        if not isinstance(self.positive, CalibrationCurve):
            raise ValueError('positive calibration must be a curve')
        if not isinstance(self.negative, CalibrationCurve):
            raise ValueError('negative calibration must be a curve')


@dataclass(frozen=True)
class VelocityCalibrationProfile:
    """Immutable, validated gait-level calibration profile."""

    schema_version: int
    profile_id: str
    provenance: str
    reference_phase_rate_hz: float
    x: AxisCalibration
    y: AxisCalibration
    yaw: AxisCalibration

    def __post_init__(self):
        if (isinstance(self.schema_version, bool)
                or not isinstance(self.schema_version, int)
                or self.schema_version != SUPPORTED_SCHEMA_VERSION):
            raise ValueError(
                'schema_version must be integer %d'
                % SUPPORTED_SCHEMA_VERSION)
        if not isinstance(self.profile_id, str) or not self.profile_id.strip():
            raise ValueError('profile_id must be a non-empty string')
        if not isinstance(self.provenance, str) or not self.provenance.strip():
            raise ValueError('provenance must be a non-empty string')
        phase_rate = _finite_float(
            self.reference_phase_rate_hz,
            'reference_phase_rate_hz',
        )
        if phase_rate <= 0.0:
            raise ValueError('reference_phase_rate_hz must be positive')
        object.__setattr__(self, 'reference_phase_rate_hz', phase_rate)

        for name, axis in (('x', self.x), ('y', self.y)):
            self._validate_axis(name, axis, LINEAR_LEVEL_MIN, LINEAR_LEVEL_MAX)
        self._validate_axis('yaw', self.yaw, YAW_LEVEL_MIN, YAW_LEVEL_MAX)

    @staticmethod
    def _validate_axis(name, axis, minimum_level, maximum_level):
        if not isinstance(axis, AxisCalibration):
            raise ValueError(
                '%s calibration must be an axis calibration' % name)
        for direction, curve in (
                ('positive', axis.positive), ('negative', axis.negative)):
            if (curve.minimum_level < minimum_level
                    or curve.maximum_level > maximum_level):
                raise ValueError(
                    '%s.%s levels must remain inside [%d, %d]'
                    % (name, direction, minimum_level, maximum_level))

    @classmethod
    def from_mapping(cls, value):
        """Build a profile from a plain mapping suitable for YAML/JSON data.

        Expected axis form::

            schema_version: 1
            profile_id: example
            provenance: measured on marked floor
            reference_phase_rate_hz: 50.0
            x:
              positive: {1: 0.01, 30: 0.30}
              negative: {1: 0.009, 30: 0.27}

        Table keys are positive raw level magnitudes and table values are
        positive SI speed magnitudes.  Yaw values use radians per second.
        """
        if not isinstance(value, Mapping):
            raise ValueError('calibration profile must be a mapping')
        try:
            schema_version = value['schema_version']
            profile_id = value['profile_id']
            provenance = value['provenance']
            reference_phase_rate_hz = value['reference_phase_rate_hz']
            x = _axis_from_mapping(value['x'], 'x')
            y = _axis_from_mapping(value['y'], 'y')
            yaw = _axis_from_mapping(value['yaw'], 'yaw')
        except KeyError as error:
            raise ValueError(
                'calibration profile is missing %s' % error.args[0]) from error
        return cls(
            schema_version=schema_version,
            profile_id=profile_id,
            provenance=provenance,
            reference_phase_rate_hz=reference_phase_rate_hz,
            x=x,
            y=y,
            yaw=yaw,
        )


def _curve_from_mapping(value, field_name):
    if not isinstance(value, Mapping) or not value:
        raise ValueError(
            '%s must be a non-empty level-to-speed mapping' % field_name)
    points = []
    levels = set()
    for raw_level, raw_speed in value.items():
        level = _positive_level(raw_level, '%s level' % field_name)
        if level in levels:
            raise ValueError(
                '%s contains duplicate level %d' % (field_name, level))
        levels.add(level)
        speed = _finite_float(raw_speed, '%s[%d]' % (field_name, level))
        points.append(CalibrationPoint(level=level, speed=speed))
    points.sort(key=lambda point: point.level)
    return CalibrationCurve(tuple(points))


def _axis_from_mapping(value, field_name):
    if not isinstance(value, Mapping):
        raise ValueError('%s must be a mapping' % field_name)
    try:
        positive = _curve_from_mapping(
            value['positive'], '%s.positive' % field_name)
        negative = _curve_from_mapping(
            value['negative'], '%s.negative' % field_name)
    except KeyError as error:
        raise ValueError(
            '%s is missing %s' % (field_name, error.args[0])) from error
    return AxisCalibration(positive=positive, negative=negative)


@dataclass(frozen=True)
class PlanarVelocity:
    linear_x_m_s: float
    linear_y_m_s: float
    angular_z_rad_s: float

    def __post_init__(self):
        object.__setattr__(
            self, 'linear_x_m_s',
            _finite_float(self.linear_x_m_s, 'linear_x_m_s'))
        object.__setattr__(
            self, 'linear_y_m_s',
            _finite_float(self.linear_y_m_s, 'linear_y_m_s'))
        object.__setattr__(
            self, 'angular_z_rad_s',
            _finite_float(self.angular_z_rad_s, 'angular_z_rad_s'))


@dataclass(frozen=True)
class MotionLevels:
    x_level: int = 0
    y_level: int = 0
    z_level: int = 0

    def __post_init__(self):
        for name in ('x_level', 'y_level', 'z_level'):
            _raw_level(getattr(self, name), name)


@dataclass(frozen=True)
class SaturationFlags:
    """Requests above the calibrated upper envelope, by physical axis."""

    x: bool = False
    y: bool = False
    yaw: bool = False
    unsupported_combination: bool = False

    @property
    def any_saturated(self):
        return self.x or self.y or self.yaw


@dataclass(frozen=True)
class ProjectionFlags:
    """Discrete-level projection, separate from upper-envelope saturation.

    An axis flag means its predicted speed differs from the requested speed.
    The ``*_to_zero`` flags identify the important sub-minimum case where zero
    was closer than the first calibrated executable level.
    """

    x: bool = False
    y: bool = False
    yaw: bool = False
    x_to_zero: bool = False
    y_to_zero: bool = False
    yaw_to_zero: bool = False

    @property
    def any_projected(self):
        return self.x or self.y or self.yaw

    @property
    def any_to_zero(self):
        return self.x_to_zero or self.y_to_zero or self.yaw_to_zero


@dataclass(frozen=True)
class VelocitySelection:
    """One executable raw-level selection and its physical interpretation."""

    requested: PlanarVelocity
    predicted: PlanarVelocity
    levels: MotionLevels
    mode: str
    saturation: SaturationFlags
    profile_id: str
    phase_rate_hz: float
    projection: ProjectionFlags = ProjectionFlags()
    detail: str = ''
    supported: bool = True
    unsupported_reason: Optional[str] = None

    @property
    def saturated(self):
        return self.saturation.any_saturated

    @property
    def projected(self):
        return self.projection.any_projected or self.saturated

    @property
    def quantized(self):
        """Whether a supported request maps to a different discrete speed."""
        return self.supported and self.projection.any_projected


@dataclass(frozen=True)
class _AxisSelection:
    level: int
    predicted: float
    saturated: bool
    projected: bool
    projected_to_zero: bool


class VelocityCalibrationMapper:
    """Select closest raw gait levels for a requested planar SI velocity."""

    def __init__(self, profile, phase_rate_hz=None):
        if not isinstance(profile, VelocityCalibrationProfile):
            raise ValueError('profile must be a VelocityCalibrationProfile')
        if phase_rate_hz is None:
            phase_rate_hz = profile.reference_phase_rate_hz
        phase_rate_hz = _finite_float(phase_rate_hz, 'phase_rate_hz')
        if phase_rate_hz <= 0.0:
            raise ValueError('phase_rate_hz must be positive')
        if not math.isclose(
                phase_rate_hz,
                profile.reference_phase_rate_hz,
                rel_tol=1e-9,
                abs_tol=1e-9):
            raise ValueError(
                'phase_rate_hz %.9g does not match profile calibration '
                'condition %.9g; use a profile calibrated for the configured '
                'phase rate'
                % (phase_rate_hz, profile.reference_phase_rate_hz))
        self._profile = profile
        self._phase_rate_hz = phase_rate_hz

    @property
    def profile(self):
        return self._profile

    @property
    def phase_rate_hz(self):
        return self._phase_rate_hz

    def select(self, linear_x_m_s=0.0, linear_y_m_s=0.0,
               angular_z_rad_s=0.0):
        requested = PlanarVelocity(
            linear_x_m_s,
            linear_y_m_s,
            angular_z_rad_s,
        )

        if (requested.linear_y_m_s != 0.0
                and (requested.linear_x_m_s != 0.0
                     or requested.angular_z_rad_s != 0.0)):
            reason = (
                'simultaneous lateral motion with forward or yaw motion is '
                'unsupported by the gait library'
            )
            return VelocitySelection(
                requested=requested,
                predicted=PlanarVelocity(0.0, 0.0, 0.0),
                levels=MotionLevels(),
                mode='standby',
                saturation=SaturationFlags(
                    unsupported_combination=True,
                ),
                profile_id=self._profile.profile_id,
                phase_rate_hz=self._phase_rate_hz,
                projection=ProjectionFlags(
                    x=requested.linear_x_m_s != 0.0,
                    y=True,
                    yaw=requested.angular_z_rad_s != 0.0,
                ),
                detail=reason,
                supported=False,
                unsupported_reason=reason,
            )

        x = self._select_axis(self._profile.x, requested.linear_x_m_s)
        y = self._select_axis(self._profile.y, requested.linear_y_m_s)
        yaw = self._select_axis(
            self._profile.yaw, requested.angular_z_rad_s)
        levels = MotionLevels(x.level, y.level, yaw.level)
        mode = self._mode_for_levels(levels)
        saturation = SaturationFlags(
            x=x.saturated,
            y=y.saturated,
            yaw=yaw.saturated,
        )
        projection = ProjectionFlags(
            x=x.projected,
            y=y.projected,
            yaw=yaw.projected,
            x_to_zero=x.projected_to_zero,
            y_to_zero=y.projected_to_zero,
            yaw_to_zero=yaw.projected_to_zero,
        )
        detail = self._selection_detail(saturation, projection)
        return VelocitySelection(
            requested=requested,
            predicted=PlanarVelocity(x.predicted, y.predicted, yaw.predicted),
            levels=levels,
            mode=mode,
            saturation=saturation,
            profile_id=self._profile.profile_id,
            phase_rate_hz=self._phase_rate_hz,
            projection=projection,
            detail=detail,
        )

    def predict_levels(self, x_level=0, y_level=0, z_level=0):
        """Predict SI velocity for raw levels inside calibrated intervals.

        This helper is strict: invalid mixed modes and levels beyond a curve's
        calibrated interval raise ``ValueError`` instead of extrapolating.
        """
        levels = MotionLevels(x_level, y_level, z_level)
        self._mode_for_levels(levels)
        return PlanarVelocity(
            self._predict_axis(self._profile.x, levels.x_level),
            self._predict_axis(self._profile.y, levels.y_level),
            self._predict_axis(self._profile.yaw, levels.z_level),
        )

    def _select_axis(self, axis, requested):
        if requested == 0.0:
            return _AxisSelection(0, 0.0, False, False, False)

        sign = 1 if requested > 0.0 else -1
        magnitude = abs(requested)
        curve = axis.positive if sign > 0 else axis.negative
        candidates = [(0, 0.0)]
        candidates.extend(
            (level, curve.speed_at_level(level))
            for level in range(curve.minimum_level, curve.maximum_level + 1)
        )
        level, predicted_magnitude = min(
            candidates,
            key=lambda candidate: (
                abs(candidate[1] - magnitude),
                candidate[0],
            ),
        )
        maximum_speed = candidates[-1][1]
        projected = not math.isclose(
            predicted_magnitude,
            magnitude,
            rel_tol=1e-12,
            abs_tol=1e-12,
        )
        return _AxisSelection(
            level=sign * level,
            predicted=sign * predicted_magnitude,
            saturated=magnitude > maximum_speed,
            projected=projected,
            projected_to_zero=projected and level == 0,
        )

    def _predict_axis(self, axis, signed_level):
        if signed_level == 0:
            return 0.0
        sign = 1 if signed_level > 0 else -1
        curve = axis.positive if sign > 0 else axis.negative
        return (
            sign
            * curve.speed_at_level(abs(signed_level))
        )

    @staticmethod
    def _mode_for_levels(levels):
        if levels.y_level and (levels.x_level or levels.z_level):
            raise ValueError(
                'raw y level cannot be combined with x or z levels')
        if levels.y_level:
            return 'move_y'
        if levels.x_level and levels.z_level:
            return 'move_xz'
        if levels.x_level:
            return 'move_x'
        if levels.z_level:
            return 'turn_z'
        return 'standby'

    @staticmethod
    def _selection_detail(saturation, projection):
        details = []
        saturated_axes = []
        if saturation.x:
            saturated_axes.append('x')
        if saturation.y:
            saturated_axes.append('y')
        if saturation.yaw:
            saturated_axes.append('yaw')
        if saturated_axes:
            details.append(
                'requested %s velocity exceeds calibrated range'
                % '/'.join(saturated_axes))

        zero_axes = []
        if projection.x_to_zero:
            zero_axes.append('x')
        if projection.y_to_zero:
            zero_axes.append('y')
        if projection.yaw_to_zero:
            zero_axes.append('yaw')
        if zero_axes:
            details.append(
                'requested %s velocity projected to zero below the minimum '
                'executable speed'
                % '/'.join(zero_axes))

        quantized_axes = []
        if projection.x and not saturation.x and not projection.x_to_zero:
            quantized_axes.append('x')
        if projection.y and not saturation.y and not projection.y_to_zero:
            quantized_axes.append('y')
        if (projection.yaw and not saturation.yaw
                and not projection.yaw_to_zero):
            quantized_axes.append('yaw')
        if quantized_axes:
            details.append(
                'requested %s velocity quantized to the nearest calibrated '
                'integer level'
                % '/'.join(quantized_axes))
        return '; '.join(details)


__all__ = (
    'AxisCalibration',
    'CalibrationCurve',
    'CalibrationPoint',
    'LINEAR_LEVEL_MAX',
    'LINEAR_LEVEL_MIN',
    'MotionLevels',
    'PlanarVelocity',
    'ProjectionFlags',
    'SaturationFlags',
    'SUPPORTED_SCHEMA_VERSION',
    'VelocityCalibrationMapper',
    'VelocityCalibrationProfile',
    'VelocitySelection',
    'YAW_LEVEL_MAX',
    'YAW_LEVEL_MIN',
)
