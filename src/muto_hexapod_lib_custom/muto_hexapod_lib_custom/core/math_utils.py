"""Coordinate transforms used by the Muto leg model."""

import math

from .base import point3d


SIN30 = 0.5
COS30 = 0.866
SIN45 = 0.7071
COS45 = 0.7071
SIN15 = 0.2588
COS15 = 0.9659


def rotate0(source):
    return point3d(source.x, source.y, source.z)


def rotate45(source):
    return point3d(
        source.x * COS45 - source.y * SIN45,
        source.x * SIN45 + source.y * COS45,
        source.z,
    )


def rotate135(source):
    return point3d(
        source.x * -COS45 - source.y * SIN45,
        source.x * SIN45 + source.y * -COS45,
        source.z,
    )


def rotate180(source):
    return point3d(-source.x, -source.y, source.z)


def rotate225(source):
    return point3d(
        source.x * -COS45 + source.y * SIN45,
        source.x * -SIN45 + source.y * -COS45,
        source.z,
    )


def rotate315(source):
    return point3d(
        source.x * COS45 + source.y * SIN45,
        source.x * -SIN45 + source.y * COS45,
        source.z,
    )


def rotate_vector(source, angle_degrees):
    angle = math.radians(angle_degrees)
    return (
        source[0] * math.cos(angle) - source[1] * math.sin(angle),
        source[0] * math.sin(angle) + source[1] * math.cos(angle),
    )
