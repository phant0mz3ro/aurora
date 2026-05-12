from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class KinematicsModel(str, Enum):
    # Planar two-wheel robot: (v, ω) kinematics; left/right tire traces use track_width.
    TWO_WHEEL = "two_wheel"
    UNICYCLE = "two_wheel"  # legacy alias — same model
    DIFFERENTIAL = "differential"
    # Four wheels; rear fixed; front steers (Ackermann). BICYCLE is legacy same as ACKERMANN.
    ACKERMANN = "ackermann"
    MECANUM = "mecanum"
    BICYCLE = "bicycle"


KinematicsKind = KinematicsModel  # backward-compatible alias


@dataclass
class UnicycleParams:
    """Two-wheel robot: unicycle *(v, ω)* kinematics; wheels drawn at ±track_width/2 (m)."""

    track_width: float = 0.4
    max_linear_speed: float = 1.0
    max_angular_speed: float = 2.0


TwoWheelParams = UnicycleParams  # preferred name for the two-wheel / (v, ω) model


@dataclass
class DifferentialParams:
    """Differential drive: wheel linear speeds (m/s at ground contact)."""

    track_width: float = 0.4
    wheel_radius: float = 0.1
    max_wheel_speed: float = 2.0


@dataclass
class AckermannParams:
    """Four-wheel Ackermann: pose at rear axle; only front wheels steer."""

    wheelbase: float = 0.55
    track_width: float = 0.35
    max_steering_angle: float = 0.55
    max_speed: float = 1.2


# Backward compatibility (older name + ``rear_track_width``).
@dataclass
class BicycleParams:
    """Deprecated: use :class:`AckermannParams` with ``track_width``."""

    wheelbase: float = 0.55
    rear_track_width: float = 0.35
    max_steering_angle: float = 0.55
    max_speed: float = 1.2

    def to_ackermann(self) -> AckermannParams:
        return AckermannParams(
            wheelbase=self.wheelbase,
            track_width=self.rear_track_width,
            max_steering_angle=self.max_steering_angle,
            max_speed=self.max_speed,
        )


@dataclass
class MecanumParams:
    """X-config mecanum; robot pose at geometric center.

    ``half_length_y`` is distance from center to front/back wheels along forward (+y body).
    ``half_width_x`` is distance from center to left/right wheels along +x body (right).
    """

    half_length_y: float = 0.25
    half_width_x: float = 0.2
    max_wheel_speed: float = 2.0
