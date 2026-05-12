from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Pose:
    """Planar pose.

    ``theta`` is in radians: ``0`` faces north (positive *y*), increasing
    counterclockwise when viewed from above (*x* right, *y* up). West is *π/2*,
    south *π*, east *3π/2* (or *-π/2*).
    """

    x: float
    y: float
    theta: float


def pose(x: float, y: float, theta: float) -> Pose:
    """Build a 2D pose from position ``(x, y)`` and heading ``theta`` (radians, CCW from north)."""
    return Pose(x=x, y=y, theta=theta)
