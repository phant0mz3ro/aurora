from __future__ import annotations

from typing import Literal

import matplotlib.pyplot as plt
import numpy as np

from .pose import Pose

Style = Literal["robot", "pointer"]


def _body_to_world(local_xy: np.ndarray, theta: float) -> np.ndarray:
    """Rotate local points into world frame; local +y is forward; θ=0 is north.

    θ is a counterclockwise rotation from north (+y): forward in world is
    ``(-sin θ, cos θ)`` (e.g. θ=π/4 is northwest, θ=π/2 is west, θ=3π/2 is east).
    """
    c, s = np.cos(theta), np.sin(theta)
    # Standard CCW rotation: local +y → (-sin θ, cos θ); local +x → (cos θ, sin θ)
    r = np.array([[c, -s], [s, c]], dtype=float)
    return (r @ local_xy.T).T


def _polygon_robot(
    length: float = 0.5,
    width: float = 0.3,
    head_fraction: float = 0.35,
) -> np.ndarray:
    """Rectangle body + triangular head; local +y is forward."""
    hw = width / 2
    hh = length * head_fraction
    # Rear at y=-L/2, neck at y=L/2-hh, tip at y=+L/2
    return np.array(
        [
            [-hw, -length / 2],
            [hw, -length / 2],
            [hw, length / 2 - hh],
            [0.0, length / 2],
            [-hw, length / 2 - hh],
        ],
        dtype=float,
    )


def _polygon_pointer(length: float = 0.55, width: float = 0.2) -> np.ndarray:
    """Arrow-like pointer; local +y is forward."""
    hw = width / 2
    return np.array(
        [
            [-hw, -length / 3],
            [hw, -length / 3],
            [0.0, 2 * length / 3],
        ],
        dtype=float,
    )


def simulate(
    p: Pose,
    *,
    style: Style = "robot",
    ax: plt.Axes | None = None,
    length: float = 0.5,
    width: float = 0.3,
    facecolor: str = "#2e7d32",
    edgecolor: str = "#1b5e20",
    show: bool = True,
) -> plt.Axes:
    """Plot the robot at ``p`` in the plane (*x* right, *y* up).

    ``p.theta`` is radians CCW from north: 0 = north, π/2 = west, π = south,
    3π/2 (or −π/2) = east.

    Parameters
    ----------
    p
        Pose from :func:`amr.pose`.
    style
        ``\"robot\"``: rounded body with triangular head; ``\"pointer\"``: simple arrowhead.
    ax
        Optional matplotlib axes; if omitted, a new figure is created.
    show
        If True, call ``plt.show()`` before returning (no-op when ``ax`` is provided).
    """
    if style == "robot":
        local = _polygon_robot(length=length, width=width)
    else:
        local = _polygon_pointer(length=length, width=width)

    xy = _body_to_world(local, p.theta)
    xy[:, 0] += p.x
    xy[:, 1] += p.y

    close = np.vstack([xy, xy[0]])
    created_fig = ax is None
    if ax is None:
        _, ax = plt.subplots(figsize=(6, 6))

    ax.fill(
        close[:, 0],
        close[:, 1],
        facecolor=facecolor,
        edgecolor=edgecolor,
        linewidth=1.5,
        zorder=5,
    )

    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.grid(True, alpha=0.3)
    margin = max(length, width) * 1.5
    ax.set_xlim(p.x - margin, p.x + margin)
    ax.set_ylim(p.y - margin, p.y + margin)

    if created_fig and show:
        plt.show()
    return ax
