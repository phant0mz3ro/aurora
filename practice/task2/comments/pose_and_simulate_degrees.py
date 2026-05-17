#!/usr/bin/env python3
"""Work in degrees for heading; convert to radians for AuroraMR.

Convention matches the library: **0°** faces north (+y), angles increase **counterclockwise**
(west = 90°, south = 180°, east = 270° or −90°).

The library’s :func:`amr.pose` expects ``theta`` in **radians** — use
``math.radians`` (or :func:`pose_deg` below) before calling it.
"""

from __future__ import annotations

import math

import AuroraMR as amr
import matplotlib.pyplot as plt


#returns the pose vector given the inital state
def pose_deg(x: float, y: float, theta_degrees: float) -> amr.Pose:
    """Build a pose with heading ``theta_degrees`` in degrees (north = 0°, CCW)."""
    return amr.pose(x, y, math.radians(theta_degrees))


# Example poses
p_north = pose_deg(0.0, 0.0, 0.0)
p_east = pose_deg(1.0, 0.0, 270.0)
p_custom = pose_deg(-0.5, 2.0, 45.0)

print("North-facing:", p_north, "theta (rad) =", p_north.theta)
print("East-facing:", p_east, "theta (rad) ≈", round(p_east.theta, 4))
print("45° from north:", p_custom)
print("Read back as degrees:", math.degrees(p_custom.theta))

#plots
fig, ax = plt.subplots(figsize=(6, 6))
fig.suptitle("Heading 45° CCW from north (degrees → radians inside pose_deg)")
amr.simulate(p_custom, ax=ax, show=False) #run simmulation
plt.show()
