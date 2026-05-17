#!/usr/bin/env python3
"""Create poses and read ``x``, ``y``, ``theta``.

Heading convention: ``theta`` is in radians; ``0`` faces north (+y); increasing
counterclockwise (west = π/2, east = 3π/2 or −π/2).

Using ``import AuroraMR as amr`` does not load matplotlib until you call
``amr.simulate``. You can also use ``import amr`` or ``from amr.pose import Pose, pose``.
"""

from __future__ import annotations

import math

import AuroraMR as amr

# Preferred: factory function
p = amr.pose(1.5, -0.5, math.pi / 6) #(x,y,theta)
print("pose() ->", p)
print("  x =", p.x, "  y =", p.y, "  theta (rad) =", p.theta)

# Instantiate ``Pose`` directly if you already have values
q = amr.Pose(x=0.0, y=0.0, theta=0.0)
print("Pose(...) ->", q)

# Common headings
north = amr.pose(0, 0, 0)
west = amr.pose(0, 0, math.pi / 2)
south = amr.pose(0, 0, math.pi)
east = amr.pose(0, 0, 3 * math.pi / 2)
print("north theta=0:", north.theta, " west theta=π/2:", west.theta)
