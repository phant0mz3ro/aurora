#!/usr/bin/env python3
"""Several poses on one axes or on separate subplots.

Each call to :func:`amr.simulate` sets axis limits around that pose. If you draw
more than one robot on the same ``ax``, set limits yourself so everyone stays
in view.
"""

from __future__ import annotations

import math

import matplotlib.pyplot as plt

import AuroraMR as amr #Aurora Library

poses = [ #list of position vectors
    amr.pose(0.0, 0.0, 0.0),  # north
    amr.pose(2.5, 1.0, math.pi / 2),  # west
    amr.pose(-1.0, 2.0, math.pi),  # south
]

# --- Same figure, one axes, multiple robots ---
fig1, ax1 = plt.subplots(figsize=(8, 8))
#multiple plots
for p in poses:
    amr.simulate(p, ax=ax1, show=False, length=0.45, width=0.28)

xs = [p.x for p in poses] #x values
ys = [p.y for p in poses] #y values
pad = 1.2

#plot customization
ax1.set_xlim(min(xs) - pad, max(xs) + pad)
ax1.set_ylim(min(ys) - pad, max(ys) + pad)
ax1.set_title("Three robots on one axes (limits set manually)")
fig1.tight_layout()

# --- One subplot per pose ---
fig2, axes = plt.subplots(1, len(poses), figsize=(4 * len(poses), 4))
for ax, p in zip(axes, poses):
    amr.simulate(p, ax=ax, show=False)
    ax.set_title(f"({p.x:.1f}, {p.y:.1f}), θ={p.theta:.2f}")
fig2.suptitle("Same poses in separate subplots")
fig2.tight_layout()

plt.show()
