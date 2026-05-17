#!/usr/bin/env python3
"""Differential drive: wheel-based moves and dotted left/right tire paths."""

from __future__ import annotations

import math
import os

import matplotlib.pyplot as plt

import AuroraMR as amr #AuroraMR LIbrary

os.environ.setdefault("MPLBACKEND", "Agg")

params = amr.DifferentialParams(track_width=0.4, wheel_radius=0.08, max_wheel_speed=2.0) #model variables
session = amr.MotionSession.create( #creating the motion model with our parameters
    amr.pose(0.0, 0.0, 0.0),
    amr.KinematicsModel.DIFFERENTIAL, #differential model
    dt=0.012,
    differential=params,
)

#motion commands
session.forward_wheels(1.2, 1.0) #both wheels
session.turn_right(math.pi / 3, 1.2)
session.differential_drive_wheels(0.3, 1.2, duration=1.5) #left and right wheel control
session.forward(0.8, 0.5)

#plot
fig, ax = plt.subplots(figsize=(8, 8))
amr.plot_motion(session, ax=ax, show=False)
fig.savefig(os.path.join(os.path.dirname(__file__), "motion_differential_demo.png"), dpi=150)
print("Saved motion_differential_demo.png")
