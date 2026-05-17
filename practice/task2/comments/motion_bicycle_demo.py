#!/usr/bin/env python3
"""Legacy alias: use :class:`AckermannParams` and :attr:`KinematicsModel.ACKERMANN` instead."""

from __future__ import annotations

import math
import os

import matplotlib.pyplot as plt

import AuroraMR as amr #AuroraMR Library

os.environ.setdefault("MPLBACKEND", "Agg") #tells matplotlib what rendering backend to use.

params = amr.BicycleParams( #variables and constraints for the bicycle model
    wheelbase=0.55,
    rear_track_width=0.36,
    max_steering_angle=0.5,
    max_speed=1.0,
)
session = amr.MotionSession.create( #create the motion model with our defined parameters
    amr.pose(0.0, 0.0, 0.0),
    amr.KinematicsModel.BICYCLE,
    dt=0.02,
    bicycle=params,
)

#motion commands
session.forward(2.0, 0.6)
session.turn_left(math.radians(40), 0.8)
session.forward(1.5, 0.5)

#creating plot
fig, ax = plt.subplots(figsize=(8, 8))
amr.plot_motion(session, ax=ax, show=False) #doesn't display the plot
fig.savefig(os.path.join(os.path.dirname(__file__), "motion_bicycle_demo.png"), dpi=150)
#saves it into a file
print("Saved motion_bicycle_demo.png (BicycleParams → Ackermann four-wheel)")
