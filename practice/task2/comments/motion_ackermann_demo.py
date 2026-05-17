#!/usr/bin/env python3
"""Four-wheel Ackermann: rear axle pose, front wheels steer; four dotted tire traces."""

from __future__ import annotations

import math
import os

import matplotlib.pyplot as plt

import AuroraMR as amr #AuroraMR Library

os.environ.setdefault("MPLBACKEND", "Agg") #tells matplotlib what rendering backend to use.
#Agg is non-interactive and renders to image files without running a GUI

params = amr.AckermannParams( #variables and constraints for our model
    wheelbase=0.55,
    track_width=0.36,
    max_steering_angle=0.5,
    max_speed=1.0,
)
session = amr.MotionSession.create( #creating a motion session with our defined conditions
    amr.pose(0.0, 0.0, 0.0),
    amr.KinematicsModel.ACKERMANN, #Ackermann drive model
    dt=0.02,
    ackermann=params,
)

#motion commands
session.forward(2.0, 0.6)
session.turn_left(math.radians(35), 0.8)
session.forward(1.5, 0.5)

#initiating plot
fig, ax = plt.subplots(figsize=(8, 8))
amr.plot_motion(session, ax=ax, show=False) #plots the motion session on the axes
fig.savefig(os.path.join(os.path.dirname(__file__), "motion_ackermann_demo.png"), dpi=150) #feature for saving the plot
print("Saved motion_ackermann_demo.png")
