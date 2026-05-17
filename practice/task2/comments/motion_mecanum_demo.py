#!/usr/bin/env python3
"""Mecanum X-config: omnidirectional motion and four wheel traces (FL, FR, RL, RR)."""

from __future__ import annotations

import math
import os

import matplotlib.pyplot as plt

import AuroraMR as amr #AuroraMR Library

os.environ.setdefault("MPLBACKEND", "Agg")

params = amr.MecanumParams(half_length_y=0.28, half_width_x=0.22, max_wheel_speed=2.5) #wheel variables and constraints
session = amr.MotionSession.create( #creating the model with our parameters
    amr.pose(0.0, 0.0, 0.0), #postion
    amr.KinematicsModel.MECANUM, #mecanum model
    dt=0.015, #time step
    mecanum=params,
)
#motion commands
session.forward(1.0, 0.7)
session.strafe_right(0.8, 0.6) #move righ
session.turn_left(math.pi / 3, 1.0)
session.mecanum_drive_wheels(0.5, -0.2, 0.0, 0.8, duration=1.0) #independent control

#inverse kinematics
goal = amr.pose(1.5, 0.5, math.pi / 6)
session.drive_to_pose(goal, linear_speed=0.6, angular_speed=1.0, position_tol=0.1, angle_tol=0.12)

#plots
fig, ax = plt.subplots(figsize=(8, 8))
amr.plot_motion(session, ax=ax, show=False)
fig.savefig(os.path.join(os.path.dirname(__file__), "motion_mecanum_demo.png"), dpi=150)
print("Saved motion_mecanum_demo.png")
