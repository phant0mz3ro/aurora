#!/usr/bin/env python3
"""Minimal live Ackermann demo: 0.5× playback speed + terminal logging.

Run from the repo root::

    PYTHONPATH=. python examples/live_ackermann_quick.py

Requires a GUI matplotlib backend to see the window.
"""

from __future__ import annotations #skips parameter type verification at runtime | escapes NameError possibilities

import AuroraMR as amr #AuroraMR library

if __name__ == "__main__":
    amr.play_motion_by_kind( 
        #imports the function play_motion_by_kind from the live_motion module in the amr folder

        "ackermann", # refernces the function _demo_ackermann
        interval_ms=28,
        playback_speed=0.5,
        show=True,
        log=True,
        log_every_n_frames=10, #prints the object's state at every step
        log_detailed=False,
    )

"""
the demo_ackermann has already been given predefined kinematics
pose(0,0,0)
dt = 0.02
s.forward(1.5,0.55)
s.turn_left(math.radians(40),0.75)
s.forward(1.0,0.45)
"""