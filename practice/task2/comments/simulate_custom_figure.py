#!/usr/bin/env python3
"""Use your own matplotlib figure: ``ax=`` and ``show=False``, then save or show.

For machines without a display, run with a non-GUI backend, e.g.:
  MPLBACKEND=Agg python simulate_custom_figure.py
"""

from __future__ import annotations

import math
import os

import matplotlib.pyplot as plt

import AuroraMR as amr #AuroraMR Library

# For a headless run: ``MPLBACKEND=Agg python simulate_custom_figure.py``

p = amr.pose(0.5, -1.0, math.radians(30)) #(x,y,theta)

#plots
fig, ax = plt.subplots(figsize=(7, 7))
fig.suptitle("Custom figure: AuroraMR simulate on supplied axes")

amr.simulate(p, ax=ax, show=True) #simulation
fig.tight_layout()
out = os.path.join(os.path.dirname(__file__), "custom_figure_example.png") #write to file
fig.savefig(out, dpi=150)
print("Wrote", out) #feedback

# If you have a display, you can still show:
# plt.show()
