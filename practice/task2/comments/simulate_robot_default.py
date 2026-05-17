#!/usr/bin/env python3
"""Plot one robot with the default style (rectangle body + triangular head)."""

from __future__ import annotations

import math

import AuroraMR as amr #AuroraMR Library

p = amr.pose(0.0, 0.0, 0.0) #position vector
# ``style="robot"`` is the default; opens a window with plt.show() unless you
# pass ``ax=`` or ``show=False``.
amr.simulate(p) #simulation
