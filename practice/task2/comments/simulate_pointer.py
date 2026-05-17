#!/usr/bin/env python3
"""Plot using the compact arrow / pointer style instead of the full robot shape."""

from __future__ import annotations

import math

import AuroraMR as amr #AuroraMR Library

p = amr.pose(1.0, 2.0, math.pi / 3) #position vectir
amr.simulate(p, style="pointer") #simulation
