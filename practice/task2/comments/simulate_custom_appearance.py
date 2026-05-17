#!/usr/bin/env python3
"""Tune robot footprint and colors: ``length``, ``width``, ``facecolor``, ``edgecolor``."""

from __future__ import annotations

import math

import AuroraMR as amr #AuroraMR Library

p = amr.pose(0.0, 0.0, 0.0) #(x,y,theta)

amr.simulate(
    p,
    style="robot",
    length=0.8,
    width=0.45,
    facecolor="#1565c0",
    edgecolor="#0d47a1",
) #customize simulation
