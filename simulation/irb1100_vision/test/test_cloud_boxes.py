#!/usr/bin/env python3
"""Self-check for scripts/cloud_boxes.py find_tops: table, pallet, a turned box on the table, a box on the pallet.
   python3 simulation/irb1100_vision/test/test_cloud_boxes.py   (needs ROS 2 Jazzy sourced, python3-opencv)"""
import os, sys
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
from cloud_boxes import find_tops


def plate(cx, cy, size, z, yaw=0.0):
    g = np.arange(-size / 2, size / 2, 0.002)
    xy = np.stack(np.meshgrid(g, g), -1).reshape(-1, 2)
    c, s = np.cos(yaw), np.sin(yaw)
    xy = xy @ np.array([[c, s], [-s, c]]) + [cx, cy]
    return np.c_[xy, np.full(len(xy), z)]


pts = np.vstack([plate(-0.2, 0.0, 0.6, 0.75), plate(-0.05, 0.0, 0.2, 0.77),       # table, pallet top (square, 200 mm)
                 plate(-0.05, 0.16, 0.025, 0.775, 0.4), plate(-0.05, 0.0, 0.025, 0.795)])
tops = sorted(find_tops(pts), key=lambda t: t[1])
assert len(tops) == 2, tops
for (x, y, z, yaw), (ex, ey, ez, eyaw) in zip(tops, [(-0.05, 0.0, 0.795, 0.0), (-0.05, 0.16, 0.775, 0.4)]):
    assert np.hypot(x - ex, y - ey) < 0.003 and abs(z - ez) < 0.002 and abs(yaw - eyaw) < np.radians(3), (x, y, z, yaw)
print("cloud_boxes find_tops: ok", [tuple(round(float(v), 3) for v in t) for t in tops])
