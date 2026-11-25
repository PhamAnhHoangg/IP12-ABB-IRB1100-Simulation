#!/usr/bin/env python3
"""Self-check for the WP1 <-> WP2 UDP message format (scripts/udp_bridge.py). Needs ROS 2 Jazzy sourced."""
import json, math, os, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
from udp_bridge import boxes_message, parse_pose

m = json.loads(boxes_message(12.5, [("red", -0.25, 0.12, 0.0125)], [math.radians(64.4)]))
assert m == {"type": "boxes", "stamp": 12.5, "frame": "base", "unit": "mm",
             "boxes": [{"class": "red", "x": -250.0, "y": 120.0, "z": 12.5, "yaw_deg": 64.4}]}, m
assert json.loads(boxes_message(1.0, [("blue", 0, 0, 0)], [None]))["boxes"][0]["yaw_deg"] is None
st, q = parse_pose(b'{"type": "robot_pose", "stamp": 3.0, "joints_deg": [180, 0, 90, 0, -45, 0]}')
assert st == 3.0 and abs(q[0] - math.pi) < 1e-9 and abs(q[4] + math.pi / 4) < 1e-9
for bad in (b'{"type": "robot_pose", "joints_deg": [1, 2, 3]}', b'{"type": "other"}', b'not json',
            b'{"type": "robot_pose", "joints_deg": [1, 2, 3, 4, 5, NaN]}'):
    try: parse_pose(bad); raise AssertionError(bad)
    except ValueError: pass
print("udp_bridge message format: ok")
