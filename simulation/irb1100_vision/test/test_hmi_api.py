#!/usr/bin/env python3
"""Self-check for scripts/hmi_api.py (WP3 API format): code reading reasons, location zones, WebSocket framing.
   python3 simulation/irb1100_vision/test/test_hmi_api.py   (needs ROS 2 Jazzy sourced, python3-opencv)"""
import json, os, sys
import cv2, numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
from hmi_api import location_of, read_code, ws_accept, ws_frame, iso

qr = cv2.QRCodeEncoder.create().encode("PKG-10023")
big = cv2.resize(qr, (qr.shape[1] * 8, qr.shape[0] * 8), interpolation=cv2.INTER_NEAREST)
big = cv2.copyMakeBorder(big, 40, 40, 40, 40, cv2.BORDER_CONSTANT, value=255)
r = read_code(big, big.shape[0]); assert r["state"] == "identified" and r["package_code"] == "PKG-10023", r
dark = (np.random.default_rng(1).integers(0, 30, big.shape)).astype(np.uint8)                   # almost black, nothing readable
assert read_code(dark, big.shape[0])["unreadable_reason"] == "bad_light"
small = cv2.resize(big, (60, 60), interpolation=cv2.INTER_AREA)
assert read_code(small, 60)["unreadable_reason"] in ("too_far", "damaged"), read_code(small, 60)
plain = np.random.default_rng(0).integers(90, 170, (200, 200)).astype(np.uint8)
assert read_code(plain, 200)["unreadable_reason"] == "covered", read_code(plain, 200)
assert read_code(big, 400, cut_off=True)["unreadable_reason"] == "unknown"
assert location_of((-50, 0, 782)) == "pallet" and location_of((-50, 150, 762)) == "source_area"
assert location_of((300, 0, 900), tcp_mm=(310, 0, 900)) == "gripper"
assert ws_accept("dGhlIHNhbXBsZSBub25jZQ==") == "s3pPLMBiTxaQ9kYGzzhZRbK+xOo="           # RFC 6455 example
assert ws_frame("hi") == b"\x81\x02hi" and ws_frame("x" * 200)[:4] == b"\x81\x7e\x00\xc8"
assert iso(0) == "1970-01-01T00:00:00.000Z"
print("hmi_api: ok")
