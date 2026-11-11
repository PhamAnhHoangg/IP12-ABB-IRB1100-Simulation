#!/usr/bin/env python3
"""Parametric camera clamp for a square tube, printable in PLA/PETG/ABS.

Clamps a square tube (default 25 mm) with one M4 screw + nut. A tilting plate
on the front carries any camera with a 1/4"-20 tripod socket.
Frame (mm): tube axis = Z, camera looks +X (plate tilted down by --tilt).

  python3 camera_mount.py --tilt 35 --out ../meshes/camera_mount_tilt35
  python3 camera_mount.py --tilt 90 --out ../meshes/camera_mount_tilt90

Writes .step (editable CAD), .stl and .obj (millimetres).
"""
import argparse, math
from build123d import (Box, Cylinder, Pos, Rot, Plane, RegularPolygon, extrude,
                       Axis, export_step, export_stl, Align)
import trimesh

ap = argparse.ArgumentParser()
ap.add_argument("--tube", type=float, default=25.0, help="square tube size, mm")
ap.add_argument("--clear", type=float, default=0.3, help="clearance per side, mm")
ap.add_argument("--wall", type=float, default=6.0)
ap.add_argument("--height", type=float, default=30.0, help="clamp height along tube, mm")
ap.add_argument("--arm", type=float, default=22.0, help="arm length in front of clamp, mm")
ap.add_argument("--tilt", type=float, default=35.0, help="camera pitch down, deg (90 = looks straight down)")
ap.add_argument("--plate_l", type=float, default=50.0)
ap.add_argument("--plate_w", type=float, default=40.0)
ap.add_argument("--plate_t", type=float, default=5.0)
ap.add_argument("--screw_hole", type=float, default=6.6, help='1/4"-20 clearance hole, mm')
ap.add_argument("--out", required=True)
a = ap.parse_args()

pocket = a.tube + 2 * a.clear
bw = pocket + 2 * a.wall                      # clamp outer square
H = a.height
slit = 1.5

body = Box(bw, bw, H)
body -= Box(pocket, pocket, H + 2)            # tube pocket
# rear slit + two ears for the M4 clamp screw (screw axis = Y)
ear_x = 14.0
for s in (-1, 1):
    body += Pos(-(bw / 2 + ear_x / 2 - 1), s * (slit / 2 + 5.0), 0) * Box(ear_x + 2, 10, H)
body -= Pos(-(bw / 2 + 5), 0, 0) * Box(bw / 2 + 12, slit, H + 2)   # slit from pocket to rear
body -= Pos(-(bw / 2 + 6), 0, 0) * (Rot(90, 0, 0) * Cylinder(2.2, 40))   # M4 clearance
hexp = extrude(RegularPolygon(4.35, 6), 3.6)                          # M4 nut trap (7 mm AF + clearance)
# nut trap opens on the outer face of the +Y ear (y = slit/2 + 10), 3.6 mm deep
body -= Pos(-(bw / 2 + 6), slit / 2 + 10, 0) * (Rot(90, 0, 0) * hexp)

# arm to the tilting camera plate
P = (bw / 2 + a.arm, 0.0, 0.0)               # plate pivot (bottom face centre)
arm = Pos(bw / 2 + a.arm / 2 - 1, 0, 0) * Box(a.arm + 2, 22, 14)
t = math.radians(a.tilt)
plate = Box(a.plate_l, a.plate_w, a.plate_t, align=(Align.CENTER, Align.CENTER, Align.MIN))
plate -= Cylinder(a.screw_hole / 2, a.plate_t + 2)                     # tripod screw hole
plate = Pos(*P) * (Rot(0, a.tilt, 0) * plate)
part = body + arm + plate

step = a.out + ".step"; stl = a.out + ".stl"; obj = a.out + ".obj"
export_step(part, step); export_stl(part, stl, tolerance=0.01, angular_tolerance=0.1)
m = trimesh.load(stl)
m.export(obj)
n = (math.sin(t), 0.0, math.cos(t))
cam_c = [P[i] + n[i] * (a.plate_t + 12.5) for i in range(3)]
print(f"{obj}: watertight={m.is_watertight} volume={m.volume/1000:.1f} cm3 "
      f"bbox={[round(x,1) for x in m.extents]} mm")
print(f"camera centre in mount frame (mm), body 25 mm tall: {[round(x,2) for x in cam_c]}")
