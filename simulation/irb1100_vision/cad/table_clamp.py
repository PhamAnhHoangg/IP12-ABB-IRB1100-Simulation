#!/usr/bin/env python3
"""Printable table-edge clamp holding a vertical square tube (the camera stand post).

Frame (mm): x = 0 at the table edge face (table is at x > 0), z = 0 at the table TOP surface,
y = 0 on the stand centre line. The tube axis is at x = -(socket/2), vertical.
Upper jaw rests on the table top; an M8 screw in the lower jaw pushes up on the underside.

  python3 table_clamp.py --table 30 --out ../meshes/table_clamp
"""
import argparse
from build123d import Box, Cylinder, Pos, Rot, RegularPolygon, extrude, export_step, export_stl
import trimesh

ap = argparse.ArgumentParser()
ap.add_argument("--tube", type=float, default=25.0)
ap.add_argument("--clear", type=float, default=0.3)
ap.add_argument("--wall", type=float, default=6.0)
ap.add_argument("--table", type=float, default=30.0, help="table top thickness, mm")
ap.add_argument("--gap", type=float, default=8.0, help="free space under the table for the M8 screw tip")
ap.add_argument("--jaw", type=float, default=10.0, help="jaw thickness")
ap.add_argument("--reach", type=float, default=45.0, help="how far the jaws go onto the table")
ap.add_argument("--up", type=float, default=90.0, help="socket height above the table top")
ap.add_argument("--out", required=True)
a = ap.parse_args()

pocket = a.tube + 2 * a.clear
bw = pocket + 2 * a.wall
z_lo = -(a.table + a.gap + a.jaw)
z_hi = a.up
mid = (z_lo + z_hi) / 2

part = Pos(-bw / 2, 0, mid) * Box(bw, bw, z_hi - z_lo)                       # socket + web
part -= Pos(-bw / 2, 0, mid) * Box(pocket, pocket, z_hi - z_lo + 2)           # tube pocket (through)
part += Pos(a.reach / 2 - 1, 0, a.jaw / 2) * Box(a.reach + 2, bw, a.jaw)       # upper jaw (on table top)
low_z = -(a.table + a.gap) - a.jaw / 2
part += Pos(a.reach / 2 - 1, 0, low_z) * Box(a.reach + 2, bw, a.jaw)           # lower jaw
sx = a.reach * 0.65
part -= Pos(sx, 0, low_z) * Cylinder(4.3, a.jaw + 2)                           # M8 clearance
nut = extrude(RegularPolygon(7.8, 6), 6.5)                                     # M8 nut trap (13 AF + clearance)
part -= Pos(sx, 0, low_z - a.jaw / 2) * nut                                    # opens on the underside
# M4 set screw through the +Y wall, captive nut in a trap on the outside face
zs = 40.0
wall_c = (pocket / 2 + bw / 2) / 2
part -= Pos(-bw / 2, wall_c, zs) * (Rot(90, 0, 0) * Cylinder(2.2, a.wall + 2))
part -= Pos(-bw / 2, bw / 2, zs) * (Rot(90, 0, 0) * extrude(RegularPolygon(4.35, 6), 3.2))

export_step(part, a.out + ".step"); export_stl(part, a.out + ".stl", tolerance=0.01, angular_tolerance=0.1)
m = trimesh.load(a.out + ".stl"); m.export(a.out + ".obj")
print(f"{a.out}.obj watertight={m.is_watertight} bbox_mm={[round(float(x),1) for x in m.extents]} "
      f"volume={m.volume/1000:.1f} cm3 | tube axis at x=-{bw/2:.1f} mm")
