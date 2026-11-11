#!/usr/bin/env python3
"""All-printed adjustable camera arm (clamp + links + head + knobs), build123d.

Every joint is a fork/tongue hinge about the Y axis closed by one M8 bolt and a printed knob
(friction joint, no teeth). Link frame (mm): origin = tongue pivot, +X along the bar, fork pivot at x=L.

  python3 arm_stand.py --out ../meshes/arm      -> arm_clamp / arm_link / arm_head / arm_knob / arm_clamp_knob (.stl .step)
"""
import argparse
import trimesh
from build123d import Box, Cylinder, Pos, Rot, RegularPolygon, Polygon, extrude, export_step, export_stl

ap = argparse.ArgumentParser()
ap.add_argument("--out", required=True)
ap.add_argument("--L", type=float, default=200.0, help="link length, pivot to pivot")
ap.add_argument("--table", type=float, default=30.0, help="max table thickness the clamp takes (min 25)")
a = ap.parse_args()

R = 17.0        # boss radius
H = 30.0        # link height (in the bending plane)
T = 10.0        # tongue thickness
CL = 0.4        # axial clearance of the fork
GAP = T + CL    # fork inner gap
CH = 8.0        # cheek thickness
W = GAP + 2 * CH  # = 26.4 outer width of the link
BORE = 8.4      # M8 clearance
WALL = 3.0


def yc(r, h, x=0, z=0, y=0):           # cylinder with its axis along Y
    return Pos(x, y, z) * (Rot(X=90) * Cylinder(r, h))


def hexpocket(af, depth):              # hex prism, axis along Z
    return extrude(RegularPolygon(af / 3 ** 0.5, 6), depth)


def tongue(L0=0.0, plate=28.0):
    p = yc(R, T) + Pos(plate / 2, 0, 0) * Box(plate, T, H)
    return p


def fork_cheeks(x, plate=28.0):
    p = None
    for s in (1, -1):
        y = s * (GAP / 2 + CH / 2)
        c = yc(R, CH, x=x, y=y) + Pos(x - plate / 2, y, 0) * Box(plate, CH, H)
        p = c if p is None else p + c
    return p


def link(L):
    p = tongue()
    p += Pos(28 + 12, 0, 0) * Box(24, W, H)                                   # solid transition block x 28..52
    p += Pos(L / 2, 0, 0) * Box(L - 104 + 2, W, H)                            # tube body x 52..L-52
    p -= Pos(L / 2, 0, 0) * Box(L - 104 - 2, W - 2 * WALL, H - 2 * WALL)      # hollow core (keeps closed ends)
    p += Pos(L - 40, 0, 0) * Box(24, W, H)                                    # solid block x L-52..L-28
    p += fork_cheeks(L)
    p -= yc(BORE / 2, 60)                                                     # tongue bore
    p -= yc(BORE / 2, 60, x=L)                                                # fork bore
    p -= Pos(L, -W / 2 + 2.75 - 0.01, 0) * (Rot(X=90) * hexpocket(13.4, 5.5))   # bolt-head hex pocket (outer -Y cheek)
    return p


def head(L=0.0):
    p = tongue(plate=30.0)
    p += Pos(30, 0, -43.75) * Box(25, 25, 112.5)       # 25x25 post: z +12.5 .. -100 (camera mount slides on)
    p -= yc(BORE / 2, 60)
    return p


def clamp(table):
    gap = table + 2.0
    jaw_hi, low = 10.0, 14.0
    zlo = -(gap + low)
    bw = 50.0
    p = Pos(-6, 0, (zlo + jaw_hi) / 2) * Box(12, bw, jaw_hi - zlo)               # back wall x -12..0
    p += Pos(24, 0, jaw_hi / 2) * Box(72, bw, jaw_hi)                            # upper jaw x -12..60, z 0..10
    p += Pos(8, 0, zlo + low / 2) * Box(104, bw, low)                            # lower jaw x -44..60 (foot behind the wall)
    for s in (1, -1):                                                           # gussets
        g = extrude(Polygon((-12, 0), (-44, 0), (-12, 40), align=None), 8)         # outside the wall, XY plane
        g = Rot(X=90) * g                                                       # now XZ plane, thickness along -Y
        p += Pos(0, 15 if s == 1 else -23, zlo + low) * g
    px, pz = 5.0, 40.0
    for s in (1, -1):
        y = s * (GAP / 2 + CH / 2)
        p += yc(R, CH, x=px, z=pz, y=y) + Pos(px, y, (jaw_hi + pz) / 2) * Box(2 * R, CH, pz - jaw_hi)
    p -= yc(BORE / 2, 60, x=px, z=pz)
    p -= Pos(px + R + 2 - R, -(GAP / 2 + CH) + 2.75 - 0.01, pz) * (Rot(X=90) * hexpocket(13.4, 5.5))
    sx = 40.0
    p -= Pos(sx, 0, zlo + low / 2) * Cylinder(5.4, low + 2)                     # M10 clearance / thread core
    p -= Pos(sx, 0, zlo + low - 8.4 / 2 + 0.01) * hexpocket(17.4, 8.4)          # M10 nut trap, open to the top
    return p


def knob(d=42.0, th=12.0, af=13.4, depth=6.5, bore=8.4, lobes=6):
    p = Cylinder(d / 2 - 3, th)
    import math
    for i in range(lobes):
        ang = 2 * math.pi * i / lobes
        p -= Pos((d / 2 + 1) * math.cos(ang), (d / 2 + 1) * math.sin(ang), 0) * Cylinder(5.5, th + 2)
    p -= Cylinder(bore / 2, th + 2)
    p -= Pos(0, 0, th / 2 - depth / 2 + 0.01) * hexpocket(af, depth)
    return p


parts = {
    "arm_link": link(a.L),
    "arm_head": head(),
    "arm_clamp": clamp(a.table),
    "arm_knob": knob(),
    "arm_clamp_knob": knob(d=52, th=14, af=17.4, depth=8, bore=10.5),
}
for name, part in parts.items():
    export_step(part, f"{a.out}_{name[4:]}.step")
    export_stl(part, f"{a.out}_{name[4:]}.stl", tolerance=0.02, angular_tolerance=0.2)
    m = trimesh.load(f"{a.out}_{name[4:]}.stl")
    e = m.bounds[1] - m.bounds[0]
    print(f"{name:15s} vol {m.volume/1000:7.1f} cm3  watertight={m.is_watertight}  bbox {e[0]:.0f} x {e[1]:.0f} x {e[2]:.0f} mm")
