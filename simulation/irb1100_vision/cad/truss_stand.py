#!/usr/bin/env python3
"""All-printed triangle truss camera stand (build123d).

Lower right triangle  A-B-C : base A-C lies on the table, upright A-B stands OUTSIDE the table edge (OUT m away), B holds Cam 1.
Upper inverted triangle K-T-E: upright K-T (same line as A-B), boom T-E over the pallet, brace E-K; E holds Cam 2 looking down.
Members are 25x25 hollow printed bars pushed into printed corner nodes / couplers and fixed with one M8 bolt per socket.
Writes STL/STEP for nodes, couplers, bars and the clamp, plus ../urdf/camera_stand.xacro and a parts list.

  python3 truss_stand.py                (run inside cad/)
"""
import json, math, os
import numpy as np
import trimesh
from build123d import Box, Cylinder, Pos, Rot, RegularPolygon, Polygon, extrude, export_step, export_stl

OUT = "../meshes"
TZ, EDGE_X = 0.75, -0.55
OUTSIDE = 0.10            # upright is this far outside the table edge
BAR, WALL = 25.0, 3.0     # bar outer size / wall (mm)
SK = 37.0                 # socket outer size
POCKET = 25.6             # socket pocket
DEPTH = 37.0              # how far a bar goes into a socket
BORE = 8.4
MAXSEG = 0.32             # max centre-to-centre length of one bar piece (m); print long bars diagonally on a 220 mm bed

# ---- layout (metres, XZ plane, y = 0) -------------------------------------------------------------
c2x, c2z = -0.05, TZ + 0.02 + 0.80          # Cam 2 camera centre
stub_x = c2x - 0.0583                       # Cam 2 post axis
c1z = TZ + 0.05 + (-0.05 - (-0.5688 + 0.05084)) * math.tan(math.radians(35)) + 0.08
AX = EDGE_X - OUTSIDE
BASEZ = TZ + 0.0228                         # base bar axis (bar sits on the clamp saddle)
BASE = 0.22
CX = AX + BASE
MOUNT_ABOVE_B = 0.14                        # camera-mount origin sits this far above node B on the upright
BZ = (c1z - 0.0175) - MOUNT_ABOVE_B
KZ = 1.39
TZ_TOP = 1.70
nodes = {
    "A": (AX, BASEZ), "B": (AX, BZ), "C": (CX, BASEZ),
    "K": (AX, KZ), "T": (AX, TZ_TOP), "E": (stub_x, TZ_TOP),
}
members = [("A", "C"), ("A", "B"), ("B", "C"), ("B", "K"), ("K", "T"), ("T", "E"), ("E", "K")]
STUB_LEN = 0.20                              # bar hanging down from E (Cam 2 mount slides on it)
STUB_END = c2z - 0.03


def unit(a, b):
    d = np.array(nodes[b]) - np.array(nodes[a]); return d / np.linalg.norm(d), float(np.linalg.norm(d))

# directions of every socket at every node
dirs = {n: [] for n in nodes}
for a, b in members:
    u, _ = unit(a, b)
    dirs[a].append(tuple(u)); dirs[b].append(tuple(-u))
dirs["E"].append((0.0, -1.0))                # stub post pointing down

def r0_for(ds):
    r = 8.0
    for i in range(len(ds)):
        for j in range(i + 1, len(ds)):
            ang = math.acos(max(-1, min(1, np.dot(ds[i], ds[j]))))
            if ang < math.pi / 2 + 0.01:
                r = max(r, 27.0 / math.sin(ang))
    return math.ceil(r + 2)
R0 = {n: r0_for(d) for n, d in dirs.items()}


def yc(r, h, x=0.0, z=0.0, y=0.0):
    return Pos(x, y, z) * (Rot(X=90) * Cylinder(r, h))


def socket_solid(r0):
    return Pos((r0 + DEPTH + 4) / 2, 0, 0) * Box(r0 + DEPTH + 4, SK, SK)


def socket_cut(r0):
    c = Pos(r0 + DEPTH / 2 + 1, 0, 0) * Box(DEPTH + 2, POCKET, POCKET)
    h = r0 + 28                                   # bar end rests 10 mm off the pocket floor; slotted +-10 mm for assembly
    c += Pos(h, 0, 0) * Box(20, SK + 10, BORE) + yc(BORE / 2, SK + 10, x=h - 10) + yc(BORE / 2, SK + 10, x=h + 10)
    return c


def rot(u):                                    # rotate a +X feature to the XZ direction u
    th = math.degrees(math.atan2(u[1], u[0]))
    return Rot(Y=-th)


def build_node(n):
    ds, r0 = dirs[n], R0[n]
    p = Box(SK, SK, SK)
    for u in ds:
        p += rot(u) * socket_solid(r0)
    for u in ds:
        p -= rot(u) * socket_cut(r0)
    return p


def build_coupler(r0=10.0):
    ds = [(1.0, 0.0), (-1.0, 0.0)]
    p = Box(SK, SK, SK)
    for u in ds:
        p += rot(u) * socket_solid(r0)
    for u in ds:
        p -= rot(u) * socket_cut(r0)
    return p


def build_bar(length_mm):
    p = Box(length_mm, BAR, BAR)
    p -= Box(length_mm + 2, BAR - 2 * WALL, BAR - 2 * WALL)
    k = 18.0
    while k < length_mm - 10:
        p -= yc(BORE / 2, BAR + 4, x=-length_mm / 2 + k)
        k += 20.0
    return p


def clamp(table=30.0):
    gap = table + 2.0
    jaw_hi, low = 10.0, 14.0
    zlo = -(gap + low)
    bw = 50.0
    p = Pos(-6, 0, (zlo + jaw_hi) / 2) * Box(12, bw, jaw_hi - zlo)
    p += Pos(24, 0, jaw_hi / 2) * Box(72, bw, jaw_hi)
    p += Pos(8, 0, zlo + low / 2) * Box(104, bw, low)
    for s in (1, -1):
        g = Rot(X=90) * extrude(Polygon((-12, 0), (-44, 0), (-12, 40), align=None), 8)
        p += Pos(0, 15 if s == 1 else -23, zlo + low) * g
    # saddle: square socket along X on top of the upper jaw; bar pocket floor at z = 10
    zc = 10 + POCKET / 2
    p += Pos(24, 0, 10 + SK / 2 - 0.5) * Box(72, SK, SK)
    p -= Pos(24, 0, zc) * Box(80, POCKET, POCKET)
    for x in (8, 48):
        p -= yc(BORE / 2, SK + 10, x=x, z=zc)
    sx = 40.0
    p -= Pos(sx, 0, zlo + low / 2) * Cylinder(5.4, low + 2)
    p -= Pos(sx, 0, zlo + low - 8.4 / 2 + 0.01) * extrude(RegularPolygon(17.4 / 3 ** 0.5, 6), 8.4)
    return p


def knob(d=52, th=14, af=17.4, depth=8, bore=10.5, lobes=6):
    p = Cylinder(d / 2 - 3, th)
    for i in range(lobes):
        a = 2 * math.pi * i / lobes
        p -= Pos((d / 2 + 1) * math.cos(a), (d / 2 + 1) * math.sin(a), 0) * Cylinder(5.5, th + 2)
    p -= Cylinder(bore / 2, th + 2)
    p -= Pos(0, 0, th / 2 - depth / 2 + 0.01) * extrude(RegularPolygon(af / 3 ** 0.5, 6), depth)
    return p


def save(part, name):
    export_step(part, f"{OUT}/{name}.step")
    export_stl(part, f"{OUT}/{name}.stl", tolerance=0.02, angular_tolerance=0.2)
    m = trimesh.load(f"{OUT}/{name}.stl"); e = m.bounds[1] - m.bounds[0]
    print(f"{name:16s} {m.volume/1000:7.1f} cm3 watertight={m.is_watertight} bbox {e[0]:.0f}x{e[1]:.0f}x{e[2]:.0f} mm")


if __name__ == "__main__":
    os.makedirs(OUT, exist_ok=True)
    for n in nodes:
        save(build_node(n), f"stand_node_{n}")
    save(build_coupler(), "stand_coupler")
    save(clamp(), "stand_clamp")
    save(knob(), "stand_clamp_knob")

    bars, couplers, links = {}, [], []
    xa = []
    for a, b in members:
        u, L = unit(a, b)
        nseg = math.ceil(L / MAXSEG - 1e-9)
        seg = L / nseg
        ra, rb = R0[a], R0[b]
        # positions of piece ends along the member (m, from node a); couplers sit at the joins, half-width 0.0185+0.010
        cuts = [i * seg for i in range(nseg + 1)]
        for i in range(nseg):
            s0 = cuts[i] + ((ra + 10) / 1000 if i == 0 else 0.020)
            s1 = cuts[i + 1] - ((rb + 10) / 1000 if i == nseg - 1 else 0.020)
            blen = round((s1 - s0) * 1000)
            bars[blen] = bars.get(blen, 0) + 1
            mid = (s0 + s1) / 2
            p = np.array(nodes[a]) + u * mid
            links.append(("bar", float(p[0]), float(p[1]), blen / 1000, math.atan2(u[1], u[0])))
        for i in range(1, nseg):
            p = np.array(nodes[a]) + u * cuts[i]
            couplers.append((float(p[0]), float(p[1]), math.atan2(u[1], u[0])))
    # stub bar
    zs0 = nodes["E"][1] - (R0["E"] + 10) / 1000
    zs1 = zs0 - STUB_LEN
    blen = round((zs0 - zs1) * 1000)
    bars[blen] = bars.get(blen, 0) + 1
    links.append(("bar", stub_x, (zs0 + zs1) / 2, blen / 1000, -math.pi / 2))
    for L in bars:
        save(build_bar(L), f"stand_bar_{L}mm")

    # --- xacro ---
    o = ['<?xml version="1.0"?>', '<robot xmlns:xacro="http://www.ros.org/wiki/xacro">', "  <!-- generated by cad/truss_stand.py -->"]
    o.append(f'  <xacro:stand_mesh name="stand_clamp" mesh="stand_clamp.stl" x="{EDGE_X}" z="{TZ}" yaw="0"/>')
    for n, (x, z) in nodes.items():
        o.append(f'  <xacro:stand_mesh name="stand_node_{n}" mesh="stand_node_{n}.stl" x="{x:.4f}" z="{z:.4f}" yaw="0"/>')
    for i, (x, z, a) in enumerate(couplers):
        o.append(f'  <xacro:stand_mesh_rot name="stand_coupler_{i}" mesh="stand_coupler.stl" x="{x:.4f}" z="{z:.4f}" pitch="{-a:.5f}"/>')
    for i, (_, x, z, L, a) in enumerate(links):
        o.append(f'  <xacro:stand_bar name="stand_bar_{i}" x="{x:.4f}" z="{z:.4f}" length="{L:.4f}" pitch="{-a:.5f}"/>')
    o.append("</robot>")
    open("../urdf/camera_stand.xacro", "w").write("\n".join(o) + "\n")
    info = {"nodes": {k: [round(v[0], 4), round(v[1], 4)] for k, v in nodes.items()}, "r0_mm": R0, "bars_mm_qty": bars,
            "couplers": len(couplers), "member_lengths_m": {f"{a}-{b}": round(unit(a, b)[1], 3) for a, b in members},
            "cam1_mount_on_upright_between": "B and K", "c1z": round(c1z, 4)}
    print(json.dumps(info, indent=1))
