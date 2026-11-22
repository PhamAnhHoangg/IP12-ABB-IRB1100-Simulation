"""ZED Mini wrist for the Zimmer LWR50L-23-00006-A as 6 printed pieces, all in the gripper STEP's coordinates
(gripper axis = Y, jaws toward -Y, cable loop on +Z, head slots and sensor slots on +-X):
  upper ring (2 halves, tongues in the head slots), lower ring (2 halves, keys in the sensor slots),
  collar (joins both rings on +X), tilt wedge (sets the camera angle), ZED box (bolted to the wedge).
The box is printed level; the wedge between collar and box tilts it TILT_DEG towards the gripper axis, so the camera
also sees the jaws. To try another angle, change TILT_DEG and reprint only the wedge (0 = level, a flat spacer).
Ring halves are split at x = 0 and go on from the sides (tongues and keys enter their slots straight in).
Joints: upper ring = ear with a screw along X on the -Z side, lap joint with screws along Y on the cable (+Z) side;
lower ring = ears with screws along X on both narrow sides. Collar and box screws run along X. All screws M3.
Run: pip install build123d && python zed_wrist_parts.py   -> one STEP + STL per piece, plus an assembly STEP
"""
from build123d import *
from math import atan2, cos, degrees, hypot, radians, sin
from OCP.BRepBuilderAPI import BRepBuilderAPI_Copy
from OCP.BRepLib import BRepLib
from OCP.TopoDS import TopoDS

M3, M3_PILOT = 3.4, 2.5                     # clearance hole / hole for an M3 self-tapping screw into the print
# upper ring: real head slot measured 5 mm tall, 12 mm deep; latches start at Y 9.5; head and plate radius 37.5
TONGUE_T, TONGUE_IN, SLOT_YC = 4.6, 31.2, 6.0
UR_IN, UR_OUT, UR_Y0, UR_Y1 = 38.5, 44.0, -9.0, SLOT_YC + 2.3
# lower ring: body 26 x 53.8 (X +-13, Z +-26.9); sensor slot centre Y -69.75, 3.0 mm opening; jaws start at Y -76
SLOT_Y, KEY_W, KEY_TIP = -69.75, 2.7, 9.6
LR_IN_X, LR_IN_Z, LR_WALL, LR_Y0, LR_Y1 = 13.2, 27.1, 4.0, -74.5, -65.0
EAR_Z = 8.0                                 # each ring half's ear is 8 thick, so a joint screw is M3 x 20 with a nut
# ZED Mini 124.5 x 30.5 x 26.5 with 1 mm play, lenses down; collar 6 thick, 24 wide
CAM_L, CAM_W, CAM_D, WALL = 125.5, 33.0, 27.5, 3.0   # width 33: 2.5 mm play so a screw head that sits proud does not press on the camera
COL_T, COL_Z = 6.0, 12.0
TILT_DEG = 25.0                             # camera tilt towards the gripper axis; only the wedge changes with it
WEDGE_T0, WEDGE_Z, BOX_SCREW_Z = 5.0, 20.0, 16.0   # wedge thickness at its thin (lower) edge, half width; box screws at z +-16

def box(x0, x1, y0, y1, z0, z1):
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(x1 - x0, y1 - y0, z1 - z0)

def hole(d, axis, at, length=80):
    rot = {"X": Rot(0, 90, 0), "Y": Rot(90, 0, 0), "Z": Rot(0, 0, 0)}[axis]
    return Pos(*at) * rot * Cylinder(d / 2, length)

def split(part):  # +X half, -X half
    return part & box(0, 200, -200, 200, -200, 200), part & box(-200, 0, -200, 200, -200, 200)

# ---- upper ring
ur = Pos(0, (UR_Y0 + UR_Y1) / 2, 0) * Rot(90, 0, 0) * (Cylinder(UR_OUT, UR_Y1 - UR_Y0) - Cylinder(UR_IN, UR_Y1 - UR_Y0))
for s in (1, -1):
    ur += box(*sorted((s * TONGUE_IN, s * (UR_IN + 1))), SLOT_YC - TONGUE_T / 2, SLOT_YC + TONGUE_T / 2, -21, 21)   # tongue
ur += box(-EAR_Z, EAR_Z, UR_Y0, UR_Y1, -(UR_OUT + 10), -(UR_OUT - 1))     # -Z ear
ur -= hole(M3, "X", (0, 0.0, -(UR_OUT + 5)))
ur += box(UR_OUT - 1, UR_OUT + 6, UR_Y0, UR_Y1, -COL_Z, COL_Z)              # flat boss for the collar on +X
ur -= hole(M3_PILOT, "X", (UR_OUT + 6, -5.0, 0), 16) + hole(M3_PILOT, "X", (UR_OUT + 6, 4.0, 0), 16)
ur -= box(-4, 4, 5.0, 20, 30, 60)                                          # notch for the cable where it leaves the head
ur_a, ur_b = split(ur)
LAP_Y, LAP_X = -2.0, 10.0                                                   # lap joint on +Z: A keeps the bottom layer, B the top
lap = box(-LAP_X, LAP_X, UR_Y0 - 1, 5.0, 37.8, 60)        # z >= 37.8 so it clears the head while the half moves in
ur_a = (ur_a - lap) + (ur & lap & box(-LAP_X, LAP_X, UR_Y0 - 1, LAP_Y, 37.8, 60))
ur_b = (ur_b - lap) + (ur & lap & box(-LAP_X, LAP_X, LAP_Y, 5.0, 37.8, 60))
lap_holes = [(x, UR_OUT - 2.75) for x in (6.0, -6.0)]
for x, z in lap_holes:
    ur_b -= hole(M3, "Y", (x, 0, z))
    ur_a -= hole(M3_PILOT, "Y", (x, 0, z))
# ---- lower ring
lo_x, lo_z = LR_IN_X + LR_WALL, LR_IN_Z + LR_WALL
lr = box(-lo_x, lo_x, LR_Y0, LR_Y1, -lo_z, lo_z) - box(-LR_IN_X, LR_IN_X, LR_Y0 - 1, LR_Y1 + 1, -LR_IN_Z, LR_IN_Z)
for s in (1, -1):
    lr += box(*sorted((s * KEY_TIP, s * (LR_IN_X + 0.5))), SLOT_Y - KEY_W / 2, SLOT_Y + KEY_W / 2, -LR_IN_Z, LR_IN_Z)  # key
    lr += box(-EAR_Z, EAR_Z, LR_Y0, LR_Y1, *sorted((s * (lo_z - 1), s * (lo_z + 10))))                                 # ear
    lr -= hole(M3, "X", (0, SLOT_Y, s * (lo_z + 5)))
lr += box(lo_x - 1, lo_x + 6, LR_Y0, -58.0, -COL_Z, COL_Z)                  # flat boss for the collar on +X
lr -= hole(M3_PILOT, "X", (lo_x + 6, -71.0, 0), 16) + hole(M3_PILOT, "X", (lo_x + 6, -61.0, 0), 16)
lr_a, lr_b = split(lr)
# ---- collar on +X: pad on each boss, straight web between them
ux, lx = UR_OUT + 6, lo_x + 6
up_pad = box(ux, ux + COL_T, UR_Y0, UR_Y1, -COL_Z, COL_Z)
lo_pad = box(lx, lx + COL_T, LR_Y0, -58.0, -COL_Z, COL_Z)
a, b = Vector(ux + COL_T / 2, UR_Y0, 0), Vector(lx + COL_T / 2, -58.0, 0)
web = Pos((a + b) / 2) * Rot(0, 0, (b - a).get_signed_angle(Vector(0, 1, 0), Vector(0, 0, -1))) * Box(COL_T, (b - a).length + 4, 2 * COL_Z)
collar = up_pad + lo_pad + web - ur - lr       # trim the web's corners where it meets the ring bosses
collar -= hole(M3, "X", (ux, -5.0, 0), 30) + hole(M3, "X", (ux, 4.0, 0), 30)
collar -= hole(6.4, "X", (ux + COL_T, -5.0, 0), 6.4) + hole(6.4, "X", (ux + COL_T, 4.0, 0), 6.4)  # 3.2 deep counterbores: these heads sit under the box wall
collar -= hole(M3, "X", (lx, -71.0, 0), 30) + hole(M3, "X", (lx, -61.0, 0), 30)
collar -= hole(M3_PILOT, "X", (ux + COL_T, -0.5, 8.5), 12) + hole(M3_PILOT, "X", (ux + COL_T, -0.5, -8.5), 12)  # box screws
# ---- ZED box: inner wall bolted to the collar, lenses facing straight down, open bottom, lips under both ends
bx0 = ux + COL_T
cy0 = UR_Y0                                         # lens face height
cx = bx0 + WALL + CAM_W / 2
zbox = box(bx0, bx0 + CAM_W + 2 * WALL, cy0, cy0 + CAM_D + WALL, -CAM_L / 2 - WALL, CAM_L / 2 + WALL)
zbox -= box(bx0 + WALL, bx0 + WALL + CAM_W, cy0 - 1, cy0 + CAM_D, -CAM_L / 2, CAM_L / 2)
for s in (1, -1):
    zbox += box(bx0, bx0 + CAM_W + 2 * WALL, cy0 - WALL, cy0, *sorted((s * (CAM_L / 2 - 6), s * (CAM_L / 2 + WALL))))   # lips
    zbox -= box(cx - 7, cx + 7, cy0 + 8, cy0 + 18, *sorted((s * CAM_L / 2 - s, s * (CAM_L / 2 + WALL + 1))))           # USB-C window
    zbox -= hole(M3, "X", (bx0, -0.5, s * BOX_SCREW_Z), 10)
    zbox -= Pos(bx0 + WALL, -0.5, s * BOX_SCREW_Z) * Rot(0, -90, 0) * Cone(3.2, 1.5, 1.7, align=(Align.CENTER, Align.CENTER, Align.MIN))  # countersink inside

collar -= box(bx0, bx0 + 50, -200, 200, -200, 200)   # flat face for the wedge: nothing of the collar past its outer face

# ---- tilt wedge: collar face (x = bx0) on one side, box wall on the other, hinged at the lower edge
def place(shape):  # level box -> tilted box: its lower inner edge (bx0, cy0) sits WEDGE_T0 off the collar, turned TILT_DEG
    return Pos(bx0 + WEDGE_T0, cy0, 0) * Rot(0, 0, -TILT_DEG) * Pos(-bx0, -cy0, 0) * shape
t = (UR_Y1 - cy0) / cos(radians(TILT_DEG))
wedge = Pos(0, 0, -WEDGE_Z) * extrude(Polygon((bx0, cy0), (bx0 + WEDGE_T0, cy0), (bx0 + WEDGE_T0 + t * sin(radians(TILT_DEG)), UR_Y1),
                                              (bx0, UR_Y1), align=None), amount=2 * WEDGE_Z)   # counter-clockwise, so it extrudes to +z
assert abs(wedge.bounding_box().min.Z + WEDGE_Z) < 0.01 and abs(wedge.bounding_box().max.Z - WEDGE_Z) < 0.01
for s in (1, -1):
    wedge -= hole(M3, "X", (bx0, -0.5, s * 8.5), 30)                                  # wedge -> collar pilot holes (M3 x 6)
    wedge -= Pos(bx0 + 2 + 10, -0.5, s * 8.5) * Rot(0, 90, 0) * Cylinder(3.2, 20)     # counterbore, 2 mm left under the head
    wedge -= place(hole(M3_PILOT, "X", (bx0, -0.5, s * BOX_SCREW_Z), 12))              # box -> wedge (M3 x 8 countersunk)
zbox_level, zbox = zbox, place(zbox)
assert wedge.distance_to(collar) < 0.01 and wedge.distance_to(zbox) < 0.01, "wedge must sit on the collar and carry the box"

parts = {"upper_ring_half_A": ur_a, "upper_ring_half_B": ur_b, "lower_ring_half_A": lr_a, "lower_ring_half_B": lr_b,
         "collar": collar, "tilt_wedge": wedge, "zed_box": zbox}
# ---- self-checks against the gripper (the STEP shows the head slots as solid blocks; the real slots are open)
def outward(s):
    if s.volume >= 0:
        return s
    w = TopoDS.Solid_s(BRepBuilderAPI_Copy(s.wrapped).Shape()) if hasattr(TopoDS, "Solid_s") else TopoDS.Solid(BRepBuilderAPI_Copy(s.wrapped).Shape())
    BRepLib.OrientClosedSolid_s(w)
    return Solid(w)
grip = [outward(s) for s in import_step("LWR50L-23-00006-A.stp").solids()
        if not (s.bounding_box().min.Y > 1.9 and s.bounding_box().max.Y < 10.1 and s.bounding_box().size.X < 15)]
slot_zone = sum((box(*sorted((s * (TONGUE_IN - 0.1), s * 37.6)), SLOT_YC - 2.4, SLOT_YC + 2.4, -21.1, 21.1) for s in (1, -1)), Part())
assert TONGUE_T <= 5.0 - 0.3 and 37.5 - TONGUE_IN <= 12, "tongue must fit the measured slot"

def overlaps(p, q):
    pb, qb = p.bounding_box(), q.bounding_box()
    if pb.min.X > qb.max.X or qb.min.X > pb.max.X or pb.min.Y > qb.max.Y or qb.min.Y > pb.max.Y or pb.min.Z > qb.max.Z or qb.min.Z > pb.max.Z:
        return False
    h = p & q
    if h is None or abs(h.volume) <= 0.01:
        return False
    if q.is_valid:
        return True
    pts = [Vector(x / 2, y / 2, z / 2) for c in h.solids() for bb in [c.bounding_box()]
           for x in range(int(bb.min.X * 2), int(bb.max.X * 2) + 1) for y in range(int(bb.min.Y * 2), int(bb.max.Y * 2) + 1)
           for z in range(int(bb.min.Z * 2), int(bb.max.Z * 2) + 1, 4)]
    return any(q.is_inside(v) and p.is_inside(v) for v in pts)   # repaired head solid: confirm by sampling

for name, p in parts.items():
    assert len(p.solids()) == 1 and p.is_valid, f"{name} should be one clean solid"
    side = 1 if name.endswith("_A") else -1
    path = (0, 5, 12, 25) if "ring" in name else (0,)       # ring halves come in sideways along X from their own side
    for dz in path:
        moved = Pos(side * dz, 0, 0) * (p - slot_zone)
        hit = [i for i, g in enumerate(grip) if overlaps(moved, g)]
        assert not hit, f"{name} hits gripper solid(s) {hit} at {dz} mm before its final position"
names = list(parts)
for i in range(len(names)):
    for j in range(i + 1, len(names)):
        h = parts[names[i]] & parts[names[j]]
        assert h is None or abs(h.volume) < 0.5, f"{names[i]} overlaps {names[j]}"
camera = place(box(cx - 15.25, cx + 15.25, cy0, cy0 + 26.5, -62.25, 62.25))
assert not any(overlaps(camera, g) for g in grip) and all((camera & p) is None or abs((camera & p).volume) < 0.01 for p in parts.values())

lens = place(Pos(cx, cy0, 0) * Box(0.01, 0.01, 0.01)).center()               # middle of the lens face
for what, (tx, ty) in (("jaw tips", (0, -109.5)), ("between the jaws", (0, -95)), ("finger tips (40 mm fingers)", (0, -149.5)), ("a box 30 cm below the jaws", (0, -409.5))):
    off = degrees(atan2(lens.X - tx, lens.Y - ty)) - TILT_DEG
    print(f"{what}: {off:+.0f} deg off the lens axis (ZED Mini vertical half-FOV about 30), {hypot(lens.X - tx, lens.Y - ty):.0f} mm away")

# break-away block under A's lap tab (it hangs 10 mm in the air when printed); only in the STL, 0.2 mm (one layer) gap so it snaps off
GAP = 0.2
breakaway = (ur & box(-LAP_X + GAP, LAP_X - GAP, LAP_Y + GAP, UR_Y1, 37.8 + GAP, 60)) - box(0, LAP_X, 5.0 - GAP, UR_Y1 + 1, 37.8, 60)  # on x > 0, A keeps a thin layer at Y 5..8.3
assert breakaway.distance_to(ur_a) > GAP - 0.01, "break-away block must not touch half A"
for name, p in parts.items():
    export_step(p, f"{name}.step")
    if name == "upper_ring_half_A":
        p = p + breakaway
    if name == "zed_box":
        p = zbox_level                                              # printed level, the wedge tilts it
    lie = Rot(0, 0, 0) if name == "collar" else Rot(0, -90, 0) if name == "tilt_wedge" else Rot(90, 0, 0) if name.startswith("lower") else Rot(-90, 0, 0)  # upper rings tongue side down, lower rings flat end down, box top down, collar on its side
    bb = (lie * p).bounding_box()
    printed = Pos(-bb.center().X, -bb.center().Y, -bb.min.Z) * lie * p
    print(f"print {name}: {bb.size.X:.1f} x {bb.size.Y:.1f} x {bb.size.Z:.1f} mm, {sum(f.area for f in printed.faces() if f.bounding_box().max.Z < 0.01) / 100:.1f} cm2 on the bed")
    export_stl(printed, f"{name}.stl")   # STL lies ready for the slicer; STEP stays in the assembly position
cam_part = Part(camera.wrapped); cam_part.label = "ZED Mini (outline)"
g = import_step("LWR50L-23-00006-A.stp"); g.label = "Zimmer LWR50L-23-00006-A"
for n, p in parts.items():
    p.label = n
export_step(Compound(children=[g, *parts.values(), cam_part], label=f"gripper with ZED wrist (tilt {TILT_DEG:g} deg)"), "gripper_with_zed_wrist_parts.step")
print("ok:", ", ".join(f"{n} {p.volume / 1000:.1f} cm3" for n, p in parts.items()))
