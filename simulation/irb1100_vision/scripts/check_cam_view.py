#!/usr/bin/env python3
"""How much of the robot arm does Cam 1 see for a given downward tilt?   (no ROS needed: pip install xacro numpy)

  python3 scripts/check_cam_view.py 0 15 20 30        # tilts in degrees (default: 0 15 20 30)

Samples random joint poses within the ABB limits, keeps those where the gripper is over the pallet
(0.80-1.10 m high, tool pointing down) and counts how often the gripper tip, the wrist (link_5) and the
elbow (link_2) are inside the Cam 1 picture. Uses link origins (points), not the meshes, so it is a first
estimate only. Re-run it after moving a camera.
"""
import math, os, sys, types, xml.etree.ElementTree as ET
import numpy as np

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
m_pkg = types.ModuleType("ament_index_python.packages"); m_pkg.get_package_share_directory = lambda n: PKG
m_top = types.ModuleType("ament_index_python"); m_top.packages = m_pkg
sys.modules["ament_index_python"] = m_top; sys.modules["ament_index_python.packages"] = m_pkg
import xacro  # noqa: E402

TILTS = [float(a) for a in sys.argv[1:]] or [0, 15, 20, 30]
N_SAMPLES = 400000
LIM = {"joint_1": (-230, 230), "joint_2": (-115, 113), "joint_3": (-205, 55),
       "joint_4": (-230, 230), "joint_5": (-125, 120), "joint_6": (-400, 400)}   # ABB IRB 1100, deg
HFOV, W, H = 1.2, 1280, 720                                                       # Cam 1 (workcell.urdf.xacro)

root = ET.fromstring(xacro.process_file(os.path.join(PKG, "urdf", "workcell.urdf.xacro"),
                     mappings={"controllers_yaml": "/x"}).toxml())


def rot(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr], [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])


joints = {}
for j in root.findall("joint"):
    o = j.find("origin")
    xyz = [float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]
    rp = [float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
    T = np.eye(4); T[:3, :3] = rot(*rp); T[:3, 3] = xyz
    ax = j.find("axis")
    joints[j.get("name")] = (j.find("parent").get("link"), j.find("child").get("link"), T,
                             np.array([float(v) for v in ax.get("xyz").split()]) if ax is not None else None)
parent_joint = {c: n for n, (_, c, _, _) in joints.items()}


def fk(q):
    cache = {}
    def get(link):
        if link in cache: return cache[link]
        n = parent_joint.get(link)
        if n is None: cache[link] = np.eye(4); return cache[link]
        p, _, T, axis = joints[n]; M = get(p) @ T
        if n in q:
            a = axis / np.linalg.norm(axis); K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
            Rt = np.eye(4); Rt[:3, :3] = np.eye(3) + math.sin(q[n])*K + (1-math.cos(q[n]))*K@K; M = M @ Rt
        cache[link] = M; return M
    return get


# Cam 1 position from the model (camera centre, world frame)
cam = joints["world_to_cam1_mount"][2][:3, 3]
mount = joints["cam1_mount_to_link"][2][:3, 3]
cam = cam + np.array([mount[0], 0, mount[2]])
cam = cam + np.array([float(os.environ.get("IP12_CAM_DX", "0")), 0, 0])   # optional: move Cam 1 along x (m), e.g. -0.10 = 10 cm farther back
tan_h = math.tan(HFOV / 2); tan_v = tan_h * H / W
KEYS = {"gripper_tcp": "gripper_tcp", "link_5": "wrist_link_5", "link_2": "elbow_link_2"}

# Sampling is done in chunks and can be resumed: set IP12_POSE_CACHE=<file> (and IP12_BUDGET_S=<seconds> per run)
import pickle, time
CACHE = os.environ.get("IP12_POSE_CACHE"); BUDGET = float(os.environ.get("IP12_BUDGET_S", "1e9"))
CHUNK = 50000; NCHUNKS = N_SAMPLES // CHUNK
state = {"done": 0, "poses": []}
if CACHE and os.path.exists(CACHE): state = pickle.load(open(CACHE, "rb"))
t0 = time.time()
while state["done"] < NCHUNKS:
    if time.time() - t0 > BUDGET:
        print(f"sampling {state['done']}/{NCHUNKS} chunks done - run the same command again to continue"); sys.exit(0)
    rng = np.random.default_rng(1 + state["done"])
    for _ in range(CHUNK):
        q = {a: rng.uniform(math.radians(lo), math.radians(hi)) for a, (lo, hi) in LIM.items()}
        g = fk(q); T = g("gripper_tcp"); P = T[:3, 3]
        if not (-0.25 < P[0] < 0.15 and -0.15 < P[1] < 0.15 and 0.80 < P[2] < 1.10) or T[2, 2] > -0.8:
            continue
        state["poses"].append({k: g(k)[:3, 3] for k in KEYS})
    state["done"] += 1
    if CACHE: pickle.dump(state, open(CACHE, "wb"))
poses = state["poses"]
print(f"Cam 1 at x={cam[0]:.3f} y={cam[1]:.3f} z={cam[2]:.3f} m, {len(poses)} sampled poses (gripper over the pallet, tool down)")


def visible(p, tilt):
    d = p - cam
    f = np.array([math.cos(tilt), 0, -math.sin(tilt)]); u = np.array([math.sin(tilt), 0, math.cos(tilt)])
    fw = d @ f
    return fw > 0.05 and abs(d[1]) / fw < tan_h and abs(d @ u) / fw < tan_v


for deg in TILTS:
    cnt = {k: sum(visible(P[k], math.radians(deg)) for P in poses) for k in KEYS}
    print(f"tilt {deg:4.0f} deg: " + "  ".join(f"{KEYS[k]} {100*cnt[k]/max(1, len(poses)):5.1f}%" for k in KEYS))
