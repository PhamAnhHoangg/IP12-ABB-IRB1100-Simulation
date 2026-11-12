#!/usr/bin/env python3
"""Contact sheet: what Cam 1 sees for each tilt option while the arm is in four poses (no ROS, no Gazebo).

  pip install xacro yourdfpy trimesh pillow numpy
  python3 scripts/render_cam1_options.py [out.png] [tilt tilt ...]      # default tilts: 0 15 20 30

Rows = Cam 1 tilt (degrees down), columns = arm poses (home, pick over pallet centre, pick at a pallet corner,
place high). Software renderer (painter's algorithm), so it shows shapes and framing only, not real image quality.
"""
import math, os, sys, tempfile, types, xml.etree.ElementTree as ET
import numpy as np
from PIL import Image, ImageDraw, ImageFont

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
m_pkg = types.ModuleType("ament_index_python.packages"); m_pkg.get_package_share_directory = lambda n: PKG
m_top = types.ModuleType("ament_index_python"); m_top.packages = m_pkg
sys.modules["ament_index_python"] = m_top; sys.modules["ament_index_python.packages"] = m_pkg
import xacro, yourdfpy, trimesh  # noqa: E402

OUT = sys.argv[1] if len(sys.argv) > 1 else os.path.join(PKG, "..", "..", "docs", "img", "cam1_options.png")
TILTS = [float(a) for a in sys.argv[2:]] or [0, 15, 20, 30]
W, H, HFOV = 640, 360, 1.2
LIM = {"joint_1": (-230, 230), "joint_2": (-115, 113), "joint_3": (-205, 55),
       "joint_4": (-230, 230), "joint_5": (-125, 120), "joint_6": (-400, 400)}
TARGETS = [("home", None), ("pick, pallet centre", (-0.05, 0.0, 0.93)),
           ("pick, pallet corner", (-0.15, -0.10, 0.90)), ("place, high", (0.0, 0.05, 1.10))]
HOME_Q = [1.5708, 0, 0, 0, 0, 0]


def expand(tilt):
    xml = xacro.process_file(os.path.join(PKG, "urdf", "workcell.urdf.xacro"),
                             mappings={"controllers_yaml": "/x", "cam1_tilt_deg": str(tilt)}).toxml()
    f = tempfile.NamedTemporaryFile("w", suffix=".urdf", delete=False); f.write(xml); f.close()
    return f.name, ET.fromstring(xml)


def load(path, root):
    mats = {m.get("name"): [float(x) for x in m.find("color").get("rgba").split()]
            for m in root.findall("material") if m.find("color") is not None}
    h = lambda fname: fname.replace("package://irb1100_vision/", PKG + "/")
    u = yourdfpy.URDF.load(path, build_scene_graph=True, load_meshes=True,
                           build_collision_scene_graph=False, filename_handler=h)
    return u, mats, h


def scene(u, mats, h):
    tris, cols, nrm = [], [], []
    for ln, l in u.link_map.items():
        T = u.get_transform(ln, "world")
        for v in l.visuals:
            g = v.geometry
            if g.box is not None: m = trimesh.creation.box(g.box.size)
            elif g.cylinder is not None: m = trimesh.creation.cylinder(g.cylinder.radius, g.cylinder.length, sections=24)
            elif g.mesh is not None:
                m = trimesh.load(h(g.mesh.filename), process=False)
                m.apply_scale(g.mesh.scale if g.mesh.scale is not None else 1)
            else: continue
            m.apply_transform(T @ (v.origin if v.origin is not None else np.eye(4)))
            c = mats.get(v.material.name, [.6, .6, .6, 1])[:3] if v.material and v.material.name else [.6, .6, .6]
            tris.append(m.vertices[m.faces]); nrm.append(m.face_normals); cols += [c] * len(m.faces)
    return np.concatenate(tris), np.concatenate(nrm), np.array(cols)


def render(S, T):
    tris, nrm, cols = S; inv = np.linalg.inv(T)
    P = ((inv[:3, :3] @ tris.reshape(-1, 3).T).T + inv[:3, 3]).reshape(-1, 3, 3)
    z = P[:, :, 2]; ok = (z > 0.05).all(1); f = (W / 2) / math.tan(HFOV / 2)
    uv = np.stack([P[:, :, 0] / np.maximum(z, 1e-6) * f + W / 2, P[:, :, 1] / np.maximum(z, 1e-6) * f + H / 2], -1)
    Lw = np.array([0.3, -0.5, 0.8]); Lw /= np.linalg.norm(Lw)
    shade = 0.45 + 0.55 * np.abs(nrm @ Lw)
    idx = np.where(ok)[0]; idx = idx[np.argsort(-z.mean(1)[idx])]
    img = Image.new("RGB", (W, H), (196, 200, 206)); d = ImageDraw.Draw(img)
    for i in idx:
        d.polygon([tuple(p) for p in uv[i]], fill=tuple(int(255 * min(1, c * shade[i])) for c in cols[i]))
    return img


# ---- arm poses: random search with a small numpy forward-kinematics --------------------------------
def make_fk(root):
    J = {}
    for j in root.findall("joint"):
        o = j.find("origin")
        xyz = [float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]
        rp = [float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
        cr, sr, cp, sp, cy, sy = math.cos(rp[0]), math.sin(rp[0]), math.cos(rp[1]), math.sin(rp[1]), math.cos(rp[2]), math.sin(rp[2])
        T = np.eye(4); T[:3, :3] = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr], [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]]
        T[:3, 3] = xyz
        ax = j.find("axis")
        J[j.find("child").get("link")] = (j.get("name"), j.find("parent").get("link"), T,
                                          np.array([float(v) for v in ax.get("xyz").split()]) if ax is not None else None)
    def tcp(q):
        chain = []; l = "gripper_tcp"
        while l in J: chain.append(l); l = J[l][1]
        M = np.eye(4)
        for l in reversed(chain):
            n, _, T, a = J[l]; M = M @ T
            if n in q:
                a = a / np.linalg.norm(a); K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
                R = np.eye(4); R[:3, :3] = np.eye(3) + math.sin(q[n])*K + (1-math.cos(q[n]))*K@K; M = M @ R
        return M
    return tcp


def find_poses(root):
    tcp = make_fk(root); rng = np.random.default_rng(3)
    names = list(LIM); best = {t[0]: (9.0, None) for t in TARGETS[1:]}
    for _ in range(120000):
        q = [rng.uniform(math.radians(a), math.radians(b)) for a, b in LIM.values()]
        M = tcp(dict(zip(names, q)))
        if M[2, 2] > -0.95: continue            # tool must point down
        for name, tg in TARGETS[1:]:
            e = np.linalg.norm(M[:3, 3] - np.array(tg))
            if e < best[name][0]: best[name] = (e, q)
    poses = [("home", HOME_Q)]
    for name, _ in TARGETS[1:]:
        e, q = best[name]; print(f"  pose '{name}': gripper tip within {e*100:.1f} cm of target", flush=True); poses.append((name, q))
    return poses


def main():
    """Resumable: poses and finished rows are cached next to the output, so if a time limit stops the run just start it again."""
    import json, time
    t0 = time.time(); budget = float(os.environ.get("IP12_BUDGET_S", "1e9"))
    cache = os.path.abspath(OUT) + ".cache"; os.makedirs(cache, exist_ok=True)
    try: font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 20)
    except Exception: font = ImageFont.load_default()
    pf = os.path.join(cache, "poses.json")
    if os.path.exists(pf): poses = json.load(open(pf))
    else:
        _, root0 = expand(TILTS[0]); poses = find_poses(root0); json.dump(poses, open(pf, "w"))
    for tilt in TILTS:
        rf = os.path.join(cache, f"row_{tilt:.0f}.png")
        if os.path.exists(rf): continue
        if time.time() - t0 > budget: print("time budget used - run the same command again to continue", flush=True); return
        path, root = expand(tilt); u, mats, h = load(path, root)
        row = Image.new("RGB", (W * len(poses), H), (255, 255, 255))
        for c, (pname, q) in enumerate(poses):
            u.update_cfg(np.array(q, float))
            row.paste(render(scene(u, mats, h), u.get_transform("cam1_optical_frame", "world")), (c * W, 0))
        row.save(rf); print(f"tilt {tilt:.0f} done", flush=True)
    sheet = Image.new("RGB", (W * len(poses) + 70, (H + 6) * len(TILTS) + 36), (255, 255, 255)); sd = ImageDraw.Draw(sheet)
    for c, (pname, _) in enumerate(poses): sd.text((70 + c * W + 10, 6), pname, fill=(0, 0, 0), font=font)
    for r, tilt in enumerate(TILTS):
        sd.text((6, 36 + r * (H + 6) + H // 2 - 10), f"{tilt:.0f} deg", fill=(0, 0, 0), font=font)
        sheet.paste(Image.open(os.path.join(cache, f"row_{tilt:.0f}.png")), (70, 36 + r * (H + 6)))
    os.makedirs(os.path.dirname(os.path.abspath(OUT)), exist_ok=True); sheet.save(OUT); print("saved", os.path.abspath(OUT), flush=True)


main()
