#!/usr/bin/env python3
# @ip12:tool  writes static/irb1100.glb and static/gripper.glb for the web 3D view (WP3 API: GET /config/robot-model)
"""Robot and gripper as glTF binaries for Babylon.js: one node per link, nested like the joints, units mm, z up.
   python3 scripts/export_web_model.py          (needs trimesh; run again after a mesh or joint change)
irb1100.glb: base > link_1 > ... > link_6, each node at its joint, so turning a node about the joint axis moves the arm
            (all link frames sit at their joint and are not rotated in the zero pose, urdf/irb1100.xacro).
gripper.glb: in the ABB flange frame tool0 (z out of the flange): Zimmer body, jaws (open), ZED mount (straight).
"""
import os
import numpy as np, trimesh

HERE = os.path.dirname(os.path.abspath(__file__))
M, OUT = os.path.join(HERE, "..", "meshes"), os.path.join(HERE, "..", "static")
# joint origins relative to the previous link, mm (urdf/irb1100.xacro)
CHAIN = [("base", None, "irb1100/base.stl", (0, 0, 0)), ("link_1", "base", "irb1100/link1.stl", (0, 0, 0)),
         ("link_2", "link_1", "irb1100/link2.stl", (0, 0, 327)), ("link_3", "link_2", "irb1100/link3.stl", (0, 0, 280)),
         ("link_4", "link_3", "irb1100/link4.stl", (0, 0, 10)), ("link_5", "link_4", "irb1100/link5.stl", (300, 0, 0)),
         ("link_6", "link_5", "irb1100/link6.stl", (0, 0, 0))]
WHITE, DARK, GREY = [235, 235, 232, 255], [45, 45, 48, 255], [150, 150, 150, 255]


def load(f, colour):
    m = trimesh.load(os.path.join(M, f), force="mesh"); m.visual.face_colors = colour; return m


def chain_scene():
    s = trimesh.Scene()
    for name, parent, f, (x, y, z) in CHAIN:
        T = np.eye(4); T[:3, 3] = (x, y, z)
        s.add_geometry(load(f, WHITE), node_name=name, geom_name=name + "_mesh", parent_node_name=parent or "world", transform=T)
    return s


def gripper_scene():
    s = trimesh.Scene()
    for name, f, c in (("gripper", "zimmer_lwr50l.stl", DARK), ("jaw_left", "zimmer_jaw_left.stl", GREY),
                       ("jaw_right", "zimmer_jaw_right.stl", GREY), ("zed_mount", "zed_wrist_mount_tilt0.stl", DARK)):
        s.add_geometry(load(f, c), node_name=name, geom_name=name + "_mesh")
    return s


if __name__ == "__main__":
    os.makedirs(OUT, exist_ok=True)
    for name, scene in (("irb1100.glb", chain_scene()), ("gripper.glb", gripper_scene())):
        p = os.path.join(OUT, name); open(p, "wb").write(trimesh.exchange.gltf.export_glb(scene))
        back = trimesh.load(p)                     # self-check: nodes survive the round trip
        print(name, f"{os.path.getsize(p) / 1e6:.1f} MB", sorted(n for n in back.graph.nodes if not n.endswith("_mesh"))[:9])
