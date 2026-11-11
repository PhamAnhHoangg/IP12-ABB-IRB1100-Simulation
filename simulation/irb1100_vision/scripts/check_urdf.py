#!/usr/bin/env python3
"""Offline check of the workcell model (no ROS install needed, only `pip install xacro`).

  python3 scripts/check_urdf.py

Expands urdf/workcell.urdf.xacro and checks: XML parses, every mesh file exists, the six arm joints
exist with the ABB axis limits, tool0 / gripper / camera frames exist, no link is orphaned.
Exit code 0 = all checks passed.
"""
import math
import os
import sys
import types
import xml.etree.ElementTree as ET

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# stand-in for ROS's ament_index so that $(find irb1100_vision) works without ROS
m_pkg = types.ModuleType("ament_index_python.packages")
m_pkg.get_package_share_directory = lambda name: PKG
m_top = types.ModuleType("ament_index_python")
m_top.packages = m_pkg
sys.modules["ament_index_python"] = m_top
sys.modules["ament_index_python.packages"] = m_pkg

import xacro  # noqa: E402

errors = []


def check(ok, msg):
    print(("PASS  " if ok else "FAIL  ") + msg)
    if not ok:
        errors.append(msg)


doc = xacro.process_file(os.path.join(PKG, "urdf", "workcell.urdf.xacro"),
                         mappings={"controllers_yaml": os.path.join(PKG, "config", "controllers.yaml")})
root = ET.fromstring(doc.toxml())

links = {l.get("name") for l in root.findall("link")}
joints = {j.get("name"): j for j in root.findall("joint")}
check(len(links) > 20, f"model expands ({len(links)} links, {len(joints)} joints)")

# meshes exist
missing = []
for mesh in root.iter("mesh"):
    f = mesh.get("filename", "")
    path = f.replace("package://irb1100_vision/", PKG + "/")
    if not os.path.isfile(path):
        missing.append(f)
check(not missing, "all mesh files exist" + (f" (missing: {missing[:3]})" if missing else ""))

# arm joints and ABB limits (deg): from ABB product specification IRB 1100
abb = {"joint_1": (-230, 230), "joint_2": (-115, 113), "joint_3": (-205, 55),
       "joint_4": (-230, 230), "joint_5": (-125, 120), "joint_6": (-400, 400)}
for name, (lo, hi) in abb.items():
    j = joints.get(name)
    if j is None:
        check(False, f"{name} exists")
        continue
    lim = j.find("limit")
    ok = (lim is not None and abs(math.degrees(float(lim.get("lower"))) - lo) < 0.1
          and abs(math.degrees(float(lim.get("upper"))) - hi) < 0.1)
    check(ok, f"{name} limits {lo}..{hi} deg")

for frame in ["tool0", "gripper_link", "cam1_link", "cam1_optical_frame",
              "cam2_left_optical_frame", "cam2_right_optical_frame"]:
    check(frame in links, f"frame {frame} exists")

# every non-root link has exactly one parent joint
children = [j.find("child").get("link") for j in joints.values()]
check(len(children) == len(set(children)), "no link has two parents")
orphans = links - set(children) - {"world"}
check(not orphans, "no orphan links" + (f" ({sorted(orphans)})" if orphans else ""))

# gazebo needs the fixed camera joints kept
gz_refs = {g.get("reference") for g in root.findall("gazebo")}
check("cam1_mount_to_link" in gz_refs, "camera mount joints are preserved for Gazebo")

print("\nALL CHECKS PASSED" if not errors else f"\n{len(errors)} CHECK(S) FAILED")
sys.exit(1 if errors else 0)
