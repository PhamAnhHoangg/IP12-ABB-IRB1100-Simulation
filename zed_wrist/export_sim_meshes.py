"""Meshes for the ROS simulation from the CAD in this folder:
  ../simulation/irb1100_vision/meshes/zimmer_lwr50l.stl   real Zimmer gripper (visual)
  ../simulation/irb1100_vision/meshes/zed_wrist_mount.stl the 7 printed parts, assembled
and prints the ZED Mini centre for urdf/zed_wrist.xacro.
STEP frame (Y = gripper axis, jaws -Y, cable +Z, mating face at Y = 18) -> gripper_link (z to the jaws, cable -y, z = 0 at the mating face):
  x = -X, y = -Z, z = 18 - Y   (mm; the URDF scales by 0.001)"""
import numpy as np, trimesh, tempfile, os
from build123d import import_step, export_stl, Compound

M = np.array([[-1, 0, 0, 0], [0, 0, -1, 0], [0, -1, 0, 18], [0, 0, 0, 1]], float)
OUT = "../simulation/irb1100_vision/meshes"

def mesh(shape, path):
    with tempfile.TemporaryDirectory() as d:
        export_stl(shape, f"{d}/m.stl", tolerance=0.08, angular_tolerance=0.3)
        m = trimesh.load(f"{d}/m.stl")
    m.apply_transform(M); m.export(path)
    print(f"{path}: {len(m.faces)} triangles, {os.path.getsize(path) / 1e6:.1f} MB")
    return m

mesh(import_step("LWR50L-23-00006-A.stp"), f"{OUT}/zimmer_lwr50l.stl")
parts = ["upper_ring_half_A", "upper_ring_half_B", "lower_ring_half_A", "lower_ring_half_B", "collar", "tilt_wedge", "zed_box"]
mesh(Compound(children=[import_step(f"{p}.step") for p in parts]), f"{OUT}/zed_wrist_mount.stl")
cam = next(c for c in import_step("gripper_with_zed_wrist_parts.step").children if "ZED" in c.label)
c = cam.bounding_box()
print("ZED Mini centre in gripper_link (m):", (M[:3, :3] @ np.array([c.center().X, c.center().Y, c.center().Z]) + M[:3, 3]).round(2) / 1000,
      "size (X, Y, Z) mm:", round(c.size.X, 1), round(c.size.Y, 1), round(c.size.Z, 1))
