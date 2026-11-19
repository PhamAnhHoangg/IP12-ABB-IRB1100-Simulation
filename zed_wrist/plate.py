"""All 6 print-ready STLs laid out on one 220 x 220 bed (5 mm apart): plate_all_parts.3mf (separate objects) + .stl."""
import trimesh
from trimesh.transformations import rotation_matrix
# name, (x, y) of the part's lower-left corner on the bed, turned 90 deg?
LAYOUT = [("zed_box", 0, 0, False), ("upper_ring_half_A", 44, 0, False), ("upper_ring_half_B", 109, 0, False),
          ("collar", 168, 0, False), ("lower_ring_half_A", 0, 136.5, True), ("lower_ring_half_B", 87, 136.5, True)]
BED = 220
scene, meshes = trimesh.Scene(), []
for name, x, y, turn in LAYOUT:
    m = trimesh.load(f"{name}.stl")
    if turn: m.apply_transform(rotation_matrix(1.5708, [0, 0, 1]))
    m.apply_translation([x - m.bounds[0][0], y - m.bounds[0][1], -m.bounds[0][2]])
    scene.add_geometry(m, node_name=name, geom_name=name); meshes.append(m)
lo, hi = trimesh.util.concatenate(meshes).bounds
assert hi[0] <= BED and hi[1] <= BED and lo.min() >= -1e-6, f"does not fit the {BED} bed: {hi}"
for a in range(len(meshes)):                    # 2D footprints must not overlap
    for b in range(a + 1, len(meshes)):
        (a0, a1), (b0, b1) = meshes[a].bounds, meshes[b].bounds
        assert a1[0] <= b0[0] or b1[0] <= a0[0] or a1[1] <= b0[1] or b1[1] <= a0[1], f"{LAYOUT[a][0]} overlaps {LAYOUT[b][0]}"
scene.export("plate_all_parts.3mf"); trimesh.util.concatenate(meshes).export("plate_all_parts.stl")
print(f"plate {hi[0]:.0f} x {hi[1]:.0f} x {hi[2]:.1f} mm, {len(meshes)} parts")
