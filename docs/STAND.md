# Camera stand (all 3D printed triangle truss)

One rigid stand clamped to the table's left edge (x = -0.55 m), made of two triangles:

* **Lower right triangle (A-B-C):** the base A-C lies on the table (0.22 m), the upright A-B stands **0.10 m outside the table edge**, the hypotenuse B-C closes it. Cam 1 (horizontal) sits on the upright just above B.
* **Upper inverted triangle (K-T-E):** the same upright continues to T, the boom T-E (0.54 m) runs over the pallet and the brace E-K closes the triangle. A short post hangs down from E and carries Cam 2 (ZED Mini) looking down, 0.8 m above the pallet.

![stand side view](img/stand_side.png)

All members are 25 x 25 mm hollow printed bars pushed into printed corner nodes and couplers, one M8 bolt per socket. Bars carry a hole every 20 mm; the sockets have 20 mm slots, so lengths can be trimmed or shifted in 20 mm steps. Camera heights are adjustable: Cam 1 mount slides on the upright between nodes B and K, Cam 2 mount slides on its 0.2 m post. Different triangle sizes: change the numbers at the top of `simulation/irb1100_vision/cad/truss_stand.py` and rerun it (it rebuilds all parts, the simulation model and this list).

Member lengths (centre to centre): A-C 0.22 m, A-B 0.277 m, B-C 0.354 m, B-K 0.34 m, K-T 0.31 m, T-E 0.542 m, E-K 0.624 m.

## Parts to print (in `simulation/irb1100_vision/meshes/`)

| File | Qty |
|---|---|
| `stand_clamp.stl` (print on its back) | 1 |
| `stand_clamp_knob.stl` | 1 |
| `stand_node_A/B/C/K/T/E.stl` | 1 each |
| `stand_coupler.stl` | 4 |

Bars (print lying flat; the longer ones diagonally on a 220 mm bed):

| File | Qty |
|---|---|
| `stand_bar_94mm.stl` | 1 |
| `stand_bar_101mm.stl` | 1 |
| `stand_bar_106mm.stl` | 1 |
| `stand_bar_110mm.stl` | 1 |
| `stand_bar_134mm.stl` | 1 |
| `stand_bar_182mm.stl` | 1 |
| `stand_bar_184mm.stl` | 1 |
| `stand_bar_200mm.stl` | 1 |
| `stand_bar_212mm.stl` | 1 |
| `stand_bar_225mm.stl` | 1 |
| `stand_bar_227mm.stl` | 1 |
| `stand_bar_248mm.stl` | 1 |

Suggested settings (not tested): PETG, 4 perimeters, 40 % infill, 0.2 mm layers.

## Hardware that is not printed

| Item | Qty |
|---|---|
| M8 x 50 bolt + nut + 2 washers | about 25 (one per socket and 2 in the clamp saddle) |
| M10 x 70 bolt + nut (clamp screw, nut sits in the lower jaw pocket) | 1 |

## Assembly order

1. Clamp on the table edge, base bar through the clamp saddle (bolt 2 holes), node A and C on its ends.
2. Upright pieces and couplers up to B, then slide the Cam 1 mount onto the upright **before** fitting node K.
3. Hypotenuse B-C. Then K, T, boom, brace; hang the Cam 2 post from node E and slide the Cam 2 mount on from below.

## Honest limits

* Nodes fix the triangle angles. Only lengths (20 mm steps) and camera heights adjust; a different shape needs reprinting the nodes.
* Planar truss: it is stiff in its own plane, much less sideways. Expect little wobble at the boom (estimated about 1 to 2 mm sideways for 70 g at the end), but this is not measured.
* Single-bolt sockets have some play. Add a second bolt through the long sockets if you feel wobble.
* Nothing has been printed or tested; print node B and one bar first to check fit. Printed sizes: the largest nodes are about 116 x 116 mm.
* Cam 1 view is clear of the brace and hypotenuse in this model (checked by geometry only, no render from the camera).
