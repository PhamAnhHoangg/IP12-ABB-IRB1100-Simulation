"""Drawing sheets (PNG) for the 6 ZED wrist parts + an exploded assembly view. Reads the STEP files next to this script."""
import numpy as np, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from build123d import import_step, Vector, Pos

NOTES = {
    "upper_ring_half_A": "Ring r38.5-44, 17.3 thick. Tongue 4.6 thick into the head slot.\nBoss for collar: 2x pilot 2.5 (M3 self-tap). Ear: hole 3.4 (M3x20 + nut).\nLap joint: 2x pilot 2.5. Print tongue side down, break-away block under the lap tab.",
    "upper_ring_half_B": "Ring r38.5-44, 17.3 thick. Tongue 4.6 thick into the head slot.\nEar: hole 3.4 (M3x20 + nut). Lap joint: 2x clearance 3.4 (M3x12).\nCable notch 8 wide. Print tongue side down.",
    "lower_ring_half_A": "Band inside 26.4 x 54.2, wall 4, 9.5 tall. Key 2.7 into the sensor slot.\nEars: 2x hole 3.4 (M3x20 + nut). Boss for collar: 2x pilot 2.5.\nPrint flat end down.",
    "lower_ring_half_B": "Band inside 26.4 x 54.2, wall 4, 9.5 tall. Key 2.7 into the sensor slot.\nEars: 2x hole 3.4 (M3x20 + nut). Print flat end down.",
    "collar": "6 thick, 24 wide. Upper pad: 2x 3.4 with 6.4 x 3.2 counterbore (M3x10).\nLower pad: 2x 3.4 (M3x12). Box face: 2x pilot 2.5 (M3x8 countersunk).\nPrint on a large flat side.",
    "tilt_wedge": "Sets the camera angle (25 deg; TILT_DEG in zed_wrist_parts.py, 0 = flat spacer). 5 thick at the lower edge, 40 wide.\nTo collar: 2x 3.4 with 6.4 counterbore, M3x6. Box screws: 2x pilot 2.5, square to the slanted face.\nPrint on the collar side (flat face down).",
    "zed_box": "Inside 33 x 125.5 x 27.5, wall 3, open bottom, 6 mm lips at both ends.\nUSB-C window 14 x 10 at both ends. 2x 3.4 countersunk from inside at z +-16 (M3x8 into the wedge).\nPrint upside down (top on bed).",
}
VIEWS = [("Front (look along -Z)", Vector(0, 0, 1), Vector(0, 1, 0), (0, 1)),
         ("Top (look down the tool axis)", Vector(0, 1, 0), Vector(0, 0, -1), (0, 2)),
         ("Side (look along -X)", Vector(1, 0, 0), Vector(0, 1, 0), (2, 1)),
         ("Isometric", Vector(1, 0.8, 1), Vector(0, 1, 0), None)]

def lines(shape, d, up):
    c = shape.bounding_box().center()
    vis, hid = shape.project_to_viewport(c + d.normalized() * 20000, up, c)
    pts = lambda es: [np.array([(p.X, p.Y) for p in e.positions(np.linspace(0, 1, 24))]) for e in es]
    return pts(vis), pts(hid)

def dim(ax, a, b, off, horiz, text):
    (x0, y0), (x1, y1) = a, b
    if horiz:
        ax.annotate("", (x0, off), (x1, off), arrowprops=dict(arrowstyle="<->", lw=0.7))
        ax.plot([x0, x0], [y0, off], [x1, x1], [y1, off], lw=0.4, c="0.4")
        ax.text((x0 + x1) / 2, off, text, ha="center", va="bottom", fontsize=8, backgroundcolor="w")
    else:
        ax.annotate("", (off, y0), (off, y1), arrowprops=dict(arrowstyle="<->", lw=0.7))
        ax.plot([x0, off], [y0, y0], [x1, off], [y1, y1], lw=0.4, c="0.4")
        ax.text(off, (y0 + y1) / 2, text, ha="right", va="center", fontsize=8, rotation=90, backgroundcolor="w")

def sheet(name):
    part = import_step(f"{name}.step")
    bb = part.bounding_box()
    fig, axs = plt.subplots(2, 2, figsize=(16.5, 11.7))   # A3 landscape
    for ax, (title, d, up, dims) in zip(axs.flat, VIEWS):
        vis, hid = lines(part, d, up)
        for l in hid: ax.plot(l[:, 0], l[:, 1], c="0.6", lw=0.5, ls="--")
        for l in vis: ax.plot(l[:, 0], l[:, 1], c="k", lw=0.9)
        allp = np.vstack(vis); (x0, y0), (x1, y1) = allp.min(0), allp.max(0)
        if dims:
            size = [bb.size.X, bb.size.Y, bb.size.Z]
            dim(ax, (x0, y0), (x1, y0), y0 - 0.12 * (y1 - y0) - 4, True, f"{size[dims[0]]:.1f}")
            dim(ax, (x0, y0), (x0, y1), x0 - 0.08 * (x1 - x0) - 4, False, f"{size[dims[1]]:.1f}")
        ax.set_title(title, fontsize=11); ax.set_aspect("equal"); ax.axis("off")
        ax.margins(0.18)
    fig.suptitle(f"IP12 ZED Mini wrist  |  {name.replace('_', ' ')}  |  {bb.size.X:.1f} x {bb.size.Y:.1f} x {bb.size.Z:.1f} mm  |  {part.volume / 1000:.1f} cm3",
                 fontsize=15, weight="bold")
    fig.text(0.02, 0.015, NOTES[name] + "\nAll dimensions in mm. Holes: 3.4 = M3 clearance, 2.5 = pilot for an M3 screw cut into the plastic. Axes: Y = gripper axis.",
             fontsize=9, family="monospace", va="bottom")
    fig.savefig(f"drawings/{name}.png", dpi=110); plt.close(fig)

def exploded():
    shift = {"upper_ring_half_A": (40, 0, 0), "upper_ring_half_B": (-40, 0, 0), "lower_ring_half_A": (30, -25, 0),
             "lower_ring_half_B": (-30, -25, 0), "collar": (45, -10, 0), "tilt_wedge": (85, 20, 0), "zed_box": (135, 45, 0)}
    fig, ax = plt.subplots(figsize=(16.5, 11.7))
    d, up = Vector(0.9, 0.5, 1.2), Vector(0, 1, 0)
    grip = import_step("LWR50L-23-00006-A.stp")
    for l in lines(grip, d, up)[0]: ax.plot(l[:, 0], l[:, 1], c="0.75", lw=0.4)
    colors = dict(zip(shift, ["tab:blue", "tab:cyan", "tab:green", "tab:olive", "tab:orange", "tab:purple", "tab:red"]))
    # project everything about one shared centre so the parts keep their places
    c = grip.bounding_box().center()
    for n, s in shift.items():
        p = Pos(*s) * import_step(f"{n}.step")
        vis, _ = p.project_to_viewport(c + d.normalized() * 20000, up, c)
        pl = [np.array([(q.X, q.Y) for q in e.positions(np.linspace(0, 1, 24))]) for e in vis]
        for l in pl: ax.plot(l[:, 0], l[:, 1], c=colors[n], lw=0.8)
        m = np.vstack(pl).max(0)
        ax.text(m[0] + 3, m[1], n.replace("_", " "), color=colors[n], fontsize=11, weight="bold")
    ax.set_aspect("equal"); ax.axis("off")
    ax.set_title("IP12 ZED Mini wrist on Zimmer LWR50L-23-00006-A  |  exploded view (grey = gripper)", fontsize=15, weight="bold")
    fig.savefig("drawings/exploded_assembly.png", dpi=110); plt.close(fig)

if __name__ == "__main__":
    for n in NOTES: sheet(n); print("drew", n)
    exploded(); print("drew exploded_assembly")
