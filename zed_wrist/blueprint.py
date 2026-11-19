"""Blueprint sheets (PNG, A3) for the 6 ZED wrist parts, first-angle projection (ISO E, as used in Germany):
front view, top view below it, left view to its right, all at one scale so every edge lines up across views
(thin projection lines, 45 deg mitre for depth). Chain dimensions with extension lines, hole sizes, small 3D view.
Reads <part>.step next to this script, writes blueprints/<part>.png."""
import numpy as np, matplotlib, os
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from build123d import import_step, Vector, GeomType
from drawings import NOTES

BG, FG, DIM, PRJ = "#123a6b", "white", "#9fd3ff", "#5f86b8"

def polylines(es, dx, dy):
    return [np.array([(p.X + dx, p.Y + dy) for p in e.positions(np.linspace(0, 1, 2 if e.geom_type == GeomType.LINE else 32))]) for e in es]

def distinct(vals, tol=0.3):
    out = []
    for v in sorted(vals):
        if not out or v - out[-1] > tol: out.append(v)
    out[0], out[-1] = min(vals), max(vals)     # the outline ends stay exact when an edge sits within tol of them
    return out

def chain(ax, vals, base, horiz, gap, sign):
    """chain dimensions edge to edge (row 1) and overall (row 2), drawn on the side `sign` (+1 above/right, -1 below/left)"""
    lo, hi = vals[0], vals[-1]
    if len(vals) == 2: base -= sign * gap      # nothing in between: overall row alone, closer in
    def dline(a, b, at, lvl):
        p, q = ((a, at), (b, at)) if horiz else ((at, a), (at, b))
        ax.annotate("", q, p, arrowprops=dict(arrowstyle="<|-|>", color=DIM, lw=0.6, mutation_scale=7, shrinkA=0, shrinkB=0))
        m, t = (a + b) / 2, f"{b - a:.1f}".rstrip("0").rstrip(".")
        off = sign * (gap * 0.1 + lvl * gap * 0.38)        # narrow gaps: label steps outward with a leader
        if lvl:
            ax.plot(*([[m, m], [at, at + off]] if horiz else [[at, at + off], [m, m]]), c=FG, lw=0.4)
        if horiz:
            ax.text(m, at + off, t, color=FG, fontsize=7.5, ha="center", va="bottom" if sign > 0 else "top")
        else:
            ax.text(at + off, m, t, color=FG, fontsize=7.5, va="center", rotation=90, ha="left" if sign > 0 else "right")
    r1, r2 = base + sign * gap, base + sign * 2.4 * gap
    ext = lambda v, a, b: ax.plot([v, v], [a, b], c=DIM, lw=0.35) if horiz else ax.plot([a, b], [v, v], c=DIM, lw=0.35)
    if len(vals) > 2:
        for v in vals: ext(v, base + sign * gap * 0.15, r1 + sign * gap * 0.15)
        lvl = 0
        for a, b in zip(vals, vals[1:]):
            lvl = (lvl + 1) % 3 if b - a < gap * 0.9 else 0
            dline(a, b, r1, lvl)
    for v in (lo, hi): ext(v, base + sign * gap * 0.15, r2 + sign * gap * 0.15)
    dline(lo, hi, r2, 0)

def project(part, d, up, dx, dy):
    """visible/hidden lines of one view, centred on the part's middle and moved to (dx, dy) on the sheet"""
    c = part.bounding_box().center()
    vis, hid = part.project_to_viewport(c + d * 20000, up, c)
    segs = [np.array([(p.X + dx, p.Y + dy) for p in e.vertices()]) for e in vis if e.geom_type == GeomType.LINE]
    us = [s[0, 0] for s in segs if len(s) == 2 and abs(s[0, 0] - s[1, 0]) < 0.01]   # edges square to the u axis
    vs = [s[0, 1] for s in segs if len(s) == 2 and abs(s[0, 1] - s[1, 1]) < 0.01]
    return polylines(vis, dx, dy), polylines(hid, dx, dy), us, vs

def holes(ax, part, d, to2d, dx, dy, gap, mid_u):
    """hole labels + centre lines for circles facing this view; returns full-hole centres for the dimension chains"""
    c = part.bounding_box().center(); cu, cv = to2d(c)
    found, arcs = {}, {}
    for e in part.edges():
        if e.geom_type != GeomType.CIRCLE or abs(abs(e.normal().dot(d)) - 1) > 1e-3: continue
        hu, hv = to2d(e.arc_center); key = (round(hu - cu + dx, 2), round(hv - cv + dy, 2)); dia = round(2 * e.radius, 1)
        if e.is_closed and dia < 12: found.setdefault(key, set()).add(dia)
        elif not e.is_closed and dia > 20: arcs.setdefault(key, set()).add(dia / 2)
    for (u, v), ds in found.items():
        r = max(ds) / 2
        ax.plot([u - r - 1.5, u + r + 1.5], [v, v], [u, u], [v - r - 1.5, v + r + 1.5], c=DIM, lw=0.4, ls="-.")
        sx = 1 if u >= mid_u else -1
        ax.annotate(" / ".join(f"Ø{x:g}" for x in sorted(ds)), (u, v), (u + sx * (gap * 0.6 + r), v + gap * 0.35), color=FG,
                    fontsize=7.5, ha="left" if sx > 0 else "right", arrowprops=dict(arrowstyle="-", color=FG, lw=0.5))
    for (u, v), rs in arcs.items():
        ax.plot(u, v, "+", c=FG, ms=6)
        ax.text(u + 2, v + 2, "\n".join(f"R{x:g}" for x in sorted(rs, reverse=True)), color=FG, fontsize=7.5, va="bottom")
    return [k[0] for k in found], [k[1] for k in found]

def sheet(name):
    part = import_step(f"{name}.step"); bb = part.bounding_box(optimal=True)
    X, Y, Z = bb.size.X, bb.size.Y, bb.size.Z
    g = max(X, Y, Z) / 14                     # dimension row spacing
    S = 4.2 * g                               # space between views, room for two dimension rows each side
    front = (0.0, 0.0)
    top = (0.0, -(Y / 2 + S + Z / 2))         # first angle: top view below the front view
    side = (X / 2 + S + Z / 2, 0.0)           # first angle: view from the left placed on the right
    fig = plt.figure(figsize=(16.5, 11.7), facecolor=BG)
    ax = fig.add_axes([0.03, 0.09, 0.66, 0.84]); ax.set_facecolor(BG)
    views = [("FRONT", Vector(0, 0, 1), Vector(0, 1, 0), lambda p: (p.X, p.Y), front),
             ("TOP", Vector(0, 1, 0), Vector(0, 0, -1), lambda p: (p.X, -p.Z), top),
             ("LEFT", Vector(-1, 0, 0), Vector(0, 1, 0), lambda p: (p.Z, p.Y), side)]
    data = {}
    for title, d, up, to2d, (dx, dy) in views:
        vis, hid, us, vs = project(part, d, up, dx, dy)
        for l in hid: ax.plot(l[:, 0], l[:, 1], c=FG, lw=0.4, ls=(0, (3, 2)), alpha=0.45)
        for l in vis: ax.plot(l[:, 0], l[:, 1], c=FG, lw=1.0)
        hu, hv = holes(ax, part, d, to2d, dx, dy, g, dx)
        w, h = (X, Y) if title == "FRONT" else (X, Z) if title == "TOP" else (Z, Y)
        u0, u1, v0, v1 = dx - w / 2, dx + w / 2, dy - h / 2, dy + h / 2
        data[title] = (distinct([u0, u1] + us + hu), distinct([v0, v1] + vs + hv), (u0, u1, v0, v1))
        ax.text(u0, v1 + (2.9 * g if title != "TOP" else 0.4 * g), f"{title} VIEW", color=FG, fontsize=11, weight="bold", va="bottom")
    # projection lines: X front<->top, Y front<->side, depth top -> 45 deg mitre -> side
    fu, fv, (fu0, fu1, fv0, fv1) = data["FRONT"]; tu, tv, (tu0, tu1, tv0, tv1) = data["TOP"]; su, sv, (su0, su1, sv0, sv1) = data["LEFT"]
    for u in distinct(fu + tu): ax.plot([u, u], [tv1, fv0], c=PRJ, lw=0.3, alpha=0.6)
    for v in distinct(fv + sv): ax.plot([fu1, su0], [v, v], c=PRJ, lw=0.3, alpha=0.6)
    xm_off = side[0] + top[1]                  # mitre: top-view height y maps to side-view x = xm_off - y
    for v in tv:
        x = xm_off - v
        ax.plot([tu1, x, x], [v, v, sv0], c=PRJ, lw=0.3, alpha=0.6)
    ys = np.array([tv0, tv1]); ax.plot(xm_off - ys, ys, c=PRJ, lw=0.6)
    # dimensions: X and Y on the front view, depth Z on the top (left side) and on the left view (top side)
    chain(ax, fu, fv1, True, g, +1);  chain(ax, fv, fu0, False, g, -1)
    chain(ax, tv, tu0, False, g, -1); chain(ax, tu, tv0, True, g, -1)
    chain(ax, su, sv1, True, g, +1);  chain(ax, sv, su1, False, g, +1)
    ax.set_aspect("equal"); ax.axis("off"); ax.margins(0.03)
    # small 3D view
    iso = fig.add_axes([0.71, 0.45, 0.27, 0.42]); iso.set_facecolor(BG)
    vis, _ = part.project_to_viewport(bb.center() + Vector(1, 0.8, 1).normalized() * 20000, Vector(0, 1, 0), bb.center())
    for l in polylines(vis, 0, 0): iso.plot(l[:, 0], l[:, 1], c=FG, lw=0.8)
    iso.set_aspect("equal"); iso.axis("off"); iso.set_title("3D VIEW", color=FG, fontsize=11, loc="left", weight="bold")
    # title block
    fig.text(0.02, 0.975, f"IP12  ZED MINI WRIST  —  {name.replace('_', ' ').upper()}", color=FG, fontsize=16, weight="bold", va="top")
    fig.text(0.71, 0.40, f"Overall   {X:.1f} (X) x {Y:.1f} (Y) x {Z:.1f} (Z) mm\nVolume    {part.volume / 1000:.1f} cm³\n"
             "Material  PLA / PETG, 0.2 mm layers\nProjection  first angle (ISO E / DIN)\nScale     one scale for all views, mm\n"
             "Y = gripper axis", color=FG, fontsize=10, family="monospace", va="top")
    fig.text(0.02, 0.012, NOTES[name] + "\nInner row: edge to edge (holes to their centre). Outer row: overall. Thin lines carry every edge across the views. "
             "Ø3.4 = M3 clearance, Ø2.5 = M3 pilot.", color=FG, fontsize=8.5, family="monospace", va="bottom")
    os.makedirs("blueprints", exist_ok=True)
    fig.savefig(f"blueprints/{name}.png", dpi=130, facecolor=BG); plt.close(fig)

if __name__ == "__main__":
    for n in NOTES: sheet(n); print("blueprint", n)
