#!/usr/bin/env python3
"""Build docs/cam1_viewer.html: one file, no install, opens in any browser (Windows, macOS, Linux, phone).

It shows what Cam 1 sees for each tilt angle and arm pose; click a tilt or a pose (or use the arrow keys) and the
picture changes at once. Pictures come from scripts/render_cam1_options.py (software render, framing only).

  python3 scripts/make_cam1_viewer.py [coverage.txt]
coverage.txt = output of scripts/check_cam_view.py (optional, adds the "in view" numbers).
"""
import base64, io, json, os, re, sys
from PIL import Image

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DOCS = os.path.join(PKG, "..", "..", "docs")
CACHE = os.path.join(DOCS, "img", "cam1_options.png.cache")
poses = json.load(open(os.path.join(CACHE, "poses.json")))
names = [p[0] for p in poses]
rows = {}
for f in os.listdir(CACHE):
    m = re.match(r"row_(\d+)\.png", f)
    if m: rows[int(m.group(1))] = os.path.join(CACHE, f)
tilts = sorted(rows)
W = 640
imgs = {}
for t in tilts:
    row = Image.open(rows[t]).convert("RGB")
    for i, n in enumerate(names):
        buf = io.BytesIO(); row.crop((i * W, 0, (i + 1) * W, row.height)).save(buf, "JPEG", quality=82)
        imgs[f"{t}|{i}"] = "data:image/jpeg;base64," + base64.b64encode(buf.getvalue()).decode()
cov = {}
if len(sys.argv) > 1 and os.path.exists(sys.argv[1]):
    for line in open(sys.argv[1]):
        m = re.match(r"tilt\s+(\d+) deg:\s+gripper_tcp\s+([\d.]+)%\s+wrist_link_5\s+([\d.]+)%\s+elbow_link_2\s+([\d.]+)%", line)
        if m: cov[int(m.group(1))] = [float(m.group(2)), float(m.group(3)), float(m.group(4))]

html = """<!DOCTYPE html><html lang="en"><head><meta charset="utf-8"><title>IP12 Cam 1 viewer</title>
<style>
body{font-family:Segoe UI,Arial,sans-serif;margin:0;background:#f4f5f7;color:#1c1f24}
header{background:#1c1f24;color:#fff;padding:10px 18px;font-size:17px}
main{max-width:1000px;margin:14px auto;padding:0 14px}
.row{display:flex;flex-wrap:wrap;gap:8px;align-items:center;margin:10px 0}
.lbl{width:110px;font-weight:600}
button{border:1px solid #9aa1ab;background:#fff;border-radius:6px;padding:7px 14px;font-size:15px;cursor:pointer}
button.on{background:#d9541e;border-color:#d9541e;color:#fff}
img{width:100%;max-width:960px;border:1px solid #c9ccd2;border-radius:6px;background:#c4c8ce;display:block}
.cov{display:flex;gap:10px;margin:10px 0;flex-wrap:wrap}
.card{background:#fff;border:1px solid #d5d8dd;border-radius:8px;padding:8px 14px;min-width:150px}
.card b{font-size:22px}.note{font-size:13px;color:#555;margin-top:12px;line-height:1.45}
</style></head><body><header>IP12 &ndash; Cam 1 view for each tilt angle and arm pose</header><main>
<div class="row"><span class="lbl">Tilt (down)</span><span id="tilts"></span></div>
<div class="row"><span class="lbl">Arm pose</span><span id="poses"></span></div>
<img id="pic" alt="Cam 1 view">
<div class="cov" id="cov"></div>
<div class="note">Click a tilt or a pose, or use the arrow keys (left/right = tilt, up/down = pose).<br>
Software render of the CAD model: it shows framing only, not real camera image quality. Arm poses were found by a search
and are within about 4&ndash;6&nbsp;cm of the described gripper position. &ldquo;In view&rdquo; = share of about 1600 random poses
(gripper over the pallet) in which the point is inside the picture (link origins only).</div></main>
<script>
const IMGS=__IMGS__,TILTS=__TILTS__,POSES=__POSES__,COV=__COV__;
let t=TILTS.indexOf(0)>=0?TILTS.indexOf(0):0,p=0;
function btn(host,items,get,set){host.innerHTML="";items.forEach((x,i)=>{const b=document.createElement("button");b.textContent=x;b.className=get()==i?"on":"";b.onclick=()=>{set(i);draw()};host.appendChild(b)})}
function draw(){
 btn(document.getElementById("tilts"),TILTS.map(x=>x+"\\u00b0"),()=>t,i=>t=i);
 btn(document.getElementById("poses"),POSES,()=>p,i=>p=i);
 document.getElementById("pic").src=IMGS[TILTS[t]+"|"+p];
 const c=COV[TILTS[t]],h=document.getElementById("cov");
 h.innerHTML=c?[["Gripper tip in view",c[0]],["Wrist in view",c[1]],["Elbow in view",c[2]]].map(a=>`<div class="card">${a[0]}<br><b>${a[1].toFixed(0)}%</b></div>`).join(""):"";
}
document.onkeydown=e=>{if(e.key=="ArrowRight")t=Math.min(TILTS.length-1,t+1);else if(e.key=="ArrowLeft")t=Math.max(0,t-1);
 else if(e.key=="ArrowDown")p=Math.min(POSES.length-1,p+1);else if(e.key=="ArrowUp")p=Math.max(0,p-1);else return;draw()};
draw();
</script></body></html>"""
html = (html.replace("__IMGS__", json.dumps(imgs)).replace("__TILTS__", json.dumps(tilts))
        .replace("__POSES__", json.dumps(names)).replace("__COV__", json.dumps(cov)))
out = os.path.join(DOCS, "cam1_viewer.html"); open(out, "w", encoding="utf-8").write(html)
print("saved", os.path.abspath(out), f"{len(html)/1e6:.1f} MB, tilts {tilts}")
