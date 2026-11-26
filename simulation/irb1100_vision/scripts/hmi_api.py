#!/usr/bin/env python3
# @ip12:vision  WP2 part of the IP12 web API (WP3 API catalogue): camera health, scans, scene, robot model, cell geometry
"""Perception and model data for the web interface, in the format of the IP12 API catalogue (WP3 UX/HMI).
   ros2 run irb1100_vision hmi_api.py --ros-args -p port:=8000
REST under /api, JSON, snake_case, lengths mm, angles deg, timestamps ISO 8601 UTC. Cell frame = world (robot base at
(200, 0, 750) mm in it, see /api/config/robot-model base_pose). Endpoints:
  GET  /api/health                camera ok / offline (our part of /health; robot and package system are not ours)
  POST /api/scans                 202 {scan_id}: takes a fresh picture + point cloud where the camera is NOW and builds
                                  the scene. Moving the robot to the scan pose is the robot side's job. Idempotency-Key ok.
  GET  /api/scans/{id}            {id, state, origin, started_at, finished_at}
  GET  /api/scene                 objects (pose, location, identification) of the last completed scan
  GET  /api/config/robot-model    joints, limits, base pose, tool, model files
  GET  /api/config/scenario       cell geometry (table, pallet, source area, robot base); slots: [] (planner's choice)
  GET  /static/irb1100.glb, /static/gripper.glb
  WS   /api/events                {type, seq, timestamp, data}: scan.status_changed, scene.updated, error.raised
Identification reads a QR code on each box top (OpenCV). unreadable_reason: bad_light (too dark / bright / flat),
too_far (the box top is too few pixels for a code), damaged (code found, not decodable), covered (no code found),
unknown (box cut off at the picture edge).
"""
import base64, hashlib, json, math, os, queue, struct, threading, time, uuid
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import cv2, numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from ip12_sim import BOX, TABLE_Z, image_to_numpy, tf_matrix
from cloud_boxes import cloud_tops

STATIC = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "share", "irb1100_vision", "static")
PALLET = {"center": (-50.0, 0.0), "size": (300.0, 200.0, 20.0)}        # mm, urdf/workcell.urdf.xacro
GRIP_DIST = 40.0                                                     # mm: a box this close to the TCP is in the gripper
MIN_CODE_PX = 3 * 29                                                 # smallest QR (21 modules + quiet zone) at 3 px/module
WS_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"


def iso(t=None):
    return datetime.fromtimestamp(time.time() if t is None else t, timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")


def location_of(p_mm, tcp_mm=None):
    """box centre (mm, cell frame) -> gripper / pallet / source_area"""
    if tcp_mm is not None and math.dist(p_mm, tcp_mm) < GRIP_DIST: return "gripper"
    (cx, cy), (lx, ly, _) = PALLET["center"], PALLET["size"]
    return "pallet" if abs(p_mm[0] - cx) < lx / 2 and abs(p_mm[1] - cy) < ly / 2 else "source_area"


def read_code(crop, box_px, cut_off=False):
    """grey crop of one box top -> identification dict (API catalogue section 3)"""
    def ident(state, code=None, conf=0.0, reason=None):
        return {"state": state, "package_code": code, "confidence": conf, "unreadable_reason": reason}
    if cut_off or crop is None or crop.size == 0: return ident("unidentified", reason="unknown")
    det = cv2.QRCodeDetector()
    text, pts, _ = det.detectAndDecode(crop)
    if text: return ident("identified", text, 0.99)
    if crop.mean() < 50 or crop.mean() > 215 or crop.std() < 12: return ident("unidentified", reason="bad_light")
    if box_px < MIN_CODE_PX: return ident("unidentified", reason="too_far")
    if pts is not None: return ident("unidentified", conf=0.3, reason="damaged")
    return ident("unidentified", reason="covered")


def ws_accept(key):
    return base64.b64encode(hashlib.sha1((key + WS_GUID).encode()).digest()).decode()


def ws_frame(text):
    """one unmasked text frame, server to client (RFC 6455)"""
    b = text.encode(); n = len(b)
    head = bytes([0x81, n]) if n < 126 else bytes([0x81, 126]) + struct.pack(">H", n) if n < 65536 else bytes([0x81, 127]) + struct.pack(">Q", n)
    return head + b


def robot_model(urdf_xml, tf):
    """GET /config/robot-model from the URDF (joints) and TF (base pose, tool)"""
    import xml.etree.ElementTree as ET
    r = ET.fromstring(urdf_xml); joints = []
    for i in range(1, 7):
        j = next(j for j in r.findall("joint") if j.get("name") == f"joint_{i}")
        o, a, lim = j.find("origin"), j.find("axis"), j.find("limit")
        xyz = [float(v) * 1000 for v in (o.get("xyz") if o is not None else "0 0 0").split()]
        joints.append({"name": f"joint_{i}", "node": f"link_{i}", "axis": [float(v) for v in a.get("xyz").split()],
                       "origin": dict(zip("xyz", [round(v, 2) for v in xyz])),
                       "min_deg": round(math.degrees(float(lim.get("lower"))), 1), "max_deg": round(math.degrees(float(lim.get("upper"))), 1),
                       "zero_offset_deg": 0})
    B = tf_matrix(tf, "world", "base_link"); F = tf_matrix(tf, "link_6", "tool0"); P = tf_matrix(tf, "tool0", "gripper_tcp")
    mm = lambda v: round(float(v) * 1000, 2)
    return {"model_url": "/static/irb1100.glb", "up_axis": "z", "units": "mm",
            "base_pose": {"x": mm(B[0, 3]), "y": mm(B[1, 3]), "z": mm(B[2, 3]), "yaw": round(math.degrees(math.atan2(B[1, 0], B[0, 0])), 2)},
            "joints": joints,
            "tool": {"model_url": "/static/gripper.glb", "frame": "tool0",
                     "mount": {"parent_node": "link_6", "x": mm(F[0, 3]), "y": mm(F[1, 3]), "z": mm(F[2, 3]),
                               "rpy_deg": [round(math.degrees(v), 2) for v in (math.atan2(F[2, 1], F[2, 2]), -math.asin(F[2, 0]), math.atan2(F[1, 0], F[0, 0]))]},
                     "tcp_offset": {"x": mm(P[0, 3]), "y": mm(P[1, 3]), "z": mm(P[2, 3])}}}


def scenario():
    """GET /config/scenario: cell geometry in the cell frame (mm). Table size is the simulation's, the real one is not measured yet."""
    (px, py), (lx, ly, h) = PALLET["center"], PALLET["size"]
    return {"pallet_id": "pallet_1", "slots": [],
            "cell": {"frame": "world", "units": "mm",
                     "table": {"center": {"x": 0, "y": 0}, "size": {"x": 1100, "y": 1100}, "top_z": TABLE_Z * 1000},
                     "pallet": {"center": {"x": px, "y": py, "z": TABLE_Z * 1000 + h}, "size": {"x": lx, "y": ly, "z": h}},
                     "source_area": "table outside the pallet",
                     "robot_base": {"x": 200, "y": 0, "z": TABLE_Z * 1000, "yaw": 0}}}


class Api(Node):
    def __init__(self):
        super().__init__("hmi_api")
        self.port = self.declare_parameter("port", 8000).value
        self.tf = Buffer(); TransformListener(self.tf, self)
        self.img = self.K = self.cloud = None; self.img_rx = self.cloud_rx = 0.0; self.urdf = None
        self.create_subscription(Image, "/cam3/left/image_raw", self.on_img, 1)
        self.create_subscription(CameraInfo, "/cam3/left/camera_info", lambda m: setattr(self, "K", np.array(m.k).reshape(3, 3)), 1)
        self.create_subscription(PointCloud2, "/cam3/points", self.on_cloud, 1)
        self.create_subscription(String, "/robot_description", lambda m: setattr(self, "urdf", m.data),
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.scans, self.keys, self.scene, self.lock = {}, {}, None, threading.Lock()
        self.ws_clients, self.seq = [], 0

    def on_img(self, m):
        self.img, self.img_rx = m, time.time()

    def on_cloud(self, m):
        self.cloud, self.cloud_rx = m, time.time()

    def camera_ok(self):
        return self.img is not None and time.time() - self.img_rx < 3.0

    # ---- events ----
    def emit(self, typ, data):
        with self.lock:
            self.seq += 1; msg = json.dumps({"type": typ, "seq": self.seq, "timestamp": iso(), "data": data})
            for q in self.ws_clients: q.put(msg)

    def error(self, code, message, recoverable=True, action=None, **related):
        e = {"code": code, "message": message, "recoverable": recoverable, "suggested_action": action,
             "related": {k: related.get(k) for k in ("object_id", "package_code", "slot_id", "task_id")}}
        return e

    # ---- scans ----
    def start_scan(self, key=None):
        if key and key in self.keys: return self.keys[key]
        sid = f"scan_{uuid.uuid4().hex[:8]}"
        self.scans[sid] = {"id": sid, "state": "running", "origin": "user", "started_at": iso(), "finished_at": None}
        if key: self.keys[key] = sid
        self.emit("scan.status_changed", {"scan_id": sid, "state": "running"})
        threading.Thread(target=self.run_scan, args=(sid, time.time()), daemon=True).start()
        return sid

    def run_scan(self, sid, t0):
        end = time.time() + 8.0            # wait for a picture and a cloud that arrived after the request
        while time.time() < end and not (self.img_rx > t0 and self.cloud_rx > t0): time.sleep(0.05)
        try:
            if not (self.img_rx > t0 and self.cloud_rx > t0 and self.K is not None): raise TimeoutError("no fresh picture or point cloud")
            scene = self.build_scene(sid, self.img, self.cloud)
            self.scene, state = scene, "completed"
        except Exception as e:
            state = "failed"
            self.emit("error.raised", self.error("SUBSYSTEM_OFFLINE", f"camera scan failed: {e}", True, "check the camera, then scan again"))
        self.scans[sid].update(state=state, finished_at=iso())
        if state == "completed": self.emit("scene.updated", self.scene)
        self.emit("scan.status_changed", {"scan_id": sid, "state": state})

    def build_scene(self, sid, img_msg, cloud_msg):
        tops = cloud_tops(cloud_msg, self.tf)
        bgr = image_to_numpy(img_msg); gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY); h, w = gray.shape
        C = np.linalg.inv(tf_matrix(self.tf, "world", img_msg.header.frame_id, rclpy.time.Time.from_msg(img_msg.header.stamp), fallback=True))
        try: tcp = tf_matrix(self.tf, "world", "gripper_tcp")[:3, 3] * 1000
        except Exception: tcp = None
        objects = []
        for i, (x, y, zt, yaw) in enumerate(tops, 1):
            c, s = math.cos(yaw), math.sin(yaw)
            corners = np.array([[x + BOX / 2 * (c * u - s * v), y + BOX / 2 * (s * u + c * v), zt, 1.0] for u, v in ((-1, -1), (1, -1), (1, 1), (-1, 1))])
            pc = (C @ corners.T)[:3]; px = (self.K @ pc)[:2] / pc[2]
            x0, y0, x1, y1 = px[0].min(), px[1].min(), px[0].max(), px[1].max()
            cut = (pc[2] <= 0).any() or x0 < 0 or y0 < 0 or x1 >= w or y1 >= h
            pad = 0.2 * (x1 - x0)
            crop = None if cut else gray[int(max(0, y0 - pad)):int(min(h, y1 + pad)), int(max(0, x0 - pad)):int(min(w, x1 + pad))]
            p_mm = (x * 1000, y * 1000, (zt - BOX / 2) * 1000)
            objects.append({"id": f"obj_{i}", "location": location_of(p_mm, tcp),
                            "pose": {"x": round(p_mm[0], 1), "y": round(p_mm[1], 1), "z": round(p_mm[2], 1), "yaw": round(math.degrees(yaw), 1)},
                            "identification": read_code(crop, max(x1 - x0, y1 - y0), cut)})
        st = img_msg.header.stamp.sec + img_msg.header.stamp.nanosec * 1e-9
        ts = iso(st if st > 1e9 else self.img_rx)       # Gazebo stamps start at 0: use the arrival time
        return {"scan_id": sid, "timestamp": ts, "objects": objects, "pallet": {"id": "pallet_1", "slots": []}}


def make_handler(api):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a): pass

        def send(self, code, body=None, ctype="application/json"):
            data = body if isinstance(body, bytes) else (b"" if body is None else json.dumps(body).encode())
            self.send_response(code); self.send_header("Content-Type", ctype); self.send_header("Content-Length", str(len(data)))
            self.send_header("Access-Control-Allow-Origin", "*"); self.end_headers(); self.wfile.write(data)

        def do_GET(self):
            p = self.path.split("?")[0]
            if p == "/api/events" and self.headers.get("Upgrade", "").lower() == "websocket": return self.websocket()
            if p == "/api/health": return self.send(200, {"camera": "ok" if api.camera_ok() else "offline"})
            if p == "/api/scene": return self.send(200, api.scene) if api.scene else self.send(204)
            if p.startswith("/api/scans/"):
                s = api.scans.get(p.rsplit("/", 1)[1])
                return self.send(200, s) if s else self.send(404, api.error("NOT_FOUND", "unknown scan id", False))
            if p == "/api/config/robot-model":
                if not api.urdf: return self.send(503, api.error("SUBSYSTEM_OFFLINE", "robot description not received yet"))
                try: return self.send(200, robot_model(api.urdf, api.tf))
                except Exception as e: return self.send(503, api.error("SUBSYSTEM_OFFLINE", f"TF not ready: {e}"))
            if p == "/api/config/scenario": return self.send(200, scenario())
            if p.startswith("/static/") and p[8:] in ("irb1100.glb", "gripper.glb"):
                return self.send(200, open(os.path.join(STATIC, p[8:]), "rb").read(), "model/gltf-binary")
            self.send(404, api.error("NOT_FOUND", f"no route {p}", False))

        def do_POST(self):
            if self.path.split("?")[0] == "/api/scans":
                if not api.camera_ok():
                    return self.send(503, api.error("SUBSYSTEM_OFFLINE", "camera not reachable", True, "check the ZED and its node"))
                return self.send(202, {"scan_id": api.start_scan(self.headers.get("Idempotency-Key"))})
            self.send(404, api.error("NOT_FOUND", f"no route {self.path}", False))

        def websocket(self):
            self.send_response(101); self.send_header("Upgrade", "websocket"); self.send_header("Connection", "Upgrade")
            self.send_header("Sec-WebSocket-Accept", ws_accept(self.headers["Sec-WebSocket-Key"])); self.end_headers()
            q = queue.Queue()
            with api.lock: api.ws_clients.append(q)
            try:
                while True:
                    try: self.wfile.write(ws_frame(q.get(timeout=15))); self.wfile.flush()
                    except queue.Empty: self.wfile.write(b"\x89\x00"); self.wfile.flush()      # ping keeps the line open
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass
            finally:
                with api.lock: api.ws_clients.remove(q)
    return H


def main():
    rclpy.init(); api = Api()
    threading.Thread(target=rclpy.spin, args=(api,), daemon=True).start()
    srv = ThreadingHTTPServer(("0.0.0.0", api.port), make_handler(api)); srv.daemon_threads = True
    api.get_logger().info(f"IP12 WP2 API on http://0.0.0.0:{api.port}/api  (events: ws://.../api/events)")
    try: srv.serve_forever()
    finally: rclpy.shutdown()


if __name__ == "__main__":
    main()
