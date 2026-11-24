#!/usr/bin/env python3
# @ip12:robot  button panel: click to move the arm to a step or play the whole pick order (smooth, streamed at 50 Hz)
"""Small window with buttons (needs: sudo apt install python3-tk).
   ros2 run irb1100_vision pick_panel.py
Every move is a minimum-jerk curve (no sudden start/stop) sent 50 times per second to /arm_controller,
using the same joint targets as config/pick_sequence.yaml.
"""
import math, os, threading, time, yaml, tkinter as tk
import numpy as np
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from ament_index_python.packages import get_package_share_directory

RATE = 50.0
VMAX = np.radians([460, 360, 280, 560, 420, 750]) * 0.9     # ABB IRB 1100 axis speeds (rad/s), with 10 % margin


class Panel(Node):
    def __init__(self):
        super().__init__("pick_panel")
        cfg = os.path.join(get_package_share_directory("irb1100_vision"), "config", "pick_sequence.yaml")
        self.cfg = yaml.safe_load(open(cfg)); self.names = self.cfg["joints"]
        self.pub = self.create_publisher(JointTrajectory, "/arm_controller/joint_trajectory", 10)
        self.grip = self.create_publisher(String, "/gripper/command", 10)   # scripts/gripper.py
        self.create_subscription(JointState, "/joint_states", self.on_js, 10)
        self.q = None            # last commanded position
        self.seg = None          # (q0, q1, t0, T)
        self.queue = []          # pending (q, secs)
        self.hold_until = 0.0
        self.pause = 0.0
        self.on_status = lambda s: None
        self.create_timer(1.0 / RATE, self.tick)

    def on_js(self, m):
        if self.q is None and set(self.names) <= set(m.name):
            self.q = np.array([m.position[m.name.index(n)] for n in self.names])

    def clear(self): self.queue = []; self.seg = None

    def add(self, q, secs, label, grip=None):   # grip: "open"/"close" once the arm is there
        self.queue.append((np.array(q, float), secs, label, grip))

    def tick(self):
        if self.q is None: return
        now = time.monotonic()
        if self.seg is None and self.queue and now >= self.hold_until:
            q1, secs, label, grip = self.queue.pop(0)
            T = max(secs, float(np.max(np.abs(q1 - self.q) / VMAX)), 0.05)
            self.seg = (self.q.copy(), q1, now, T, grip); self.on_status(label)
        if self.seg is None: return
        q0, q1, t0, T, grip = self.seg
        tau = min(1.0, (now - t0) / T)
        s = 10 * tau**3 - 15 * tau**4 + 6 * tau**5
        ds = (30 * tau**2 - 60 * tau**3 + 30 * tau**4) / T
        self.q = q0 + (q1 - q0) * s
        dt = 1.0 / RATE
        m = JointTrajectory(); m.joint_names = self.names
        m.points = [JointTrajectoryPoint(positions=[float(v) for v in self.q], velocities=[float(v) for v in (q1 - q0) * ds],
                                         time_from_start=Duration(sec=0, nanosec=int(dt * 1e9)))]
        self.pub.publish(m)
        if tau >= 1.0:
            self.seg = None; self.hold_until = now + self.pause
            if grip: self.grip.publish(String(data=grip)); self.hold_until = now + max(self.pause, 0.5)   # jaws need ~0.2 s


def main():
    rclpy.init(); node = Panel()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    steps = node.cfg["steps"]
    root = tk.Tk(); root.title("IP12 arm control")
    secs = tk.DoubleVar(value=0.5); loop = tk.BooleanVar(value=False)
    status = tk.StringVar(value="waiting for /joint_states ...")
    node.on_status = lambda s: status.set(s)
    state = {"playing": False}

    def stop():
        state["playing"] = False; node.clear(); status.set("stopped")

    def one(i):
        stop(); node.pause = 0.0; node.add(steps[i]["q"], secs.get(), f"moving to: {steps[i]['name']}", steps[i].get("grip"))

    def fill():                                    # keeps the queue topped up while playing
        if state["playing"] and not node.queue and node.seg is None:
            if state["i"] >= len(steps):
                if loop.get(): state["i"] = 1
                else: state["playing"] = False; status.set("sequence finished"); return
            st = steps[state["i"]]; node.add(st["q"], secs.get(), f"[{state['i'] + 1}/{len(steps)}] {st['name']}", st.get("grip")); state["i"] += 1
        root.after(30, fill)

    def play():
        stop(); node.pause = min(0.6, secs.get() * 0.5); state.update(playing=True, i=0)

    tk.Button(root, text="PLAY full order", bg="#2e7d32", fg="white", font=("Arial", 12, "bold"), command=play).pack(fill="x", padx=8, pady=(8, 2))
    tk.Button(root, text="STOP", bg="#c62828", fg="white", font=("Arial", 12, "bold"), command=stop).pack(fill="x", padx=8, pady=2)
    tk.Checkbutton(root, text="repeat forever", variable=loop).pack(anchor="w", padx=8)
    tk.Scale(root, from_=0.05, to=2.0, resolution=0.05, orient="horizontal", label="seconds per move (fastest the arm allows is used if too small)", variable=secs).pack(fill="x", padx=8)
    g = tk.Frame(root); g.pack(fill="x", padx=8, pady=(6, 0))
    tk.Button(g, text="close gripper", command=lambda: node.grip.publish(String(data="close"))).pack(side="left", expand=True, fill="x")
    tk.Button(g, text="open gripper", command=lambda: node.grip.publish(String(data="open"))).pack(side="left", expand=True, fill="x")
    tk.Label(root, text="Go to one step:").pack(anchor="w", padx=8, pady=(6, 0))
    for i, st in enumerate(steps[:-1]):
        tk.Button(root, text=st["name"].replace("_", " "), command=lambda i=i: one(i)).pack(fill="x", padx=8, pady=1)
    tk.Button(root, text="home", command=lambda: one(len(steps) - 1)).pack(fill="x", padx=8, pady=1)
    tk.Label(root, textvariable=status, fg="#555").pack(pady=6)
    fill(); root.mainloop(); rclpy.shutdown()


if __name__ == "__main__":
    main()
