#!/usr/bin/env python3
# @ip12:vision  WP2 <-> WP1 link: box positions out, robot pose in, over UDP JSON (docs/INTERFACE.md)
"""UDP bridge between the WP2 perception (ROS 2 inside) and the WP1 robot control (Python, no ROS 2).
   ros2 run irb1100_vision udp_bridge.py --ros-args -p wp1_host:=127.0.0.1             # boxes out only (Gazebo)
   ros2 run irb1100_vision udp_bridge.py --ros-args -p wp1_host:=127.0.0.1 -p pose_in:=true   # real cell: pose in too
Out, to wp1_host:boxes_port, one datagram per camera picture:
   {"type": "boxes", "stamp": 1764100000.123, "frame": "base", "unit": "mm",
    "boxes": [{"class": "red", "x": -250.0, "y": 120.0, "z": 12.5, "yaw_deg": 64.4}]}
   x, y, z = box centre in the robot base frame (RobotWare base, mm); yaw_deg 0-90 or null if the point cloud missed it
In, on pose_port (only with pose_in:=true; in Gazebo the simulator publishes /joint_states itself):
   {"type": "robot_pose", "stamp": 1764100000.120, "joints_deg": [180.0, 0.0, 34.4, 0.0, 42.9, 0.0]}
   -> /joint_states (radians). stamp = when the angles were read (Unix time, s); 0 = use the arrival time.
"""
import json, math, socket
import numpy as np, rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener
from ip12_sim import JOINTS, tf_matrix

YAW_MATCH = 0.015   # m: a point-cloud box this close to a detection gives it its yaw


def boxes_message(stamp, boxes_base, yaws):
    """boxes_base: [(class, x, y, z) in metres, base frame]; yaws: [rad or None] -> JSON bytes (mm, deg)."""
    return json.dumps({"type": "boxes", "stamp": round(stamp, 6), "frame": "base", "unit": "mm",
                       "boxes": [{"class": c, "x": round(x * 1000, 1), "y": round(y * 1000, 1), "z": round(z * 1000, 1),
                                  "yaw_deg": None if w is None else round(math.degrees(w), 1)}
                                 for (c, x, y, z), w in zip(boxes_base, yaws)]}).encode()


def parse_pose(data):
    """JSON bytes from WP1 -> (stamp s, [6 joint angles in rad]); raises ValueError on anything malformed."""
    m = json.loads(data)
    q = m.get("joints_deg") if m.get("type") == "robot_pose" else None
    if not isinstance(q, list) or len(q) != 6 or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in q):
        raise ValueError("expected {'type': 'robot_pose', 'joints_deg': [6 numbers]}")
    return float(m.get("stamp") or 0.0), [math.radians(v) for v in q]


class Bridge(Node):
    def __init__(self):
        super().__init__("udp_bridge")
        self.wp1 = (self.declare_parameter("wp1_host", "127.0.0.1").value, self.declare_parameter("boxes_port", 5005).value)
        self.tf = Buffer(); TransformListener(self.tf, self)
        self.out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.cloud = []                                                   # [(x, y, yaw)] in world, latest
        self.create_subscription(MarkerArray, "/cam3/detections/markers", self.on_boxes, 5)
        self.create_subscription(PoseArray, "/cam3/cloud_boxes", self.on_cloud, 5)
        if self.declare_parameter("pose_in", False).value:
            self.inp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.inp.bind(("0.0.0.0", self.declare_parameter("pose_port", 5006).value)); self.inp.setblocking(False)
            self.js = self.create_publisher(JointState, "/joint_states", 10)
            self.create_timer(0.002, self.read_pose)

    def on_cloud(self, m):
        self.cloud = [(p.position.x, p.position.y, 2 * math.atan2(p.orientation.z, p.orientation.w)) for p in m.poses]

    def on_boxes(self, m):
        mk = [k for k in m.markers if k.action == Marker.ADD]
        if not mk: return
        try:
            B = np.linalg.inv(tf_matrix(self.tf, "world", "base_link"))   # world -> robot base
        except Exception as e:
            self.get_logger().info(f"waiting for TF: {e}", throttle_duration_sec=2.0); return
        boxes, yaws = [], []
        for k in mk:
            p = np.array([k.pose.position.x, k.pose.position.y, k.pose.position.z, 1.0])
            near = [w for x, y, w in self.cloud if math.hypot(x - p[0], y - p[1]) < YAW_MATCH]
            boxes.append((k.text or "box", *(B @ p)[:3])); yaws.append(near[0] if near else None)
        st = mk[0].header.stamp
        self.out.sendto(boxes_message(st.sec + st.nanosec * 1e-9, boxes, yaws), self.wp1)

    def read_pose(self):
        while True:
            try: data = self.inp.recv(4096)
            except BlockingIOError: return
            try: stamp, q = parse_pose(data)
            except ValueError as e:
                self.get_logger().warn(f"bad robot_pose from WP1 ignored: {e}", throttle_duration_sec=2.0); continue
            js = JointState(name=JOINTS, position=q)
            js.header.stamp = (rclpy.time.Time(nanoseconds=int(stamp * 1e9)) if stamp else self.get_clock().now()).to_msg()
            self.js.publish(js)


def main():
    rclpy.init(); rclpy.spin(Bridge())


if __name__ == "__main__":
    main()
