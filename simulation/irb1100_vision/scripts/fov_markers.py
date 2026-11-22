#!/usr/bin/env python3
# @ip12:camera  simulation: camera view cones (RViz2) for cam1 side, cam2 ZED Mini top-down, cam3 ZED Mini on the wrist
"""Publish translucent view-frustum pyramids for the cameras (RViz2 'View cones').

Markers live in each camera's optical frame (z forward), so they follow TF.
Parameters: cameras (list of names), and per camera <name>.frame/.hfov/.aspect/.range/.color
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

DEFAULTS = {
    "cam1": dict(frame="cam1_optical_frame", hfov=1.2, aspect=16 / 9, range=1.0, color=[0.96, 0.62, 0.10]),
    "cam2": dict(frame="cam2_left_optical_frame", hfov=1.5708, aspect=16 / 9, range=0.85, color=[0.10, 0.65, 0.52]),
    "cam3": dict(frame="cam3_left_optical_frame", hfov=1.5708, aspect=16 / 9, range=0.40, color=[0.55, 0.35, 0.85]),  # ZED Mini on the wrist
}


class FovMarkers(Node):
    def __init__(self):
        super().__init__("fov_markers")
        self.declare_parameter("cameras", list(DEFAULTS))
        self.cams = {}
        for name in self.get_parameter("cameras").value:
            d = DEFAULTS.get(name, DEFAULTS["cam1"])
            cfg = {}
            for key, default in d.items():
                self.declare_parameter(f"{name}.{key}", default)
                cfg[key] = self.get_parameter(f"{name}.{key}").value
            self.cams[name] = cfg
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(MarkerArray, "/vision/fov_cones", qos)
        self.create_timer(1.0, self.tick)

    def corners(self, cfg):
        r = cfg["range"]
        hw = r * math.tan(cfg["hfov"] / 2)
        hh = hw / cfg["aspect"]
        return [Point(x=-hw, y=-hh, z=r), Point(x=hw, y=-hh, z=r),
                Point(x=hw, y=hh, z=r), Point(x=-hw, y=hh, z=r)]

    def tick(self):
        arr = MarkerArray()
        for i, (name, cfg) in enumerate(self.cams.items()):
            c = self.corners(cfg)
            o = Point()
            base = lambda mid, mtype: self._marker(name, cfg, mid, mtype)
            faces = base(2 * i, Marker.TRIANGLE_LIST)
            for a, b in ((0, 1), (1, 2), (2, 3), (3, 0)):
                faces.points += [o, c[a], c[b]]
            faces.color.a = 0.18
            edges = base(2 * i + 1, Marker.LINE_LIST)
            edges.scale.x = 0.004
            for k in range(4):
                edges.points += [o, c[k]]
            for a, b in ((0, 1), (1, 2), (2, 3), (3, 0)):
                edges.points += [c[a], c[b]]
            edges.color.a = 0.9
            arr.markers += [faces, edges]
        self.pub.publish(arr)

    def _marker(self, name, cfg, mid, mtype):
        m = Marker()
        m.header.frame_id = cfg["frame"]
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = f"fov_{name}"
        m.id = mid
        m.type = mtype
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 1.0
        m.color.r, m.color.g, m.color.b = cfg["color"]
        m.frame_locked = True
        return m


def main():
    rclpy.init()
    rclpy.spin(FovMarkers())


if __name__ == "__main__":
    main()
