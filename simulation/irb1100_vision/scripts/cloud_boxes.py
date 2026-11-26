#!/usr/bin/env python3
# @ip12:vision  finds the boxes from the ZED point cloud alone (shape, no colour), in robot coordinates
"""Box finder on the 3D point cloud: works without colours or a trained model, so it also checks the colour/YOLO detector.
   ros2 run irb1100_vision cloud_boxes.py
1. /cam3/points -> world frame with TF (the camera -> robot transform, the same chain aruco_locate.py checks)
2. drop the robot base and points at table height, make a height map (highest point per 3 mm cell)
3. grow flat patches: neighbour cells less than `flat` metres apart in height
4. a patch whose smallest rectangle is box-sized (25 mm +- tol) is a box top: centre, height - 12.5 mm, yaw
Out: /cam3/cloud_boxes (PoseArray, world, yaw in the orientation). In simulation it logs the error against /sim/<box>/pose.
"""
from collections import deque
import cv2, numpy as np, rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points_numpy
from tf2_ros import Buffer, TransformListener
from ip12_sim import BOX, TABLE_Z, BoxPoses, tf_matrix

CELL = 0.003
# robot base footprint in base_link (meshes/irb1100/base.stl bounds + 10 mm): its 22 mm top ring looks like a box top
BASE_MIN, BASE_MAX = np.array([-0.16, -0.09, -0.01]), np.array([0.09, 0.09, 0.17])


def find_tops(pts, flat=0.0025, tol=0.008):
    """pts: Nx3 world points -> list of (x, y, z_top, yaw). Pure numpy/OpenCV, no ROS (tested below)."""
    pts = pts[(pts[:, 2] > TABLE_Z + 0.004) & (pts[:, 2] < TABLE_Z + 0.4)]
    if len(pts) == 0: return []
    ij = np.floor(pts[:, :2] / CELL).astype(int); ij -= ij.min(0)
    H = np.full(ij.max(0) + 1, np.nan)
    order = np.argsort(pts[:, 2])                                  # write low to high: the last (highest) wins
    H[ij[order, 0], ij[order, 1]] = pts[order, 2]
    seen, out = np.isnan(H), []
    origin = np.floor(pts[:, :2].min(0) / CELL) * CELL
    for start in zip(*np.nonzero(~seen)):
        if seen[start]: continue
        seen[start] = True; q, cells = deque([start]), []
        while q:
            a, b = q.popleft(); cells.append((a, b))
            for c, d in ((a + 1, b), (a - 1, b), (a, b + 1), (a, b - 1)):
                if 0 <= c < H.shape[0] and 0 <= d < H.shape[1] and not seen[c, d] and abs(H[c, d] - H[a, b]) < flat:
                    seen[c, d] = True; q.append((c, d))
        if len(cells) < 20: continue
        cells = np.array(cells)
        xy = (cells + 0.5) * CELL + origin
        (cx, cy), (w, h), ang = cv2.minAreaRect(xy.astype(np.float32))
        w, h = w + CELL, h + CELL                                  # cell centres sit half a cell inside the edges
        if abs(w - BOX) < tol and abs(h - BOX) < tol:
            out.append((cx, cy, float(np.median(H[cells[:, 0], cells[:, 1]])), np.radians(ang) % (np.pi / 2)))
    return out


def cloud_tops(m, tf):
    """PointCloud2 + tf2 Buffer -> box tops (x, y, z_top, yaw) in world. Raises if TF is not there yet."""
    T = tf_matrix(tf, "world", m.header.frame_id, rclpy.time.Time.from_msg(m.header.stamp), fallback=True)
    p = read_points_numpy(m, field_names=("x", "y", "z"), skip_nans=True).astype(np.float64)
    p = p[np.isfinite(p).all(1)] @ T[:3, :3].T + T[:3, 3]
    try:
        B = np.linalg.inv(tf_matrix(tf, "world", "base_link"))
        pb = p @ B[:3, :3].T + B[:3, 3]
        p = p[~((pb > BASE_MIN) & (pb < BASE_MAX)).all(1)]
    except Exception:
        pass
    return find_tops(p)


class CloudBoxes(Node):
    def __init__(self):
        super().__init__("cloud_boxes")
        self.tf = Buffer(); TransformListener(self.tf, self)
        self.truth = BoxPoses(self); self.cloud = None
        self.create_subscription(PointCloud2, "/cam3/points", lambda m: setattr(self, "cloud", m), 1)
        self.pub = self.create_publisher(PoseArray, "/cam3/cloud_boxes", 5)
        self.create_timer(1.0, self.tick)

    def tick(self):
        m, self.cloud = self.cloud, None
        if m is None: return
        try:
            tops = cloud_tops(m, self.tf)
        except Exception as e:
            self.get_logger().info(f"waiting for TF: {e}", throttle_duration_sec=2.0); return
        poses, log = PoseArray(), []
        poses.header.frame_id, poses.header.stamp = "world", m.header.stamp
        for x, y, z, yaw in tops:
            pose = Pose(); pose.position.x, pose.position.y, pose.position.z = x, y, z - BOX / 2
            pose.orientation.z, pose.orientation.w = np.sin(yaw / 2), np.cos(yaw / 2)
            poses.poses.append(pose)
            near = min(self.truth.T.items(), key=lambda kv: np.linalg.norm(kv[1][:3, 3] - [x, y, z - BOX / 2]), default=None)
            err = ""
            if near is not None:
                t = near[1]; true_yaw = np.arctan2(t[1, 0], t[0, 0]) % (np.pi / 2)
                dyaw = np.degrees(min(abs(yaw - true_yaw), np.pi / 2 - abs(yaw - true_yaw)))
                err = f" ({near[0]}: {np.linalg.norm(t[:3, 3] - [x, y, z - BOX / 2]) * 1000:.0f} mm, {dyaw:.0f} deg off)"
            log.append(f"({x:.3f}, {y:.3f}, {z - BOX / 2:.3f}) yaw {np.degrees(yaw):.0f} deg{err}")
        self.pub.publish(poses)
        self.get_logger().info(f"{len(log)} boxes: " + " | ".join(log), throttle_duration_sec=2.0)


def main():
    rclpy.init(); rclpy.spin(CloudBoxes())


if __name__ == "__main__":
    main()
