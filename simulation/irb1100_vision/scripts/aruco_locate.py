#!/usr/bin/env python3
# @ip12:vision  finds where the wrist camera is from one ArUco marker on the table (camera -> robot coordinate transform)
"""Camera position from an ArUco marker whose place on the table is known (your professor's method).
   ros2 run irb1100_vision aruco_locate.py                     # arm must look at the marker; it does not move the arm
   ros2 run irb1100_vision aruco_locate.py --ros-args -p frames:=20 -p out:=$HOME/ip12_aruco.yaml
The chain:
   T_world_marker  known: measured once (on the real cell: touch the marker centre with the TCP; here: worlds/workcell.sdf)
   T_cam_marker    from the picture: detect the 4 corners, solvePnP with the 50 mm side length
   T_world_cam   = T_world_marker . inverse(T_cam_marker)            camera in robot coordinates
   T_gripper_cam = inverse(T_world_gripper) . T_world_cam            camera on the gripper (hand-eye from one view)
In simulation both are compared with the URDF (TF) and the error is logged. Real robot: same script, real ZED, and
T_world_gripper comes from the robot's /joint_states through TF.
"""
import os, cv2, numpy as np, rclpy, rclpy.parameter, yaml
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from ip12_sim import image_to_numpy, mat_to_quat, tf_matrix

MARKER_ID, SIDE = 0, 0.050                                       # DICT_4X4_50 id 0, black square side (m)
# Marker centre on the table top and its yaw, worlds/workcell.sdf. Gazebo lays the picture turned by 90 deg, so the
# marker's own x axis points along world -y (yaw -90 deg). On the real cell this pose is what you measure.
MARKER_XYZ, MARKER_YAW = (-0.17, -0.15, 0.752), -np.pi / 2
DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
# marker corners in the marker frame, in ArUco's order: top-left, top-right, bottom-right, bottom-left (x right, y up)
OBJ = np.array([[-1, 1, 0], [1, 1, 0], [1, -1, 0], [-1, -1, 0]], np.float32) * SIDE / 2


def detect(gray):
    """-> 4x2 corner pixels of MARKER_ID or None (OpenCV 4.6 in ROS Jazzy has the old API, 4.7+ the new one)."""
    new = hasattr(cv2.aruco, "ArucoDetector")
    prm = cv2.aruco.DetectorParameters() if new else cv2.aruco.DetectorParameters_create()
    prm.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX          # corners to a fraction of a pixel
    if new:
        corners, ids, _ = cv2.aruco.ArucoDetector(DICT, prm).detectMarkers(gray)
    else:
        corners, ids, _ = cv2.aruco.detectMarkers(gray, DICT, parameters=prm)
    for c, i in zip(corners, [] if ids is None else ids.ravel()):
        if i == MARKER_ID: return c.reshape(4, 2)
    return None


def pose_error(A, B):
    """translation (mm) and rotation (deg) between two 4x4 poses"""
    dr = np.degrees(np.arccos(np.clip((np.trace(A[:3, :3].T @ B[:3, :3]) - 1) / 2, -1, 1)))
    return np.linalg.norm(A[:3, 3] - B[:3, 3]) * 1000, dr


class Locate(Node):
    def __init__(self):
        super().__init__("aruco_locate", parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.declare_parameter("frames", 10)
        self.declare_parameter("out", "")
        self.tf = Buffer(); TransformListener(self.tf, self)
        self.img = self.K = None
        self.create_subscription(Image, "/cam3/left/image_raw", lambda m: setattr(self, "img", m), 1)
        self.create_subscription(CameraInfo, "/cam3/left/camera_info", lambda m: setattr(self, "K", np.array(m.k).reshape(3, 3)), 1)

    def run(self):
        W_M = np.eye(4); c, s = np.cos(MARKER_YAW), np.sin(MARKER_YAW)
        W_M[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]; W_M[:3, 3] = MARKER_XYZ
        cams, grips, last = [], [], None
        while rclpy.ok() and len(cams) < self.get_parameter("frames").value:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.img is None or self.K is None or self.img is last: continue
            last = m = self.img
            px = detect(cv2.cvtColor(image_to_numpy(m), cv2.COLOR_BGR2GRAY))
            if px is None: self.get_logger().info("marker not in the picture", throttle_duration_sec=2.0); continue
            ok, rvec, tvec = cv2.solvePnP(OBJ, px.astype(np.float32), self.K, None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
            C_M = np.eye(4); C_M[:3, :3] = cv2.Rodrigues(rvec)[0]; C_M[:3, 3] = tvec.ravel()
            W_C = W_M @ np.linalg.inv(C_M)
            stamp = rclpy.time.Time.from_msg(m.header.stamp)
            try:
                W_G = tf_matrix(self.tf, "world", "gripper_link", stamp, fallback=True)
                W_C_tf = tf_matrix(self.tf, "world", m.header.frame_id, stamp, fallback=True)
            except Exception as e:
                self.get_logger().info(f"waiting for TF: {e}", throttle_duration_sec=2.0); continue
            cams.append((W_C, W_C_tf)); grips.append(np.linalg.inv(W_G) @ W_C)
        if not cams: return
        W_C, W_C_tf = cams[-1]
        G_C, G_C_cad = grips[-1], tf_matrix(self.tf, "gripper_link", "cam3_left_optical_frame")
        e_cam, e_grip = pose_error(W_C, W_C_tf), pose_error(G_C, G_C_cad)
        spread = np.ptp([c[0][:3, 3] for c in cams], axis=0).max() * 1000
        self.get_logger().info(f"camera in world: {np.round(W_C[:3, 3], 4)} m, error vs TF {e_cam[0]:.1f} mm / {e_cam[1]:.2f} deg")
        self.get_logger().info(f"camera on gripper_link: {np.round(G_C[:3, 3], 4)} m, error vs CAD {e_grip[0]:.1f} mm / {e_grip[1]:.2f} deg "
                               f"({len(cams)} frames, spread {spread:.1f} mm)")
        if self.get_parameter("out").value:
            yaml.safe_dump({"parent": "gripper_link", "child": "cam3_left_optical_frame",
                            "translation_m": [round(float(v), 5) for v in G_C[:3, 3]],
                            "quaternion_xyzw": [round(float(v), 6) for v in mat_to_quat(G_C[:3, :3])],
                            "error_vs_cad_mm": round(float(e_grip[0]), 2), "error_vs_cad_deg": round(float(e_grip[1]), 3)},
                           open(os.path.expanduser(self.get_parameter("out").value), "w"), sort_keys=False)


def main():
    rclpy.init(); node = Locate()
    try: node.run()
    finally: rclpy.shutdown()


if __name__ == "__main__":
    main()
