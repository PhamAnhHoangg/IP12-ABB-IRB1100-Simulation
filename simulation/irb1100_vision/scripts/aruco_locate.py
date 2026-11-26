#!/usr/bin/env python3
# @ip12:vision  finds where the wrist camera is from ArUco markers at a known place (camera -> robot coordinate transform)
"""Camera position from ArUco markers whose place is known (your professor's method), one picture, all markers at once.
   ros2 run irb1100_vision aruco_locate.py                     # arm must look at the sheet; it does not move the arm
   ros2 run irb1100_vision aruco_locate.py --ros-args -p frames:=20 -p out:=$HOME/ip12_aruco.yaml
   real cell, calibration square docs/print/aruco_square18.pdf (5 markers, the default layout; A4 sheet: -p layout:=sheet5):
   ros2 run irb1100_vision aruco_locate.py --ros-args -p marker_xyz:="[x, y, z]" -p marker_yaw:=yaw
   check markers around the work area (config/aruco_corners.yaml, IDs 5-8, measured one by one with the TCP):
   ros2 run irb1100_vision aruco_locate.py --ros-args -p layout:=corners [-p corners_file:=/path/to/measured.yaml]
   real cell, single marker docs/print/aruco_id0_DICT_4X4_50.pdf:
   ros2 run irb1100_vision aruco_locate.py --ros-args -p layout:=single -p side:=0.1 -p marker_xyz:="[x, y, z]" -p marker_yaw:=yaw
The chain:
   T_world_sheet  known: measured once (real cell: touch the centre of marker 0 with the TCP; here: worlds/workcell.sdf)
   T_cam_sheet    from the picture: every visible marker gives 4 corners whose place on the sheet is known; one
                  solvePnP over all of them (5 markers = 20 corners spread over the sheet, far steadier than 4)
   T_world_cam   = T_world_sheet . inverse(T_cam_sheet)              camera in robot coordinates
   T_gripper_cam = inverse(T_world_gripper) . T_world_cam            camera on the gripper (hand-eye from one view)
In simulation both are compared with the URDF (TF) and the error is logged. Real robot: same script, real ZED, and
T_world_gripper comes from the robot's /joint_states through TF.
"""
import os, cv2, numpy as np, rclpy, rclpy.parameter, yaml
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from ip12_sim import image_to_numpy, mat_to_quat, tf_matrix

DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
# id -> (centre x, centre y, side) in metres on the sheet (x right, y up, origin = centre of marker 0)
LAYOUTS = {"square18": {0: (0.0, 0.0, 0.04), 1: (-0.06, 0.06, 0.045), 2: (0.06, 0.06, 0.045),   # docs/print/aruco_square18.pdf
                        3: (-0.06, -0.06, 0.045), 4: (0.06, -0.06, 0.045)},
           "sheet5": {0: (0.0, 0.0, 0.06), 1: (-0.10, 0.06, 0.06), 2: (0.10, 0.06, 0.06),   # docs/print/aruco_sheet5_A4.pdf
                      3: (-0.10, -0.06, 0.06), 4: (0.10, -0.06, 0.06)},
           "single": {0: (0.0, 0.0, None)}}                                                # side from the parameter
# Defaults = the Gazebo sheet (worlds/workcell.sdf): on the pallet, sheet x along world +x. Real cell: pass what you measure.
SIDE, MARKER_XYZ, MARKER_YAW = 0.050, [-0.05, 0.0, 0.772], 0.0
# marker corners around its centre, in ArUco's order: top-left, top-right, bottom-right, bottom-left
CORNERS = np.array([[-1, 1, 0], [1, 1, 0], [1, -1, 0], [-1, -1, 0]], np.float32) / 2


def detect(gray):
    """-> {id: 4x2 corner pixels} of every marker found (OpenCV 4.6 in ROS Jazzy has the old API, 4.7+ the new one)."""
    new = hasattr(cv2.aruco, "ArucoDetector")
    prm = cv2.aruco.DetectorParameters() if new else cv2.aruco.DetectorParameters_create()
    prm.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX          # corners to a fraction of a pixel
    if new:
        corners, ids, _ = cv2.aruco.ArucoDetector(DICT, prm).detectMarkers(gray)
    else:
        corners, ids, _ = cv2.aruco.detectMarkers(gray, DICT, parameters=prm)
    return {int(i): c.reshape(4, 2) for c, i in zip(corners, [] if ids is None else ids.ravel())}


def correspondences(found, layout, side=SIDE):
    """matched (points Nx3, pixels Nx2) for the markers of `layout` that were found; an entry is (x, y, side) on the sheet
    or (x, y, side, yaw) for a marker turned by yaw (corners layout). All points lie in one plane z = 0."""
    obj, px = [], []
    for i, (cx, cy, s, *yaw) in layout.items():
        if i in found:
            c, sn = (np.cos(yaw[0]), np.sin(yaw[0])) if yaw else (1.0, 0.0)
            obj.append(CORNERS * (s or side) @ np.array([[c, sn, 0], [-sn, c, 0], [0, 0, 1]]) + [cx, cy, 0]); px.append(found[i])
    return (np.concatenate(obj).astype(np.float32), np.concatenate(px).astype(np.float32)) if obj else (None, None)


def pose_error(A, B):
    """translation (mm) and rotation (deg) between two 4x4 poses"""
    dr = np.degrees(np.arccos(np.clip((np.trace(A[:3, :3].T @ B[:3, :3]) - 1) / 2, -1, 1)))
    return np.linalg.norm(A[:3, 3] - B[:3, 3]) * 1000, dr


class Locate(Node):
    def __init__(self):
        super().__init__("aruco_locate", parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.declare_parameter("frames", 10)
        self.declare_parameter("out", "")
        self.declare_parameter("layout", "square18")       # square18 (18 x 18 cm), sheet5 (A4), single, corners
        self.declare_parameter("corners_file", "")          # layout corners: markers measured one by one (default config/aruco_corners.yaml)
        self.declare_parameter("side", SIDE)               # layout single: black square side (m), measure the print
        self.declare_parameter("marker_xyz", MARKER_XYZ)   # centre of marker 0 in world (m)
        self.declare_parameter("marker_yaw", MARKER_YAW)   # direction of the sheet's x axis in world (rad)
        self.tf = Buffer(); TransformListener(self.tf, self)
        self.img = self.K = None
        self.create_subscription(Image, "/cam3/left/image_raw", lambda m: setattr(self, "img", m), 1)
        self.create_subscription(CameraInfo, "/cam3/left/camera_info", lambda m: setattr(self, "K", np.array(m.k).reshape(3, 3)), 1)

    def run(self):
        yaw, name = self.get_parameter("marker_yaw").value, self.get_parameter("layout").value
        if name == "corners":   # each marker has its own measured place: work in the world frame at table height
            from ament_index_python.packages import get_package_share_directory
            f = self.get_parameter("corners_file").value or os.path.join(get_package_share_directory("irb1100_vision"), "config", "aruco_corners.yaml")
            cfg = yaml.safe_load(open(f))
            layout = {int(i): (m["xyz"][0], m["xyz"][1], cfg["side"], m.get("yaw", 0.0)) for i, m in cfg["markers"].items()}
            z0 = float(np.mean([m["xyz"][2] for m in cfg["markers"].values()]))
            self.set_parameters([rclpy.parameter.Parameter("marker_xyz", value=[0.0, 0.0, z0]),
                                 rclpy.parameter.Parameter("marker_yaw", value=0.0)]); yaw = 0.0
        else:
            layout = LAYOUTS[name]
        W_M = np.eye(4); c, s = np.cos(yaw), np.sin(yaw)
        W_M[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]; W_M[:3, 3] = self.get_parameter("marker_xyz").value
        cams, grips, last = [], [], None
        while rclpy.ok() and len(cams) < self.get_parameter("frames").value:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.img is None or self.K is None or self.img is last: continue
            last = m = self.img
            obj, px = correspondences(detect(cv2.cvtColor(image_to_numpy(m), cv2.COLOR_BGR2GRAY)), layout,
                                      self.get_parameter("side").value)
            if obj is None: self.get_logger().info("no marker of the sheet in the picture", throttle_duration_sec=2.0); continue
            ok, rvec, tvec = cv2.solvePnP(obj, px, self.K, None, flags=cv2.SOLVEPNP_IPPE)   # all corners lie in one plane
            used = len(obj) // 4
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
                               f"({len(cams)} frames, {used} markers = {used * 4} corners, spread {spread:.1f} mm)")
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
