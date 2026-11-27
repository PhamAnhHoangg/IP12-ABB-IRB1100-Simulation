#!/usr/bin/env python3
# @ip12:vision  hand-eye calibration of the wrist camera (where exactly the camera sits on the gripper)
"""Hand-eye calibration (eye-in-hand) in Gazebo, the same steps as on the real robot.
   ros2 launch irb1100_vision gazebo.launch.py panel:=false            # first, in another terminal
   ros2 run irb1100_vision handeye_calib.py --ros-args -p views:=15 -p out:=$HOME/ip12_handeye.yaml
1. puts the checkerboard (9 x 6 squares of 20 mm) on the pallet (the ArUco sheet goes aside meanwhile), boxes back to their start places
2. moves the arm to random poses looking at it; keeps the views where all 8 x 5 inner corners are found
3. per view: board pose in the camera (solvePnP) + gripper pose in the world (TF)  ->  cv2.calibrateHandEye
4. result = camera (cam3_left_optical_frame) in gripper_link; in simulation it is compared with the CAD value from the URDF
On the real robot: same board, gripper pose from the robot controller instead of TF, real ZED images; then put the result
into urdf/zed_wrist.xacro (camera origin) so detections land where the robot really is.
"""
import os, cv2, numpy as np, rclpy, rclpy.parameter, yaml
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from trajectory_msgs.msg import JointTrajectory
from tf2_ros import Buffer, TransformListener
from ip12_sim import BOX_START, arm_command, box_resting_z, image_to_numpy, mat_to_quat, set_model_pose, tf_matrix

PATTERN, SQUARE = (8, 5), 0.020          # inner corners of the 9 x 6 board in worlds/workcell.sdf
BOARD_ON_PALLET, BOARD_PARK = (-0.05, 0.0, 0.771), (-0.40, -0.30, 0.751)   # worlds/workcell.sdf
# the ArUco sheet lies on the pallet too; its corner markers around the board confuse the corner finder, so it moves away
SHEET_ON_PALLET, SHEET_PARK = (-0.05, 0.0, 0.771), (-0.40, 0.30, 0.751)
# views start from these poses (random one each time): hover_pallet (config/pick_sequence.yaml) and two poses that look at the
# pallet centre 30 deg from the side. A camera looking straight down sees the board dead-on, which fixes tilt and distance
# badly (11 mm error with the straight camera); the side views give the perspective the solver needs.
LOOKS = np.array([[3.1416, 0.1379, 0.6841, 0.0, 0.7487, 0.0],
                  [2.4601, 0.1024, 0.7426, 0.4463, 1.1161, 0.0925], [3.6587, 0.1165, 0.6640, -0.5025, 1.1215, -0.0255]])
SPREAD = np.array([0.25, 0.10, 0.10, 0.35, 0.25, 0.8])


class Calib(Node):
    def __init__(self):
        super().__init__("handeye_calib", parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.declare_parameter("views", 15)
        self.declare_parameter("out", os.path.expanduser("~/ip12_handeye.yaml"))
        self.tf = Buffer(); TransformListener(self.tf, self)
        self.img = self.K = None
        self.create_subscription(Image, "/cam3/left/image_raw", lambda m: setattr(self, "img", m), 1)
        self.create_subscription(CameraInfo, "/cam3/left/camera_info", lambda m: setattr(self, "K", np.array(m.k).reshape(3, 3)), 1)
        self.arm = self.create_publisher(JointTrajectory, "/arm_controller/joint_trajectory", 10)

    def wait(self, sec):
        end = self.get_clock().now().nanoseconds + sec * 1e9
        while rclpy.ok() and self.get_clock().now().nanoseconds < end: rclpy.spin_once(self, timeout_sec=0.05)

    def fresh(self):
        t0 = self.get_clock().now().nanoseconds; self.img = None
        while rclpy.ok() and not (self.img and rclpy.time.Time.from_msg(self.img.header.stamp).nanoseconds > t0):
            rclpy.spin_once(self, timeout_sec=0.05)
        return self.img

    def run(self):
        while rclpy.ok() and self.K is None: rclpy.spin_once(self, timeout_sec=0.1)
        for name, (x, y, yaw) in BOX_START.items(): set_model_pose(name, x, y, box_resting_z(x, y), yaw)
        set_model_pose("aruco_sheet", *SHEET_PARK); set_model_pose("checkerboard", *BOARD_ON_PALLET)
        obj = np.zeros((PATTERN[0] * PATTERN[1], 3), np.float32)
        obj[:, :2] = np.mgrid[0:PATTERN[0], 0:PATTERN[1]].T.reshape(-1, 2) * SQUARE
        Rg, tg, Rc, tc = [], [], [], []
        rng = np.random.default_rng(1); want = self.get_parameter("views").value
        for i in range(want * 4):
            if len(Rg) >= want: break
            self.arm.publish(arm_command(LOOKS[rng.integers(len(LOOKS))] + rng.uniform(-1, 1, 6) * SPREAD, 1.5)); self.wait(2.5)
            m = self.fresh(); gray = cv2.cvtColor(image_to_numpy(m), cv2.COLOR_BGR2GRAY)
            ok, corners = cv2.findChessboardCorners(gray, PATTERN)
            if not ok: self.get_logger().info(f"pose {i + 1}: board not found"); continue
            corners = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3))
            ok, rvec, tvec = cv2.solvePnP(obj, corners, self.K, None)
            G = tf_matrix(self.tf, "world", "gripper_link", rclpy.time.Time.from_msg(m.header.stamp), fallback=True)
            Rg.append(G[:3, :3]); tg.append(G[:3, 3]); Rc.append(cv2.Rodrigues(rvec)[0]); tc.append(tvec.ravel())
            self.get_logger().info(f"pose {i + 1}: view {len(Rg)}/{want} ok")
        set_model_pose("checkerboard", *BOARD_PARK); set_model_pose("aruco_sheet", *SHEET_ON_PALLET)
        if len(Rg) < 5: self.get_logger().error(f"only {len(Rg)} good views, need at least 5"); return
        R, t = cv2.calibrateHandEye(Rg, tg, Rc, tc, method=cv2.CALIB_HAND_EYE_TSAI)
        truth = tf_matrix(self.tf, "gripper_link", "cam3_left_optical_frame")
        dt = np.linalg.norm(t.ravel() - truth[:3, 3]) * 1000
        dr = np.degrees(np.arccos(np.clip((np.trace(R.T @ truth[:3, :3]) - 1) / 2, -1, 1)))
        res = {"parent": "gripper_link", "child": "cam3_left_optical_frame", "views": len(Rg),
               "translation_m": [round(float(v), 5) for v in t.ravel()], "quaternion_xyzw": [round(float(v), 6) for v in mat_to_quat(R)],
               "cad_translation_m": [round(float(v), 5) for v in truth[:3, 3]], "error_mm": round(float(dt), 2), "error_deg": round(float(dr), 3)}
        yaml.safe_dump(res, open(self.get_parameter("out").value, "w"), sort_keys=False)
        self.get_logger().info(f"camera in gripper_link: t = {np.round(t.ravel(), 4)} m, error vs CAD {dt:.1f} mm / {dr:.2f} deg "
                               f"({len(Rg)} views) -> {self.get_parameter('out').value}")


def main():
    rclpy.init(); node = Calib()
    try: node.run()
    finally: rclpy.shutdown()


if __name__ == "__main__":
    main()
