#!/usr/bin/env python3
# @ip12:ml  writes a labelled YOLO dataset from the simulated wrist camera (no hand labelling)
"""Auto-labelled training pictures for YOLO, made in Gazebo.
   ros2 launch irb1100_vision gazebo.launch.py panel:=false            # first, in another terminal
   ros2 run irb1100_vision make_dataset.py --ros-args -p count:=300 -p out:=$HOME/ip12_dataset
Each sample: the boxes are put at random spots/angles on the table and pallet, the arm goes to a random pose above the
table, the wrist camera picture is saved and every visible box gets a YOLO label (class x_centre y_centre width height,
0..1), computed from the true box pose and the camera calibration. Boxes hidden behind something (depth check) or cut by
the picture edge by more than half are left out. 80 % train / 20 % val, plus dataset.yaml for Ultralytics:
   yolo train model=yolo11n.pt data=$HOME/ip12_dataset/dataset.yaml epochs=50 imgsz=640
At the end the boxes go back to their start places, so the pick sequence still works.
"""
import os, cv2, numpy as np, rclpy
import rclpy.parameter
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from trajectory_msgs.msg import JointTrajectory
from tf2_ros import Buffer, TransformListener
from ip12_sim import BOX, BOX_START, BOXES, CLASSES, BoxPoses, arm_command, box_resting_z, image_to_numpy, set_model_pose, tf_matrix

LOOK = [3.1416, 0.1379, 0.6841, 0.0, 0.7487, 0.0]  # hover_pallet (config/pick_sequence.yaml): straight-down camera ~30 cm above the pallet
SPREAD = [0.45, 0.15, 0.12, 0.05, 0.20, 1.5]       # random +- per joint around LOOK (rad)
CORNERS = np.array([[x * BOX / 2, y * BOX / 2, z * BOX / 2, 1] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)])


class Maker(Node):
    def __init__(self):
        super().__init__("make_dataset", parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.declare_parameter("count", 200)
        self.declare_parameter("out", os.path.expanduser("~/ip12_dataset"))
        self.declare_parameter("seed", 0)
        self.tf = Buffer(); TransformListener(self.tf, self)
        self.truth = BoxPoses(self)
        self.img = self.depth = self.K = None
        self.create_subscription(Image, "/cam3/left/image_raw", lambda m: setattr(self, "img", m), 1)
        self.create_subscription(Image, "/cam3/depth/image_raw", lambda m: setattr(self, "depth", m), 1)
        self.create_subscription(CameraInfo, "/cam3/left/camera_info", lambda m: setattr(self, "K", np.array(m.k).reshape(3, 3)), 1)
        self.arm = self.create_publisher(JointTrajectory, "/arm_controller/joint_trajectory", 10)

    def wait(self, sec):                                   # sim time, so slow PCs still settle
        end = self.get_clock().now().nanoseconds + sec * 1e9
        while rclpy.ok() and self.get_clock().now().nanoseconds < end: rclpy.spin_once(self, timeout_sec=0.05)

    def fresh_frame(self):
        t0 = self.get_clock().now().nanoseconds; self.img = self.depth = None
        while rclpy.ok() and not (self.img and self.depth and rclpy.time.Time.from_msg(self.img.header.stamp).nanoseconds > t0):
            rclpy.spin_once(self, timeout_sec=0.05)
        return self.img, self.depth

    def labels(self, img, depth):
        T_cam = np.linalg.inv(tf_matrix(self.tf, "world", img.header.frame_id, rclpy.time.Time.from_msg(img.header.stamp)))
        D = image_to_numpy(depth); h, w = D.shape; out = []
        for name, cls in BOXES.items():
            Tb = self.truth.T.get(name)
            if Tb is None: continue
            pc = (T_cam @ Tb @ CORNERS.T)[:3]                       # 8 corners in the optical frame
            if (pc[2] <= 0.05).any(): continue
            uv = (self.K @ pc)[:2] / pc[2]
            x1, y1, x2, y2 = uv[0].min(), uv[1].min(), uv[0].max(), uv[1].max()
            cx1, cy1, cx2, cy2 = max(x1, 0), max(y1, 0), min(x2, w - 1), min(y2, h - 1)
            if cx2 <= cx1 or cy2 <= cy1 or (cx2 - cx1) * (cy2 - cy1) < 0.5 * (x2 - x1) * (y2 - y1): continue
            c = (T_cam @ Tb[:, 3])[:3]; u, v = (self.K @ c)[:2] / c[2]            # centre visible? (nothing in front of it)
            if not (0 <= u < w and 0 <= v < h) or D[int(v), int(u)] < c[2] - BOX: continue
            out.append(f"{cls} {(cx1 + cx2) / 2 / w:.6f} {(cy1 + cy2) / 2 / h:.6f} {(cx2 - cx1) / w:.6f} {(cy2 - cy1) / h:.6f}")
        return out

    def run(self):
        n, out = self.get_parameter("count").value, self.get_parameter("out").value
        rng = np.random.default_rng(self.get_parameter("seed").value)
        for split in ("train", "val"):
            for sub in ("images", "labels"): os.makedirs(f"{out}/{sub}/{split}", exist_ok=True)
        while rclpy.ok() and (self.K is None or len(self.truth.T) < len(BOXES)): rclpy.spin_once(self, timeout_sec=0.1)
        made = 0
        for i in range(n):
            spots = []
            for name in BOXES:                                      # random spots, at least 6 cm apart
                while True:
                    x, y = rng.uniform(-0.32, 0.10), rng.uniform(-0.22, 0.22)
                    if all(np.hypot(x - a, y - b) > 0.06 for a, b in spots): break
                spots.append((x, y)); set_model_pose(name, x, y, box_resting_z(x, y), rng.uniform(-np.pi, np.pi))
            self.arm.publish(arm_command(np.array(LOOK) + rng.uniform(-1, 1, 6) * SPREAD, 1.5))
            self.wait(2.5)
            img, depth = self.fresh_frame()
            lab = self.labels(img, depth)
            split = "val" if i % 5 == 4 else "train"
            cv2.imwrite(f"{out}/images/{split}/{i:05d}.png", image_to_numpy(img))
            open(f"{out}/labels/{split}/{i:05d}.txt", "w").write("\n".join(lab) + ("\n" if lab else ""))
            made += bool(lab)
            self.get_logger().info(f"{i + 1}/{n}: {len(lab)} box(es)")
        open(f"{out}/dataset.yaml", "w").write(f"path: {out}\ntrain: images/train\nval: images/val\nnames:\n" +
                                               "".join(f"  {i}: {c}\n" for i, c in enumerate(CLASSES)))
        for name, (x, y, yaw) in BOX_START.items(): set_model_pose(name, x, y, box_resting_z(x, y), yaw)
        self.get_logger().info(f"done: {n} pictures ({made} with boxes) in {out}, boxes back at their start places")


def main():
    rclpy.init(); node = Maker()
    try: node.run()
    finally: rclpy.shutdown()


if __name__ == "__main__":
    main()
