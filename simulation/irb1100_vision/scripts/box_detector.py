#!/usr/bin/env python3
# @ip12:vision  finds the boxes in the wrist ZED picture and gives their position in robot (world) coordinates
"""Box detector for the wrist camera (simulation now, the real ZED Mini later: same topics and frames).
   ros2 run irb1100_vision box_detector.py                                       # colour detector (red/blue/green boxes)
   ros2 run irb1100_vision box_detector.py --ros-args -p model:=/path/best.pt    # YOLO weights (pip install ultralytics),
                                                                                  #  e.g. trained on scripts/make_dataset.py output
Pipeline: 2D box in the image -> depth at its centre (ZED depth image) -> 3D point in cam3_left_optical_frame -> TF -> world.
Out:  /cam3/detections (PoseArray, world frame, one pose per box), /cam3/detections/markers (RViz; marker text = class),
      /cam3/detections/image (picture with the boxes drawn).
In simulation it also compares with the true box positions (/sim/<box>/pose) and logs the error in mm.
"""
import cv2, numpy as np, rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray
from sensor_msgs.msg import CameraInfo, Image
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener
from ip12_sim import BOX, BOXES, CLASSES, BoxPoses, image_to_numpy, tf_matrix

# HSV ranges for the coloured boxes (OpenCV hue 0-180); red wraps around 0
HSV = {"red": [((0, 120, 70), (8, 255, 255)), ((172, 120, 70), (180, 255, 255))],
       "blue": [((100, 120, 60), (130, 255, 255))],
       "green": [((40, 100, 50), (85, 255, 255))]}
RGB = {"red": (0.9, 0.1, 0.1), "blue": (0.1, 0.3, 0.9), "green": (0.1, 0.7, 0.2)}


class Detector(Node):
    def __init__(self):
        super().__init__("box_detector")
        self.declare_parameter("model", "")            # YOLO .pt; empty = colour detector
        self.declare_parameter("min_area", 60)         # pixels
        self.declare_parameter("world_frame", "world")
        self.yolo = None
        if self.get_parameter("model").value:
            from ultralytics import YOLO               # only needed with a model
            self.yolo = YOLO(self.get_parameter("model").value)
        self.tf = Buffer(); TransformListener(self.tf, self)
        self.truth = BoxPoses(self)
        self.K = None; self.depth = None
        self.create_subscription(CameraInfo, "/cam3/left/camera_info", lambda m: setattr(self, "K", np.array(m.k).reshape(3, 3)), 1)
        self.create_subscription(Image, "/cam3/depth/image_raw", lambda m: setattr(self, "depth", image_to_numpy(m)), 1)
        self.create_subscription(Image, "/cam3/left/image_raw", self.on_image, 1)
        self.pub = self.create_publisher(PoseArray, "/cam3/detections", 5)
        self.mpub = self.create_publisher(MarkerArray, "/cam3/detections/markers", 5)
        self.ipub = self.create_publisher(Image, "/cam3/detections/image", 2)

    def detect(self, bgr):
        """-> list of (class name, x1, y1, x2, y2)"""
        if self.yolo:
            r = self.yolo(bgr, verbose=False)[0]
            return [(r.names[int(c)], *map(int, b)) for b, c in zip(r.boxes.xyxy.tolist(), r.boxes.cls.tolist())]
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV); out = []
        for name, ranges in HSV.items():
            mask = sum(cv2.inRange(hsv, lo, hi) for lo, hi in ranges).clip(0, 255).astype(np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
            for c in cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]:
                if cv2.contourArea(c) >= self.get_parameter("min_area").value:
                    x, y, w, h = cv2.boundingRect(c); out.append((name, x, y, x + w, y + h))
        return out

    def on_image(self, m):
        if self.K is None or self.depth is None: return
        bgr = image_to_numpy(m)
        world = self.get_parameter("world_frame").value
        try:
            T = tf_matrix(self.tf, world, m.header.frame_id, rclpy.time.Time.from_msg(m.header.stamp))
        except Exception:
            try: T = tf_matrix(self.tf, world, m.header.frame_id)       # latest, if the exact stamp is not in the buffer yet
            except Exception: return
        poses = PoseArray(); poses.header.frame_id = world; poses.header.stamp = m.header.stamp
        markers = MarkerArray(markers=[Marker(action=Marker.DELETEALL)]); log = []
        for i, (name, x1, y1, x2, y2) in enumerate(self.detect(bgr)):
            u, v = (x1 + x2) // 2, (y1 + y2) // 2
            win = self.depth[max(0, v - 3):v + 4, max(0, u - 3):u + 4]
            z = float(np.nanmedian(win[np.isfinite(win)])) if np.isfinite(win).any() else np.nan
            col = tuple(int(255 * c) for c in RGB.get(name, (0.0, 0.0, 0.0))[::-1])   # class colour, BGR (white vanished on the table)
            cv2.rectangle(bgr, (x1 - 3, y1 - 3), (x2 + 3, y2 + 3), col, 2)
            cv2.putText(bgr, name, (x1, max(12, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2)
            if not np.isfinite(z): continue
            z += BOX / 2                                                  # the depth hits the face; the centre is half a box further
            p_cam = np.array([(u - self.K[0, 2]) * z / self.K[0, 0], (v - self.K[1, 2]) * z / self.K[1, 1], z, 1.0])
            p = (T @ p_cam)[:3]
            pose = Pose(); pose.position.x, pose.position.y, pose.position.z = map(float, p); pose.orientation.w = 1.0
            poses.poses.append(pose)
            mk = Marker(header=poses.header, ns="boxes", id=i, type=Marker.CUBE, pose=pose, text=name)   # text = class (udp_bridge reads it)
            mk.scale.x = mk.scale.y = mk.scale.z = BOX; mk.color.r, mk.color.g, mk.color.b = RGB.get(name, (1.0, 1.0, 1.0)); mk.color.a = 0.6
            markers.markers.append(mk)
            truth = self.truth.T.get(f"box_{name}")
            err = f", error {np.linalg.norm(truth[:3, 3] - p) * 1000:.0f} mm" if truth is not None else ""
            log.append(f"{name} at ({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f}){err}")
        self.pub.publish(poses); self.mpub.publish(markers)
        out = Image(header=m.header, height=bgr.shape[0], width=bgr.shape[1], encoding="bgr8", step=bgr.shape[1] * 3, data=bgr.tobytes())
        self.ipub.publish(out)
        if log: self.get_logger().info(" | ".join(log), throttle_duration_sec=2.0)


def main():
    rclpy.init(); rclpy.spin(Detector())


if __name__ == "__main__":
    main()
