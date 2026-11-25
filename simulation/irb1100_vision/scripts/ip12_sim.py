# @ip12:tools  shared helpers for the simulation scripts (box_detector, make_dataset, handeye_calib)
"""Small helpers: ROS image/TF messages -> numpy, the boxes of worlds/workcell.sdf, moving the arm and the boxes."""
import subprocess
import numpy as np
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

BOX = 0.025                                                   # box edge (m), worlds/workcell.sdf
BOXES = {"box_red": 0, "box_blue": 1, "box_green": 2}         # name -> class id (YOLO labels, detector output)
CLASSES = ["red", "blue", "green"]
JOINTS = [f"joint_{i}" for i in range(1, 7)]
TABLE_Z, PALLET_TOP = 0.75, 0.77
BOX_START = {"box_red": (-0.05, 0.12, -0.4468), "box_blue": (-0.05, -0.12, 0.4468), "box_green": (-0.30, 0.10, 0.0)}  # x, y, yaw in worlds/workcell.sdf


def quat_to_mat(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def mat_to_quat(R):
    w = np.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = np.copysign(np.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = np.copysign(np.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = np.copysign(np.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    return x, y, z, w


def tf_matrix(buf, target, source, time=None, fallback=False):
    """4x4 that maps points in `source` into `target` (tf2 Buffer).
    fallback: if `time` is not in the buffer (yet), use the latest transform; fine while the arm stands still."""
    import rclpy
    try:
        t = buf.lookup_transform(target, source, time or rclpy.time.Time()).transform
    except Exception:
        if not fallback or time is None: raise
        t = buf.lookup_transform(target, source, rclpy.time.Time()).transform
    T = np.eye(4); T[:3, :3] = quat_to_mat(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w)
    T[:3, 3] = [t.translation.x, t.translation.y, t.translation.z]
    return T


def image_to_numpy(m):
    """sensor_msgs/Image (rgb8, bgr8 or 32FC1) -> numpy; colour comes back as BGR for OpenCV."""
    if m.encoding == "32FC1":
        return np.frombuffer(bytes(m.data), np.float32).reshape(m.height, m.width)
    a = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width, 3)
    return a[:, :, ::-1].copy() if m.encoding == "rgb8" else a.copy()


class BoxPoses:
    """Ground-truth box poses from Gazebo (/sim/<box>/pose): name -> 4x4 in the world frame."""
    def __init__(self, node):
        self.T = {}
        for b in BOXES:
            node.create_subscription(PoseStamped, f"/sim/{b}/pose", lambda m, b=b: self._cb(b, m), 10)

    def _cb(self, b, m):
        p, q = m.pose.position, m.pose.orientation
        T = np.eye(4); T[:3, :3] = quat_to_mat(q.x, q.y, q.z, q.w); T[:3, 3] = [p.x, p.y, p.z]; self.T[b] = T


def arm_command(q, secs=2.0):
    m = JointTrajectory(); m.joint_names = JOINTS
    m.points = [JointTrajectoryPoint(positions=[float(v) for v in q], time_from_start=Duration(sec=int(secs), nanosec=int(secs % 1 * 1e9)))]
    return m


def set_model_pose(name, x, y, z, yaw=0.0, world="workcell"):
    """Teleport a Gazebo model (gz service, needs the gz CLI that comes with ROS 2 Jazzy + gz)."""
    req = (f'name: "{name}" position {{x: {x} y: {y} z: {z}}} '
           f'orientation {{x: 0 y: 0 z: {np.sin(yaw / 2)} w: {np.cos(yaw / 2)}}}')
    return subprocess.run(["gz", "service", "-s", f"/world/{world}/set_pose", "--reqtype", "gz.msgs.Pose",
                           "--reptype", "gz.msgs.Boolean", "--timeout", "3000", "--req", req],
                          capture_output=True, text=True).stdout.strip() == "data: true"


def box_resting_z(x, y):
    """Centre height of a box lying on the table or on the pallet (0.30 x 0.20 at x = -0.05, y = 0)."""
    on_pallet = abs(x + 0.05) < 0.15 and abs(y) < 0.10
    return (PALLET_TOP if on_pallet else TABLE_Z) + BOX / 2
