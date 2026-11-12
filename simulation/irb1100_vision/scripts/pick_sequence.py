#!/usr/bin/env python3
# @ip12:robot  plays the fixed pick order from config/pick_sequence.yaml on /arm_controller
"""Fixed demo order: home -> left -> touch table -> lift -> right -> touch -> lift -> pallet -> touch -> home.
   ros2 run irb1100_vision pick_sequence.py                       # once
   ros2 run irb1100_vision pick_sequence.py --ros-args -p loops:=0 -p move_s:=4.0   # loops:=0 = forever
Boxes are not in the model yet, so "touch" = the gripper tip stops 5 mm above the table top.
"""
import os, yaml, rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from ament_index_python.packages import get_package_share_directory


class Pick(Node):
    def __init__(self):
        super().__init__("pick_sequence", parameter_overrides=[])
        self.declare_parameter("loops", 1); self.declare_parameter("move_s", 3.0)
        self.declare_parameter("pause_s", 1.0); self.declare_parameter("start_delay_s", 8.0)
        cfg = os.path.join(get_package_share_directory("irb1100_vision"), "config", "pick_sequence.yaml")
        self.cfg = yaml.safe_load(open(cfg))
        self.pub = self.create_publisher(JointTrajectory, "/arm_controller/joint_trajectory", 10)
        self.i = 0; self.loop = 0; self.timer = self.create_timer(float(self.get_parameter("start_delay_s").value), self.step)

    def step(self):
        steps = self.cfg["steps"]; move = float(self.get_parameter("move_s").value)
        st = steps[self.i]
        msg = JointTrajectory(); msg.joint_names = self.cfg["joints"]
        pt = JointTrajectoryPoint(positions=st["q"], time_from_start=Duration(sec=int(move), nanosec=int((move % 1) * 1e9)))
        msg.points = [pt]; self.pub.publish(msg)
        self.get_logger().info(f"[{self.i + 1}/{len(steps)}] {st['name']}")
        self.timer.cancel()
        self.i += 1
        if self.i >= len(steps):
            self.loop += 1; self.i = 1
            loops = int(self.get_parameter("loops").value)
            if loops and self.loop >= loops:
                self.get_logger().info("sequence finished"); self.timer = self.create_timer(move + 1.0, lambda: rclpy.shutdown()); return
        pause = float(self.get_parameter("pause_s").value)
        self.timer = self.create_timer(move + min(pause, move * 0.5), self.step)


def main():
    rclpy.init(); n = Pick()
    try: rclpy.spin(n)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException): pass


if __name__ == "__main__":
    main()
