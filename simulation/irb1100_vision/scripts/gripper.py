#!/usr/bin/env python3
# @ip12:robot  simulation: opens/closes the Zimmer jaws and holds the box between them
"""Gripper for the Gazebo simulation.
   ros2 topic pub --once /gripper/command std_msgs/msg/String "{data: close}"     # or: open
Close: the jaws move in; if a box is between the jaws (within 30 mm of gripper_tcp) the jaws stop at its width and the
box is fixed to the wrist (gz DetachableJoint), so it is carried reliably. Open: jaws back to open, box released.
Publishes /gripper/holding (String, box name or "") for other scripts.
At start the gz plugin attaches every box (its default), so this node releases them once the robot is up.
"""
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, Float64MultiArray, String
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener

BOXES = ["box_red", "box_blue", "box_green"]   # must match worlds/workcell.sdf and the grip_box plugins in zimmer_lwr50l.xacro
BOX = 0.025          # box width (m)
OPEN_GAP = 0.0328    # jaw gap when open (m)
STROKE = 0.010       # per jaw (m)
REACH = 0.030        # a box this close to gripper_tcp counts as "between the jaws" (m)


class Gripper(Node):
    def __init__(self):
        super().__init__("gripper")
        self.jaws = self.create_publisher(Float64MultiArray, "/gripper_controller/commands", 10)
        self.holding_pub = self.create_publisher(String, "/gripper/holding", 10)
        self.att = {b: self.create_publisher(Empty, f"/grip/{b}/attach", 10) for b in BOXES}
        self.det = {b: self.create_publisher(Empty, f"/grip/{b}/detach", 10) for b in BOXES}
        self.poses = {}
        self.holding = ""
        self.tf = Buffer(); TransformListener(self.tf, self)
        for b in BOXES:
            self.create_subscription(PoseStamped, f"/sim/{b}/pose", lambda m, b=b: self.on_pose(b, m), 10)
        self.create_subscription(String, "/gripper/command", self.on_cmd, 10)
        self.released = 0
        self.create_timer(1.0, self.startup_release)

    def startup_release(self):
        if self.released < 3:            # a few times: the plugin may attach a moment after the first message
            for b in BOXES: self.det[b].publish(Empty())
            self.jaws.publish(Float64MultiArray(data=[0.0, 0.0]))
            self.released += 1
        self.holding_pub.publish(String(data=self.holding))

    def on_pose(self, box, m):
        p = m.pose.position; self.poses[box] = (p.x, p.y, p.z)

    def on_cmd(self, m):
        if m.data == "open":
            if self.holding: self.det[self.holding].publish(Empty()); self.get_logger().info(f"released {self.holding}")
            self.holding = ""
            self.jaws.publish(Float64MultiArray(data=[0.0, 0.0]))
        elif m.data == "close":
            try:
                t = self.tf.lookup_transform("world", "gripper_tcp", rclpy.time.Time()).transform.translation
            except Exception as e:
                self.get_logger().warn(f"no gripper_tcp pose yet: {e}"); return
            near = [(math.dist((t.x, t.y, t.z), p), b) for b, p in self.poses.items()]
            d, box = min(near) if near else (1.0, "")
            if d < REACH:
                self.jaws.publish(Float64MultiArray(data=[(OPEN_GAP - BOX) / 2] * 2))
                self.att[box].publish(Empty()); self.holding = box
                self.get_logger().info(f"gripped {box} ({d * 1000:.0f} mm from the TCP)")
            else:
                self.jaws.publish(Float64MultiArray(data=[STROKE, STROKE]))
                self.get_logger().info(f"closed, no box between the jaws (nearest {box or '-'} {d * 1000:.0f} mm from the TCP)")
        self.holding_pub.publish(String(data=self.holding))


def main():
    rclpy.init(); rclpy.spin(Gripper())


if __name__ == "__main__":
    main()
