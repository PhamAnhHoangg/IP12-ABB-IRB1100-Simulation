#!/usr/bin/env python3
# @ip12:camera  simulation: puts the Gazebo point cloud of the wrist ZED into the right frame
"""/cam3/points_gz -> /cam3/points with frame cam3_left_cloud_frame.
Gazebo's RGB-D sensor writes the cloud with x-forward axes but labels it with the optical frame (z-forward), so RViz/TF
would turn it 90 deg. Only the header changes; the data is passed through."""
import rclpy
from sensor_msgs.msg import PointCloud2


def main():
    rclpy.init(); n = rclpy.create_node("points_relay")
    pub = n.create_publisher(PointCloud2, "/cam3/points", 5)
    def cb(m):
        m.header.frame_id = "cam3_left_cloud_frame"; pub.publish(m)
    n.create_subscription(PointCloud2, "/cam3/points_gz", cb, 5)
    rclpy.spin(n)


if __name__ == "__main__":
    main()
