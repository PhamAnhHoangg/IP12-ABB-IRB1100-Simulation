# @ip12:robot  simulation: ABB IRB 1100 + Zimmer gripper workcell in RViz2 (joint sliders)
"""RViz2 only (no Gazebo): robot, workcell, camera frames and view cones.
   ros2 launch irb1100_vision display.launch.py
   ros2 launch irb1100_vision display.launch.py cam1_tilt_deg:=15     # Cam 1 tilted 15 deg down (see config/cam1_poses/)
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

HOME = {"joint_1": 1.5708, "joint_2": 0.0, "joint_3": 0.0,
        "joint_4": 0.0, "joint_5": 0.0, "joint_6": 0.0}


def generate_launch_description():
    share = get_package_share_directory("irb1100_vision")
    xacro_file = os.path.join(share, "urdf", "workcell.urdf.xacro")
    robot_description = ParameterValue(Command(["xacro ", xacro_file, " cam1_tilt_deg:=", LaunchConfiguration("cam1_tilt_deg")]), value_type=str)
    return LaunchDescription([
        DeclareLaunchArgument("cam1_tilt_deg", default_value="0"),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": robot_description}]),
        Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui",
             parameters=[{"zeros": HOME}]),
        Node(package="irb1100_vision", executable="fov_markers.py"),
        Node(package="rviz2", executable="rviz2", output="screen",
             arguments=["-d", os.path.join(share, "rviz", "vision.rviz")]),
    ])
