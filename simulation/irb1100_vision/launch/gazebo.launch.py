# @ip12:robot  simulation: same workcell in Gazebo with ros2_control and camera bridges
"""Gazebo (gz sim) + ros2_control + camera bridges + RViz2.
   ros2 launch irb1100_vision gazebo.launch.py
   ros2 launch irb1100_vision gazebo.launch.py gz_args:="-r -s --headless-rendering"   # no gz GUI
"""
import os
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share = get_package_share_directory("irb1100_vision")
    xacro_file = os.path.join(share, "urdf", "workcell.urdf.xacro")
    controllers = os.path.join(share, "config", "controllers.yaml")
    world = os.path.join(share, "worlds", "workcell.sdf")

    robot_description = ParameterValue(
        Command(["xacro ", xacro_file, " controllers_yaml:=", controllers]), value_type=str)

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": LaunchConfiguration("gz_args")}.items())

    rsp = Node(package="robot_state_publisher", executable="robot_state_publisher",
               parameters=[{"robot_description": robot_description, "use_sim_time": True}])

    spawn = Node(package="ros_gz_sim", executable="create", output="screen",
                 arguments=["-topic", "robot_description", "-name", "irb1100_workcell"])

    bridge = Node(package="ros_gz_bridge", executable="parameter_bridge",
                  parameters=[{"config_file": os.path.join(share, "config", "bridge.yaml")}])
    images = Node(package="ros_gz_image", executable="image_bridge",
                  arguments=["/cam1/image_raw", "/cam2/left/image_raw", "/cam2/right/image_raw"])

    jsb = Node(package="controller_manager", executable="spawner",
               arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"])
    arm = Node(package="controller_manager", executable="spawner",
               arguments=["arm_controller", "--controller-manager", "/controller_manager"])

    fov = Node(package="irb1100_vision", executable="fov_markers.py",
               parameters=[{"use_sim_time": True}])
    rviz = Node(package="rviz2", executable="rviz2", output="screen",
                arguments=["-d", os.path.join(share, "rviz", "vision.rviz")],
                parameters=[{"use_sim_time": True}])

    return LaunchDescription([
        DeclareLaunchArgument("gz_args", default_value=f"-r {world}"),
        # lets gz resolve package://irb1100_vision/meshes/...
        AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.path.dirname(share)),
        gz, rsp, spawn, bridge, images, fov, rviz,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb])),
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[arm])),
    ])
