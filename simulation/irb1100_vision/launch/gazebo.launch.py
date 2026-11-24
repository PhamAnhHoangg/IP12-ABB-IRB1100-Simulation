# @ip12:robot  simulation: same workcell in Gazebo with ros2_control and camera bridges
"""Gazebo (gz sim) + ros2_control + camera bridges + RViz2. Gripper: ros2 topic pub --once /gripper/command std_msgs/msg/String "{data: close}" (or open)
   ros2 launch irb1100_vision gazebo.launch.py
   ros2 launch irb1100_vision gazebo.launch.py sliders:=true          # joint slider panel: click/drag to move the arm
   ros2 launch irb1100_vision gazebo.launch.py                          # opens the button panel (panel:=false to hide)
   ros2 launch irb1100_vision gazebo.launch.py lite:=true rviz_cfg:=vision_lite.rviz   # smoothest with camera images
   ros2 launch irb1100_vision gazebo.launch.py rviz_cfg:=vision_noimg.rviz              # smoothest arm, no camera images in RViz
   ros2 launch irb1100_vision gazebo.launch.py lite:=true               # light cameras (640x360, 10 fps) for smooth motion
   ros2 launch irb1100_vision gazebo.launch.py stereo:=false lite:=true # wrist ZED left eye only, light
   ros2 launch irb1100_vision gazebo.launch.py sequence:=true loops:=0   # fixed pick order, repeat forever
   ros2 launch irb1100_vision gazebo.launch.py gz_args:="-r -s --headless-rendering"   # no gz GUI
"""
import os
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share = get_package_share_directory("irb1100_vision")
    xacro_file = os.path.join(share, "urdf", "workcell.urdf.xacro")
    controllers = os.path.join(share, "config", "controllers.yaml")
    world = os.path.join(share, "worlds", "workcell.sdf")

    robot_description = ParameterValue(
        Command(["xacro ", xacro_file, " controllers_yaml:=", controllers,
                 " cam_right:=", LaunchConfiguration("stereo"),
                 " cam_w:=", PythonExpression(["'640' if '", LaunchConfiguration("lite"), "'=='true' else '1280'"]),
                 " cam_h:=", PythonExpression(["'360' if '", LaunchConfiguration("lite"), "'=='true' else '720'"]),
                 " cam_fps:=", PythonExpression(["'10' if '", LaunchConfiguration("lite"), "'=='true' else '15'"])]), value_type=str)

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
                  arguments=["/cam3/right/image_raw"])   # left eye (RGB-D) is in config/bridge.yaml

    jsb = Node(package="controller_manager", executable="spawner",
               arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager",
                          "--controller-manager-timeout", "60", "--switch-timeout", "60"])
    arm = Node(package="controller_manager", executable="spawner",
               arguments=["arm_controller", "--controller-manager", "/controller_manager",
                          "--controller-manager-timeout", "60", "--switch-timeout", "60"])

    grip_ctrl = Node(package="controller_manager", executable="spawner",
                     arguments=["gripper_controller", "--controller-manager", "/controller_manager",
                                "--controller-manager-timeout", "60", "--switch-timeout", "60"])
    gripper = Node(package="irb1100_vision", executable="gripper.py", output="screen", parameters=[{"use_sim_time": True}])
    points = Node(package="irb1100_vision", executable="points_relay.py")
    detect = Node(package="irb1100_vision", executable="box_detector.py", parameters=[{"use_sim_time": True}],
                  condition=IfCondition(LaunchConfiguration("detect")))

    sliders = Node(package="rqt_joint_trajectory_controller", executable="rqt_joint_trajectory_controller",
                   condition=IfCondition(LaunchConfiguration("sliders")))

    seq = Node(package="irb1100_vision", executable="pick_sequence.py", output="screen",
               parameters=[{"use_sim_time": True, "loops": ParameterValue(LaunchConfiguration("loops"), value_type=int)}],
               condition=IfCondition(LaunchConfiguration("sequence")))

    panel = Node(package="irb1100_vision", executable="pick_panel.py", output="screen",
                 condition=IfCondition(LaunchConfiguration("panel")))

    fov = Node(package="irb1100_vision", executable="fov_markers.py",
               parameters=[{"use_sim_time": True}])
    rviz = Node(package="rviz2", executable="rviz2", output="screen",
                arguments=["-d", PathJoinSubstitution([share, "rviz", LaunchConfiguration("rviz_cfg")])],
                parameters=[{"use_sim_time": True}])

    return LaunchDescription([
        DeclareLaunchArgument("sliders", default_value="false"),
        DeclareLaunchArgument("rviz_cfg", default_value="vision.rviz"),   # vision_lite.rviz = no ZED right image, 15 fps; vision_noimg.rviz = no camera images
        DeclareLaunchArgument("stereo", default_value="true"),    # false = only the wrist ZED left eye (one image stream)
        DeclareLaunchArgument("lite", default_value="false"),       # true = cameras 640x360 @10 fps: much smoother on slow PCs
        DeclareLaunchArgument("panel", default_value="true"),       # button window: play / stop / go to a step
        DeclareLaunchArgument("sequence", default_value="false"),   # true = play the fixed pick order
        DeclareLaunchArgument("loops", default_value="1"),          # 0 = repeat forever
        DeclareLaunchArgument("detect", default_value="true"),      # colour box detector on the wrist camera
        DeclareLaunchArgument("gz_args", default_value=f"-r {world}"),
        # lets gz resolve package://irb1100_vision/meshes/...
        AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.path.dirname(share)),
        gz, rsp, spawn, bridge, images, points, detect, fov, rviz, sliders, panel,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb])),
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[arm])),
        RegisterEventHandler(OnProcessExit(target_action=arm, on_exit=[grip_ctrl])),
        RegisterEventHandler(OnProcessExit(target_action=grip_ctrl, on_exit=[gripper, seq])),
    ])
