# irb1100_vision - ABB IRB 1100 + Zimmer LWR50L + ZED Mini wrist camera (RViz2 and Gazebo)

Target: ROS 2 Jazzy + Gazebo Harmonic (`gz sim`). Other distros: package names are the same, but
`gz_ros2_control` / `ros_gz_*` versions must match your Gazebo.

## Build and run
    cd ~/ros2_ws/src && unzip irb1100_vision.zip
    cd ~/ros2_ws && colcon build --packages-select irb1100_vision && source install/setup.bash
    ros2 launch irb1100_vision display.launch.py      # RViz2 only: robot, workcell, wrist camera view cone, sliders
    ros2 launch irb1100_vision gazebo.launch.py       # Gazebo + wrist camera images in RViz2

Camera topics (Gazebo): /cam3/left/image_raw, /cam3/right/image_raw, /cam3/left/camera_info, /cam3/right/camera_info
Depth: /cam3/depth/image_raw, /cam3/points. Detections: /cam3/detections. Gripper: /gripper/command (open/close).
Frames: cam3_link, cam3_left_optical_frame, cam3_right_optical_frame. Move the arm: publish to /arm_controller/joint_trajectory.
Tools: make_dataset.py (YOLO dataset), handeye_calib.py (hand-eye calibration), box_detector.py.

## Layout (world frame = floor, metres)
| item | position |
|---|---|
| table top | 1.1 x 1.1 m square, 25 mm thick (from you), top surface z = 0.75 (assumed) |
| pallet (blocks removed) | centre (-0.05, 0) |
| robot base | (0.20, 0.00, 0.75) |
| OmniCore E10 (under the top) | centre (0.20, 0.00), 449 x 338 x 89 mm per ABB datasheet; position and orientation assumed |
| wrist camera (cam3) | ZED Mini on the printed wrist mount, fixed to the gripper (moves with the arm), 25 deg tilt towards the jaws, 1280x720, hfov 90 deg, baseline 63 mm |
Edit these in `urdf/workcell.urdf.xacro`.

## 3D-printed parts
The wrist mount is designed in `zed_wrist/` (repo root); `zed_wrist/export_sim_meshes.py` writes `meshes/zed_wrist_mount.stl` and `meshes/zimmer_lwr50l.stl`.

## What is NOT exact (read before trusting numbers)
* Table height (0.75 m), E10 position and the pallet position are assumptions.
* Not yet run on a real ROS 2 / Gazebo install (checked offline: xacro expands, URDF parses, kinematics checked).
* See the arm and gripper sections below for what comes from CAD / datasheet and what is estimated.

## Arm model (ABB CAD)
Visuals, joint axes and origins come from ABB's official CAD (IRB 1100-4/0.58, STD base). Zero pose: tool0 at (0.364, 0, 0.617) from axis 1.
NOT from the CAD: link masses/inertia (21.1 kg spread by volume), positive rotation directions (ABB convention, check on the real robot), torque limits.
Collision uses convex hulls. Preview renders: scripts/render_preview_views.py (software painter's-algorithm render, not Gazebo).

## Gripper (Zimmer LWR50L-23-00006-A)
Looks: the real Zimmer CAD (`meshes/zimmer_lwr50l.stl`, made from the STEP in `zed_wrist/` by `zed_wrist/export_sim_meshes.py`). Collision and mass: simple shapes from the datasheet (plate dia 75, body 53.8 x 22.3, jaws 10 x 10.5 x 21.5, gap 32.8 open, 0.725 kg).
Cam 3 = ZED Mini on the 3D-printed wrist mount (`urdf/zed_wrist.xacro`): mount, camera and view cone in RViz2, frames `cam3_link`, `cam3_left/right_optical_frame`; Gazebo stereo sensors, left eye also gives depth.
Jaws are prismatic joints (0 to 10 mm each, `/gripper/command` open/close), no fingers, LWR50F head not modelled (a 3D-printed adapter will replace it). Jaws open along tool Y (left-right at zero pose).
TCP = jaw end face (0.1275 m from the mating face); add the finger length for the fingertip.
