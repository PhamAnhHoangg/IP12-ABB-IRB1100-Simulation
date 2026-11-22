# irb1100_vision - ABB IRB 1100 + Zimmer LWR50L + 2 cameras (RViz2 and Gazebo)

Target: ROS 2 Jazzy + Gazebo Harmonic (`gz sim`). Other distros: package names are the same, but
`gz_ros2_control` / `ros_gz_*` versions must match your Gazebo.

## Build and run
    cd ~/ros2_ws/src && unzip irb1100_vision.zip
    cd ~/ros2_ws && colcon build --packages-select irb1100_vision && source install/setup.bash
    ros2 launch irb1100_vision display.launch.py      # RViz2 only: robot, workcell, view cones, sliders
    ros2 launch irb1100_vision gazebo.launch.py       # Gazebo + camera images in RViz2

Camera topics (Gazebo): /cam1/image_raw, /cam1/camera_info, /cam2/left/image_raw, /cam2/right/image_raw, /cam2/left/camera_info, /cam2/right/camera_info
Offline preview of the ZED view: python3 scripts/render_cam_preview.py (needs yourdfpy, trimesh, pillow; edit paths inside). Frames: cam1_optical_frame, cam2_left_optical_frame, cam2_right_optical_frame. Move the arm: publish to /arm_controller/joint_trajectory.

## Layout (world frame = floor, metres)
One stand, clamped to the table's left edge (x = -0.55), carries both cameras.
| item | position |
|---|---|
| table top | 1.1 x 1.1 m square, 25 mm thick (from you), top surface z = 0.75 (assumed) |
| pallet (blocks removed) | centre (-0.05, 0) |
| robot base | (0.20, 0.00, 0.75) |
| OmniCore E10 (under the top) | centre (0.20, 0.00), 449 x 338 x 89 mm per ABB datasheet; position and orientation assumed |
| stand post | 25 mm square tube, axis at x = -0.569, y = 0 |
| Cam 1 side view | (-0.528, 0, 1.208): on the post, level (pitch 0), looks along +X, 1280x720, hfov 1.2 rad |
| Cam 2 top-down | (-0.05, 0, 1.57): 0.8 m above the pallet top, on a boom from the post top, looking straight down |
Edit these in `urdf/workcell.urdf.xacro`.

## 3D-printed parts (meshes/, units mm)
* `table_clamp.obj`: table-edge clamp with a socket for the 25 mm post. Upper jaw sits on the table,
  M8 screw (>= 40 mm) + nut in the lower jaw pushes up on the underside, M4 set screw locks the post.
  Fits a 25 mm thick table top (`--table`).
* `camera_mount_tilt0.obj`: clamps on the post, camera looks horizontally (Cam 1). `camera_mount_tilt35.*` is the old 35 deg version.
* `camera_mount_tilt90.obj`: clamps on the short stub at the boom end, camera looks straight down (Cam 2).
Camera plate has a 6.6 mm hole for a 1/4"-20 tripod screw. Regenerate with `cad/camera_mount.py` and
`cad/table_clamp.py` (pip install build123d trimesh). Print the clamp on its side (lying on a +Y face).
The post and boom are plain 25x25 aluminium tube (3 pieces; boom connects to the post top with an
off-the-shelf tube corner connector).

## What is NOT exact (read before trusting numbers)
* Table height (0.75 m), E10 position, cam1 model and the pallet position are assumptions.
* Not yet run on a real ROS 2 / Gazebo install (checked offline: xacro expands, URDF parses, kinematics checked).
* See the arm and gripper sections below for what comes from CAD / datasheet and what is estimated.

## Arm model (ABB CAD)
Visuals, joint axes and origins come from ABB's official CAD (IRB 1100-4/0.58, STD base). Zero pose: tool0 at (0.364, 0, 0.617) from axis 1.
NOT from the CAD: link masses/inertia (21.1 kg spread by volume), positive rotation directions (ABB convention, check on the real robot), torque limits.
Collision uses convex hulls. Preview renders: scripts/render_preview_views.py (software painter's-algorithm render, not Gazebo).

## Gripper (Zimmer LWR50L-23-00006-A)
Looks: the real Zimmer CAD (`meshes/zimmer_lwr50l.stl`, made from the STEP in `zed_wrist/` by `zed_wrist/export_sim_meshes.py`). Collision and mass: simple shapes from the datasheet (plate dia 75, body 53.8 x 22.3, jaws 10 x 10.5 x 21.5, gap 32.8 open, 0.725 kg).
Cam 3 = ZED Mini on the 3D-printed wrist mount (`urdf/zed_wrist.xacro`): mount, camera and view cone in RViz2, frames `cam3_link`, `cam3_left/right_optical_frame`; no Gazebo sensor yet.
Jaws fixed open, no fingers, LWR50F head not modelled (a 3D-printed adapter will replace it). Jaws open along tool Y (left-right at zero pose).
TCP = jaw end face (0.1275 m from the mating face); add the finger length for the fingertip.
