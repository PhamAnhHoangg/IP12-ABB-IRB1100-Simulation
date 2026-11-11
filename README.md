# IP12 ABB IRB 1100 simulation (RViz2 + Gazebo)

[![Check robot model](../../actions/workflows/check-model.yml/badge.svg)](../../actions/workflows/check-model.yml)

ROS 2 Jazzy workcell for the **IP12 palletising project** (HSRW): an ABB IRB 1100 robot with a Zimmer gripper,
a square table and two cameras. It is used to plan the camera placement and to move the arm in simulation
before the real setup is built.

![scene](docs/img/sim_gripper_on_arm.png)

## What is inside

| Part | Source |
|---|---|
| ABB IRB 1100-4/0.58 (OmniCore) arm, 6 joints, ABB axis limits | ABB official CAD (STEP, rev04), converted to meshes |
| Zimmer LWR50L-23-00006-A gripper | Zimmer datasheet drawings (approximate shapes, not the exact CAD) |
| Table 1.1 m x 1.1 m, pallet | measured / assumed (see limits) |
| All-printed triangle truss camera stand (Cam 1 side, Cam 2 ZED Mini top-down) | designed in this project, see [docs/STAND.md](docs/STAND.md) |
| Launch files, controllers, camera bridges | this project |

```
simulation/irb1100_vision/   ROS 2 package (ament_cmake)
  urdf/        workcell, arm, gripper, cameras (xacro)
  launch/      display.launch.py (RViz2), gazebo.launch.py (Gazebo + RViz2)
  config/      ros2_control controllers, ROS<->Gazebo bridge
  meshes/      ABB arm meshes, camera mounts, printed stand parts (STL/STEP)
  cad/         scripts that generate the printable mounts and arms (build123d)
  scripts/     fov_markers.py (view cones), check_urdf.py (model check)
  worlds/      Gazebo world
docs/          SIMULATION.md / SIMULATION.pdf (full description)
```

## Requirements

- Ubuntu 24.04 (native or WSL2 on Windows 11 with WSLg for windows)
- ROS 2 **Jazzy** and Gazebo **Harmonic**
- A graphics driver that works in WSL/Linux. Gazebo runs at a low real-time factor on weak GPUs (about 8 % was seen on a laptop).
- Packages:

```bash
sudo apt install -y ros-jazzy-rviz2 ros-jazzy-xacro ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher-gui python3-colcon-common-extensions
# for Gazebo:
sudo apt install -y ros-jazzy-ros-gz ros-jazzy-gz-ros2-control \
  ros-jazzy-ros2-control ros-jazzy-ros2-controllers
```

## Install and run

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/PhamAnhHoangg/IP12-ABB-IRB1100-Simulation.git
cp -r IP12-ABB-IRB1100-Simulation/simulation/irb1100_vision .
cd ~/ros2_ws && source /opt/ros/jazzy/setup.bash
colcon build --packages-select irb1100_vision && source install/setup.bash

ros2 launch irb1100_vision display.launch.py     # RViz2 only: robot, table, view cones, joint sliders
ros2 launch irb1100_vision gazebo.launch.py      # Gazebo + RViz2 with the camera pictures
```

Move the arm in Gazebo (joint targets in radians, 4 s):

```bash
ros2 topic pub --once /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
 "{joint_names: [joint_1,joint_2,joint_3,joint_4,joint_5,joint_6], points: [{positions: [0.0,0.4,0.3,0.0,0.6,0.0], time_from_start: {sec: 4}}]}"
```

Camera topics: `/cam1/image_raw`, `/cam2/left/image_raw`, `/cam2/right/image_raw` (+ `camera_info`).

## Testing

| Check | What it covers | Where it runs |
|---|---|---|
| `python3 simulation/irb1100_vision/scripts/check_urdf.py` (needs only `pip install xacro`) | model expands, all meshes exist, six joints with ABB limits, tool/gripper/camera frames exist, no orphan links, Gazebo-needed camera joints kept | on every push via GitHub Actions ([`check-model.yml`](.github/workflows/check-model.yml)) |
| RViz2 launch | robot and cameras display, joint sliders move the arm | checked by hand on the author's PC (WSL2, Jazzy) |
| Gazebo launch | model spawns, controllers `joint_state_broadcaster` and `arm_controller` start, camera pictures appear | checked by hand on the author's PC; takes about a minute to start |

The GitHub check does **not** run ROS or Gazebo. It only guards the model files against breaking.

## Known limits

- Table top height (0.75 m), pallet size and position, base variant (STD vs BTM) are assumptions.
- Cam 1 is a 90 x 25 x 25 mm placeholder, camera model not chosen. Cam 1 is level, 1.208 m high.
- Gripper shape is approximate, jaws are fixed open, no fingers, no adapter plate (3D-printed part not modelled).
- Link masses, inertia and torque limits are estimates, so Gazebo dynamics are only roughly right.
- No inverse kinematics or MoveIt, no connection to the real OmniCore controller.
- OmniCore E10 controller box is not modelled.

More detail: [`docs/SIMULATION.md`](docs/SIMULATION.md) ([PDF](docs/SIMULATION.pdf)).

## Credits and sources

- **ABB**: IRB 1100 robot and CAD model, downloaded from ABB's website: *CAD models, STEP, IRB 1100, 4-580, OmniCore, rev04, comp*,
  product page <https://new.abb.com/products/robotics/nl/onze-robots/articulated-robots/irb-1100>.
  The arm meshes in this repo are derived from that CAD model. Robot data sheet: <https://library.e.abb.com/public/6810bdc2bc0744edb5f050a6494daaea/Datenblatt_IRB1100_lowres.pdf>.
  All rights to the ABB robot and its CAD model belong to ABB.
- **Zimmer Group**: datasheet for MATCH LWR50L-23-00006-A (dimensions, mass, jaw stroke).
- **Stereolabs**: ZED Mini datasheet (size, baseline, field of view).
- **Open Robotics / ROS 2 community**: ROS 2 Jazzy, RViz2, ros2_control, gz_ros2_control, ros_gz, Gazebo Harmonic.
- Python tools used for the mesh and mount work: build123d, trimesh, yourdfpy, numpy, Pillow.
- Project: HSRW IP12 palletising with AI perception. Repository owner: PhamAnhHoangg.
- Built with the help of Claude (Anthropic).

## License and third-party material

No license has been chosen yet. Third-party material belongs to its owners and is credited above: the ABB IRB 1100 CAD model
(source link above), the Zimmer LWR50L dimensions and the Stereolabs ZED Mini dimensions. The ABB-derived meshes are used here for the
HSRW IP12 university project, with ABB credited as the source.
