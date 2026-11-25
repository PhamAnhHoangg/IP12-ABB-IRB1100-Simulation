# WP1 ↔ WP2 interface

What WP1 (robot and automation) and WP2 (perception, simulation, data) send each other. As long as both sides
stick to this page, each group can test on its own: WP2 against Gazebo, WP1 against the real controller, then both
together on the cell. Change it only when both groups agree.

## Who owns what

| WP1: robot and automation | WP2: perception, simulation, data |
|---|---|
| Python control (pick-and-place, state machine, safety checks) | ZED Mini driver, depth and point cloud |
| Link to the OmniCore C30 (RWS 2.0 / RAPID), gripper, conveyor, E-stop | Box detection (colour / YOLO, point cloud), ArUco and hand-eye calibration |
| Measuring the ArUco marker place with the TCP | Simulation (Gazebo for tests and CI, Isaac Sim for training images) |
| Sending the robot pose to WP2 | The UDP bridge (`scripts/udp_bridge.py`) and this message format |

WP2 runs ROS 2 inside. WP1 does not need ROS 2: everything between the groups is plain UDP with JSON.

## WP2 → WP1: box positions

UDP to WP1's PC, port **5005**, one datagram per camera picture (a few per second):

```json
{"type": "boxes", "stamp": 1764100000.123, "frame": "base", "unit": "mm",
 "boxes": [{"class": "red",  "x": -249.7, "y": 122.2,  "z": 12.9, "yaw_deg": 63.4},
           {"class": "blue", "x": -249.7, "y": -121.0, "z": 12.9, "yaw_deg": 26.6}]}
```

| Field | Meaning |
|---|---|
| `stamp` | when the picture was taken, Unix time in seconds (in Gazebo: simulation seconds) |
| `frame`, `unit` | always the robot base frame (RobotWare base: floor plane under axis 1) in millimetres |
| `class` | box type from the detector (`red`, `blue`, `green` now; YOLO class names later) |
| `x`, `y`, `z` | centre of the box. The top face is `z + 12.5` (25 mm boxes) |
| `yaw_deg` | rotation of the box around z, 0 to 90 (a cube looks the same every 90°). `null` if the point cloud did not see the box well enough: do not grip it at an unknown angle |

An empty `boxes` list is not sent; no message means no box seen.

## WP1 → WP2: robot pose

UDP to the WP2 PC, port **5006**, as often as WP1 reads it, at least 20 times a second:

```json
{"type": "robot_pose", "stamp": 1764100000.120, "joints_deg": [180.0, 0.0, 34.4, 0.0, 42.9, 0.0]}
```

| Field | Meaning |
|---|---|
| `stamp` | when the angles were read from the controller, Unix time in seconds. `0` = WP2 uses the arrival time |
| `joints_deg` | axis 1 to 6 in degrees, exactly as the FlexPendant shows them |

Why WP2 needs it: the ZED sits on the robot's wrist. To turn a box seen in the picture into robot coordinates,
WP2 has to know where the arm was when the picture was taken. Old or missing poses give wrong box positions as soon
as the arm moves. The safest pictures are taken while the arm stands still.

Malformed messages are ignored and logged on the WP2 side.

## Clocks

Both stamps are Unix time. If WP1 and WP2 run on the same Linux PC this is automatic. On two PCs, both need NTP
(or `chrony`) so their clocks agree within a few milliseconds.

## One-time calibration (both groups)

1. WP2 prints `docs/print/aruco_sheet5_A4.pdf` (A4 landscape, 100 %, check the 100 mm bar) and glues it flat on a
   plate. It goes where the camera looks from the start pose, with nothing covering it; the gripper jaws hide the
   bottom of the picture, so put it a little further away than the pick spot.
2. WP1 touches the centre of marker 0 (the middle one) with the TCP and reads x, y, z (mm), and the direction of the
   "sheet x" arrow.
3. WP2 runs `aruco_locate.py` with those values (converted to `world`, metres) and checks the camera position
   against the CAD value. Check again from two or three other arm poses; the result should stay the same.

## Inside WP2 (ROS 2, for reference)

| Topic | Type | Content |
|---|---|---|
| `/cam3/detections` | `geometry_msgs/PoseArray` | box centres, frame `world` (metres); the bridge converts to base / mm |
| `/cam3/detections/markers` | `visualization_msgs/MarkerArray` | same boxes for RViz2; marker `text` = class |
| `/cam3/cloud_boxes` | `geometry_msgs/PoseArray` | box centres and yaw from the point cloud alone |
| `/joint_states` | `sensor_msgs/JointState` | from Gazebo in simulation, from `udp_bridge.py` (`pose_in:=true`) on the real cell |

`world` is the cell frame of the simulation; the robot base sits at `(0.20, 0, 0.75)` m in it with no rotation.

## Run and test

```bash
ros2 launch irb1100_vision gazebo.launch.py panel:=false                       # WP2 side in simulation
ros2 run irb1100_vision cloud_boxes.py                                          # adds the yaw
ros2 run irb1100_vision udp_bridge.py --ros-args -p wp1_host:=<WP1 PC IP>       # boxes to WP1
# real cell: add  -p pose_in:=true  so the robot pose from WP1 becomes /joint_states
```

WP1 can test without WP2 by listening on UDP 5005 while Gazebo runs, and WP2 can test the pose input by sending
the JSON above to port 5006 (see `test/test_udp_bridge.py`).

## Still open

- Robot motion over RWS 2.0 or EGM (only matters for how fast WP1 can send the pose).
- Does the conveyor move while the robot picks? If yes, WP2 also has to send the box speed.
