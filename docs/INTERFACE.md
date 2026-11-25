# Interface between perception (WP2) and robot motion

This is what the perception side and the robot driver side agree on. As long as both stick to it, each side can be
tested on its own: perception against Gazebo, the driver against RobotStudio, and later both on the real cell.

All values are SI units: metres, seconds, radians. RobotWare works in degrees and millimetres, so the driver converts
at its own border and nowhere else.

## Topics

| Topic | Type | Published by | Read by | Content |
|---|---|---|---|---|
| `/cam3/detections` | `geometry_msgs/PoseArray` | perception (`box_detector.py`) | motion (pick logic) | one pose per box, centre of the box, frame `world`, metres. Orientation is not filled in (identity). |
| `/cam3/cloud_boxes` | `geometry_msgs/PoseArray` | perception (`cloud_boxes.py`) | motion, optional | box centres from the point cloud only, frame `world`, with the box yaw in the orientation (0 to 90 deg, a cube looks the same every 90 deg) |
| `/joint_states` | `sensor_msgs/JointState` | motion (robot driver) | perception (TF, through `robot_state_publisher`) | `joint_1` ... `joint_6`, radians, stamped with the time the angles were measured. At least 50 Hz. |

Perception needs `/joint_states` to know where the wrist camera is at the moment of each picture. Without it, or with
old timestamps, the box positions are wrong while the arm moves.

## Frames

| Frame | What it is |
|---|---|
| `world` | the cell frame. `z` = up, the table top is at `z = 0.75`. All box positions are given here. |
| `base_link` | the IRB 1100 base, at `(0.20, 0, 0.75)` in `world`, same orientation (no rotation). RobotWare's base frame is this frame in millimetres. |
| `gripper_link`, `gripper_tcp` | the Zimmer gripper on the flange and the point between the jaw tips |
| `cam3_left_optical_frame` | the left ZED eye: `z` forward out of the lens, `x` right, `y` down (camera convention) |

To turn a box position from `world` into RobotWare's base frame: subtract `(0.20, 0, 0.75)`, then multiply by 1000 for mm.
On the real cell the offset between `world` and `base_link` has to be measured once (or `world` is simply set equal to
the base frame in `urdf/workcell.urdf.xacro`).

## Box order and colours

`PoseArray` has no class names. `box_detector.py` publishes the boxes in the order it logs them (red, blue, green when
the colour detector runs). If the pick logic needs to know which box is which, the topic should move to
`vision_msgs/Detection3DArray` (class + score per box). That change has to be agreed by both sides first.

## Moving the arm from perception tools

`handeye_calib.py` and `make_dataset.py` move the arm by publishing `trajectory_msgs/JointTrajectory` on
`/arm_controller/joint_trajectory` (radians). In Gazebo that is the `arm_controller`. On the real cell the driver
either accepts the same topic, or the calibration poses are driven by hand and the script only takes the pictures.

## Test it without the lab

```bash
ros2 launch irb1100_vision gazebo.launch.py          # Gazebo publishes /joint_states, the detector /cam3/detections
ros2 topic echo /cam3/detections                     # what the driver side will receive
ros2 topic echo /joint_states --field position       # what perception expects back
```
