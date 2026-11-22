#!/bin/bash
# Record both simulated ZED Mini (wrist) streams (+ camera_info, TF, joint states) to a rosbag2 folder.
# Usage: ./record_cameras.sh [output_name]     Stop with Ctrl+C.
source /opt/ros/jazzy/setup.bash
OUT=${1:-cameras_$(date +%Y%m%d_%H%M%S)}
ros2 bag record -o "$OUT" \
  /cam3/left/image_raw /cam3/left/camera_info \
  /cam3/right/image_raw /cam3/right/camera_info \
  /tf /tf_static /joint_states
