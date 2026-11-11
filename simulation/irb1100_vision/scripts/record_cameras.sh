#!/bin/bash
# Record all three simulated camera streams (+ camera_info, TF, joint states) to a rosbag2 folder.
# Usage: ./record_cameras.sh [output_name]     Stop with Ctrl+C.
source /opt/ros/jazzy/setup.bash
OUT=${1:-cameras_$(date +%Y%m%d_%H%M%S)}
ros2 bag record -o "$OUT" \
  /cam1/image_raw /cam1/camera_info \
  /cam2/left/image_raw /cam2/left/camera_info \
  /cam2/right/image_raw /cam2/right/camera_info \
  /tf /tf_static /joint_states
