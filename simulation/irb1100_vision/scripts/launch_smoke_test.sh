#!/bin/bash
# Starts RViz2 and then Gazebo on a virtual screen and checks they really work:
#   RViz2:  rviz2 + robot_state_publisher come up and stay up, joint states are published
#   Gazebo: robot spawns, controllers start, the arm moves to a commanded pose, Cam 1 sends a picture
# Used by CI inside the Docker image (.github/workflows/docker-build.yml). Locally:
#   docker compose -f docker/docker-compose.yml run --rm --entrypoint bash ip12 /root/ros2_ws/src/irb1100_vision/scripts/launch_smoke_test.sh
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1            # no GPU in CI: software OpenGL
Xvfb :99 -screen 0 1280x800x24 >/dev/null 2>&1 &
export DISPLAY=:99
sleep 2
share=$(ros2 pkg prefix irb1100_vision)/share/irb1100_vision
errors=0
check() { if eval "$2"; then echo "PASS  $1"; else echo "FAIL  $1"; errors=$((errors + 1)); fi; }
stop() { kill -INT "$1" 2>/dev/null; sleep 5; pkill -9 -f "gz sim|ruby|rviz2|robot_state_publisher|parameter_bridge|image_bridge" 2>/dev/null; wait "$1" 2>/dev/null; }

echo "=== RViz2 (display.launch.py) ==="
ros2 launch irb1100_vision display.launch.py > /tmp/rviz.log 2>&1 &
pid=$!
for _ in $(seq 60); do ros2 node list 2>/dev/null | grep -q '^/rviz2$' && break; sleep 1; done
sleep 15                                  # RViz2 must stay up after loading the config and meshes, not crash later
nodes=$(ros2 node list 2>/dev/null)
check "rviz2 node running" 'grep -q "^/rviz2$" <<< "$nodes"'
check "robot_state_publisher running" 'grep -q "^/robot_state_publisher$" <<< "$nodes"'
check "joint states published" 'timeout 15 ros2 topic echo --once /joint_states > /dev/null 2>&1'
check "launch still alive" 'kill -0 $pid 2>/dev/null'
stop $pid

echo "=== Gazebo (gazebo.launch.py, server only, light cameras) ==="
ros2 launch irb1100_vision gazebo.launch.py gz_args:="-r -s --headless-rendering $share/worlds/workcell.sdf" \
  panel:=false lite:=true stereo:=false rviz_cfg:=vision_noimg.rviz > /tmp/gz.log 2>&1 &
pid=$!
for _ in $(seq 180); do timeout 5 ros2 topic echo --once /joint_states > /dev/null 2>&1 && break; sleep 1; done
check "robot spawned, joint_state_broadcaster publishing" 'timeout 10 ros2 topic echo --once /joint_states > /dev/null 2>&1'

# same command as in the README; sent again every 15 s until arm_controller is active and the arm gets there
reached() {
  timeout 10 ros2 topic echo --once /joint_states 2>/dev/null | python3 -c '
import sys, yaml
m = yaml.safe_load(sys.stdin.read().split("---")[0])
q = dict(zip(m["name"], m["position"]))
sys.exit(0 if abs(q["joint_2"] - 0.4) < 0.05 and abs(q["joint_5"] - 0.6) < 0.05 else 1)'
}
moved=1
for _ in $(seq 16); do
  ros2 topic pub --once /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
    "{joint_names: [joint_1,joint_2,joint_3,joint_4,joint_5,joint_6], points: [{positions: [0.0,0.4,0.3,0.0,0.6,0.0], time_from_start: {sec: 4}}]}" > /dev/null 2>&1
  for _ in $(seq 15); do reached && { moved=0; break; }; sleep 1; done
  [ $moved -eq 0 ] && break
done
check "arm_controller moves the arm to the commanded pose" '[ $moved -eq 0 ]'
check "Cam 1 sends a picture (640 x 360)" '[ "$(timeout 90 ros2 topic echo --once /cam1/image_raw --field width 2>/dev/null | head -1)" = "640" ]'
check "launch still alive" 'kill -0 $pid 2>/dev/null'
stop $pid

if [ $errors -gt 0 ]; then
  echo "--- rviz.log (end) ---"; tail -40 /tmp/rviz.log
  echo "--- gz.log (end) ---"; tail -60 /tmp/gz.log
  echo "$errors check(s) failed"; exit 1
fi
echo "all launch checks passed"
