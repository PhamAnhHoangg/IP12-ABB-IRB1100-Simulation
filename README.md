# IP12 ABB IRB 1100 simulation (RViz2 + Gazebo)

ROS 2 Jazzy workcell for the IP12 palletising project: ABB IRB 1100-4/0.58 with a Zimmer LWR50L gripper,
a 1.1 m x 1.1 m table, and two cameras (side camera and ZED Mini top-down) on one stand.

- Package: [`simulation/irb1100_vision`](simulation/irb1100_vision)
- Document: [`docs/SIMULATION.md`](docs/SIMULATION.md) (PDF copy: [`docs/SIMULATION.pdf`](docs/SIMULATION.pdf))

Quick start (WSL Ubuntu 24.04, ROS 2 Jazzy):

```bash
mkdir -p ~/ros2_ws/src && cp -r simulation/irb1100_vision ~/ros2_ws/src/
cd ~/ros2_ws && source /opt/ros/jazzy/setup.bash
colcon build --packages-select irb1100_vision && source install/setup.bash
ros2 launch irb1100_vision display.launch.py     # RViz2 only
ros2 launch irb1100_vision gazebo.launch.py      # Gazebo + RViz2 with camera images
```

Status: runs in RViz2 and Gazebo on the author's PC. Table height, cam 1 model and gripper shape are not final (see the document).
