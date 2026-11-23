# IP12 workcell in a container: runs on Windows, macOS (Intel + Apple Silicon) and Linux.
# The desktop (RViz2, Gazebo) is shown in your web browser at http://localhost:6080/vnc.html
# Builds and runs on Windows (Docker Desktop); checked in CI by docker-build.yml (build + RViz2/Gazebo launch test).
FROM osrf/ros:jazzy-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-ros-gz ros-jazzy-gz-ros2-control \
      ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
      ros-jazzy-xacro ros-jazzy-robot-state-publisher \
      ros-jazzy-joint-state-publisher-gui ros-jazzy-rosbag2 \
      python3-colcon-common-extensions python3-tk \
      xvfb x11vnc fluxbox novnc websockify \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /root/ros2_ws
COPY simulation/irb1100_vision src/irb1100_vision
RUN . /opt/ros/jazzy/setup.sh && colcon build --packages-select irb1100_vision
# so a shell opened with "docker compose exec ip12 bash" can start RViz2/Gazebo straight away
ENV DISPLAY=:1 LIBGL_ALWAYS_SOFTWARE=1
RUN echo "source /opt/ros/jazzy/setup.bash && source /root/ros2_ws/install/setup.bash" >> /root/.bashrc

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
EXPOSE 6080
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
