#!/bin/bash
# Start a virtual screen + VNC + browser access, then run the given command.
export DISPLAY=:1
export LIBGL_ALWAYS_SOFTWARE=1      # no GPU in the container: software rendering (slow but works)
Xvfb :1 -screen 0 1600x900x24 &
sleep 1
fluxbox &
x11vnc -display :1 -forever -shared -nopw -quiet &
websockify --web /usr/share/novnc 6080 localhost:5900 &
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
echo "Ready. 1) Open http://localhost:6080/vnc.html in your browser and click Connect."
echo "       2) In a second terminal: docker compose -f docker/docker-compose.yml exec ip12 bash"
echo "       3) In that shell: ros2 launch irb1100_vision gazebo.launch.py lite:=true stereo:=false   (or display.launch.py)"
exec "$@"
