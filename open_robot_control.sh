#!/usr/bin/env bash
set -e
docker start cyberdog2025 >/dev/null 2>&1 || true
docker exec -it cyberdog2025 bash -lc "cd /home/cyberdog_sim && source /opt/ros/galactic/setup.bash && source install/setup.bash && ros2 launch cyberdog_gazebo cyberdog_control_launch.py"
