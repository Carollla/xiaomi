#!/usr/bin/env bash
set -e
docker start cyberdog2025 >/dev/null 2>&1 || true
docker exec cyberdog2025 bash -lc "pkill -f 'python3 main.py|gzclient|gzserver|rviz2|launchsim.py|ros2 launch cyberdog_gazebo' || true"
docker exec -it -e DISPLAY="$DISPLAY" cyberdog2025 bash -lc "cd /home/cyberdog_sim && source /opt/ros/galactic/setup.bash && source install/setup.bash && ros2 launch cyberdog_gazebo race_gazebo.launch.py use_lidar:=true"
