#!/usr/bin/env bash
set -e

docker start cyberdog2025 >/dev/null 2>&1 || true

docker exec cyberdog2025 bash -lc "pkill -f 'python3 main.py|gzclient|gzserver|rviz2|launchsim.py' || true"

docker exec -d -e DISPLAY="$DISPLAY" cyberdog2025 bash -lc "
  source /opt/ros/galactic/setup.bash >/dev/null 2>&1
  source /home/cyberdog_sim/install/setup.bash >/dev/null 2>&1
  gzserver /home/cyberdog_sim/install/share/cyberdog_gazebo/world/race.world \
    -s libgazebo_ros_init.so \
    -s libgazebo_ros_factory.so \
    -s libgazebo_ros_force_system.so \
    > /tmp/gzserver_demo.log 2>&1
"

sleep 3

docker exec -d -e DISPLAY="$DISPLAY" cyberdog2025 bash -lc "gzclient > /tmp/gzclient_demo.log 2>&1"

sleep 3

docker exec cyberdog2025 bash -lc "
  source /opt/ros/galactic/setup.bash >/dev/null 2>&1
  source /home/cyberdog_sim/install/setup.bash >/dev/null 2>&1
  ros2 service call /reset_world std_srvs/srv/Empty '{}' >/dev/null 2>&1 || true
  ros2 service call /reset_simulation std_srvs/srv/Empty '{}' >/dev/null 2>&1 || true
"

docker exec -d cyberdog2025 bash -lc "
  cd /workspace/cyberdog_nudt/right_all
  source /opt/ros/galactic/setup.bash >/dev/null 2>&1
  source /home/cyberdog_sim/install/setup.bash >/dev/null 2>&1
  python3 main.py > /tmp/cyberdog_demo.log 2>&1
"

echo "DISPLAY=$DISPLAY"
echo "2025赛道和机器狗程序已启动。"
echo "如果Gazebo窗口已打开但没看到机器狗：按 Home -> Scene -> 选中 cyberdog -> 按 F。"
