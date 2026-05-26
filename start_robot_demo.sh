#!/usr/bin/env bash
set -e
docker start cyberdog2025 >/dev/null 2>&1 || true
docker exec cyberdog2025 bash -lc "source /opt/ros/galactic/setup.bash && source /home/cyberdog_sim/install/setup.bash && ros2 service call /reset_world std_srvs/srv/Empty '{}'" || true
docker exec cyberdog2025 bash -lc "source /opt/ros/galactic/setup.bash && source /home/cyberdog_sim/install/setup.bash && ros2 service call /reset_simulation std_srvs/srv/Empty '{}'" || true
docker exec -it cyberdog2025 bash -lc "cd /workspace/cyberdog_nudt/right_all && source /opt/ros/galactic/setup.bash && source /home/cyberdog_sim/install/setup.bash && python3 main.py"
