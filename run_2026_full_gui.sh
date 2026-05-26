#!/usr/bin/env bash
set -eo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

cat <<EOF
This project is intended to be demonstrated in foreground GUI mode.

Open three visible terminals and run these commands.

Terminal 1 - Gazebo GUI world:
  cd "$REPO_DIR"
  ./run_2026_world_gui.sh

Terminal 2 - CyberDog controller:
  cd /home/cyberdog_sim
  source /opt/ros/galactic/setup.bash
  source /home/cyberdog_sim/install/setup.bash
  ros2 launch cyberdog_gazebo cyberdog_control_launch.py

Terminal 3 - Race code:
  cd "$REPO_DIR"
  ./run_2026_race.sh

Do not run the GUI demo with nohup or background redirection if you want
judges or teammates to see the robot moving in Gazebo.
EOF
