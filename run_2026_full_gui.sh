#!/usr/bin/env bash
set -eo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

pkill -f 'gzserver|gzclient|ros2 launch cyberdog_gazebo|main_2026.py|cyberdog_control' 2>/dev/null || true
sleep 1

cd "$REPO_DIR"
./run_2026_world_gui.sh > /tmp/race2026_world_gui.log 2>&1 &
sleep "${WORLD_START_DELAY:-16}"

source /opt/ros/galactic/setup.bash
source /home/cyberdog_sim/install/setup.bash
cd /home/cyberdog_sim
ros2 launch cyberdog_gazebo cyberdog_control_launch.py > /tmp/race2026_control.log 2>&1 &
sleep "${CONTROL_START_DELAY:-12}"

cd "$REPO_DIR"
./run_2026_race.sh
