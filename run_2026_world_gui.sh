#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/galactic/setup.bash
source /home/cyberdog_sim/install/setup.bash

cd /workspace/xiaomi_cup
./patch_2026_camera_sensors.sh

cd /home/cyberdog_sim
ros2 launch cyberdog_gazebo race_gazebo.launch.py \
  wname:=race \
  headless:=False \
  gui:=True \
  paused:=False
