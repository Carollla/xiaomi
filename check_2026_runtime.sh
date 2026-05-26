#!/usr/bin/env bash
set -eo pipefail

if [ -f /opt/ros/galactic/setup.bash ]; then
  source /opt/ros/galactic/setup.bash
fi

if [ -f /home/cyberdog_sim/install/setup.bash ]; then
  source /home/cyberdog_sim/install/setup.bash
fi

export PYTHONPATH="/usr/local/lib/python3.8/site-packages:${PYTHONPATH:-}"

echo "ROS topics:"
ros2 topic list -t | sort | grep -E 'camera|image|D435|rgb|imu|robot|motion|voice' || true

echo
echo "IMU sample:"
timeout 5 ros2 topic echo /imu --qos-reliability best_effort 2>/dev/null | head -40 || true

echo
echo "Configured sensor topics:"
echo "RGB_TOPIC=${RGB_TOPIC:-/rgb_camera/image_raw}"
echo "D435_TOPIC=${D435_TOPIC:-/D435_camera/depth/image_raw}"

cd "$(dirname "${BASH_SOURCE[0]}")/right_all"
CAPTURE_TIMEOUT="${CAPTURE_TIMEOUT:-10}" python3 tools/capture_sensor_frames.py || true
