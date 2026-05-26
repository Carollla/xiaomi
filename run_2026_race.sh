#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ -f /opt/ros/galactic/setup.bash ]; then
  source /opt/ros/galactic/setup.bash
fi

if [ -f /home/cyberdog_sim/install/setup.bash ]; then
  source /home/cyberdog_sim/install/setup.bash
fi

export PYTHONPATH="/usr/local/lib/python3.8/site-packages:${PYTHONPATH:-}"

set -u

cd "$SCRIPT_DIR/right_all"
python3 main_2026.py
