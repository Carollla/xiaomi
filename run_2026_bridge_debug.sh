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
export RACE2026_DEBUG_STATE="${RACE2026_DEBUG_STATE:-SEG5_BRIDGE}"

gz model -m robot -x "${RACE2026_DEBUG_X:-0.0}" \
  -y "${RACE2026_DEBUG_Y:-11.72}" \
  -z "${RACE2026_DEBUG_Z:-0.32}" \
  -R 0 -P 0 -Y 1.57 || true

cd "$SCRIPT_DIR/right_all"
python3 main_2026.py
