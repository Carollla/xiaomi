# Xiaomi Cup CyberDog 2026

This repository contains the modified CyberDog race code for the 2026 Xiaomi Cup simulation track.

## Main Entry

- State machine: `right_all/robot_state_machine_2026.py`
- Motion wrapper: `right_all/motion.py`
- Headless world: `run_2026_world_headless.sh`
- Race launcher: `run_2026_race.sh`

## Verified Result

The latest full simulation run reached the finish with a real Gazebo pose, not a timeout shortcut.

Key log lines:

```text
状态切换: SEG6_SOCCER -> SEG6_FINISH_CIRCLE
final_lane target_y=15.60 pose=(3.62,15.65) ...
状态切换: SEG6_FINISH_CIRCLE -> FINISH
```

The finish transition is gated by `pose.y > 15.55`.

## Run Order

Inside the Docker container:

```bash
cd /workspace/xiaomi_cup
./run_2026_world_headless.sh
```

In another shell:

```bash
cd /home/cyberdog_sim
source /opt/ros/galactic/setup.bash
source /home/cyberdog_sim/install/setup.bash
ros2 launch cyberdog_gazebo cyberdog_control_launch.py
```

Then start the race:

```bash
cd /workspace/xiaomi_cup
./run_2026_race.sh
```
