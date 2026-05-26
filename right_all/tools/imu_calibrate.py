#!/usr/bin/env python3
import os
import sys
import time

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(SCRIPT_DIR)
if ROOT_DIR not in sys.path:
    sys.path.insert(0, ROOT_DIR)

import rclpy
from rclpy.executors import SingleThreadedExecutor

from detect.IMU_provider import IMUProvider
from motion import MotionController


def sample_yaw(imu_provider, executor, seconds=2.0):
    deadline = time.time() + seconds
    values = []
    while time.time() < deadline:
        executor.spin_once(timeout_sec=0.05)
        yaw = imu_provider.get_yaw()
        values.append(yaw)
    if not values:
        return 0.0
    return sum(values) / len(values)


def main():
    turn_seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 2.0
    settle_seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0

    rclpy.init()
    imu_provider = IMUProvider()
    motion_controller = MotionController()
    executor = SingleThreadedExecutor()
    executor.add_node(imu_provider.node)
    executor.add_node(motion_controller)

    try:
        time.sleep(1.0)
        yaw0 = sample_yaw(imu_provider, executor, 2.0)
        print(f"YAW_START={yaw0:.3f}", flush=True)

        motion_controller.execute_right_turn()
        t_end = time.time() + turn_seconds
        while time.time() < t_end:
            executor.spin_once(timeout_sec=0.05)

        motion_controller.execute_stand()
        t_settle = time.time() + settle_seconds
        while time.time() < t_settle:
            executor.spin_once(timeout_sec=0.05)

        yaw1 = sample_yaw(imu_provider, executor, 2.0)
        print(f"YAW_AFTER_RIGHT={yaw1:.3f}", flush=True)

        delta = (yaw1 - yaw0 + 540.0) % 360.0 - 180.0
        print(f"DELTA_RIGHT={delta:.3f}", flush=True)
    finally:
        motion_controller.execute_stand()
        motion_controller.destroy_node()
        imu_provider.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
