#!/usr/bin/env python3
import os
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor

RIGHT_ALL_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if RIGHT_ALL_DIR not in sys.path:
    sys.path.insert(0, RIGHT_ALL_DIR)

from detect.D435_provider import D435Provider
from detect.RGB_provider import RGBProvider


def main():
    out_dir = os.environ.get("CAPTURE_DIR", "/tmp/xiaomi_cup_frames")
    os.makedirs(out_dir, exist_ok=True)
    rclpy.init()
    rgb = RGBProvider()
    depth = D435Provider()
    nodes = [rgb.node, depth.node]
    executor = MultiThreadedExecutor(num_threads=2)
    for node in nodes:
        executor.add_node(node)
    deadline = time.time() + float(os.environ.get("CAPTURE_TIMEOUT", "20"))
    try:
        while rclpy.ok() and time.time() < deadline:
            executor.spin_once(timeout_sec=0.1)
            image = rgb.get_latest_image()
            depth_image = depth.get_latest_depth()
            if image is not None:
                cv2.imwrite(os.path.join(out_dir, "rgb_latest.jpg"), image)
            if depth_image is not None:
                valid = depth_image[np.isfinite(depth_image)]
                if valid.size:
                    print(
                        "depth stats:",
                        "shape=", depth_image.shape,
                        "min=", float(np.min(valid)),
                        "median=", float(np.median(valid)),
                        "max=", float(np.max(valid)),
                    )
                np.save(os.path.join(out_dir, "depth_latest.npy"), depth_image)
            if image is not None and depth_image is not None:
                print("captured", os.path.join(out_dir, "rgb_latest.jpg"))
                return 0
        print("timeout waiting for rgb/depth frames")
        return 1
    finally:
        for node in nodes:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
