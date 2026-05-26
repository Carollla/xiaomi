#!/usr/bin/env python3
import math
import re
import subprocess


def quat_to_yaw(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def main():
    out = subprocess.check_output(
        ["gz", "model", "-m", "robot", "-p"],
        stderr=subprocess.DEVNULL,
        text=True,
        timeout=1.0,
    ).strip()
    if not out:
        print("robot_pose unavailable")
        return
    vals = [float(v) for v in re.split(r"\s+", out)[:6]]
    print("robot_pose x={:.3f} y={:.3f} z={:.3f} roll={:.3f} pitch={:.3f} yaw={:.3f}".format(*vals))


if __name__ == "__main__":
    main()
