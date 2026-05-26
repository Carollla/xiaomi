#!/usr/bin/env python3
import math
import re
import subprocess
import time
from dataclasses import dataclass
from typing import Optional


@dataclass
class GazeboPose:
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float


class GazeboPoseProvider:
    def __init__(self, min_interval=0.5):
        self.min_interval = min_interval
        self.last_query_at = 0.0
        self.last_pose: Optional[GazeboPose] = None
        self.last_success_at = 0.0
        self.max_stale_sec = 6.0
        self.world_names = ("earth", "race", "default")

    def get_pose(self) -> Optional[GazeboPose]:
        now = time.time()
        if self.last_pose is not None and now - self.last_query_at < self.min_interval:
            return self.last_pose
        self.last_query_at = now
        try:
            out = ""
            for world in self.world_names:
                try:
                    out = subprocess.check_output(
                        ["gz", "model", "-w", world, "-m", "robot", "-p"],
                        stderr=subprocess.DEVNULL,
                        text=True,
                        timeout=4.5,
                    ).strip()
                    if out:
                        break
                except Exception:
                    continue
            vals = [float(v) for v in re.split(r"\s+", out)[:6]]
            if len(vals) != 6 or not all(math.isfinite(v) for v in vals):
                return self.last_pose if now - self.last_success_at <= self.max_stale_sec else None
            self.last_pose = GazeboPose(*vals)
            self.last_success_at = now
        except Exception:
            return self.last_pose if now - self.last_success_at <= self.max_stale_sec else None
        return self.last_pose
