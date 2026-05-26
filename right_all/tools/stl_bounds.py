#!/usr/bin/env python3
import glob
import os
import re
import struct
import sys

import numpy as np


def read_stl_vertices(path):
    data = open(path, "rb").read()
    if len(data) >= 84:
        tri_count = struct.unpack("<I", data[80:84])[0]
        if 84 + tri_count * 50 == len(data):
            points = []
            offset = 84
            for _ in range(tri_count):
                vals = struct.unpack("<12fH", data[offset:offset + 50])[:12]
                points.extend((vals[3:6], vals[6:9], vals[9:12]))
                offset += 50
            return np.asarray(points, dtype=float)

    text = data.decode("utf-8", "ignore")
    points = [
        tuple(map(float, match))
        for match in re.findall(
            r"vertex\s+([-+eE0-9.]+)\s+([-+eE0-9.]+)\s+([-+eE0-9.]+)",
            text,
        )
    ]
    return np.asarray(points, dtype=float)


def main():
    base = sys.argv[1] if len(sys.argv) > 1 else "."
    for path in sorted(glob.glob(os.path.join(base, "*.stl"))):
        points = read_stl_vertices(path) * 0.01
        if points.size == 0:
            continue
        print(
            f"{os.path.basename(path):18s} "
            f"min={points.min(axis=0)} max={points.max(axis=0)} mean={points.mean(axis=0)}"
        )


if __name__ == "__main__":
    main()
