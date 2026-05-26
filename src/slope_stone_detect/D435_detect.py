#!/usr/bin/env python3
"""
仅使用D435深度相机的深度数据检测斜坡并确保对准
"""
import numpy as np
import time
import cv2
from D435_provider import D435Provider

def detect_top_bar(depth_image, distance_threshold=1.0, top_height_ratio=0.05, max_valid_depth=5.0, mode = "yellow_light"):
    """
    只检测顶部区域是否有障碍物
    """
    if depth_image is None:
        print("depth_image is None")
        return False
    h, w = depth_image.shape
    top_h = int(h * top_height_ratio)
    top_bar = depth_image[:top_h, :]
    # 只保留0~max_valid_depth米的点
    valid_mask = (top_bar > 0) & (top_bar < max_valid_depth)
    top_depths = top_bar[valid_mask]
    if len(top_depths) == 0:
        return False
    top_avg = np.mean(top_depths)
    print(f"顶部平均距离: {top_avg:.3f}米")
    print(f"top_depths min: {np.min(top_depths)}, max: {np.max(top_depths)}")
    if mode == "yellow_light":
        return top_avg < distance_threshold
    elif mode == "limit_bar":
        return np.min(top_depths) < distance_threshold

def main():
    provider = D435Provider()
    print("等待深度相机数据... 按q退出")
    while True:
        depth_image = provider.get_latest_depth()
        if depth_image is None:
            continue
        # 只检测顶部
        has_top_bar = detect_top_bar(depth_image, distance_threshold=1.0, top_height_ratio=0.5, max_valid_depth=5.0)
        print(f"顶部有障碍物: {has_top_bar}")
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    provider.shutdown()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

