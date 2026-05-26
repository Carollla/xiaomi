#!/usr/bin/env python3
"""
Vision and depth helpers for the 2026 Xiaomi Cup "Wild Treasure Hunt" track.

The detector intentionally uses classical CV only.  It is fast enough for the
official ROS2/Gazebo image topics and does not require training data inside the
race container.
"""
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import cv2
import numpy as np


@dataclass
class Detection:
    found: bool = False
    cx: float = 0.0
    cy: float = 0.0
    area_ratio: float = 0.0
    bbox: Tuple[int, int, int, int] = (0, 0, 0, 0)
    confidence: float = 0.0
    distance: Optional[float] = None


class Race2026Detector:
    def __init__(self):
        self.kernel3 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        self.kernel5 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

    def _mask(self, image, lower, upper):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array(lower, dtype=np.uint8), np.array(upper, dtype=np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel3)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel5)
        return mask

    def _largest_blob(self, mask, min_area_ratio=0.0008, roundish=False) -> Detection:
        h, w = mask.shape
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best = None
        best_score = 0.0
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < min_area_ratio * h * w:
                continue
            x, y, bw, bh = cv2.boundingRect(cnt)
            fill = area / max(1.0, float(bw * bh))
            aspect = bw / max(1.0, float(bh))
            round_score = max(0.0, 1.0 - abs(1.0 - aspect)) if roundish else 1.0
            score = area * fill * round_score
            if score > best_score:
                best = (cnt, x, y, bw, bh, area, fill, round_score)
                best_score = score

        if best is None:
            return Detection()

        _, x, y, bw, bh, area, fill, round_score = best
        cx = (x + bw / 2.0) / w
        cy = (y + bh / 2.0) / h
        area_ratio = area / float(h * w)
        confidence = min(1.0, 0.35 + 25.0 * area_ratio + 0.25 * fill + 0.2 * round_score)
        return Detection(True, cx, cy, area_ratio, (x, y, bw, bh), confidence)

    def detect_orange_ball(self, image) -> Detection:
        if image is None:
            return Detection()
        mask1 = self._mask(image, (5, 80, 80), (24, 255, 255))
        mask2 = self._mask(image, (0, 80, 80), (8, 255, 255))
        mask = cv2.bitwise_or(mask1, mask2)
        return self._largest_blob(mask, min_area_ratio=0.00025, roundish=True)

    def detect_yellow_border(self, image) -> Detection:
        if image is None:
            return Detection()
        mask = self._mask(image, (20, 80, 90), (40, 255, 255))
        h, _ = mask.shape
        roi = mask[int(h * 0.35):, :]
        result = self._largest_blob(roi, min_area_ratio=0.001)
        if result.found:
            result.cy = 0.35 + result.cy * 0.65
        return result

    def detect_red_bar(self, image) -> Detection:
        if image is None:
            return Detection()
        mask1 = self._mask(image, (0, 90, 80), (8, 255, 255))
        mask2 = self._mask(image, (170, 90, 80), (179, 255, 255))
        mask = cv2.bitwise_or(mask1, mask2)
        h, _ = mask.shape
        roi = mask[: int(h * 0.55), :]
        result = self._largest_blob(roi, min_area_ratio=0.0008)
        if result.found:
            result.cy *= 0.55
        return result

    def detect_soccer(self, image) -> Detection:
        if image is None:
            return Detection()
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        white = cv2.inRange(hsv, np.array((0, 0, 120), dtype=np.uint8), np.array((179, 80, 255), dtype=np.uint8))
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        dark = cv2.inRange(gray, 0, 75)
        mask = cv2.morphologyEx(cv2.bitwise_or(white, dark), cv2.MORPH_OPEN, self.kernel3)
        result = self._largest_blob(mask, min_area_ratio=0.0009, roundish=True)
        if result.found and result.cy < 0.25:
            result.confidence *= 0.55
        return result

    def detect_coke_bottle(self, image) -> Detection:
        if image is None:
            return Detection()
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        red1 = cv2.inRange(hsv, np.array((0, 70, 50), dtype=np.uint8), np.array((10, 255, 255), dtype=np.uint8))
        red2 = cv2.inRange(hsv, np.array((170, 70, 50), dtype=np.uint8), np.array((179, 255, 255), dtype=np.uint8))
        dark = cv2.inRange(hsv, np.array((0, 0, 20), dtype=np.uint8), np.array((179, 120, 110), dtype=np.uint8))
        mask = cv2.morphologyEx(cv2.bitwise_or(cv2.bitwise_or(red1, red2), dark), cv2.MORPH_CLOSE, self.kernel5)
        result = self._largest_blob(mask, min_area_ratio=0.001)
        if result.found:
            _, _, bw, bh = result.bbox
            if bh < bw * 1.2:
                result.confidence *= 0.55
        return result

    def detect_targets(self, image) -> Dict[str, Detection]:
        return {
            "orange_ball": self.detect_orange_ball(image),
            "soccer": self.detect_soccer(image),
            "coke": self.detect_coke_bottle(image),
            "red_bar": self.detect_red_bar(image),
        }

    def attach_depth(self, detection: Detection, depth_image, window=13) -> Detection:
        if not detection.found or depth_image is None:
            return detection
        h, w = depth_image.shape[:2]
        x = int(max(0, min(w - 1, detection.cx * w)))
        y = int(max(0, min(h - 1, detection.cy * h)))
        half = max(2, window // 2)
        roi = depth_image[max(0, y - half): min(h, y + half + 1), max(0, x - half): min(w, x + half + 1)]
        valid = roi[np.isfinite(roi) & (roi > 0.05) & (roi < 6.0)]
        if valid.size:
            detection.distance = float(np.median(valid))
        return detection

    def obstacle_depth(self, depth_image, near_threshold=0.75):
        if depth_image is None:
            return False, 0.0, 0.0
        h, w = depth_image.shape[:2]
        roi = depth_image[int(h * 0.35): int(h * 0.70), int(w * 0.25): int(w * 0.75)]
        valid = roi[np.isfinite(roi) & (roi > 0.05) & (roi < 4.0)]
        if valid.size < 40:
            return False, 0.0, 0.0
        near_ratio = float(np.mean(valid < near_threshold))
        median = float(np.median(valid))
        return near_ratio > 0.22, near_ratio, median
