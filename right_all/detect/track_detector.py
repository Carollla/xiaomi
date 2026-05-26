#!/usr/bin/env python3
"""
Robust visual track detector for the 2025 Xiaomi Cup race.

This module upgrades the old "yellow exists / yellow missing" logic to:
1. HSV segmentation
2. Morphology cleanup
3. 8-neighborhood connected component filtering
4. Multi-row centerline extraction
5. Track error / heading / confidence output
"""
from collections import deque
from dataclasses import dataclass
from typing import List, Optional, Tuple

import cv2
import numpy as np


@dataclass
class TrackResult:
    found: bool
    error: float = 0.0
    heading_deg: float = 0.0
    confidence: float = 0.0
    center_points: Optional[List[Tuple[int, int]]] = None
    component_area: int = 0
    debug_image: Optional[np.ndarray] = None
    mask: Optional[np.ndarray] = None


class TrackDetector:
    """
    Extracts a stable centerline from yellow track boundaries / road region.
    The implementation is intentionally conservative and tuned for simulator robustness.
    """

    def __init__(self):
        self.lower_yellow = np.array([20, 80, 80], dtype=np.uint8)
        self.upper_yellow = np.array([40, 255, 255], dtype=np.uint8)
        self.kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        self.history = deque(maxlen=5)

    def get_yellow_mask(self, image: np.ndarray) -> np.ndarray:
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)
        return mask

    def _pick_main_component(self, mask: np.ndarray) -> Tuple[Optional[np.ndarray], int]:
        h, w = mask.shape
        num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
        best_label = None
        best_score = -1.0

        # Prefer large components that also reach the lower half of the image.
        for label in range(1, num_labels):
            area = int(stats[label, cv2.CC_STAT_AREA])
            left = int(stats[label, cv2.CC_STAT_LEFT])
            top = int(stats[label, cv2.CC_STAT_TOP])
            width = int(stats[label, cv2.CC_STAT_WIDTH])
            height = int(stats[label, cv2.CC_STAT_HEIGHT])
            bottom = top + height

            if area < 300:
                continue

            center_x = left + width / 2.0
            center_weight = 1.0 - min(abs(center_x - w / 2.0) / (w / 2.0), 1.0)
            bottom_weight = 1.0 if bottom >= int(h * 0.75) else 0.3
            score = area * (0.6 + 0.4 * center_weight) * bottom_weight

            if score > best_score:
                best_score = score
                best_label = label

        if best_label is None:
            return None, 0

        component = np.where(labels == best_label, 255, 0).astype(np.uint8)
        area = int(stats[best_label, cv2.CC_STAT_AREA])
        return component, area

    def _extract_centerline(self, component: np.ndarray) -> List[Tuple[int, int]]:
        h, w = component.shape
        centers: List[Tuple[int, int]] = []

        # Bottom-up row sampling gives better control relevance than contour center.
        for y in range(h - 10, int(h * 0.45), -12):
            row = component[y]
            xs = np.where(row > 0)[0]
            if xs.size < 6:
                continue

            # Split by largest contiguous run to avoid side noise.
            runs = []
            start = xs[0]
            prev = xs[0]
            for x in xs[1:]:
                if x == prev + 1:
                    prev = x
                    continue
                runs.append((start, prev))
                start = x
                prev = x
            runs.append((start, prev))
            run = max(runs, key=lambda pair: pair[1] - pair[0])
            cx = (run[0] + run[1]) // 2
            centers.append((int(cx), int(y)))

        return centers

    def _fit_heading(self, centers: List[Tuple[int, int]]) -> float:
        if len(centers) < 2:
            return 0.0
        pts = np.array(centers, dtype=np.float32)
        vx, vy, _, _ = cv2.fitLine(pts, cv2.DIST_L2, 0, 0.01, 0.01)
        vx = float(vx)
        vy = float(vy)
        # image y axis points down; convert to intuitive heading sign
        return float(np.degrees(np.arctan2(vx, -vy)))

    def detect(self, image: np.ndarray) -> TrackResult:
        if image is None:
            return TrackResult(found=False)

        h, w = image.shape[:2]
        roi = image[int(h * 0.35):, :]
        mask = self.get_yellow_mask(roi)
        component, area = self._pick_main_component(mask)

        debug = roi.copy()
        if component is None:
            return TrackResult(found=False, debug_image=debug, mask=mask)

        centers = self._extract_centerline(component)
        if len(centers) < 3:
            return TrackResult(found=False, debug_image=debug, mask=component, component_area=area)

        bottom_center = centers[0][0]
        raw_error = (bottom_center - (w / 2.0)) / max(1.0, (w / 2.0))
        heading = self._fit_heading(centers)

        self.history.append(raw_error)
        filtered_error = float(np.mean(self.history))

        confidence = min(1.0, 0.35 + 0.35 * (len(centers) / 12.0) + 0.30 * min(area / 25000.0, 1.0))

        debug_component = cv2.cvtColor(component, cv2.COLOR_GRAY2BGR)
        overlay = cv2.addWeighted(debug, 0.7, debug_component, 0.3, 0)
        cv2.line(overlay, (w // 2, 0), (w // 2, overlay.shape[0]), (255, 0, 0), 1)

        for pt in centers:
            cv2.circle(overlay, pt, 4, (0, 0, 255), -1)

        cv2.putText(
            overlay,
            f"err={filtered_error:.3f} heading={heading:.1f} conf={confidence:.2f}",
            (20, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 255),
            2,
        )

        return TrackResult(
            found=True,
            error=filtered_error,
            heading_deg=heading,
            confidence=confidence,
            center_points=centers,
            component_area=area,
            debug_image=overlay,
            mask=component,
        )
