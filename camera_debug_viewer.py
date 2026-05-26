#!/usr/bin/env python3
import argparse
import os

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image


class CameraDebugViewer(Node):
    def __init__(self, rgb_topic: str, depth_topic: str, show_depth: bool):
        super().__init__("camera_debug_viewer")
        self.bridge = CvBridge()
        self.latest_rgb = None
        self.latest_depth = None
        self.show_depth = show_depth

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            depth=1,
        )

        self.create_subscription(Image, rgb_topic, self.rgb_callback, qos)
        if show_depth:
            self.create_subscription(Image, depth_topic, self.depth_callback, qos)

        self.qr_detector = self._build_qr_detector()
        self.get_logger().info(f"subscribed rgb topic: {rgb_topic}")
        if show_depth:
            self.get_logger().info(f"subscribed depth topic: {depth_topic}")

    def _build_qr_detector(self):
        model_dir = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            "right_all",
            "detect",
            "QR_row",
        )
        if hasattr(cv2, "wechat_qrcode") and os.path.isdir(model_dir):
            try:
                return cv2.wechat_qrcode.WeChatQRCode(
                    detector_caffe_model_path=os.path.join(model_dir, "detect.caffemodel"),
                    detector_prototxt_path=os.path.join(model_dir, "detect.prototxt"),
                    super_resolution_caffe_model_path=os.path.join(model_dir, "sr.caffemodel"),
                    super_resolution_prototxt_path=os.path.join(model_dir, "sr.prototxt"),
                )
            except Exception as exc:
                self.get_logger().warn(f"WeChatQRCode init failed, fallback to QRCodeDetector: {exc}")
        return cv2.QRCodeDetector()

    def rgb_callback(self, msg: Image):
        try:
            self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as exc:
            self.get_logger().error(f"rgb conversion failed: {exc}")

    def depth_callback(self, msg: Image):
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as exc:
            self.get_logger().error(f"depth conversion failed: {exc}")

    def _yellow_mask(self, image: np.ndarray) -> np.ndarray:
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower = np.array([20, 80, 80], dtype=np.uint8)
        upper = np.array([40, 255, 255], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower, upper)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return mask

    def _largest_component_center(self, mask: np.ndarray):
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)
        if num_labels <= 1:
            return None
        best_label = None
        best_area = 0
        for label in range(1, num_labels):
            area = stats[label, cv2.CC_STAT_AREA]
            if area > best_area:
                best_area = area
                best_label = label
        if best_label is None:
            return None
        cx, cy = centroids[best_label]
        return int(cx), int(cy), int(best_area)

    def _draw_qr(self, image: np.ndarray):
        text = None
        try:
            if hasattr(self.qr_detector, "detectAndDecode") and self.qr_detector.__class__.__name__ == "QRCodeDetector":
                text, points, _ = self.qr_detector.detectAndDecode(image)
                if points is not None and len(points) > 0:
                    pts = np.int32(points).reshape(-1, 2)
                    cv2.polylines(image, [pts], True, (0, 255, 0), 2)
            else:
                texts, points = self.qr_detector.detectAndDecode(image)
                if texts:
                    text = texts[0]
                if points is not None and len(points) > 0:
                    pts = np.int32(points[0]).reshape(-1, 2)
                    cv2.polylines(image, [pts], True, (0, 255, 0), 2)
        except Exception as exc:
            self.get_logger().warn(f"qr decode failed: {exc}")
        return text

    def render(self):
        if self.latest_rgb is None:
            return

        rgb = self.latest_rgb.copy()
        mask = self._yellow_mask(rgb)
        info = self._largest_component_center(mask)

        h, w = rgb.shape[:2]
        cv2.line(rgb, (w // 2, 0), (w // 2, h), (255, 0, 0), 1)
        if info is not None:
            cx, cy, area = info
            error = (cx - w // 2) / max(1, w // 2)
            cv2.circle(rgb, (cx, cy), 6, (0, 0, 255), -1)
            cv2.putText(rgb, f"track_error={error:.3f} area={area}", (20, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

        qr_text = self._draw_qr(rgb)
        if qr_text:
            cv2.putText(rgb, f"qr={qr_text}", (20, 65),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        stacked = np.hstack([rgb, mask_bgr])

        if self.show_depth and self.latest_depth is not None:
            depth = self.latest_depth.copy()
            finite = np.isfinite(depth)
            depth_vis = np.zeros_like(depth, dtype=np.uint8)
            if np.any(finite):
                valid = depth[finite]
                clipped = np.clip(depth, np.percentile(valid, 5), np.percentile(valid, 95))
                depth_vis = cv2.normalize(clipped, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
            depth_vis = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
            stacked = np.vstack([stacked, cv2.resize(depth_vis, (stacked.shape[1], h))])

        cv2.imshow("cyberdog_camera_debug", stacked)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--rgb-topic", default="/rgb_camera/image_raw")
    parser.add_argument("--depth-topic", default="/D435_camera/depth/image_raw")
    parser.add_argument("--show-depth", action="store_true")
    args = parser.parse_args()

    rclpy.init()
    node = CameraDebugViewer(args.rgb_topic, args.depth_topic, args.show_depth)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            node.render()
            if cv2.waitKey(1) & 0xFF == 27:
                break
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
