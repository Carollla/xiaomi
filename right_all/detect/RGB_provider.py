#!/usr/bin/env python3
"""
ROS图像提供者模块
从ROS话题获取图像并提供给检测算法
"""
import threading
from cv_bridge import CvBridge
import rclpy
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import os
import cv2
import numpy as np

try:
    from cv2.wechat_qrcode import WeChatQRCode
except Exception:
    WeChatQRCode = None

class RGBProvider:
    """
    从ROS话题获取图像并提供给检测算法
    """
    def __init__(self):
        self.node = rclpy.create_node('rgb_provider')    
        self.bridge = CvBridge()
        self.latest_image = None
        self.image_lock = threading.Lock()
        self.topic_name = os.environ.get("RGB_TOPIC", "/rgb_camera/image_raw")
        self.sub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            depth=1
        )
        self.image_sub = self.node.create_subscription(
            Image,
            self.topic_name,
            self.image_callback,
            self.sub_qos
        )
        # 初始化二维码检测器
        script_dir = os.path.dirname(os.path.abspath(__file__))
        model_dir = os.path.join(script_dir, "QR_row")
        if WeChatQRCode is not None:
            self.qrcode_detector = WeChatQRCode(
                detector_caffe_model_path=os.path.join(model_dir, "detect.caffemodel"),
                detector_prototxt_path=os.path.join(model_dir, "detect.prototxt"),
                super_resolution_caffe_model_path=os.path.join(model_dir, "sr.caffemodel"),
                super_resolution_prototxt_path=os.path.join(model_dir, "sr.prototxt")
            )
            self._fallback_qrcode_detector = None
        else:
            self.qrcode_detector = None
            self._fallback_qrcode_detector = cv2.QRCodeDetector()
            self.node.get_logger().warn("OpenCV WeChatQRCode不可用，二维码识别回退到QRCodeDetector")
        self.node.get_logger().info(f"等待图像数据: {self.topic_name}")
    
    def image_callback(self, msg):
        """图像回调函数"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            with self.image_lock:
                self.latest_image = cv_image
        except Exception as e:
            self.node.get_logger().error(f"图像处理失败: {str(e)}")
    
    def get_latest_image(self):
        """获取最新图像"""
        with self.image_lock:
            if self.latest_image is not None:
                return self.latest_image.copy()
            return None
        
    def detect_qrcode(self, img):
        """检测二维码"""
        if self.qrcode_detector is not None:
            res, points = self.qrcode_detector.detectAndDecode(img)
        else:
            res, points, _ = self._fallback_qrcode_detector.detectAndDecode(img)
        if res:
            if points is not None:
                points = np.array(points).astype(int)
                cv2.polylines(img, [points], True, (0, 255, 0), 3)
            return True, res
        return False, None
    
    def shutdown(self):
        """关闭资源"""
        self.node.destroy_node() 
