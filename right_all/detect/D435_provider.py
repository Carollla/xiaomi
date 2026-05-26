#!/usr/bin/env python3
"""
D435深度相机数据提供模块
从ROS话题获取深度数据并提供给检测算法
"""
import threading
import time
import numpy as np
import os
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class D435Provider:
    """
    从ROS话题获取D435相机的深度数据并提供给检测算法
    """
    def __init__(self):
        self.node = rclpy.create_node('d435_provider')
        self.bridge = CvBridge()
        self.latest_depth_image = None
        self.depth_lock = threading.Lock()
        self.topic_name = os.environ.get("D435_TOPIC", "/D435_camera/depth/image_raw")
        self.sub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            depth=1
        )
        self.depth_sub = self.node.create_subscription(
            Image,
            self.topic_name,
            self.depth_callback,
            self.sub_qos
        )
        self.node.get_logger().info(f"等待D435深度相机数据: {self.topic_name}")
    
    def depth_callback(self, msg):
        """深度图像回调函数"""
        try:
            # 转换图像格式 (32FC1 - 浮点型深度值，单位通常为米)
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            with self.depth_lock:
                self.latest_depth_image = depth_image
        except Exception as e:
            self.node.get_logger().error(f"深度图像处理失败: {str(e)}")
    
    def get_latest_depth(self):
        """获取最新深度图像"""
        with self.depth_lock:
            if self.latest_depth_image is not None:
                return self.latest_depth_image.copy()
            return None
    
    def shutdown(self):
        """关闭资源"""
        self.node.destroy_node()
