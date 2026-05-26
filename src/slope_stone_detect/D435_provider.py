#!/usr/bin/env python3
"""
D435深度相机数据提供模块
从ROS话题获取深度数据并提供给检测算法
"""
import threading
import time
import numpy as np
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
        # 创建ROS节点
        rclpy.init(args=None)
        self.node = rclpy.create_node('d435_provider')
        
        # 图像相关
        self.bridge = CvBridge()
        self.latest_depth_image = None
        self.depth_lock = threading.Lock()
        
        # 图像传输配置
        self.sub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            depth=1
        )
        
        # 订阅深度图像话题
        self.depth_sub = self.node.create_subscription(
            Image,
            '/D435_camera/depth/image_raw',
            self.depth_callback,
            self.sub_qos
        )
        
        # 创建ROS线程
        self.ros_thread = threading.Thread(target=self.ros_spin)
        self.ros_thread.daemon = True  # 后台运行
        self.running = True
        
        # 启动ROS线程
        self.ros_thread.start()
        
        # 等待首次数据
        self.node.get_logger().info("等待D435深度相机数据...")
    
    def depth_callback(self, msg):
        """深度图像回调函数"""
        try:
            # 转换图像格式 (32FC1 - 浮点型深度值，单位通常为米)
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            
            # 更新最新深度图像
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
    
    def ros_spin(self):
        """ROS线程函数"""
        try:
            while self.running and rclpy.ok():
                rclpy.spin_once(self.node, timeout_sec=0.1)
        except Exception as e:
            self.node.get_logger().error(f"ROS循环异常: {str(e)}")
    
    def shutdown(self):
        """关闭资源"""
        self.running = False
        if self.ros_thread.is_alive():
            self.ros_thread.join(timeout=1.0)
        self.node.destroy_node()
