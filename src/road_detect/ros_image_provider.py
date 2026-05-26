#!/usr/bin/env python3
"""
ROS图像提供者模块
从ROS话题获取图像并提供给检测算法
"""
import threading
import time
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class RoadImageProvider:
    """
    从ROS话题获取图像并提供给检测算法
    """
    def __init__(self):
        # 初始化ROS
        rclpy.init(args=None)
        
        # 创建ROS节点（但不暴露为服务）
        self.node = rclpy.create_node('road_image_provider')
        
        # 图像相关
        self.bridge = CvBridge()
        self.latest_image = None
        self.image_lock = threading.Lock()
        
        # 图像传输配置
        self.sub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            depth=1
        )
        
        # 订阅图像话题
        self.image_sub = self.node.create_subscription(
            Image,
            '/rgb_camera/image_raw',
            self.image_callback,
            self.sub_qos
        )
        
        # 创建ROS线程
        self.ros_thread = threading.Thread(target=self.ros_spin)
        self.ros_thread.daemon = True  # 后台运行
        self.running = True
        
        # 启动ROS线程
        self.ros_thread.start()
        
        # 等待首次图像
        self.node.get_logger().info("等待图像数据...")
    
    def image_callback(self, msg):
        """图像回调函数"""
        try:
            # 转换图像格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # 更新最新图像
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
        rclpy.shutdown() 