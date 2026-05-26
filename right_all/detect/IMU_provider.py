#!/usr/bin/env python3
"""
IMU数据提供模块
从ROS话题获取IMU数据并提供给方向判断、坡度检测等功能
"""
import threading
import time
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy

class IMUProvider:
    """
    IMU数据提供器
    从ROS话题获取IMU数据并提供方向
    """
    def __init__(self):
        self.node = rclpy.create_node('imu_provider')
        self.latest_imu_data = None
        self.data_lock = threading.Lock()
        self.best_effort_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.imu_sub = self.node.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            self.best_effort_qos
        )
        self.node.get_logger().info("IMU提供器初始化完成，等待IMU数据...")
    
    def imu_callback(self, msg):
        """IMU数据回调函数"""
        try:
            with self.data_lock:
                self.latest_imu_data = msg
                # print(f"IMU数据: {self.latest_imu_data}")
        except Exception as e:
            self.node.get_logger().error(f"IMU数据处理失败: {str(e)}")
    
    def _quaternion_to_euler(self, x, y, z, w):
        """将四元数转换为欧拉角 (Roll, Pitch, Yaw)"""
        if abs(x) < 1e-8 and abs(y) < 1e-8 and abs(z) < 1e-8 and abs(w-1.0) < 1e-8:
            return 0.0, 0.0, 0.0
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1.0:
            pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw
    
    def get_pitch(self):
        """获取俯仰角(度)"""
        with self.data_lock:
            if self.latest_imu_data is None:
                return 0.0
            quat = self.latest_imu_data.orientation
            _, pitch, _ = self._quaternion_to_euler(quat.x, quat.y, quat.z, quat.w)
            return math.degrees(pitch)
    
    def get_yaw(self):
        """获取偏航角(度)"""
        with self.data_lock:
            if self.latest_imu_data is None:
                return 0.0
            quat = self.latest_imu_data.orientation
            _, _, yaw = self._quaternion_to_euler(quat.x, quat.y, quat.z, quat.w)
            return math.degrees(yaw)
    
    def shutdown(self):
        """关闭资源"""
        self.node.destroy_node()
