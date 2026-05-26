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
import transforms3d as t3d

class IMUProvider:
    """
    IMU数据提供器
    从ROS话题获取IMU数据并提供方向、角速度、加速度等数据
    """
    def __init__(self):
        # 初始化ROS
        #rclpy.init(args=None)
        self.node = rclpy.create_node('imu_provider')
        
        # IMU数据相关
        self.latest_imu_data = None
        self.data_lock = threading.Lock()
        
        # 设置QoS配置
        self.best_effort_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # 订阅IMU话题
        self.imu_sub = self.node.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            self.best_effort_qos
        )
        
        # 创建ROS线程
        self.ros_thread = threading.Thread(target=self.ros_spin)
        self.ros_thread.daemon = True  # 后台运行
        self.running = True
        
        # 启动ROS线程
        self.ros_thread.start()
        
        # 记录日志
        self.node.get_logger().info("IMU提供器初始化完成，等待IMU数据...")
    
    def imu_callback(self, msg):
        """IMU数据回调函数"""
        try:
            with self.data_lock:
                self.latest_imu_data = msg
        except Exception as e:
            self.node.get_logger().error(f"IMU数据处理失败: {str(e)}")
    
    def _quaternion_to_euler(self, x, y, z, w):
        """
        将四元数转换为欧拉角 (Roll, Pitch, Yaw)
        返回弧度值
        """
        if abs(x) < 1e-8 and abs(y) < 1e-8 and abs(z) < 1e-8 and abs(w-1.0) < 1e-8:
            return 0.0, 0.0, 0.0
        # 返回值是按照(roll, pitch, yaw)顺序的元组
        return t3d.euler.quat2euler([w, x, y, z], 'rxyz')
    
    def ros_spin(self):
        """ROS线程函数"""
        try:
            while self.running and rclpy.ok():
                rclpy.spin_once(self.node, timeout_sec=0.1)
        except Exception as e:
            self.node.get_logger().error(f"ROS循环异常: {str(e)}")
    
    def get_pitch(self):
        """
        获取俯仰角(度)
        """
        with self.data_lock:
            if self.latest_imu_data is None:
                return 0.0
            quat = self.latest_imu_data.orientation
            _, pitch, _ = self._quaternion_to_euler(quat.x, quat.y, quat.z, quat.w)
            return math.degrees(pitch)
    
    def get_yaw(self):
        """
        获取偏航角(度)
        """
        with self.data_lock:
            if self.latest_imu_data is None:
                return 0.0
            quat = self.latest_imu_data.orientation
            _, _, yaw = self._quaternion_to_euler(quat.x, quat.y, quat.z, quat.w)
            return math.degrees(yaw)
    
    def shutdown(self):
        """关闭资源"""
        self.running = False
        if hasattr(self, 'ros_thread') and self.ros_thread.is_alive():
            self.ros_thread.join(timeout=1.0)
        if hasattr(self, 'node'):
            self.node.destroy_node()
        self.node.get_logger().info("IMU提供器已关闭")

def main(args=None):
    print("启动IMU提供器")
    
    try:
        provider = IMUProvider()
        print("IMU提供器已创建，等待用户中断...")
        
        try:
            # 等待用户中断
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("程序被用户中断")
    except Exception as e:
        print(f"运行时错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if 'provider' in locals():
            provider.shutdown()
        rclpy.shutdown()
        print("IMU提供器已关闭")

if __name__ == '__main__':
    import sys
    main()
