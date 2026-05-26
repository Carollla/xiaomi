#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class YellowDetector(Node):
    def __init__(self):
        super().__init__('yellow_detector')
        
        # 图像传输配置
        self.sub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            depth=1
        )
        
        # 初始化组件
        self.bridge = CvBridge()
        
        # 通信设置
        self.image_sub = self.create_subscription(Image,'/rgb_camera/image_raw',self.image_callback,self.sub_qos )
        self.detection_pub = self.create_publisher(Bool, '/yellow_detection', 10)

        
        # HSV颜色范围定义
        self.lower_yellow = np.array([20, 100, 100])  # 黄色下限
        self.upper_yellow = np.array([30, 255, 255]) # 黄色上限
        
        # 形态学处理核
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
        
        self.get_logger().info("黄色检测节点已启动")

    def image_callback(self, msg):
        try:
            # 转换图像格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # 转换为HSV颜色空间
            hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            # 颜色阈值处理
            mask = cv2.inRange(hsv_img, self.lower_yellow, self.upper_yellow)
            
            # 形态学去噪
            processed = cv2.erode(mask, self.kernel)
            processed = cv2.dilate(processed, self.kernel)
            
            # 统计有效像素数量
            count = cv2.countNonZero(processed)
            
            # 创建并发布检测结果
            result = Bool()
            result.data = count > 50 
            self.detection_pub.publish(result)
            self.get_logger().info(f"检测结果: {int(result.data)}", 
                                throttle_duration_sec=1.0)
                
        except Exception as e:
            self.get_logger().error(f"图像处理失败: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = YellowDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()