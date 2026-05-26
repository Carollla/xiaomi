#!/usr/bin/env python3
"""
视觉矫正模块
功能：检测黄线并计算机器人需要旋转的矫正角度
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy


class VisionCorrector(Node):
    """
    ROS2视觉矫正节点
    """
    def __init__(self):
        super().__init__('vision_corrector')
        
        # 图像传输配置（优化带宽）
        angle_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,  # 关键控制数据需要可靠传输
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        debug_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,  # 调试数据允许丢帧
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        
        # 初始化组件
        self.bridge = CvBridge()
        self.analyzer = LineAnalyzer()
        
        # 创建订阅/发布
        self.image_sub = self.create_subscription(
            Image,
            '/rgb_camera/image_raw',
            self.image_callback,
            debug_qos)
            
        self.angle_pub = self.create_publisher(
            Float32,
            '/vision/correction_angle',
            angle_qos)
            
        # 调试信息发布
        self.debug_pub = self.create_publisher(
            Image,
            '/vision/debug_output',
            debug_qos)
            
        self.get_logger().info("视觉矫正节点已启动")

    def image_callback(self, msg):
        try:
            # 转换图像格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # 执行分析
            correction_angle = self.analyzer.process(cv_image)
            
            if correction_angle is not None:
                # 发布矫正角度
                self.publish_angle(correction_angle)
                
                # 生成调试图像
                debug_img = self.generate_debug_image(cv_image, correction_angle)
                self.publish_debug_image(debug_img)
                
        except Exception as e:
            self.get_logger().error(f"图像处理失败: {str(e)}")

    def publish_angle(self, angle):
        """发布矫正角度"""
        msg = Float32()
        msg.data = float(-angle)  # 取反得到需要旋转的角度
        self.angle_pub.publish(msg)
        self.get_logger().info(f"发布矫正角度: {msg.data:.2f}度", throttle_duration_sec=1)

    def generate_debug_image(self, cv_image, angle):
        """生成调试可视化图像"""
        # 绘制基准线
        debug_img = cv_image.copy()
        h, w = debug_img.shape[:2]
        cv2.line(debug_img, (w//2,0), (w//2,h), (0,255,0), 2)
        
        # 绘制预测角度
        text = f"Req Rotation: {-angle:.1f}deg"
        cv2.putText(debug_img, text, (10,30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255), 2)
                   
        # 绘制角度指示线
        pt1 = (w//2, h//2)
        pt2 = (int(w//2 + 100 * np.sin(np.radians(angle))), 
              int(h//2 - 100 * np.cos(np.radians(angle))))
        cv2.arrowedLine(debug_img, pt1, pt2, (255,0,0), 3, tipLength=0.3)
        
        return debug_img

    def publish_debug_image(self, cv_image):
        """发布调试图像"""
        try:
            msg = self.bridge.cv2_to_imgmsg(cv_image, 'bgr8')
            self.debug_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"调试图像发布失败: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = VisionCorrector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()