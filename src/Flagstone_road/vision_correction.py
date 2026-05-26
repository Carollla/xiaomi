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

class LineAnalyzer:
    """
    黄线分析核心算法类
    算法特点：
    - 使用自适应HSV阈值
    - 基于最小二乘法的角度拟合
    - 包含噪声过滤机制
    """
    def __init__(self):
        # 动态阈值初始化值
        self.lower_hsv = np.array([20, 100, 100])
        self.upper_hsv = np.array([30, 255, 255])
        
        # 图像处理参数
        self.erode_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
        self.dilate_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
        
        # 角度计算参数
        self.min_line_points = 20    # 有效检测最小点数
        self.angle_filter = AngleFilter()  # 角度滤波器

    def dynamic_threshold(self, hsv_img):
        """
        自适应阈值调整
        根据图像亮度动态调整V通道阈值
        """
        v_channel = hsv_img[:,:,2]
        v_mean = np.mean(v_channel)
        
        # 动态调整V阈值
        self.lower_hsv[2] = max(50, int(v_mean*0.6))
        self.upper_hsv[2] = min(250, int(v_mean*1.4))
        
        return cv2.inRange(hsv_img, self.lower_hsv, self.upper_hsv)

    def find_contour_centroid(self, mask):
        """
        寻找最大连通域的质心轨迹
        返回：质心坐标列表 (x,y)
        """
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
            
        # 选择面积最大的轮廓
        max_contour = max(contours, key=cv2.contourArea)
        moments = cv2.moments(max_contour)
        
        # 沿y轴采样质心
        centroids = []
        for y in range(0, mask.shape[0], 5):  # 5像素间隔采样
            slice_mask = mask[y:y+5, :]
            if np.any(slice_mask):
                x_center = np.mean(np.where(slice_mask)[1])
                centroids.append((x_center, y+2))  # y+2取中间位置
        return np.array(centroids)

    def calculate_angle(self, points, img_width):
        """
        基于最小二乘法的角度计算（兼容旧版本NumPy）
        返回：旋转角度（度）
        """
        if len(points) < self.min_line_points:
            return None
        
        # 转换为归一化坐标
        y_norm = (points[:,1] / img_width).reshape(-1,1)
        x_norm = points[:,0] / img_width
    
        # 生成设计矩阵
        A = np.hstack([y_norm, np.ones_like(y_norm)])
    
        # 加权处理（兼容性实现）
        weights = np.linspace(0.5, 1.0, len(x_norm))
    
        # 手动应用权重（替代weights参数）
        sqrt_weights = np.sqrt(weights)
        A_weighted = A * sqrt_weights[:, np.newaxis]  # 对A矩阵加权
        x_weighted = x_norm * sqrt_weights            # 对观测值加权
    
        try:
            # 执行最小二乘计算
            coef = np.linalg.lstsq(A_weighted, x_weighted, rcond=None)[0]
        except np.linalg.LinAlgError as e:
            self.get_logger().warn(f"矩阵奇异值问题: {str(e)}")
            return None
        
        # 计算角度（考虑图像坐标系）
        angle_rad = np.arctan(coef[0]) 
        return np.degrees(angle_rad)

    def process(self, cv_image):
        """
        完整处理流程
        返回：矫正角度（度）
        """
        # 预处理
        hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = self.dynamic_threshold(hsv_img)
        
        # 形态学操作
        processed = cv2.erode(mask, self.erode_kernel)
        processed = cv2.dilate(processed, self.dilate_kernel)
        
        # 寻找质心轨迹
        centroids = self.find_contour_centroid(processed)
        if centroids is None:
            return None
            
        # 计算原始角度
        raw_angle = self.calculate_angle(centroids, cv_image.shape[1])
        if raw_angle is None:
            return None
            
        # 角度滤波处理
        return self.angle_filter.update(raw_angle)

class AngleFilter:
    """
    角度滤波器
    功能：使用滑动窗口均值滤波消除抖动
    """
    def __init__(self, window_size=5):
        self.window = []
        self.window_size = window_size

    def update(self, new_angle):
        if new_angle is None:
            return None
            
        self.window.append(new_angle)
        if len(self.window) > self.window_size:
            self.window.pop(0)
            
        return np.mean(self.window)

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