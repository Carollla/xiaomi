#!/usr/bin/env python3
"""
微调角度检测器
专注于处理小角度的轨道微调场景
"""
import cv2
import numpy as np

from road_detector_base import RoadDetectorBase, DIRECTION_FORWARD, DIRECTION_SLIGHT_LEFT, DIRECTION_SLIGHT_RIGHT

class SlightAdjustmentDetector(RoadDetectorBase):
    """微调角度检测器类"""
    def __init__(self):
        # 调用父类初始化方法
        super().__init__()
        
        # 微调角度阈值
        self.slight_threshold = 10.0   # 微调阈值，小于此值认为是微调
        self.forward_threshold = 2.0   # 直行阈值，小于此值认为是直行
        
        # 微调模式专用参数
        self.lateral_error_weight = 0.7   # 横向误差权重
        self.angle_error_weight = 0.3     # 角度误差权重
        
        # 调试选项
        self.show_lateral_error = True    # 显示横向误差

    def analyze_path(self, path_results):
        """
        分析路径方向，专注于微调角度
        覆盖基类方法
        """
        if not path_results or len(path_results) == 0:
            return DIRECTION_FORWARD  # 默认直行
            
        # 获取主路径
        primary_path = path_results[0]
        angle = primary_path.angle
        
        if angle is None:
            return DIRECTION_FORWARD
            
        # 微调级别方向判断
        abs_angle = abs(angle)
        
        # 根据角度大小判断方向
        if abs_angle <= self.forward_threshold:
            # 即使角度很小，也计算横向偏移再决定
            return self.analyze_lateral_error(primary_path)
        elif abs_angle <= self.slight_threshold:
            # 微调级别
            if angle < 0:
                return DIRECTION_SLIGHT_LEFT
            else:
                return DIRECTION_SLIGHT_RIGHT
        else:
            # 超出微调范围，仍返回微调方向，但可能需要更大的修正
            if angle < 0:
                return DIRECTION_SLIGHT_LEFT
            else:
                return DIRECTION_SLIGHT_RIGHT
    
    def analyze_lateral_error(self, path):
        """分析横向误差，用于精细判断直行时的微调需求"""
        if path.center_point is None:
            return DIRECTION_FORWARD
            
        # 获取路径中心点
        center_x, _ = path.center_point
        
        # 获取图像宽度
        _, w = self.get_last_image_size()
        image_center_x = w // 2
        
        # 计算横向误差（像素）
        lateral_error = center_x - image_center_x
        
        # 归一化横向误差（以图像宽度的10%为阈值）
        lateral_threshold = w * 0.05  # 5%宽度作为阈值
        
        if abs(lateral_error) < lateral_threshold:
            return DIRECTION_FORWARD
        elif lateral_error < 0:
            # 中心线在图像中心左侧，需右转微调
            return DIRECTION_SLIGHT_RIGHT
        else:
            # 中心线在图像中心右侧，需左转微调
            return DIRECTION_SLIGHT_LEFT
    
    def get_last_image_size(self):
        """获取最近处理图像的尺寸"""
        # 如果没有记录图像大小，返回默认值
        return (480, 640)  # 默认高度和宽度
    
    def draw_main_path_info(self, img, path, angle, direction_text):
        """
        增强版主路径信息绘制，添加微调相关信息
        覆盖基类方法
        """
        # 首先调用父类方法
        super().draw_main_path_info(img, path, angle, direction_text)
        
        # 添加微调专用可视化信息
        h, w = img.shape[:2]
        
        if self.show_lateral_error and path.center_point is not None:
            # 绘制图像中心垂直线
            cv2.line(img, (w//2, 0), (w//2, h), (0, 255, 255), 1)
            
            # 绘制路径中心垂直线
            center_x, _ = path.center_point
            cv2.line(img, (center_x, h), (center_x, h-100), (255, 0, 255), 2)
            
            # 计算横向误差
            lateral_error = center_x - w//2
            
            # 绘制误差连接线
            cv2.line(img, (w//2, h-50), (center_x, h-50), (255, 0, 255), 2)
            
            # 标注横向误差值
            cv2.putText(img, f"Lateral: {lateral_error}px", (10, 150), 
                      cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)
            
            # 微调模式信息
            cv2.putText(img, "Mode: Slight Adjustment", (w-280, 30), 
                      cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2) 