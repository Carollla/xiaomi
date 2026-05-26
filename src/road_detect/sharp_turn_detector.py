#!/usr/bin/env python3
"""
大转弯检测器
专注于处理大角度弯道和直角转弯场景
"""
import cv2
import numpy as np
import math

from road_detector_base import RoadDetectorBase, DIRECTION_FORWARD, DIRECTION_LEFT, DIRECTION_RIGHT
from road_detector_base import DIRECTION_SHARP_LEFT, DIRECTION_SHARP_RIGHT

class SharpTurnDetector(RoadDetectorBase):
    """大转弯检测器类"""
    def __init__(self):
        # 调用父类初始化方法
        super().__init__()
        
        # 大转弯角度阈值
        self.sharp_threshold = 30.0     # 大转弯阈值，大于此值认为是大转弯
        self.critical_threshold = 60.0  # 临界阈值，大于此值认为是直角转弯
        
        # 大转弯专用参数
        self.corner_detection_enabled = True  # 启用角点检测
        self.corner_detection_quality = 0.01  # 角点检测质量参数
        self.corner_detection_min_dist = 10   # 角点检测最小距离
        self.max_corners = 50                 # 最大角点数量
        
        # 局部特征参数
        self.corner_density_threshold = 0.6   # 角点密度阈值
        self.path_curvature_weight = 0.5      # 路径曲率权重
        
        # 调试选项
        self.show_corners = True              # 显示检测到的角点
        self.show_curvature = True            # 显示路径曲率
        
        # 状态变量
        self.last_corners = None              # 上一帧检测到的角点

    def analyze_path(self, path_results):
        """
        分析路径方向，专注于大转弯识别
        覆盖基类方法
        """
        if not path_results or len(path_results) == 0:
            return DIRECTION_FORWARD  # 默认直行
            
        # 获取主路径
        primary_path = path_results[0]
        angle = primary_path.angle
        
        if angle is None:
            return DIRECTION_FORWARD
            
        # 分析曲率
        curvature = self.analyze_path_curvature(primary_path)
        
        # 大转弯级别方向判断
        abs_angle = abs(angle)
        
        # 检测角点密度，用于辅助判断是否是急转弯
        corner_density = self.analyze_corner_density(primary_path)
        
        # 综合判断
        if abs_angle > self.critical_threshold:
            # 直角转弯
            if angle < 0:
                return DIRECTION_SHARP_LEFT
            else:
                return DIRECTION_SHARP_RIGHT
        elif abs_angle > self.sharp_threshold:
            # 大转弯
            if angle < 0:
                return DIRECTION_SHARP_LEFT
            else:
                return DIRECTION_SHARP_RIGHT
        else:
            # 角度不大，但根据曲率和角点密度可能是大转弯的开始
            if curvature > 0.7 or corner_density > self.corner_density_threshold:
                if angle < 0:
                    return DIRECTION_SHARP_LEFT
                elif angle > 0:
                    return DIRECTION_SHARP_RIGHT
        
        # 如果以上条件都不满足，返回基本方向
        if abs_angle <= 2.0:
            return DIRECTION_FORWARD
        elif angle < 0:
            return DIRECTION_LEFT
        else:
            return DIRECTION_RIGHT
    
    def analyze_path_curvature(self, path):
        """
        分析路径曲率
        返回归一化曲率值(0-1)，值越大表示曲率越大
        """
        if path.left_lines is None or path.right_lines is None:
            return 0.0
        
        # 提取所有线段点
        all_points = []
        
        for line in path.left_lines:
            x1, y1, x2, y2 = line
            all_points.append((x1, y1))
            all_points.append((x2, y2))
            
        for line in path.right_lines:
            x1, y1, x2, y2 = line
            all_points.append((x1, y1))
            all_points.append((x2, y2))
            
        if len(all_points) < 3:
            return 0.0
            
        # 计算平均距离偏差作为曲率估计
        # 直线上的点到直线的距离应该是0，曲线则会有偏差
        
        # 使用主线的斜率和截距
        slope = None
        intercept = None
        
        if path.left_params is not None and path.right_params is not None:
            left_slope, left_intercept = path.left_params
            right_slope, right_intercept = path.right_params
            slope = (left_slope + right_slope) / 2
            intercept = (left_intercept + right_intercept) / 2
        else:
            # 如果没有主线参数，尝试拟合直线
            pts = np.array(all_points)
            vx, vy, x0, y0 = cv2.fitLine(pts, cv2.DIST_L2, 0, 0.01, 0.01)
            
            # 避免除零错误
            if abs(vx) < 0.001:
                vx = 0.001
                
            slope = vy / vx
            intercept = y0 - slope * x0
        
        # 计算点到直线的距离
        distances = []
        for x, y in all_points:
            # 计算点到直线的距离
            if slope is not None and intercept is not None:
                distance = abs(y - (slope * x + intercept)) / math.sqrt(1 + slope * slope)
                distances.append(distance)
        
        if not distances:
            return 0.0
            
        # 计算平均距离并归一化
        avg_distance = sum(distances) / len(distances)
        
        # 根据经验值归一化，50像素距离认为是曲率为1
        normalized_curvature = min(1.0, avg_distance / 50.0)
        
        return normalized_curvature
    
    def analyze_corner_density(self, path):
        """
        分析角点密度，用于辅助判断是否是急转弯
        返回归一化密度值(0-1)，值越大表示角点越多
        """
        if not self.corner_detection_enabled or path.bbox is None:
            return 0.0
            
        # 提取ROI区域
        x, y, w, h = path.bbox
        
        # 创建路径掩码图像
        mask = np.zeros((h, w), dtype=np.uint8)
        
        # 绘制路径轮廓到掩码上
        if path.contour is not None:
            # 调整轮廓坐标
            contour_shifted = path.contour.copy()
            contour_shifted[:,:,0] -= x
            contour_shifted[:,:,1] -= y
            cv2.drawContours(mask, [contour_shifted], 0, 255, -1)
        
        # 检测角点
        corners = cv2.goodFeaturesToTrack(mask, self.max_corners, 
                                         self.corner_detection_quality, 
                                         self.corner_detection_min_dist)
        
        if corners is None:
            return 0.0
            
        # 保存角点供绘制用
        self.last_corners = corners
        
        # 计算角点密度：角点数 / 区域面积的平方根
        area = w * h
        if area <= 0:
            return 0.0
            
        corner_count = len(corners)
        density = corner_count / math.sqrt(area)
        
        # 归一化密度，每100像素区域1个角点认为是密度0.5
        normalized_density = min(1.0, density * 10.0)
        
        return normalized_density
    
    def draw_main_path_info(self, img, path, angle, direction_text):
        """
        增强版主路径信息绘制，添加大转弯相关信息
        覆盖基类方法
        """
        # 首先调用父类方法
        super().draw_main_path_info(img, path, angle, direction_text)
        
        # 添加大转弯专用可视化信息
        h, w = img.shape[:2]
        
        # 绘制曲率信息
        if self.show_curvature:
            curvature = self.analyze_path_curvature(path)
            cv2.putText(img, f"Curvature: {curvature:.2f}", (10, 190), 
                      cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
        
        # 绘制角点密度信息
        if self.show_corners and path.bbox is not None:
            x, y, _, _ = path.bbox
            
            # 绘制检测到的角点
            if self.last_corners is not None:
                for corner in self.last_corners:
                    cx, cy = corner.ravel()
                    # 调整角点坐标
                    cx = int(cx + x)
                    cy = int(cy + y)
                    cv2.circle(img, (cx, cy), 3, (0, 255, 0), -1)
            
            # 显示角点密度
            corner_density = self.analyze_corner_density(path)
            cv2.putText(img, f"Corner Density: {corner_density:.2f}", (10, 230), 
                      cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
        
        # 大转弯模式信息
        if abs(angle) > self.sharp_threshold:
            cv2.putText(img, "Mode: Sharp Turn", (w-280, 30), 
                      cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2) 