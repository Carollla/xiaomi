#!/usr/bin/env python3
"""
S弯检测器
专注于处理S型弯道场景和多路径选择
"""
import cv2
import numpy as np

from road_detector_base import RoadDetectorBase, DIRECTION_FORWARD, DIRECTION_LEFT, DIRECTION_RIGHT, DIRECTION_S_CURVE

class SCurveDetector(RoadDetectorBase):
    """S弯检测器类"""
    def __init__(self):
        # 调用父类初始化方法
        super().__init__()
        
        # S弯参数
        self.s_curve_angle_diff = 25.0    # S弯道的角度差异阈值
        self.s_curve_opposite_required = True  # 是否要求两段路径角度符号相反
        self.s_curve_min_confidence = 0.7  # S弯最小置信度
        
        # 多路径参数
        self.max_paths = 3                # 最多处理的路径数量
        self.min_path_distance = 50       # 相邻路径的最小距离
        self.prefer_closer_paths = True   # 是否偏好更近的路径
        
        # 调试选项
        self.show_path_connections = True  # 显示路径连接
        self.show_s_curve_indicators = True  # 显示S弯指示器
        
        # 状态变量
        self.is_in_s_curve = False       # 是否正在S弯中
        self.s_curve_confidence = 0.0    # S弯置信度
        self.s_curve_direction = 0       # S弯方向

    def analyze_path(self, path_results):
        """
        分析路径方向，专注于S弯识别
        覆盖基类方法
        """
        if not path_results or len(path_results) == 0:
            self.is_in_s_curve = False
            return DIRECTION_FORWARD  # 默认直行
        
        # 首先检查是否有多条路径
        if len(path_results) < 2:
            self.is_in_s_curve = False
            # 只有一条路径，使用基本方向判断
            return super().analyze_path(path_results)
        
        # 获取最近的两条路径
        path1 = path_results[0]
        path2 = path_results[1]
        
        # 检测S弯
        is_s_curve, s_direction, confidence = self.detect_s_curve(path1, path2)
        
        # 保存S弯状态
        self.is_in_s_curve = is_s_curve
        self.s_curve_confidence = confidence
        self.s_curve_direction = s_direction
        
        if is_s_curve:
            return DIRECTION_S_CURVE
        
        # 如果不是S弯，返回基本方向
        return super().analyze_path([path_results[0]])
    
    def detect_s_curve(self, path1, path2):
        """
        检测两条路径是否构成S弯
        返回: (是否是S弯, S弯方向, 置信度)
        """
        # 检查路径角度
        angle1 = path1.angle
        angle2 = path2.angle
        
        if angle1 is None or angle2 is None:
            return False, 0, 0.0
        
        # 计算角度差
        angle_diff = abs(angle1 - angle2)
        
        # 计算路径距离，检查是否足够近
        distance_ok = True
        if path1.center_point is not None and path2.center_point is not None:
            cx1, cy1 = path1.center_point
            cx2, cy2 = path2.center_point
            path_distance = np.sqrt((cx1-cx2)**2 + (cy1-cy2)**2)
            distance_ok = path_distance < self.min_path_distance * 3  # 允许更大的距离
        
        # S弯判断条件:
        # 1. 两条路径角度差较大
        # 2. 角度符号相反(如果要求)
        # 3. 路径距离适中
        is_angle_diff_enough = angle_diff > self.s_curve_angle_diff
        is_opposite_direction = (angle1 * angle2 < 0) if self.s_curve_opposite_required else True
        
        is_s_curve = is_angle_diff_enough and is_opposite_direction and distance_ok
        
        # 计算置信度
        confidence = 0.0
        if is_s_curve:
            # 角度差贡献(最大0.7)
            angle_diff_norm = min(1.0, angle_diff / 90.0) * 0.7
            
            # 方向一致性贡献(0.3)
            direction_factor = 0.3 if is_opposite_direction else 0.0
            
            confidence = angle_diff_norm + direction_factor
        
        # 确定S弯方向：根据第一段路径的方向
        s_direction = -1 if angle1 < 0 else 1  # -1表示先左后右，1表示先右后左
        
        return is_s_curve, s_direction, confidence
    
    def get_path_connection_points(self, path1, path2):
        """计算两条路径的连接点"""
        if path1.center_point is None or path2.center_point is None:
            return None
            
        cx1, cy1 = path1.center_point
        cx2, cy2 = path2.center_point
        
        # 计算连接中点
        mid_x = (cx1 + cx2) // 2
        mid_y = (cy1 + cy2) // 2
        
        return [(cx1, cy1), (mid_x, mid_y), (cx2, cy2)]
    
    def draw_main_path_info(self, img, path, angle, direction_text):
        """
        增强版主路径信息绘制，添加S弯相关信息
        覆盖基类方法
        """
        # 首先调用父类方法
        super().draw_main_path_info(img, path, angle, direction_text)
        
        # 添加S弯专用可视化信息
        h, w = img.shape[:2]
        
        # 如果是S弯，绘制特殊标记
        if self.is_in_s_curve and self.show_s_curve_indicators:
            # 显示S弯置信度
            cv2.putText(img, f"S-curve: {self.s_curve_confidence:.2f}", (10, 190), 
                      cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            # 显示大警告标志
            cv2.putText(img, "S-CURVE DETECTED!", (w//2-150, 50), 
                      cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)
            
            # 显示S弯方向
            s_dir_text = "Left then Right" if self.s_curve_direction < 0 else "Right then Left"
            cv2.putText(img, s_dir_text, (w//2-120, 90), 
                      cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
    
    def draw_all_paths(self, img, path_results):
        """
        增强版路径绘制，添加S弯路径连接
        覆盖基类方法
        """
        # 首先调用父类方法
        super().draw_all_paths(img, path_results)
        
        # 如果有多条路径且需要显示连接，则绘制路径连接
        if len(path_results) >= 2 and self.show_path_connections:
            # 获取最近的两条路径
            path1 = path_results[0]
            path2 = path_results[1]
            
            # 检测是否是S弯
            is_s_curve, _, _ = self.detect_s_curve(path1, path2)
            
            # 获取连接点
            connection_points = self.get_path_connection_points(path1, path2)
            if connection_points:
                # 如果是S弯，使用红色；否则使用黄色
                color = (0, 0, 255) if is_s_curve else (0, 255, 255)
                
                # 绘制连接线（贝塞尔曲线近似）
                p1, p2, p3 = connection_points
                
                # 将连接点转换为数组
                pts = np.array([p1, p2, p3], dtype=np.int32)
                
                # 绘制平滑曲线
                cv2.polylines(img, [pts], False, color, 2)
                
                # 在连接线上添加标记
                if is_s_curve:
                    cv2.putText(img, "S", (p2[0]-10, p2[1]-10), 
                              cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2) 