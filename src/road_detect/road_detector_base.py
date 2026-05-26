#!/usr/bin/env python3
"""
轨道检测算法基类
提供基础图像处理和线段检测功能
"""
import cv2
import numpy as np

# 方向枚举常量
# 基础方向
DIRECTION_UNKNOWN = 0      # 未知/无效
DIRECTION_FORWARD = 1      # 直行前进
DIRECTION_LEFT = 2         # 左转
DIRECTION_RIGHT = 3        # 右转

# 微调方向
DIRECTION_SLIGHT_LEFT = 10   # 微调左转
DIRECTION_SLIGHT_RIGHT = 11  # 微调右转

# 大转弯方向
DIRECTION_SHARP_LEFT = 20    # 大角度左转
DIRECTION_SHARP_RIGHT = 21   # 大角度右转

# 特殊方向
DIRECTION_S_CURVE = 30     # S弯道


class AngleFilter:
    """角度滤波器"""
    def __init__(self, window_size=5):
        self.window = []
        self.window_size = window_size
        self.prev_filtered_angle = None

    def update(self, new_angle):
        if new_angle is None:
            return self.prev_filtered_angle
            
        self.window.append(new_angle)
        if len(self.window) > self.window_size:
            self.window.pop(0)
        
        # 中值滤波 + 平均滤波的组合，去除异常值并平滑结果
        if len(self.window) >= 3:  # 至少需要几个样本才能进行中值滤波
            # 首先排序并去除最大和最小值（去除异常值）
            sorted_angles = sorted(self.window)
            trimmed_angles = sorted_angles[1:-1] if len(sorted_angles) > 3 else sorted_angles
            
            # 然后计算平均值
            filtered_angle = np.mean(trimmed_angles)
            
            # 应用低通滤波，平滑角度变化
            if self.prev_filtered_angle is not None:
                # 限制变化速率，alpha越小，平滑效果越明显
                alpha = 0.3  
                filtered_angle = alpha * filtered_angle + (1 - alpha) * self.prev_filtered_angle
        else:
            filtered_angle = np.mean(self.window)
        
        self.prev_filtered_angle = filtered_angle
        return filtered_angle


class PathInfo:
    """路径信息类，储存检测到的路径数据"""
    def __init__(self):
        self.left_lines = []         # 左侧线段列表
        self.right_lines = []        # 右侧线段列表
        self.left_params = None      # 左侧线参数 (斜率,截距)
        self.right_params = None     # 右侧线参数 (斜率,截距) 
        self.center_point = None     # 中心点坐标 (x,y)
        self.angle = None            # 路径角度
        self.width = 0               # 路径宽度
        self.distance = 0            # 到图像底部的距离
        self.contour = None          # 路径轮廓
        self.bbox = None             # 边界框 (x,y,w,h)
        self.direction = DIRECTION_UNKNOWN  # 方向枚举
        self.confidence = 0.0        # 置信度


class RoadDetectorBase:
    """道路检测器基类"""
    def __init__(self):
        # HSV阈值参数
        self.lower_yellow = np.array([20, 100, 100])  # 黄色下限
        self.upper_yellow = np.array([30, 255, 255])  # 黄色上限
        
        # 形态学处理核
        self.kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        
        # 霍夫线参数
        self.hough_threshold = 50      # 最小投票数
        self.min_line_length = 80      # 最小线段长度
        self.max_line_gap = 15         # 最大线段间隙
        
        # 角度滤波器
        self.angle_filter = AngleFilter(window_size=15)
        
        # 常量和安全参数
        self.min_slope = 0.001         # 避免除零
        self.max_angle_change = 3.0    # 最大角度变化
        self.last_angle = None         # 上一帧角度
        
        # 多路径处理参数
        self.max_paths = 3             # 最多处理的路径数量
        self.min_contour_area = 500    # 最小轮廓面积
        
        # 调试选项
        self.show_all_paths = True     # 显示所有检测到的路径
        self.show_contours = True      # 显示轮廓

    def process_image(self, image):
        """
        处理图像，检测轨道方向
        返回: (角度, 方向枚举, 路径类型, 调试图像, 掩码图像)
        """
        if image is None:
            return None, DIRECTION_UNKNOWN, None, None, None
            
        # 预处理和掩码生成
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
        
        # 形态学操作（去噪）
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)
        
        # ROI提取（只关注图像下半部分）
        h, w = mask.shape
        roi_mask = mask[h//2:h, :]
        
        # 创建调试图像
        debug_img = image.copy()
        mask_color = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        
        # 检测多条路径
        path_results = self.detect_multiple_paths(roi_mask, h, w)
        
        if not path_results or len(path_results) == 0:
            return None, DIRECTION_UNKNOWN, None, debug_img, mask_color
        
        # 绘制所有检测到的路径
        if self.show_all_paths:
            self.draw_all_paths(debug_img, path_results)
        
        # 检查主路径
        primary_path = path_results[0]  # 最近的路径
        
        # 分析路径
        direction = self.analyze_path(path_results)
        
        # 对主路径进行角度滤波
        angle = primary_path.angle
        if angle is not None:
            # 应用角度变化限制
            if self.last_angle is not None:
                angle_diff = angle - self.last_angle
                if abs(angle_diff) > self.max_angle_change:
                    sign = 1 if angle_diff > 0 else -1
                    angle = self.last_angle + sign * self.max_angle_change
                    
            # 应用滤波
            filtered_angle = self.angle_filter.update(angle)
            
            # 更新上一帧角度
            self.last_angle = filtered_angle
            angle = filtered_angle
            
            # 更新主路径角度
            primary_path.angle = angle
        
        # 获取方向文本
        direction_text = self.get_direction_text(direction)
        
        # 绘制主路径信息
        self.draw_main_path_info(debug_img, primary_path, angle, direction_text)
        
        return angle, direction, direction_text, debug_img, mask_color
    
    def analyze_path(self, path_results):
        """
        分析路径方向，基类版本仅提供基础分析
        子类中应重写此方法以提供特定类型的分析
        """
        if not path_results or len(path_results) == 0:
            return DIRECTION_UNKNOWN
            
        primary_path = path_results[0]
        angle = primary_path.angle
        
        if angle is None:
            return DIRECTION_UNKNOWN
            
        # 基本方向判断
        if abs(angle) <= 2.0:
            return DIRECTION_FORWARD
        elif angle < 0:
            return DIRECTION_LEFT
        else:
            return DIRECTION_RIGHT
    
    def detect_multiple_paths(self, roi_mask, img_height, img_width):
        """
        检测图像中的多条路径
        返回: 按距离排序的路径列表
        """
        # 查找轮廓
        contours, _ = cv2.findContours(roi_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # 过滤轮廓
        valid_paths = []
        for contour in contours:
            # 计算轮廓面积
            area = cv2.contourArea(contour)
            if area < self.min_contour_area:
                continue
                
            # 获取边界矩形
            x, y, w, h = cv2.boundingRect(contour)
            
            # 调整坐标（考虑ROI偏移）
            y += img_height // 2
            
            # 根据轮廓创建路径掩码
            path_mask = np.zeros_like(roi_mask)
            cv2.drawContours(path_mask, [contour], 0, 255, -1)
            
            # 在该轮廓区域检测线段
            lines = cv2.HoughLinesP(path_mask, 1, np.pi/180, self.hough_threshold,
                                   minLineLength=self.min_line_length,
                                   maxLineGap=self.max_line_gap)
                                   
            if lines is None or len(lines) < 2:
                continue
                
            # 分类左右轨道线
            left_lines = []
            right_lines = []
            center_x = x + w // 2
            
            for line in lines:
                x1, y1, x2, y2 = line[0]
                
                # 调整坐标（考虑ROI偏移）
                y1 += img_height // 2
                y2 += img_height // 2
                
                # 计算线段中点
                mid_x = (x1 + x2) // 2
                
                # 过滤垂直线
                if x2 - x1 == 0:
                    continue
                    
                slope = (y2 - y1) / (x2 - x1)
                if abs(slope) > 5.0:  # 过滤斜率过大的线
                    continue
                
                # 分类左右线
                if mid_x < center_x:
                    left_lines.append(line[0])
                else:
                    right_lines.append(line[0])
            
            # 确保有足够的线段
            if len(left_lines) == 0 or len(right_lines) == 0:
                continue
                
            # 计算左右轨道线的参数
            left_params, right_params = self.get_average_lines(left_lines, right_lines)
            if left_params is None or right_params is None:
                continue
                
            # 计算中心线斜率和角度
            center_slope = (left_params[0] + right_params[0]) / 2
            center_intercept = (left_params[1] + right_params[1]) / 2
            angle = np.arctan(center_slope) * 180 / np.pi
            
            # 计算路径底部宽度和中心点
            bottom_y = img_height
            left_x = int((bottom_y - left_params[1]) / left_params[0])
            right_x = int((bottom_y - right_params[1]) / right_params[0])
            
            # 约束坐标范围
            left_x = max(0, min(img_width-1, left_x))
            right_x = max(0, min(img_width-1, right_x))
            
            path_width = abs(right_x - left_x)
            path_center_x = (left_x + right_x) // 2
            
            # 估计距离（使用底部中心点的y坐标）
            distance_score = bottom_y - (y + h) # 距底部越近，分数越低
            
            # 创建路径信息
            path_info = PathInfo()
            path_info.left_lines = left_lines
            path_info.right_lines = right_lines
            path_info.left_params = left_params
            path_info.right_params = right_params
            path_info.center_point = (path_center_x, bottom_y)
            path_info.bbox = (x, y, w, h)
            path_info.width = path_width
            path_info.angle = angle
            path_info.distance = distance_score
            path_info.contour = contour
            
            valid_paths.append(path_info)
        
        # 按距离排序（距离小的排前面）
        valid_paths.sort(key=lambda p: p.distance)
        
        # 限制路径数量
        return valid_paths[:self.max_paths]
        
    def get_average_lines(self, left_lines, right_lines):
        """
        计算左右轨道线的平均斜率和截距
        """
        # 计算左侧线段参数
        left_slopes = []
        left_intercepts = []
        
        for line in left_lines:
            x1, y1, x2, y2 = line
            if x2 - x1 == 0:
                continue
            slope = (y2 - y1) / (x2 - x1)
            if abs(slope) > 5.0:  # 过滤异常斜率
                continue
            intercept = y1 - slope * x1
            left_slopes.append(slope)
            left_intercepts.append(intercept)
            
        # 计算右侧线段参数
        right_slopes = []
        right_intercepts = []
        
        for line in right_lines:
            x1, y1, x2, y2 = line
            if x2 - x1 == 0:
                continue
            slope = (y2 - y1) / (x2 - x1)
            if abs(slope) > 5.0:  # 过滤异常斜率
                continue
            intercept = y1 - slope * x1
            right_slopes.append(slope)
            right_intercepts.append(intercept)
            
        # 检查是否有足够的数据
        if not left_slopes or not right_slopes:
            return None, None
            
        # 过滤异常值
        left_slopes = self.filter_outliers(left_slopes)
        left_intercepts = self.filter_outliers(left_intercepts)
        right_slopes = self.filter_outliers(right_slopes)
        right_intercepts = self.filter_outliers(right_intercepts)
        
        if not left_slopes or not right_slopes:
            return None, None
            
        # 计算中位数
        left_avg_slope = np.median(left_slopes)
        left_avg_intercept = np.median(left_intercepts)
        right_avg_slope = np.median(right_slopes)
        right_avg_intercept = np.median(right_intercepts)
        
        # 确保斜率不为零
        if abs(left_avg_slope) < self.min_slope:
            left_avg_slope = self.min_slope if left_avg_slope >= 0 else -self.min_slope
        
        if abs(right_avg_slope) < self.min_slope:
            right_avg_slope = self.min_slope if right_avg_slope >= 0 else -self.min_slope
        
        return (left_avg_slope, left_avg_intercept), (right_avg_slope, right_avg_intercept)
    
    def filter_outliers(self, values):
        """过滤异常值"""
        if len(values) <= 2:
            return values
            
        # 四分位数过滤
        q1, q3 = np.percentile(values, [25, 75])
        iqr = q3 - q1
        lower_bound = q1 - 1.5 * iqr
        upper_bound = q3 + 1.5 * iqr
        
        return [x for x in values if lower_bound <= x <= upper_bound]
    
    def get_direction_text(self, direction):
        """获取方向的文本描述"""
        if direction == DIRECTION_FORWARD:
            return "直行"
        elif direction == DIRECTION_LEFT:
            return "左转"
        elif direction == DIRECTION_RIGHT:
            return "右转"
        elif direction == DIRECTION_SLIGHT_LEFT:
            return "微调左转"
        elif direction == DIRECTION_SLIGHT_RIGHT:
            return "微调右转"
        elif direction == DIRECTION_SHARP_LEFT:
            return "大角度左转"
        elif direction == DIRECTION_SHARP_RIGHT:
            return "大角度右转"
        elif direction == DIRECTION_S_CURVE:
            return "S弯道"
        else:
            return "未知"
            
    def draw_all_paths(self, img, path_results):
        """绘制所有检测到的路径"""
        colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0)]  # 不同路径使用不同颜色
        
        for i, path in enumerate(path_results):
            color = colors[i % len(colors)]
            
            # 绘制轮廓
            if self.show_contours and path.contour is not None:
                cv2.drawContours(img, [path.contour], 0, color, 2)
            
            # 绘制边界框
            if path.bbox is not None:
                x, y, w, h = path.bbox
                cv2.rectangle(img, (x, y), (x+w, y+h), color, 1)
            
            # 绘制路径中心点
            if path.center_point is not None:
                cv2.circle(img, path.center_point, 5, color, -1)
            
            # 绘制路径线
            if path.left_params is not None and path.right_params is not None:
                self.draw_path_lines(img, path.left_params, path.right_params, color)
            
            # 标注路径编号
            if path.center_point is not None:
                cx, cy = path.center_point
                cv2.putText(img, f"Path {i+1}", (cx-30, cy-10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                
            # 标注角度
            if path.angle is not None:
                cx, cy = path.center_point
                cv2.putText(img, f"{path.angle:.1f}°", (cx-30, cy+20),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
    
    def draw_path_lines(self, img, left_params, right_params, color, thickness=2):
        """绘制路径线"""
        h, w = img.shape[:2]
        left_slope, left_intercept = left_params
        right_slope, right_intercept = right_params
        
        # 计算两点坐标
        y1 = h
        y2 = int(h * 0.6)
        
        # 确保斜率不为零
        if abs(left_slope) < self.min_slope:
            left_slope = self.min_slope if left_slope >= 0 else -self.min_slope
        
        if abs(right_slope) < self.min_slope:
            right_slope = self.min_slope if right_slope >= 0 else -self.min_slope
        
        # 计算x坐标
        x1_left = int((y1 - left_intercept) / left_slope)
        x2_left = int((y2 - left_intercept) / left_slope)
        x1_right = int((y1 - right_intercept) / right_slope)
        x2_right = int((y2 - right_intercept) / right_slope)
        
        # 限制坐标范围
        x1_left = max(0, min(w-1, x1_left))
        x2_left = max(0, min(w-1, x2_left))
        x1_right = max(0, min(w-1, x1_right))
        x2_right = max(0, min(w-1, x2_right))
        
        # 绘制线段
        cv2.line(img, (x1_left, y1), (x2_left, y2), color, thickness)
        cv2.line(img, (x1_right, y1), (x2_right, y2), color, thickness)
        
        # 计算中心线
        center_slope = (left_slope + right_slope) / 2
        center_intercept = (left_intercept + right_intercept) / 2
        
        if abs(center_slope) < self.min_slope:
            center_slope = self.min_slope if center_slope >= 0 else -self.min_slope
            
        x1_center = int((y1 - center_intercept) / center_slope)
        x2_center = int((y2 - center_intercept) / center_slope)
        
        x1_center = max(0, min(w-1, x1_center))
        x2_center = max(0, min(w-1, x2_center))
        
        # 绘制中心线（颜色略淡一些）
        center_color = tuple([min(255, c + 80) for c in color])
        cv2.line(img, (x1_center, y1), (x2_center, y2), center_color, thickness+1)
    
    def draw_main_path_info(self, img, path, angle, direction_text):
        """绘制主路径信息"""
        h, w = img.shape[:2]
        
        # 绘制路径线（粗一些）
        self.draw_path_lines(img, path.left_params, path.right_params, (0, 0, 255), 3)
        
        # 绘制参考线（垂直中线）
        cv2.line(img, (w//2, 0), (w//2, h), (0, 255, 255), 1)
        
        # 显示角度信息
        cv2.putText(img, f"Angle: {angle:.1f}°", (10, 30), 
                  cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        # 显示方向信息
        cv2.putText(img, f"Direction: {direction_text}", (10, 70), 
                  cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        # 显示路径宽度
        cv2.putText(img, f"Width: {path.width:.1f} px", (10, 110), 
                  cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2) 