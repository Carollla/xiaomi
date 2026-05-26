#!/usr/bin/env python3
"""
轨道检测核心算法模块
提供角度滤波和道路检测的核心功能
"""
import cv2
import numpy as np

# 方向枚举（用整数表示）
DIRECTION_LEFT = 1    # 偏左
DIRECTION_FORWARD = 0  # 正向
DIRECTION_RIGHT = -1    # 偏右
DIRECTION_UNKNOWN = 2  # 未知/无效

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

class RoadDetector:
    """道路检测算法"""
    def __init__(self):
        # HSV阈值参数
        self.lower_yellow = np.array([20, 100, 100])  # 黄色下限
        self.upper_yellow = np.array([30, 255, 255])  # 黄色上限
        
        # 形态学处理核
        self.kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        
        # 霍夫线参数
        self.hough_threshold = 50      # 最小投票数
        self.min_line_length = 80      # 最小线段长度 (增加长度要求，提高稳定性)
        self.max_line_gap = 15         # 最大线段间隙 (略微增加允许的间隙)
        
        # 角度滤波器
        self.angle_filter = AngleFilter(window_size=15)  # 增加滤波窗口大小
        
        # 方向判断阈值
        self.forward_threshold = 2.0  # 正向阈值（小于此角度视为正向）
        
        # 多路径选择参数
        self.prefer_center_weight = 0.8  # 中心区域权重 (0-1) (增加中心区域权重)
        self.prefer_width_weight = 0.2   # 宽度权重 (0-1)
        
        # 是否显示所有检测到的路径
        self.show_all_paths = False
        
        # 安全斜率阈值（避免除零错误）
        self.min_slope = 0.001
        
        # 角度变化限制 (新增参数，限制相邻帧之间角度变化幅度)
        self.max_angle_change = 3.0  # 度
        self.last_angle = None  # 记录上一帧的角度

    def process_image(self, image):
        """
        处理图像，检测轨道方向
        返回: (角度, 方向枚举, 调试图像, 掩码图像)
        """
        if image is None:
            return None, DIRECTION_UNKNOWN, None, None
            
        # 预处理
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
        
        # 保存中间结果（用于调试）
        mask_color = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        
        # 霍夫直线检测
        lines = cv2.HoughLinesP(roi_mask, 1, np.pi/180, self.hough_threshold, 
                              minLineLength=self.min_line_length, 
                              maxLineGap=self.max_line_gap)
        
        if lines is None or len(lines) < 2:
            # 未检测到足够的轨道线
            return None, DIRECTION_UNKNOWN, debug_img, mask_color
            
        # 分类左右轨道线
        left_lines = []
        right_lines = []
        center_x = w // 2
        
        for line in lines:
            x1, y1, x2, y2 = line[0]
            # 调整坐标（考虑ROI偏移）
            y1 += h//2
            y2 += h//2
            
            # 计算线段中点
            mid_x = (x1 + x2) // 2
            
            # 绘制所有检测到的线（可选）
            if self.show_all_paths:
                cv2.line(debug_img, (x1, y1), (x2, y2), (0, 0, 255), 2)
            
            # 计算线段斜率，过滤垂直或接近垂直的线
            if x2 - x1 == 0:
                continue  # 跳过垂直线
                
            slope = (y2 - y1) / (x2 - x1)
            # 过滤掉斜率过大的线段（几乎垂直的线可能是噪声）
            if abs(slope) > 5.0:  # 大约等于约80度
                continue
                
            # 分类左右轨道线
            if mid_x < center_x:
                left_lines.append(line[0])
            else:
                right_lines.append(line[0])
        
        # 如果检测到多组轨道，选择最优的一组
        if len(left_lines) > 0 and len(right_lines) > 0:
            left_lines, right_lines = self.select_best_path(left_lines, right_lines, center_x, w)
        
        # 计算左右轨道线的平均斜率和截距
        left_params, right_params = self.get_average_lines(left_lines, right_lines, h)
        
        if left_params is None or right_params is None:
            # 无法计算有效轨道线
            return None, DIRECTION_UNKNOWN, debug_img, mask_color
        
        try:   
            # 绘制最终选择的轨道线
            self.draw_lines(debug_img, left_params, right_params, h)
            
            # 计算中心线参数
            center_slope = (left_params[0] + right_params[0]) / 2
            center_intercept = (left_params[1] + right_params[1]) / 2
            
            # 计算角度（弧度转度）
            angle = np.arctan(center_slope) * 180 / np.pi
            
            # 应用角度变化限制
            if self.last_angle is not None:
                # 限制单帧角度变化
                angle_diff = angle - self.last_angle
                if abs(angle_diff) > self.max_angle_change:
                    # 如果变化太大，限制变化量
                    sign = 1 if angle_diff > 0 else -1
                    angle = self.last_angle + sign * self.max_angle_change
            
            # 应用滤波
            filtered_angle = self.angle_filter.update(angle)
            
            # 更新上一帧角度
            self.last_angle = filtered_angle
            
            # 判断方向枚举值
            direction = self.determine_direction(filtered_angle)
            
            # 判断是否正向（向前）
            is_forward = direction == DIRECTION_FORWARD
            
            # 绘制中心线
            self.draw_center_line(debug_img, center_slope, center_intercept, h, is_forward)
            
            # 显示角度信息
            cv2.putText(debug_img, f"Angle: {filtered_angle:.1f}", (10, 30), 
                      cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            # 显示方向信息
            direction_text = self.get_direction_text(direction)
            cv2.putText(debug_img, f"Direction: {direction_text}", (10, 70), 
                      cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        except Exception as e:
            # 如果绘图过程中出错，记录错误并返回空结果
            print(f"处理图像时出错: {str(e)}")
            return None, DIRECTION_UNKNOWN, debug_img, mask_color
            
        return filtered_angle, direction, debug_img, mask_color
    
    def select_best_path(self, left_lines, right_lines, center_x, image_width):
        """
        当检测到多条可能路径时，选择最优路径
        使用启发式规则：
        1. 优先选择靠近图像中心的路径
        2. 优先选择宽度合适的路径
        """
        # 如果只有一条路径，直接返回
        if len(left_lines) <= 1 or len(right_lines) <= 1:
            return left_lines, right_lines
            
        # 生成所有可能的左右线组合
        path_candidates = []
        
        for left in left_lines:
            for right in right_lines:
                # 计算左右线中点
                left_x1, left_y1, left_x2, left_y2 = left
                right_x1, right_y1, right_x2, right_y2 = right
                
                # 计算底部点(y较大的点)
                left_bottom_x = left_x1 if left_y1 >= left_y2 else left_x2
                right_bottom_x = right_x1 if right_y1 >= right_y2 else right_x2
                
                # 确保左线在右线左侧
                if left_bottom_x >= right_bottom_x:
                    continue
                
                # 计算路径中心点
                path_center_x = (left_bottom_x + right_bottom_x) / 2
                
                # 计算路径宽度
                path_width = right_bottom_x - left_bottom_x
                
                # 路径中心到图像中心的距离（归一化）
                center_dist = abs(path_center_x - center_x) / (image_width / 2)
                center_score = 1.0 - center_dist  # 越靠近中心，分数越高
                
                # 路径宽度得分（归一化，假设理想宽度为图像宽度的1/3）
                ideal_width = image_width / 3
                width_score = 1.0 - min(abs(path_width - ideal_width) / ideal_width, 1.0)
                
                # 综合评分
                score = (self.prefer_center_weight * center_score + 
                         self.prefer_width_weight * width_score)
                
                # 添加到候选
                path_candidates.append({
                    'left': left,
                    'right': right,
                    'score': score,
                    'center_x': path_center_x,
                    'width': path_width
                })
        
        # 如果没有有效路径，返回原始线段
        if not path_candidates:
            return left_lines, right_lines
            
        # 按分数排序
        path_candidates.sort(key=lambda x: x['score'], reverse=True)
        
        # 选择分数最高的路径
        best_path = path_candidates[0]
        return [best_path['left']], [best_path['right']]

    def determine_direction(self, angle):
        """根据角度确定方向枚举值"""
        if angle is None:
            return DIRECTION_UNKNOWN
        
        # 使用更加平滑的方向判断
        if abs(angle) <= self.forward_threshold:
            return DIRECTION_FORWARD
        elif angle < -self.forward_threshold:
            # 角度为负，需要向左修正
            # 角度越大，向左修正的程度越大
            if angle < -8.0:  # 特别大的角度可能是异常值
                return DIRECTION_UNKNOWN
            return DIRECTION_LEFT
        else:
            # 角度为正，需要向右修正
            # 角度越大，向右修正的程度越大
            if angle > 8.0:  # 特别大的角度可能是异常值
                return DIRECTION_UNKNOWN
            return DIRECTION_RIGHT
    
    def get_direction_text(self, direction):
        """获取方向的文本描述"""
        if direction == DIRECTION_FORWARD:
            return "Forward"
        elif direction == DIRECTION_LEFT:
            return "Turn Left"
        elif direction == DIRECTION_RIGHT:
            return "Turn Right"
        else:
            return "Unknown"

    def get_average_lines(self, left_lines, right_lines, img_height):
        """
        计算左右轨道线的平均斜率和截距
        """
        # 检查是否有足够的线段
        if len(left_lines) == 0 or len(right_lines) == 0:
            return None, None
            
        # 计算左侧线段的平均斜率和截距
        left_slopes = []
        left_intercepts = []
        
        for line in left_lines:
            x1, y1, x2, y2 = line
            
            # 避免除零错误
            if x2 - x1 == 0:
                continue
                
            slope = (y2 - y1) / (x2 - x1)
            
            # 过滤异常斜率
            if abs(slope) > 5.0:  # 斜率太大，几乎垂直
                continue
                
            intercept = y1 - slope * x1
            
            left_slopes.append(slope)
            left_intercepts.append(intercept)
            
        # 计算右侧线段的平均斜率和截距
        right_slopes = []
        right_intercepts = []
        
        for line in right_lines:
            x1, y1, x2, y2 = line
            
            # 避免除零错误
            if x2 - x1 == 0:
                continue
                
            slope = (y2 - y1) / (x2 - x1)
            
            # 过滤异常斜率
            if abs(slope) > 5.0:  # 斜率太大，几乎垂直
                continue
                
            intercept = y1 - slope * x1
            
            right_slopes.append(slope)
            right_intercepts.append(intercept)
            
        # 检查是否存在有效的斜率和截距
        if not left_slopes or not right_slopes:
            return None, None
        
        # 过滤异常值
        def remove_outliers(values):
            if len(values) <= 2:
                return values
            # 计算四分位数
            q1, q3 = np.percentile(values, [25, 75])
            # 计算IQR（四分位数间距）
            iqr = q3 - q1
            # 定义异常值界限
            lower_bound = q1 - 1.5 * iqr
            upper_bound = q3 + 1.5 * iqr
            # 过滤异常值
            return [x for x in values if lower_bound <= x <= upper_bound]
        
        # 应用异常值过滤
        if len(left_slopes) > 2:
            left_slopes = remove_outliers(left_slopes)
        if len(left_intercepts) > 2:
            left_intercepts = remove_outliers(left_intercepts)
        if len(right_slopes) > 2:
            right_slopes = remove_outliers(right_slopes)
        if len(right_intercepts) > 2:
            right_intercepts = remove_outliers(right_intercepts)
            
        # 检查过滤后是否还有足够的数据
        if not left_slopes or not right_slopes:
            return None, None
            
        # 计算平均值
        left_avg_slope = np.median(left_slopes)  # 使用中位数代替平均值，更鲁棒
        left_avg_intercept = np.median(left_intercepts)
        
        right_avg_slope = np.median(right_slopes)
        right_avg_intercept = np.median(right_intercepts)
        
        # 确保斜率不为零（避免除零错误）
        if abs(left_avg_slope) < self.min_slope:
            left_avg_slope = self.min_slope if left_avg_slope >= 0 else -self.min_slope
        
        if abs(right_avg_slope) < self.min_slope:
            right_avg_slope = self.min_slope if right_avg_slope >= 0 else -self.min_slope
        
        return (left_avg_slope, left_avg_intercept), (right_avg_slope, right_avg_intercept)

    def draw_lines(self, img, left_params, right_params, img_height):
        """
        绘制左右轨道线
        """
        left_slope, left_intercept = left_params
        right_slope, right_intercept = right_params
        
        # 计算两点坐标，以绘制线段
        y1 = img_height
        y2 = int(img_height * 0.6)
        
        # 确保斜率不为零（避免除零错误）
        if abs(left_slope) < self.min_slope:
            left_slope = self.min_slope if left_slope >= 0 else -self.min_slope
        
        if abs(right_slope) < self.min_slope:
            right_slope = self.min_slope if right_slope >= 0 else -self.min_slope
        
        # 左侧线
        x1_left = int((y1 - left_intercept) / left_slope)
        x2_left = int((y2 - left_intercept) / left_slope)
        
        # 右侧线
        x1_right = int((y1 - right_intercept) / right_slope)
        x2_right = int((y2 - right_intercept) / right_slope)
        
        # 限制坐标在图像范围内
        h, w = img.shape[:2]
        x1_left = max(0, min(w-1, x1_left))
        x2_left = max(0, min(w-1, x2_left))
        x1_right = max(0, min(w-1, x1_right))
        x2_right = max(0, min(w-1, x2_right))
        
        # 绘制线段
        cv2.line(img, (x1_left, y1), (x2_left, y2), (255, 0, 0), 2)
        cv2.line(img, (x1_right, y1), (x2_right, y2), (255, 0, 0), 2)

    def draw_center_line(self, img, slope, intercept, img_height, is_forward):
        """
        绘制中心线
        """
        # 计算两点坐标
        y1 = img_height
        y2 = int(img_height * 0.6)
        
        # 确保斜率不为零（避免除零错误）
        if abs(slope) < self.min_slope:
            slope = self.min_slope if slope >= 0 else -self.min_slope
        
        # 计算x坐标
        x1 = int((y1 - intercept) / slope)
        x2 = int((y2 - intercept) / slope)
        
        # 限制坐标在图像范围内
        h, w = img.shape[:2]
        x1 = max(0, min(w-1, x1))
        x2 = max(0, min(w-1, x2))
        
        # 根据是否正向选择颜色
        color = (0, 255, 0) if is_forward else (0, 0, 255)
        
        # 绘制中心线
        cv2.line(img, (x1, y1), (x2, y2), color, 3)
        
        # 绘制参考线（垂直中线）
        cv2.line(img, (w//2, 0), (w//2, h), (0, 255, 255), 1) 