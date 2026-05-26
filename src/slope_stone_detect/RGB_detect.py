import cv2
import numpy as np
from RGB_provider import RoadImageProvider

def get_yellow_mask(image):
    """
    识别黄色区域，返回掩码
    颜色范围可根据实际图片微调
    """
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # 黄色常用HSV范围 H: 20-40, S: 80-255, V: 80-255
    lower = np.array([20, 80, 80])
    upper = np.array([40, 255, 255])
    mask = cv2.inRange(hsv, lower, upper)
    return mask

def detect_yellow(image, x_start=0, x_end=1.0, y_start=0.7, y_end=1.0, yellow_thresh=0.01):
    """
    判断指定区域是否没有黄色。
    Args:
        image: 输入图像
        x_start: x轴起始位置比例（0-1之间）
        x_end: x轴结束位置比例（0-1之间）
        y_start: y轴起始位置比例（0-1之间）
        y_end: y轴结束位置比例（0-1之间）
        yellow_thresh: 黄色像素占比阈值，低于此值认为没有黄色
    Returns:
        bool: True表示指定区域有黄色，False表示没有黄色
    """
    mask = get_yellow_mask(image)
    h, w = mask.shape
    x0 = int(w * x_start)
    x1 = int(w * x_end)
    y0 = int(h * y_start)
    y1 = int(h * y_end)
    roi = mask[y0:y1, x0:x1]
    yellow_pixels = np.count_nonzero(roi)
    total_pixels = roi.size
    ratio = yellow_pixels / total_pixels if total_pixels > 0 else 0
    # 调试信息
    print(f"区域({x_start:.1f}-{x_end:.1f}, {y_start:.1f}-{y_end:.1f})黄色像素占比: {ratio:.4f}")
    return ratio > yellow_thresh

def preprocess_arrow(img):
    """预处理图像用于箭头检测"""
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower_green = np.array([35, 43, 46])
    upper_green = np.array([77, 255, 255])
    mask = cv2.inRange(hsv, lower_green, upper_green)
    green_img = cv2.bitwise_and(img, img, mask=mask)
    gray = cv2.cvtColor(green_img, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 1)
    _, thresh = cv2.threshold(blurred, 50, 255, cv2.THRESH_BINARY)
    kernel = np.ones((3, 3), np.uint8)
    dilated = cv2.dilate(thresh, kernel, iterations=2)
    eroded = cv2.erode(dilated, kernel, iterations=1)
    return eroded

def find_arrow_tip(points, convex_hull):
    """找到箭头的尖端"""
    length = len(points)
    indices = np.setdiff1d(range(length), convex_hull)
    if len(indices) != 2:
        return None
    tip_index = indices[0] if points[indices[0], 1] < points[indices[1], 1] else indices[1]
    return points[tip_index]

def detect_arrow(img):
    """
    检测图像中的绿色箭头
    返回: (是否检测到箭头, 箭头方向, 箭头轮廓, 箭头尖端)
    """
    processed_image = preprocess_arrow(img)
    contours, hierarchy = cv2.findContours(processed_image, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

    for cnt in contours:
        peri = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, 0.025 * peri, True)
        hull = cv2.convexHull(approx, returnPoints=False)
        sides = len(hull)
        if 6 > sides > 3 and sides + 2 == len(approx):
            arrow_tip = find_arrow_tip(approx[:, 0, :], hull.squeeze())
            if arrow_tip is not None:
                arrow_dir = np.array(arrow_tip) - np.array(approx.mean(axis=0)[0])
                arrow_direction = "Right" if arrow_dir[0] > 0 else "Left"
                print(f"箭头方向: {arrow_direction}")
                cv2.drawContours(img, [approx], -1, (0, 255, 0), 3)
                cv2.circle(img, tuple(arrow_tip), 5, (0, 0, 255), -1)
                return True, arrow_direction, approx, arrow_tip
    return False, None, None, None

def calculate_curvature(points):
    """
    计算线段的曲率
    Args:
        points: 线段上的点集
    Returns:
        float: 曲率值，值越大表示越弯曲
    """
    if len(points) < 3:
        return 0
    
    # 计算相邻点之间的角度变化
    angles = []
    for i in range(len(points)-2):
        v1 = points[i+1] - points[i]
        v2 = points[i+2] - points[i+1]
        angle = np.arccos(np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2)))
        angles.append(angle)
    
    return np.mean(angles) if angles else 0

def detect_yellow_line(image, x_start=0, x_end=1.0, y_start=0.7, y_end=1.0, min_line_length=100):
    """
    检测指定区域内的黄色曲线/直线
    Args:
        image: 输入图像
        x_start: x轴起始位置比例（0-1之间）
        x_end: x轴结束位置比例（0-1之间）
        y_start: y轴起始位置比例（0-1之间）
        y_end: y轴结束位置比例（0-1之间）
        min_line_length: 最小线段长度
    Returns:
        tuple: (是否检测到线, 直线列表, 曲线列表, 最下面的线)
    """
    # 获取黄色掩码
    mask = get_yellow_mask(image)
    h, w = mask.shape
    
    # 计算ROI区域
    x0 = int(w * x_start)
    x1 = int(w * x_end)
    y0 = int(h * y_start)
    y1 = int(h * y_end)
    roi = mask[y0:y1, x0:x1]
    
    # 形态学操作，连接相近的黄色区域
    kernel = np.ones((5,5), np.uint8)
    roi = cv2.morphologyEx(roi, cv2.MORPH_CLOSE, kernel)
    
    # 查找轮廓
    contours, _ = cv2.findContours(roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    straight_lines = []
    curved_lines = []
    bottom_line = None
    max_y = -1
    
    for cnt in contours:
        # 过滤太小的轮廓
        if cv2.arcLength(cnt, False) < min_line_length:
            continue
            
        # 获取轮廓的边界框
        x, y, w, h = cv2.boundingRect(cnt)
        if w < 10:  # 过滤太窄的轮廓
            continue
            
        # 将轮廓点转换回原图坐标
        cnt = cnt + np.array([x0, y0])
        
        # 使用最小二乘法拟合直线
        points = cnt.reshape(-1, 2)
        x = points[:, 0]
        y = points[:, 1]
        
        # 计算直线拟合的误差
        if len(x) > 1:
            z = np.polyfit(x, y, 1)
            p = np.poly1d(z)
            y_fit = p(x)
            error = np.mean(np.abs(y - y_fit))
            
            # 计算轮廓的曲率
            peri = cv2.arcLength(cnt, False)
            approx = cv2.approxPolyDP(cnt, 0.02 * peri, False)
            hull = cv2.convexHull(approx, returnPoints=False)
            defects = cv2.convexityDefects(approx, hull) if hull is not None else None
            
            # 判断是直线还是曲线
            is_straight = error < 5 and (defects is None or len(defects) < 3)
            
            # 获取轮廓的起点和终点
            start_point = tuple(cnt[0][0])
            end_point = tuple(cnt[-1][0])
            
            if is_straight:
                straight_lines.append((start_point[0], start_point[1], end_point[0], end_point[1]))
                cv2.line(image, start_point, end_point, (0, 255, 0), 2)
            else:
                curved_lines.append((start_point[0], start_point[1], end_point[0], end_point[1]))
                cv2.polylines(image, [cnt], False, (0, 0, 255), 2)
            
            # 更新最下面的线
            current_max_y = max(start_point[1], end_point[1])
            if current_max_y > max_y:
                max_y = current_max_y
                bottom_line = (start_point[0], start_point[1], end_point[0], end_point[1])
    
    # 特别标记最下面的线
    if bottom_line:
        x1, y1, x2, y2 = bottom_line
        cv2.line(image, (x1, y1), (x2, y2), (255, 0, 0), 3)  # 蓝色标记
    
    return len(straight_lines) + len(curved_lines) > 0, straight_lines, curved_lines, bottom_line

def main():
    provider = RoadImageProvider()
    print("等待图像数据... 按q退出")
    while True:
        img = provider.get_latest_image()
        if img is None:
            continue
            
        # 检测黄色线
        has_line, straight_lines, curved_lines, bottom_line = detect_yellow_line(img, 0, 1, 0.9, 1)
        if has_line:
            print(f"检测到直线{len(straight_lines)}条，曲线{len(curved_lines)}条")
            if bottom_line:
                x1, y1, x2, y2 = bottom_line
                print(f"最下面的线: 从({x1},{y1})到({x2},{y2})")
            
        # 显示原图
        cv2.imshow('检测结果', img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    provider.shutdown()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
