import cv2
import numpy as np
# ------------------- 检测黄色 -------------------#
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

# ------------------- 检测灰绿色 -------------------#
def get_greenish_mask(image):
    """
    识别灰绿色主色区域，返回掩码
    颜色范围可根据实际图片微调
    """
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # 经验范围，主色大致在H: 80-100, S: 20-60, V: 70-120
    lower = np.array([80, 20, 70])
    upper = np.array([100, 60, 130])
    mask = cv2.inRange(hsv, lower, upper)
    return mask

def slope_move_right_end(image, region_ratio=2/3, green_thresh=0.01):
    """
    判断图像右侧2/3区域是否无灰绿色。
    region_ratio: 右侧区域占宽的比例（默认2/3）
    green_thresh: 灰绿色像素占比阈值，低于此值认为没有灰绿色
    返回: True表示右侧2/3无灰绿色，False表示有灰绿色
    """
    mask = get_greenish_mask(image)
    h, w = mask.shape
    x0 = int(w * (1 - region_ratio))
    roi = mask[:, x0:]
    green_pixels = np.count_nonzero(roi)
    total_pixels = roi.size
    ratio = green_pixels / total_pixels if total_pixels > 0 else 0
    print(f"右侧2/3灰绿色像素占比: {ratio:.4f}")
    return ratio < green_thresh

# ------------------- 检测绿色箭头 -------------------#
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