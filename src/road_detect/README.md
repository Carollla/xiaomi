# 轨道检测库

该库提供了一系列轨道检测和角度计算功能，专为机器人导航设计。

## 主要功能

- 轨道线检测和角度计算
- 多种路径场景分类
- 轨道方向和转弯类型识别
- 可视化调试支持

## 快速入门

### 安装依赖

```bash
pip install numpy opencv-python
```

### 基本用法

最简单的使用方式是通过API接口获取轨道信息：

```python
from road_detect import detect_road, cleanup

try:
    # 获取轨道信息
    angle, direction, debug_img, mask = detect_road()
    
    # 使用结果
    if angle is not None:
        print(f"轨道角度: {angle}度, 方向代码: {direction}")
finally:
    # 清理资源
    cleanup()
```

## 高级用法

### 1. 微调角度检测器

适用于直线和轻微弯道场景，可检测小角度偏移：

```python
from road_detect import SlightAdjustmentDetector
from road_detect import DIRECTION_SLIGHT_LEFT, DIRECTION_SLIGHT_RIGHT

# 创建微调检测器
detector = SlightAdjustmentDetector()

# 自定义参数
detector.slight_threshold = 10.0  # 微调角度阈值
detector.forward_threshold = 2.0  # 直行阈值

# 处理图像
angle, direction, direction_text, debug_img, mask = detector.process_image(image)

# 判断微调方向
if direction == DIRECTION_SLIGHT_LEFT:
    # 微调左转控制
    pass
elif direction == DIRECTION_SLIGHT_RIGHT:
    # 微调右转控制
    pass
```

### 2. 大转弯检测器

适用于大角度转弯和直角弯道：

```python
from road_detect import SharpTurnDetector
from road_detect import DIRECTION_SHARP_LEFT, DIRECTION_SHARP_RIGHT

# 创建大转弯检测器
detector = SharpTurnDetector()

# 自定义参数
detector.sharp_threshold = 30.0     # 大转弯阈值
detector.critical_threshold = 60.0  # 直角转弯阈值

# 特殊功能参数
detector.corner_detection_enabled = True  # 启用角点检测
detector.corner_detection_quality = 0.01  # 角点检测质量

# 处理图像
angle, direction, direction_text, debug_img, mask = detector.process_image(image)

# 判断转弯类型
if direction == DIRECTION_SHARP_LEFT:
    # 大角度左转控制
    pass
elif direction == DIRECTION_SHARP_RIGHT:
    # 大角度右转控制
    pass
```

### 3. S弯检测器

适用于识别复杂的S型弯道和多路径选择：

```python
from road_detect import SCurveDetector
from road_detect import DIRECTION_S_CURVE

# 创建S弯检测器
detector = SCurveDetector()

# 自定义参数
detector.s_curve_angle_diff = 25.0    # S弯道角度差异阈值
detector.max_paths = 3                # 最多处理的路径数量

# 处理图像
angle, direction, direction_text, debug_img, mask = detector.process_image(image)

# 判断是否为S弯
if direction == DIRECTION_S_CURVE:
    # S弯道专用控制
    # 可以通过detector.s_curve_direction获取更多信息
    s_direction = detector.s_curve_direction  # -1:先左后右, 1:先右后左
    pass
```

## 运行示例

使用提供的示例代码测试功能：

```bash
# 使用默认自动模式
python example_usage.py

# 使用微调角度检测器
python example_usage.py --detector slight

# 使用大转弯检测器
python example_usage.py --detector sharp

# 使用S弯检测器
python example_usage.py --detector scurve

# 使用视频文件测试
python example_usage.py --mode video --detector sharp --video test_video.mp4
```

## 方向常量

库提供以下方向枚举常量：

- `DIRECTION_UNKNOWN = 0`: 未知/无效
- `DIRECTION_FORWARD = 1`: 直行前进
- `DIRECTION_LEFT = 2`: 左转
- `DIRECTION_RIGHT = 3`: 右转
- `DIRECTION_SLIGHT_LEFT = 10`: 微调左转
- `DIRECTION_SLIGHT_RIGHT = 11`: 微调右转
- `DIRECTION_SHARP_LEFT = 20`: 大角度左转
- `DIRECTION_SHARP_RIGHT = 21`: 大角度右转
- `DIRECTION_S_CURVE = 30`: S弯道

## 调试与可视化

所有检测器都提供可视化调试功能：

```python
# 启用/禁用特定可视化
detector.show_all_paths = True      # 显示所有检测到的路径
detector.show_contours = True       # 显示轮廓

# 特定检测器的调试选项
slight_detector.show_lateral_error = True   # 显示横向误差
sharp_detector.show_corners = True          # 显示检测到的角点
sharp_detector.show_curvature = True        # 显示路径曲率
s_detector.show_path_connections = True     # 显示路径连接
s_detector.show_s_curve_indicators = True   # 显示S弯指示器
``` 