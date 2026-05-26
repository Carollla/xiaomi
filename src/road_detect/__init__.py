"""
轨道检测包
提供用于机器狗轨道线检测的功能
"""

# 从API模块导出主要接口
from .road_detector_api import detect_road, cleanup

# 从基础模块导出所有方向常量
from .road_detector_base import (
    DIRECTION_FORWARD, 
    DIRECTION_LEFT, 
    DIRECTION_RIGHT, 
    DIRECTION_UNKNOWN,
    DIRECTION_SLIGHT_LEFT,
    DIRECTION_SLIGHT_RIGHT,
    DIRECTION_SHARP_LEFT,
    DIRECTION_SHARP_RIGHT,
    DIRECTION_S_CURVE
)

# 导出各种检测器类
from .slight_adjustment_detector import SlightAdjustmentDetector
from .sharp_turn_detector import SharpTurnDetector
from .s_curve_detector import SCurveDetector

# 版本信息
__version__ = '1.1.0' 