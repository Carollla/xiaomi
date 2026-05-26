#!/usr/bin/env python3
"""
基于IMU数据实现方向判断、旋转计算等功能
使用"度"作为角度单位
"""
def get_current_yaw(imu_provider):
    """获取当前偏航角(度)，返回标准化后的角度(0-360)"""
    if not imu_provider:
        print("WARNING: IMU提供器未初始化，无法获取偏航角")
        return 0.0
    return _normalize_angle(imu_provider.get_yaw())

def is_facing_cardinal_direction(imu_provider, target_angle, tolerance_deg=0.5):
    """返回: (是否面向该角度, 当前角度, 与目标角度的差值)，用来微调转弯的角度，针对初始位置顺时针方向考虑
    Args:
        imu_provider: IMU数据提供者
        target_angle: 目标角度值(0-360)
        tolerance_deg: 允许的角度误差范围
    """
    current_yaw = get_current_yaw(imu_provider)
    # 计算角度差
    angle_diff = abs(current_yaw - target_angle)
    if angle_diff > 180:
        angle_diff = 360 - angle_diff
    return angle_diff <= tolerance_deg, current_yaw, angle_diff

def get_adjustment_direction(imu_provider, target_angle, tolerance_deg=0.2):
    """根据当前朝向和目标角度，返回调整方向
    Args:
        imu_provider: IMU数据提供者
        target_angle: 目标角度值(0-360)
    Returns:
        tuple: (是否需要调整, 调整方向, 角度差)
            - 是否需要调整: bool
            - 调整方向: str ('left'/'right')
            - 角度差: float
    """
    current_angle = get_current_yaw(imu_provider)
    angle_diff = (current_angle - target_angle + 360) % 360
    if angle_diff > 180:
        angle_diff = angle_diff - 360
        
    if abs(angle_diff) <= tolerance_deg:
        return False, None, angle_diff
    if angle_diff > 0:  # 偏右，需要向左调整
        return True, 'left', angle_diff
    else:  # 偏左，需要向右调整
        return True, 'right', angle_diff

def _normalize_angle(angle):
    """将角度标准化到0-360范围"""
    angle = angle % 360
    if angle < 0:
        angle += 360
    return angle
