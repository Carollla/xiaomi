#!/usr/bin/env python3
"""
处理旋转与方向判断的工具模块
基于IMU数据实现方向判断、旋转计算等功能
使用"度"作为角度单位
"""
import time
from imu_provider import IMUProvider

def get_current_yaw(imu_provider):
    """
    获取当前偏航角(度)
    """
    if not imu_provider:
        print("WARNING: IMU提供器未初始化，无法获取偏航角")
        return 0.0
    return imu_provider.get_yaw()

def normalize_angle(angle):
    """
    将角度标准化到0-360范围
    """
    angle = angle % 360
    if angle < 0:
        angle += 360
    return angle

def is_facing_cardinal_direction(imu_provider, direction, tolerance_deg=1.0):
    """
    判断是否面向基本方向（东、南、西、北）
    direction: 目标方向，可选值："东"、"南"、"西"、"北"
    tolerance_deg: 容差角度，默认5度
    返回: (是否面向该方向, 当前角度, 与目标角度的差值)
    """
    current_yaw = normalize_angle(get_current_yaw(imu_provider))
    direction_angles = {
        "北": 0.0,
        "东": 90.0,
        "南": 180.0,
        "西": 270.0
    }
    if direction not in direction_angles:
        return False, current_yaw, 0.0
    target_angle = direction_angles[direction]
    # 计算角度差
    angle_diff = abs(current_yaw - target_angle)
    if angle_diff > 180:
        angle_diff = 360 - angle_diff
    return angle_diff <= tolerance_deg, current_yaw, angle_diff

def print_orientation(imu_provider, logger=None):
    """
    打印当前朝向信息
    """
    yaw_deg = imu_provider.get_yaw()
    pitch_deg = imu_provider.get_pitch()
    directions = ["东", "南", "西", "北"]
    for direction in directions:
        is_facing, current_angle, angle_diff = is_facing_cardinal_direction(imu_provider, direction)
    print(f"当前朝向: Pitch={pitch_deg:.2f}° Yaw={yaw_deg:.2f}°")
    print(f"是否正对{direction}方向: {is_facing}，角度差{angle_diff:.2f}°，当前角度{current_angle:.2f}°")

def main():
    print("启动旋转与方向判断工具...")
    
    try:
        # 创建IMU提供器
        imu_provider = IMUProvider()
        print("创建IMU提供器完成，等待数据初始化...")
        time.sleep(2.0)
        running = True
        
        try:
            while running:
                print("\n==== 方向数据更新 ====")
                print_orientation(imu_provider)
                print("======================")
                time.sleep(2.0)
        except KeyboardInterrupt:
            print("程序被用户中断")
        
    except Exception as e:
        print(f"出现异常: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if 'imu_provider' in locals():
            imu_provider.shutdown()
        print("程序已退出")

if __name__ == '__main__':
    main()
