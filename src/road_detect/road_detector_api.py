#!/usr/bin/env python3
"""
道路检测API模块
整合角度检测和ROS图像，提供简单接口
"""
import cv2
import time

# 导入模块
from angle_detector_core import RoadDetector, DIRECTION_FORWARD, DIRECTION_LEFT, DIRECTION_RIGHT, DIRECTION_UNKNOWN
from ros_image_provider import RoadImageProvider

# 单例模式全局对象
_image_provider = None
_detector = None

def get_image_provider():
    """获取图像提供者单例"""
    global _image_provider
    if _image_provider is None:
        _image_provider = RoadImageProvider()
    return _image_provider

def get_road_detector():
    """获取道路检测器单例"""
    global _detector
    if _detector is None:
        _detector = RoadDetector()
    return _detector

def detect_road(image=None):
    """
    检测图像中的轨道线
    参数:
        image: 可选，输入图像。如果为None则自动获取最新图像
    返回:
        (角度, 方向枚举, 调试图像, 掩码图像)
    """
    if image is None:
        # 获取最新图像
        provider = get_image_provider()
        image = provider.get_latest_image()
        if image is None:
            return None, DIRECTION_UNKNOWN, None, None
    
    # 获取检测器并处理图像
    detector = get_road_detector()
    return detector.process_image(image)

def cleanup():
    """清理资源"""
    global _image_provider
    if _image_provider is not None:
        _image_provider.shutdown()
        _image_provider = None

# 示例使用代码
if __name__ == "__main__":
    try:
        print("道路检测库演示")
        print("按 'q' 键退出")
        
        # 循环获取和处理图像
        while True:
            # 获取并处理图像
            angle, direction, debug_img, mask = detect_road()
            
            # 检查是否成功获取和处理
            if debug_img is None:
                print("等待图像数据...")
                time.sleep(0.5)
                continue
                
            # 显示结果
            if angle is not None:
                direction_text = "未知"
                if direction == DIRECTION_FORWARD:
                    direction_text = "直行"
                elif direction == DIRECTION_LEFT:
                    direction_text = "向左转"
                elif direction == DIRECTION_RIGHT:
                    direction_text = "向右转"
                    
                print(f"轨道角度: {angle:.1f}度, 方向: {direction_text}")
                
            # 显示图像
            cv2.imshow('掩码', mask)
            cv2.imshow('检测结果', debug_img)
            
            # 按 'q' 键退出
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
    except KeyboardInterrupt:
        print("程序已中断")
    finally:
        # 清理资源
        cleanup()
        cv2.destroyAllWindows() 