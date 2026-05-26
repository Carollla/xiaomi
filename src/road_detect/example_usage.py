#!/usr/bin/env python3
"""
道路检测库使用示例
演示如何集成道路检测库到您的应用程序
"""
import time
import cv2
import os
import argparse

# 导入道路检测库API和方向常量
from road_detector_api import detect_road, cleanup
from road_detector_base import (
    DIRECTION_FORWARD, DIRECTION_LEFT, DIRECTION_RIGHT, DIRECTION_UNKNOWN,
    DIRECTION_SLIGHT_LEFT, DIRECTION_SLIGHT_RIGHT,
    DIRECTION_SHARP_LEFT, DIRECTION_SHARP_RIGHT,
    DIRECTION_S_CURVE
)

# 导入特定检测器
from slight_adjustment_detector import SlightAdjustmentDetector
from sharp_turn_detector import SharpTurnDetector
from s_curve_detector import SCurveDetector

def get_direction_description(direction):
    """获取方向的详细描述"""
    if direction == DIRECTION_FORWARD:
        return "直行"
    elif direction == DIRECTION_LEFT:
        return "向左转"
    elif direction == DIRECTION_RIGHT:
        return "向右转"
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
    else:  # DIRECTION_UNKNOWN
        return "未知方向"

def get_control_params(direction, angle=None):
    """根据方向和角度获取控制参数"""
    # 默认参数
    speed = 0.0
    turn_rate = 0.0
    
    # 根据方向调整参数
    if direction == DIRECTION_FORWARD:
        speed = 0.5
        turn_rate = 0.0
    elif direction == DIRECTION_LEFT:
        speed = 0.3
        turn_rate = -0.2
    elif direction == DIRECTION_RIGHT:
        speed = 0.3
        turn_rate = 0.2
    elif direction == DIRECTION_SLIGHT_LEFT:
        speed = 0.4
        turn_rate = -0.1
    elif direction == DIRECTION_SLIGHT_RIGHT:
        speed = 0.4
        turn_rate = 0.1
    elif direction == DIRECTION_SHARP_LEFT:
        speed = 0.2
        turn_rate = -0.3
    elif direction == DIRECTION_SHARP_RIGHT:
        speed = 0.2
        turn_rate = 0.3
    elif direction == DIRECTION_S_CURVE:
        speed = 0.2
        turn_rate = -0.15  # 初始转向，实际应基于S弯情况动态调整
    
    # 如果有角度信息，可以用于微调转向率
    if angle is not None:
        # 可以根据角度大小进一步调整转向率
        pass
        
    return speed, turn_rate

def test_video_file(detector, video_path):
    """测试视频文件"""
    if not os.path.exists(video_path):
        print(f"错误：视频文件 {video_path} 不存在")
        return
        
    # 打开视频文件
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print(f"错误：无法打开视频文件 {video_path}")
        return
    
    # 图像显示配置
    cv2.namedWindow('检测结果', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('检测结果', 800, 600)
    
    # 帧计数和时间测量
    frame_count = 0
    total_time = 0
    
    try:
        while True:
            # 读取一帧
            ret, frame = cap.read()
            if not ret:
                print("视频播放结束")
                break
                
            # 处理图像
            start_time = time.time()
            angle, direction, direction_text, debug_img, mask = detector.process_image(frame)
            process_time = time.time() - start_time
            
            total_time += process_time
            frame_count += 1
            
            # 显示处理结果
            if angle is not None:
                fps = 1.0 / process_time if process_time > 0 else 0
                print(f"帧: {frame_count}, 角度: {angle:.1f}°, 方向: {direction_text}, 处理时间: {process_time*1000:.1f}ms, FPS: {fps:.1f}")
            
            # 在图像上添加帧计数
            cv2.putText(debug_img, f"Frame: {frame_count}", (10, 350), 
                      cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # 显示调试图像
            cv2.imshow('检测结果', debug_img)
            if mask is not None:
                cv2.imshow('掩码', mask)
            
            # 按 'q' 键退出，按空格暂停
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord(' '):
                print("暂停，按任意键继续...")
                cv2.waitKey(0)
    
    finally:
        # 清理资源
        cap.release()
        cv2.destroyAllWindows()
        
        # 显示统计信息
        if frame_count > 0:
            avg_time = total_time / frame_count
            avg_fps = frame_count / total_time if total_time > 0 else 0
            print(f"统计信息: 处理 {frame_count} 帧, 平均时间: {avg_time*1000:.1f}ms, 平均FPS: {avg_fps:.1f}")

def main():
    """主函数"""
    # 解析命令行参数
    parser = argparse.ArgumentParser(description='轨道检测示例')
    parser.add_argument('--mode', type=str, default='live', choices=['live', 'video'],
                       help='运行模式: live (实时摄像头) 或 video (视频文件)')
    parser.add_argument('--detector', type=str, default='auto', 
                       choices=['auto', 'slight', 'sharp', 'scurve'],
                       help='检测器类型: auto (自动), slight (微调), sharp (大转弯), scurve (S弯)')
    parser.add_argument('--video', type=str, default='test_video.mp4',
                       help='视频文件路径 (仅video模式)')
    args = parser.parse_args()
    
    try:
        print("道路检测使用示例")
        print(f"模式: {args.mode}, 检测器: {args.detector}")
        print("按 'q' 键退出")
        
        # 创建选择的检测器
        if args.detector == 'slight':
            detector = SlightAdjustmentDetector()
            print("使用微调角度检测器")
        elif args.detector == 'sharp':
            detector = SharpTurnDetector()
            print("使用大转弯检测器")
        elif args.detector == 'scurve':
            detector = SCurveDetector()
            print("使用S弯检测器")
        else:
            # 自动模式使用默认API检测
            detector = None
            print("使用自动检测模式")
        
        # 视频文件模式
        if args.mode == 'video' and detector is not None:
            test_video_file(detector, args.video)
            return
        
        # 图像显示配置
        cv2.namedWindow('检测结果', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('检测结果', 800, 600)
        
        # 主循环
        while True:
            # 根据模式选择不同的处理方式
            if args.detector == 'auto' or detector is None:
                # 使用标准API
                angle, direction, debug_img, mask = detect_road()
                direction_text = get_direction_description(direction)
            else:
                # 使用自定义检测器
                # 从ROS获取图像
                from ros_image_provider import RoadImageProvider
                image_provider = RoadImageProvider()
                image = image_provider.get_latest_image()
                
                if image is None:
                    print("等待图像数据...")
                    time.sleep(0.5)
                    continue
                    
                # 使用选择的检测器处理图像
                angle, direction, direction_text, debug_img, mask = detector.process_image(image)
            
            # 检查是否成功获取和处理图像
            if debug_img is None:
                print("等待图像数据...")
                time.sleep(0.5)
                continue
                
            # 使用检测结果
            if angle is not None:
                speed, turn_rate = get_control_params(direction, angle)
                print(f"轨道角度: {angle:.1f}°, 方向: {direction_text}, 速度: {speed:.1f}, 转向率: {turn_rate:.2f}")
                
                # 在这里可以发送控制命令
                # control_robot(speed, turn_rate)
            else:
                # 未检测到有效轨道
                print("未检测到有效轨道")
                
            # 显示调试图像
            cv2.imshow('检测结果', debug_img)
            if mask is not None:
                cv2.imshow('掩码', mask)
            
            # 按 'q' 键退出
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
    except KeyboardInterrupt:
        print("程序已中断")
    finally:
        # 清理资源
        cleanup()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main() 