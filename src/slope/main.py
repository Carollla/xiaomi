#!/usr/bin/env python3
'''
四足机器人爬坡ROS2集成程序
'''
import rclpy
from rclpy.executors import MultiThreadedExecutor
from lidar_rec import LidarAnalyzer
from robot_state_machine import RobotStateMachine
from motion import MotionController
import signal
import sys

def main():
    # 设置信号处理
    signal.signal(signal.SIGINT, lambda sig, frame: sys.exit(0))  # 确保Ctrl+C可以强制退出
    signal.signal(signal.SIGTERM, lambda sig, frame: sys.exit(0))
    
    print("启动四足机器人爬坡系统...")
    rclpy.init()
    nodes = []
    try:
        executor = MultiThreadedExecutor(num_threads=3)
        # 创建并添加节点
        print("创建节点...")
        for node_class in [LidarAnalyzer, RobotStateMachine, MotionController]:
            node = node_class()
            executor.add_node(node)
            nodes.append(node)
            print(f"- 已添加节点: {node.get_name()}")
        print("所有节点已启动，开始运行（按Ctrl+C退出）")
        executor.spin()
    except KeyboardInterrupt:
        print("\n收到退出信号，正在关闭系统...")
    finally:
        print("正在关闭节点...")
        for node in reversed(nodes):
            try:
                node_name = node.get_name()
                node.destroy_node()
                print(f"- 已关闭节点: {node_name}")
            except Exception:
                pass
        rclpy.shutdown()
        print("系统已完全关闭")

if __name__ == '__main__':
    main()
