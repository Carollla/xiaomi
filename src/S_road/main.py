#!/usr/bin/env python3
'''
四足机器人基础控制ROS2集成程序
包含：状态机管理、基础运动控制、视觉检测调整
'''
import rclpy
from rclpy.executors import MultiThreadedExecutor
from robot_state_machine import RobotStateMachine
from motion import MotionController


import signal
import sys
import time

def setup_signal_handler():
    def signal_handler(sig, frame):
        print('收到退出信号，准备关闭节点...')
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

def main():
    start_time = time.time()
    setup_signal_handler()
    print('初始化ROS2环境...')
    rclpy.init()
    nodes = []
    try:
        from imu_provider import IMUProvider
        imu_provider = IMUProvider()

        print('创建节点和任务执行器...')
        executor = MultiThreadedExecutor(num_threads=3)
        executor.add_node(imu_provider.node)
        
        print('初始化状态机节点...')
        state_machine = RobotStateMachine(imu_provider)
        executor.add_node(state_machine)
        nodes.append(state_machine)
        print(f'- 已启动节点: {state_machine.get_name()}')

        print('初始化运动控制节点...')
        motion_controller = MotionController()
        executor.add_node(motion_controller)
        nodes.append(motion_controller)
        print(f'- 已启动节点: {motion_controller.get_name()}')

        '''
        print('初始化黄色检测节点...')
        yellow_detector = YellowDetector()
        executor.add_node(yellow_detector)
        nodes.append(yellow_detector)
        print(f'- 已启动节点: {yellow_detector.get_name()}')
        '''

        startup_time = time.time() - start_time
        print(f'系统准备就绪 (初始化耗时 {startup_time:.2f}秒)')
        print('='*50)
        print('机器人站立控制程序已启动')
        print('节点列表:')
        print(f'1. {state_machine.get_name()} - 状态管理')
        print(f'2. {motion_controller.get_name()} - 运动执行')
        print('='*50)
        print('等待站立流程完成...')
        print('按 Ctrl+C 停止程序')

        executor.spin()

    except KeyboardInterrupt:
        print('\n接收到终止信号，开始关闭流程...')
    except Exception as e:
        print(f'发生未预期异常: {str(e)}')
        import traceback
        traceback.print_exc()
    finally:
        print('\n开始清理资源...')
        # 逆序关闭节点
        for node in reversed(nodes):
            try:
                node_name = node.get_name()
                node.destroy_node()
                print(f'成功关闭节点: {node_name}')
            except Exception as e:
                print(f'关闭节点时发生错误: {str(e)}')
        
        print('关闭ROS2上下文...')
        rclpy.shutdown()
        
        total_time = time.time() - start_time
        print(f'程序运行总时长: {total_time:.2f}秒')
        print('系统已安全关闭')

if __name__ == '__main__':
    main()