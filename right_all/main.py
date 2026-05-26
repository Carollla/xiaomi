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
import time
import os
from std_msgs.msg import Bool

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
        # print('创建节点和任务执行器...')
        executor = MultiThreadedExecutor(num_threads=5)
        from detect.IMU_provider import IMUProvider
        from detect.D435_provider import D435Provider
        from detect.RGB_provider import RGBProvider
        imu_provider = IMUProvider()
        d435_provider = D435Provider()
        rgb_provider = RGBProvider()
        executor.add_node(imu_provider.node)
        executor.add_node(d435_provider.node)
        executor.add_node(rgb_provider.node)
        nodes.extend([imu_provider.node, d435_provider.node, rgb_provider.node])
        # print('初始化状态机节点...')
        state_machine = RobotStateMachine(
            imu_provider=imu_provider,
            d435_provider=d435_provider,
            rgb_provider=rgb_provider
        )
        executor.add_node(state_machine)
        nodes.append(state_machine)
        # state_machine.state = "WAREHOUSE_A_1_TURN_LEFT_TO_NORTH"
        # print('初始化运动控制节点...')
        motion_controller = MotionController()
        executor.add_node(motion_controller)
        nodes.append(motion_controller)
        print('初始化完成，开始运行...', nodes)

        # 创建完成信号订阅者
        completion_received = False
        def completion_callback(msg):
            nonlocal completion_received
            if msg.data:
                completion_received = True
                print('收到完成信号，准备退出...')
        
        completion_sub = state_machine.create_subscription(
            Bool,
            '/robot/completion',
            completion_callback,
            10
        )

        # 主循环
        while rclpy.ok() and not completion_received:
            executor.spin_once(timeout_sec=0.0)  # 设置超时为0，立即返回

        print('任务完成，开始清理资源...')

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
        # 强制退出程序
        os._exit(0)

if __name__ == '__main__':
    main()
