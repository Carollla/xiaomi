#!/usr/bin/env python3
"""
Entry point for the 2026 Xiaomi Cup autonomous race strategy.
"""
import os
import signal
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Bool

from motion import MotionController
from robot_state_machine_2026 import RobotStateMachine2026


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
        executor = MultiThreadedExecutor(num_threads=6)
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

        state_machine = RobotStateMachine2026(
            imu_provider=imu_provider,
            d435_provider=d435_provider,
            rgb_provider=rgb_provider,
        )
        executor.add_node(state_machine)
        nodes.append(state_machine)

        motion_controller = MotionController()
        executor.add_node(motion_controller)
        nodes.append(motion_controller)
        print('2026赛道程序初始化完成，开始运行...')

        completion_received = False

        def completion_callback(msg):
            nonlocal completion_received
            if msg.data:
                completion_received = True
                print('收到完成信号，准备退出...')

        state_machine.create_subscription(Bool, '/robot/completion', completion_callback, 10)

        while rclpy.ok() and not completion_received:
            executor.spin_once(timeout_sec=0.05)

        print('任务完成，开始清理资源...')

    except KeyboardInterrupt:
        print('\n接收到终止信号，开始关闭流程...')
    except Exception as e:
        print(f'发生未预期异常: {str(e)}')
        import traceback
        traceback.print_exc()
    finally:
        try:
            from std_msgs.msg import Int32MultiArray
            if 'state_machine' in locals():
                msg = Int32MultiArray()
                msg.data = [0]
                state_machine.motion_cmd_publisher.publish(msg)
                executor.spin_once(timeout_sec=0.1)
        except Exception:
            pass
        for node in reversed(nodes):
            try:
                node_name = node.get_name()
                node.destroy_node()
                print(f'成功关闭节点: {node_name}')
            except Exception as e:
                print(f'关闭节点时发生错误: {str(e)}')
        if rclpy.ok():
            rclpy.shutdown()
        total_time = time.time() - start_time
        print(f'程序运行总时长: {total_time:.2f}秒')
        os._exit(0)


if __name__ == '__main__':
    main()
