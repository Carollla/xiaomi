#!/usr/bin/env python3
'''
视觉矫正节点测试程序
功能：验证视觉矫正节点输出角度是否合理
'''
import rclpy
from rclpy.executors import MultiThreadedExecutor
from vision_correction import VisionCorrector
import signal
import sys
import time
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

class CorrectionTester(Node):
    """测试节点：监听并显示矫正角度"""
    def __init__(self):
        super().__init__('correction_tester')

        angle_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # 订阅矫正角度
        self.angle_sub = self.create_subscription(
            Float32,
            '/vision/correction_angle',
            self.angle_callback,
            qos_profile=angle_qos)
        # 订阅调试图像
        debug_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.debug_sub = self.create_subscription(
            Image,
            '/vision/debug_output',
            self.debug_callback,
            qos_profile=debug_qos)
        self.get_logger().info("测试节点已就绪")

    def angle_callback(self, msg):
        """角度数据回调"""
        self.get_logger().info(
            f"收到矫正角度: {msg.data:.2f} 度",
            throttle_duration_sec=1  # 节流输出
        )

    def debug_callback(self, msg):
        """调试图像回调（仅标记接收）"""
        self.get_logger().info("收到调试图像帧", throttle_duration_sec=2)

def setup_signal_handler():
    def signal_handler(sig, frame):
        print('\n收到退出信号，准备关闭节点...')
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
        print('创建节点和任务执行器...')
        executor = MultiThreadedExecutor(num_threads=2)

        print('初始化视觉矫正节点...')
        corrector = VisionCorrector()
        executor.add_node(corrector)
        nodes.append(corrector)
        print(f'- 已启动节点: {corrector.get_name()}')

        print('初始化测试节点...')
        tester = CorrectionTester()
        executor.add_node(tester)
        nodes.append(tester)
        print(f'- 已启动节点: {tester.get_name()}')

        startup_time = time.time() - start_time
        print(f'\n系统准备就绪 (初始化耗时 {startup_time:.2f}秒)')
        print('='*50)
        print('测试说明:')
        print('1. 确保外部相机节点正在发布 /rgb_camera/image_raw')
        print('2. 观察矫正角度输出是否合理')
        print('3. 可用rqt_image_view查看/vision/debug_output话题')
        print('='*50)
        print('按 Ctrl+C 停止测试\n')

        executor.spin()

    except KeyboardInterrupt:
        print('\n测试终止')
    except Exception as e:
        print(f'发生未预期异常: {str(e)}')
        import traceback
        traceback.print_exc()
    finally:
        print('\n清理资源...')
        for node in reversed(nodes):
            try:
                node_name = node.get_name()
                node.destroy_node()
                print(f'已关闭节点: {node_name}')
            except Exception as e:
                print(f'关闭节点失败: {str(e)}')
        
        rclpy.shutdown()
        print(f'测试总时长: {time.time()-start_time:.1f}秒')

if __name__ == '__main__':
    main()