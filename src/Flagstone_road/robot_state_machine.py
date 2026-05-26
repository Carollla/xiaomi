#!/usr/bin/env python3
'''
机器人状态机管理模块 - 仅站立部分
负责处理机器人初始化站立状态
'''
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32MultiArray
import time

class RobotStateMachine(Node):
    '''
    精简版状态机流程：
    WAITING_ROBOT_INIT -> STANDING_COMPLETED
    '''
    def __init__(self):
        super().__init__('robot_state_machine')
               
        # 订阅机器人反馈数据
        self.create_subscription(Int32MultiArray, '/robot/status', self.robot_status_callback, 10)
        self.create_subscription(Bool, '/robot/ready', self.robot_ready_callback, 10)
        self.create_subscription(Bool, '/yellow_detection', self.yellow_detection_callback, 10)
        
        # 创建命令发布器
        self.motion_cmd_publisher = self.create_publisher(Int32MultiArray, '/robot/motion_cmd', 10)
        
        # 状态机变量（仅站立相关）
        self.state = "WAITING_ROBOT_INIT"  # 初始状态
        self.robot_initialized = False
        self.init_start_time = time.time()  # 初始化开始时间
        self.last_robot_mode = -1           # 最后已知机器人模式
        self.yellow_detected = None
        
        # 创建定时器
        self.publish_timer = self.create_timer(0.05, self.publish_control_command)

        # 命令发送标记
        self._stand_cmd_sent = False
        self._walking_with_head_down_cmd_sent = False
        self._turn_cmd_sent = False
        self._left_walk_cmd_sent = False
        self._flagstone_walk_cmd_sent = False
        self._down_walk_cmd_sent = False
        
        self.get_logger().info("[初始化] 机器人状态机节点启动")

    def yellow_detection_callback(self, msg):
        """处理黄色检测结果回调"""
        self.yellow_detected = msg.data

    def robot_ready_callback(self, msg):
        """
        处理机器人就绪状态回调
        输入：/robot/ready话题的Bool消息
        """
        if msg.data and not self.robot_initialized:
            self.robot_initialized = True
            self.get_logger().info("[初始化] 收到机器人就绪信号，系统准备就绪")
            if self.state == "WAITING_ROBOT_INIT":
                self.get_logger().info("[状态切换] 从 WAITING_ROBOT_INIT 切换到 WAITING_INITIAL_DATA")
                self.state = "WAITING_INITIAL_DATA"
                self.init_start_time = time.time()
        else:
            self.robot_initialized = False
            self.get_logger().warn("[警告] 机器人变为未就绪状态，返回等待初始化")
            self.state = "WAITING_ROBOT_INIT"
            self.init_start_time = time.time()

    def robot_status_callback(self, msg):
        """
        处理机器人状态反馈回调
        输入：/robot/status话题的Int32MultiArray消息
        数据格式：[当前模式, 进度百分比]
        """
        if len(msg.data) >= 2:
            current_mode = msg.data[0]
            progress = msg.data[1]
            self.last_robot_mode = current_mode
            
            # 站立完成检测
            if current_mode == 0 and progress >= 95:
                if self.state == "STANDING":
                    self.get_logger().info("[状态变更] 检测到站立完成，切换到WALKING_WITH_HEAD_DOWN状态")
                    self.state = "WALKING_WITH_HEAD_DOWN"
                    self._walking_with_head_down_cmd_sent = False
                    self._turn_cmd_sent = False
            elif current_mode ==  2 and progress >= 95:
                time.sleep(3)
                self.state = "RIGHT_TURN"
                self.get_logger().info("[状态变更] 检测到右转弯状态，下面进行左移动作")
                self._left_walk_cmd_sent = False
            elif current_mode == 3 and progress >= 95:
                self.get_logger().info("[状态变更] 检测到左移动，下面进行石板路步态状态")
                time.sleep(3)
                self.state = "LEFT_WALK"
                self._left_walk_cmd_sent = False
            elif current_mode ==4 and progress >= 95:
                self.get_logger().info("[状态变更] 检测到石板路步态状态")
                time.sleep(55)
                self.state = "FLAGSTONE_WALK"
                self._down_walk_cmd_sent = False




    def publish_control_command(self):
        """
        主控制命令发布函数
        根据当前状态发送相应命令
        """
        if not self.robot_initialized:
            return

        cmd_msg = Int32MultiArray()

        if self.state == "WAITING_INITIAL_DATA":
            #cmd_msg.data = [0, 0]  # 发送站立指令
            #self.motion_cmd_publisher.publish(cmd_msg)
            self.get_logger().info("[模式指令] 准备发布站立指令: mode=0")
            self.state = "STANDING"  # 切换至STANDING状态

        elif self.state == "STANDING" and not self._stand_cmd_sent:
            cmd_msg.data = [0, 0]
            self.motion_cmd_publisher.publish(cmd_msg)
            self._stand_cmd_sent = True
            self.get_logger().info("[模式指令] 发布站立指令: mode=0")

        elif self.state == "WALKING_WITH_HEAD_DOWN":
            # 优先处理黄色检测逻辑
            if self.yellow_detected is False and not self._turn_cmd_sent:
                cmd_msg.data = [2, 0]  # 发送转向指令
                self.motion_cmd_publisher.publish(cmd_msg)
                self.get_logger().info("[模式指令] 检测到无黄色，发布转向指令: mode=2")
                self._turn_cmd_sent = True
                
            elif not self._walking_with_head_down_cmd_sent:
                cmd_msg.data = [1, 0]  # 正常低头行走指令
                self.motion_cmd_publisher.publish(cmd_msg)
                self.get_logger().info("[模式指令] 发布低头行走指令: mode=1")
                self._walking_with_head_down_cmd_sent = True
                
        elif self.state == "RIGHT_TURN" and not self._left_walk_cmd_sent:
            cmd_msg.data = [3, 0]
            self.motion_cmd_publisher.publish(cmd_msg)
            self._left_walk_cmd_sent = True
            self.get_logger().info("[模式指令] 发布左移动指令: mode=3")
        
        elif self.state == "LEFT_WALK" and not self._flagstone_walk_cmd_sent:
            cmd_msg.data = [4,0]
            self.motion_cmd_publisher.publish(cmd_msg)
            self._flagstone_walk_cmd_sent = True
            self.get_logger().info("[模式指令] 发布石板路步态指令: mode=4")
        
        elif self.state == "FLAGSTONE_WALK" and not self._down_walk_cmd_sent:
            cmd_msg.data = [5,0]
            self.motion_cmd_publisher.publish(cmd_msg)
            self._down_walk_cmd_sent = True
            self.get_logger().info("[模式指令] 发布趴下走路步态指令: mode=5")
            

def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    node = RobotStateMachine()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("销毁机器人状态机节点...")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()