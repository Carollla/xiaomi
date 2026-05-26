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
        self.create_subscription(Int32MultiArray, 'Qrcode1', self.qrcode_detection_callback,10)
        #参数设定
        # 初始化参数
        self.declare_parameter('init_timeout', 5.0)     # 初始化超时
        self.declare_parameter('head_down_angle', 0.3)  # 低头角度(弧度)
        self.declare_parameter('walking_speed', 0.2)    # 行走速度(m/s)
        # 获取参数
        self.head_down_angle = self.get_parameter('head_down_angle').get_parameter_value().double_value
        self.walking_speed = self.get_parameter('walking_speed').get_parameter_value().double_value
        self.init_timeout = self.get_parameter('init_timeout').get_parameter_value().double_value
        # 创建命令发布器
        self.motion_cmd_publisher = self.create_publisher(Int32MultiArray, '/robot/motion_cmd', 10)
        
        # 状态机变量（仅站立相关）
        self.state = "WAITING_ROBOT_INIT"  # 初始状态
        self.robot_initialized = False
        self.init_start_time = time.time()  # 初始化开始时间
        self.last_robot_mode = -1           # 最后已知机器人模式
        self.yellow_detected = None
        self.qrcode_info=None
        
        # 创建定时器
        self.publish_timer = self.create_timer(0.05, self.publish_control_command)

        # 命令发送标记
        self._stand_cmd_sent = False
        self._walking_cmd_sent = False
        self._turn_cmd_sent = False
        self._standing_after_walk_cmd_sent=False
        self._walking_after_turn_cmd_sent=False
        self._qrcode_detected=False
        self._turning_30_cmd_sent=False
        self._walking_third_sent=False
        self._turn_third_sent=False
        self._walking_fourth_sent=False
        self._standing_atA_sent=False
        self._walking_for_turn=False
        self._get_down=False
        self._get_up=False
        self._walking_back=False
        self._turning_back=False
        self._walking_after_turn_back=False
        self._turning_to_main=False
        self._walking_to_s=False
        self._turning_to_s=False
        self.get_logger().info("[初始化] 机器人状态机节点启动")

    def yellow_detection_callback(self, msg):
        """处理黄色检测结果回调"""
        self.yellow_detected = msg.data
    def qrcode_detection_callback(self,msg):
        self.qrcode_info=msg.data[0]
        if self.qrcode_info == 0:
            self.get_logger().info("未检测到二维码")
            self._qrcode_detected = False
        else:
            self._qrcode_detected=True
            self.get_logger().info("已检测到二维码")
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
        else:
            self.robot_initialized = False
            self.get_logger().warn("[警告] 机器人变为未就绪状态，返回等待初始化")
            self.state = "WAITING_ROBOT_INIT"

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
                    self.get_logger().info("[状态变更] 检测到站立完成，切换到WALKING状态")
                    self.state = "WALKING"
                    self._walking_cmd_sent = False
                    self._turn_cmd_sent = False
            elif current_mode == 2 and progress >= 95 and self.state == "TURNING":
                self.get_logger().info("[状态变更] 转向完成，切换到二次行走状态")
                self.state = "WALKING_AFTER_TURN"
                self._walking_after_turn_cmd_sent = False
            # 行走完成检测
            elif current_mode == 1 and progress >= 95 and self.state == "WALKING":
                self.get_logger().info("[状态变更] 行走完成，发布转向指令")
                self.state = "TURNING"
                self._turn_cmd_sent = False
            # 二次行走完成检测
            elif current_mode == 1 and progress >= 95 and self.state == "WALKING_AFTER_TURN":
                self.get_logger().info("[状态变更] 二次行走完成，发布站立指令")
                self.state = "WAITING_VISUAL_DATA"
                self._standing_after_walk_cmd_sent = False
            elif current_mode == 2 and progress >= 95 and self.state == "TURNING_30":
                self.get_logger().info("[状态变更] 转向完成，切换到三次行走状态")
                self.state = "WALKING_THIRD"
                self._walking_third_sent = False
            # 行走完成检测
            elif current_mode == 1 and progress >= 95 and self.state == "WALKING_THIRD":
                self.get_logger().info("[状态变更] 三次行走完成，发布转向指令")
                self.state = "TURNING_THIRD"
                self._turn_third_sent = False
            #二维码识别完成检测
            elif current_mode == 3 and progress >=95 and self.state == "WAITING_VISUAL_DATA" and self._qrcode_detected == True and self.qrcode_info == 2:
                self.get_logger().info("[状态变更] 二维码识别完成，发布行走指令")
                self.state="WALKING_FOR_TURN"
                self._walking_for_turn=False
            #未识别到二维码
            elif current_mode ==3 and progress >= 95 and self.state == "WAITING_VISUAL_DATA" and self._qrcode_detected == False:
                self.get_logger().info("[状态维持] 等待二维码识别信息")
            elif current_mode ==1 and progress >=95 and self.state =="WALKING_FOR_TURN":
                self.get_logger().info("[状态变更] 行走过渡完成，发布转向指令")
                self.state="TURNING_30"
                self._turning_30_cmd_sent=False
            elif current_mode == 2 and progress >= 95 and self.state == "TURNING_THIRD":
                self.get_logger().info("[状态变更] 三次转向完成，切换到四次行走状态")
                self.state = "WALKING_FOURTH"
                self._walking_fourth_sent =False
            elif current_mode == 1 and progress >= 95 and self.state == "WALKING_FOURTH":
                self.get_logger().info("[状态变更] 四次行走完成，发布站立指令")
                self.state = "STANDING_AT_A"
                self._standing_atA_sent = False
            elif current_mode == 3 and progress >=95 and self.state == "STANDING_AT_A":
                self.get_logger().info("[状态变更] 站立完成，发布趴下指令")
                self.state = "GETTING_DOWN"
                self._get_down=False
            elif current_mode == 4 and progress >=95 and self.state == "GETTING_DOWN":
                self.get_logger().info("[状态变更] 装货完成，发布站立指令")
                self.state = "GETTING_UP"
                self._get_up=False
            elif current_mode == 3 and progress >=95 and self.state == "GETTING_UP":
                self.get_logger().info("[状态变更] 站立完成，发布后退指令")
                self.state = "WALKING_BACK"
                self._walking_back=False
            elif current_mode == 5 and progress >=95 and self.state == "WALKING_BACK":
                self.get_logger().info("[状态变更] 后退完成，发布逆转弯指令")
                self.state = "TURNING_BACK"
                self._turning_back=False
            elif current_mode == 2 and progress >=95 and self.state == "TURNING_BACK":
                self.get_logger().info("[状态变更] 逆转弯完成，发布逆转弯后行走指令")
                self.state = "WALKING_AFTER_TURN_BACK"
                self._walking_after_turn_back=False
            elif current_mode == 1 and progress >=95 and self.state == "WALKING_AFTER_TURN_BACK":
                self.get_logger().info("[状态变更] 行走完成，发布行走后转弯指令")
                self.state = "TURNING_TO_MAIN"
                self._turning_to_main=False
            elif current_mode == 6 and progress >=95 and self.state == "TURNING_TO_MAIN":
                self.get_logger().info("[状态变更] 转向完成，发布转向后行走指令")
                self.state = "WALKING_TO_S"
                self._walking_to_s=False
            elif current_mode == 1 and progress >=95 and self.state == "WALKING_TO_S":
                self.get_logger().info("[状态变更] 行走完成，发布行走后转向指令")
                self.state = "TURNING_TO_S"
                self._turning_to_s=False

    def publish_control_command(self):
        """
        主控制命令发布函数
        根据当前状态发送相应命令
        """
        if not self.robot_initialized:
            return

        cmd_msg = Int32MultiArray()

        if self.state == "WAITING_INITIAL_DATA":
            cmd_msg.data = [0, 0]  # 发送站立指令
            self.motion_cmd_publisher.publish(cmd_msg)
            self.get_logger().info("[模式指令] 发布站立指令: mode=0")
            self.state = "STANDING"  # 切换至STANDING状态
            self._stand_cmd_sent = True

        elif self.state == "STANDING" and not self._stand_cmd_sent:
            cmd_msg.data = [0, 0]
            self.motion_cmd_publisher.publish(cmd_msg)
            self._stand_cmd_sent = True
            self.get_logger().info("[模式指令] 发布站立指令: mode=0")

        elif self.state == "WALKING":
            # 优先发送行走指令
            if not self._walking_cmd_sent:
                cmd_msg.data = [1, 4600]  # 正常低头行走指令
                self.motion_cmd_publisher.publish(cmd_msg)
                self.get_logger().info("[模式指令] 发布低头行走指令: mode=1")
                self._walking_cmd_sent = True
        elif self.state == "TURNING":
            if not self._turn_cmd_sent:
                cmd_msg = Int32MultiArray()
                cmd_msg.data = [2, 4000]
                self.motion_cmd_publisher.publish(cmd_msg)
                self.get_logger().info("[转向指令] 发布转向指令: mode=2")
                self._turn_cmd_sent=True
        elif self.state == "WALKING_AFTER_TURN":
            if not self._walking_after_turn_cmd_sent:
                cmd_msg.data = [1, 3600]  # 继续发送行走指令
                self.motion_cmd_publisher.publish(cmd_msg)
                self.get_logger().info("[模式指令] 发布转向后行走指令")
                self._walking_after_turn_cmd_sent = True
        elif self.state == "WAITING_VISUAL_DATA":
            if not self._standing_after_walk_cmd_sent:
                cmd_msg.data = [3, 2000]  # 发送站立指令
                self.motion_cmd_publisher.publish(cmd_msg)
                self.get_logger().info("[模式指令] 发布站立指令: mode=3")
                self._standing_after_walk_cmd_sent = True
        elif self.state == "WALKING_FOR_TURN":
            if not self._walking_for_turn:
                cmd_msg.data = [1, 1200]  # 继续发送行走指令
                self.motion_cmd_publisher.publish(cmd_msg)
                self.get_logger().info("[模式指令] 发布转向后行走指令：mode=1")
                self._walking_for_turn = True
        elif self.state == "TURNING_30":
            if not self._turning_30_cmd_sent:
                cmd_msg.data = [2, 900]
                self.motion_cmd_publisher.publish(cmd_msg)
                self.get_logger().info("[转向指令] 发布转向30度指令: mode=2")
                self._turning_30_cmd_sent=True
        elif self.state == "WALKING_THIRD":
            if not self._walking_third_sent:
                cmd_msg.data = [1, 7000]  # 继续发送行走指令
                self.motion_cmd_publisher.publish(cmd_msg)
                self.get_logger().info("[模式指令] 发布转向后行走指令：mode=1")
                self._walking_third_sent = True
        elif self.state == "TURNING_THIRD":
            if not self._turn_third_sent:
                cmd_msg.data = [2, 6000]
                self.motion_cmd_publisher.publish(cmd_msg)
                self.get_logger().info("[转向指令] 发布转向110度指令: mode=2")
                self._turn_third_sent=True
        elif self.state == "WALKING_FOURTH":
            if not self._walking_fourth_sent:
                cmd_msg.data = [1, 6000]  # 继续发送行走指令
                self.motion_cmd_publisher.publish(cmd_msg)
                self.get_logger().info("[模式指令] 发布转向后行走指令：mode=1")
                self._walking_fourth_sent = True
        elif self.state == "STANDING_AT_A":
            if not self._standing_atA_sent:
                cmd_msg.data = [3, 1000]  # 继续发送站立指令
                self.motion_cmd_publisher.publish(cmd_msg)
                self.get_logger().info("[模式指令] 发布站立指令: mode=3")
                self._standing_atA_sent = True
        elif self.state == "GETTING_DOWN":
            if not self._get_down:
                cmd_msg.data = [4, 3000]
                self.motion_cmd_publisher.publish(cmd_msg)
                self.get_logger().info("[模式指令] 发布趴下指令: mode=4")
                self._get_down =True
        elif self.state == "GETTING_UP":
            if not self._get_up:
                cmd_msg.data = [3, 2000]
                self.motion_cmd_publisher.publish(cmd_msg)
                self.get_logger().info("[模式指令] 发布站立指令: mode=3")
                self._get_up =True
        elif self.state == "WALKING_BACK":
            if not self._walking_back:
                cmd_msg.data = [5, 6000]
                self.motion_cmd_publisher.publish(cmd_msg)
                self.get_logger().info("[模式指令] 发布后退指令: mode=5")
                self._walking_back =True
        elif self.state == "TURNING_BACK":
            if not self._turning_back:
                cmd_msg.data = [2, 2400]
                self.motion_cmd_publisher.publish(cmd_msg)
                self.get_logger().info("[模式指令] 发布转向指令: mode=2")
                self._turning_back =True
        elif self.state == "WALKING_AFTER_TURN_BACK":
            if not self._walking_after_turn_back:
                cmd_msg.data = [1, 7000]
                self.motion_cmd_publisher.publish(cmd_msg)
                self.get_logger().info("[模式指令] 发布行走指令: mode=1")
                self._walking_after_turn_back =True
        elif self.state == "TURNING_TO_MAIN":
            if not self._turning_to_main:
                cmd_msg.data = [6, 1400]
                self.motion_cmd_publisher.publish(cmd_msg)
                self.get_logger().info("[模式指令] 发布逆转向指令: mode=6")
                self._turning_to_main=True
        elif self.state =="WALKING_TO_S":
            if not self._walking_to_s:
                cmd_msg.data = [1, 5200]
                self.motion_cmd_publisher.publish(cmd_msg)
                self.get_logger().info("[模式指令] 发布行走指令: mode=1")
                self._walking_to_s=True
        elif self.state =="TURNING_TO_S":
            if not self._turning_to_s:
                cmd_msg.data = [2, 4000]
                self.motion_cmd_publisher.publish(cmd_msg)
                self.get_logger().info("[模式指令] 发布转向指令: mode=2")
                self._turning_to_s=True

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