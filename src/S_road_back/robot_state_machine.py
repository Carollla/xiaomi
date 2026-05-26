#!/usr/bin/env python3
'''
机器人状态机管理模块 - 仅站立部分
负责处理机器人初始化站立状态
'''
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32MultiArray
import time

from solve_turn import normalize_angle, get_current_yaw

class RobotStateMachine(Node):
    '''
    精简版状态机流程：
    WAITING_ROBOT_INIT -> STANDING_COMPLETED
    '''
    def __init__(self, imu_provider):
        super().__init__('robot_state_machine')
               
        # 订阅机器人反馈数据
        self.create_subscription(Int32MultiArray, '/robot/status', self.robot_status_callback, 10)
        self.create_subscription(Bool, '/robot/ready', self.robot_ready_callback, 10)

        self.imu_provider = imu_provider
        
        # 创建命令发布器
        self.motion_cmd_publisher = self.create_publisher(Int32MultiArray, '/robot/motion_cmd', 10)
        
        # 状态机变量（仅站立相关）
        #self.state = "WAITING_ROBOT_INIT"  # 初始状态
        self.state = "RIGHT_ROUND_WALK"
        self.robot_initialized = False
        self.init_start_time = time.time()  # 初始化开始时间
        self.last_robot_mode = -1           # 最后已知机器人模式
        self.yellow_detected = None
        
        # 创建定时器
        self.publish_timer = self.create_timer(0.05, self.publish_control_command)

        self.publish_timer = self.create_timer(0.05,self.turned_angles)

        # 命令发送标记
        self._stand_cmd_sent = False
        self._left_round_walk_cmd_sent = False
        self._right_round_walk_cmd_sent = False
        self._left_turn_cmd_sent = False
        
        #判断角度转过情况
        self.current_angle = -1
        self.turned_angle = 0
        self.count = 0
        self.turn_mode = 0
        self.right_count = 0
        self.left_count = 0
        self.stand_count = 0

        self.get_logger().info("[初始化] 机器人状态机节点启动")  

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
            if current_mode == 0 :
                if self.stand_count == 0:
                    self.stand_count += 1
                    self.get_logger().info("[状态变更] 检测到站立完成，下面进左圆形走步态")
                    self.current_angle = normalize_angle(get_current_yaw(self.imu_provider))
                    self.get_logger().info("当前角度为：%s" % self.current_angle)
                    self.state = "LEFT_ROUND_WALK"
                    #self.state = "RIGHT_ROUND_WALK"
                    self._left_round_walk_cmd_sent = False
                else:
                    self.get_logger().info("[状态变更] 开始罚站")
            elif current_mode == 1:
                if self.right_count == 0 and abs(self.turned_angles()-100) <= 1:
                    self.right_count += 1
                    self.state = "RIGHT_ROUND_WALK"
                    self.get_logger().info("[状态变更] 检测到左圆形走完成，下面进右圆形走步态")
                    self.current_angle = normalize_angle(get_current_yaw(self.imu_provider))
                    self.turned_angle = 0
                    self.turn_mode = 1
                    self.get_logger().info("当前角度为：%s" % self.current_angle)                    
                    self._right_round_walk_cmd_sent = False
                elif self.right_count == 1 and abs(self.turned_angles()-267) <= 1:
                    self.right_count += 1
                    self.state = "RIGHT_ROUND_WALK_2"
                    self.get_logger().info("[状态变更] 检测到左圆形走完成，下面进右圆形走步态")
                    self.current_angle = normalize_angle(get_current_yaw(self.imu_provider))
                    self.turned_angle = 0
                    self.turn_mode = 1
                    self.get_logger().info("当前角度为：%s" % self.current_angle)                    
                    self._right_round_walk_cmd_sent = False
                
            elif current_mode == 2 :
                if self.left_count == 0 and abs(self.turned_angles()-225) <= 1:
                    self.left_count += 1
                    self.get_logger().info("[状态变更] 检测到右圆形走完成，下面进左圆形走步态")
                    self.current_angle = normalize_angle(get_current_yaw(self.imu_provider))
                    self.turned_angle = 0
                    self.turn_mode = 0
                    self.get_logger().info("当前角度为：%s" % self.current_angle)
                    self.state = "LEFT_ROUND_WALK_2"
                    self._left_round_walk_cmd_sent = False
                elif self.left_count == 1 and abs(self.turned_angles()-215) <= 1:
                    self.get_logger().info("[状态变更] 检测到右圆形走完成，下面进行站立步态")
                    self.current_angle = normalize_angle(get_current_yaw(self.imu_provider))
                    self.turned_angle = 0
                    self.turn_mode = 0
                    self.get_logger().info("当前角度为：%s" % self.current_angle)
                    self.state = "STANDING"
                    self._left_turn_cmd_sent = False
            '''
            elif current_mode == 3 and abs(self.turned_angles()-10) <= 1:
                self.get_logger().info("[状态变更] 检测到左转完成，下面进站立步态")
                self.state = "STANDING"
                self._stand_cmd_sent = False
            '''

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

        elif (self.state == "LEFT_ROUND_WALK" or self.state == "LEFT_ROUND_WALK_2") and not self._left_round_walk_cmd_sent:
            cmd_msg.data = [1, 0]
            self.motion_cmd_publisher.publish(cmd_msg)
            self._left_round_walk_cmd_sent = True
            self.get_logger().info("[模式指令] 发布左圆形走指令: mode=1")

        elif (self.state == "RIGHT_ROUND_WALK" or self.state == "RIGHT_ROUND_WALK_2") and not self._right_round_walk_cmd_sent:
            cmd_msg.data = [2, 0]
            self.motion_cmd_publisher.publish(cmd_msg)
            self._right_round_walk_cmd_sent = True
            self.get_logger().info("[模式指令] 发布右圆形走指令: mode=2")
        '''
        elif self.state == "LEFT_TURN" and not self._left_turn_cmd_sent:
            cmd_msg.data = [3,0]
            self.motion_cmd_publisher.publish(cmd_msg)
            self._left_turn_cmd_sent = True
            self.get_logger().info("[模式指令] 发布左转指令: mode=3")
        '''


    def turned_angles(self):
        if self.current_angle == -1:
            return -1
        current_angle = normalize_angle(get_current_yaw(self.imu_provider))
        #mode = 0
        '''
        if(current_angle<self.current_angle):
            self.turned_angle = 360-self.current_angle + current_angle
            mode = 1
        else:
            self.turned_angle = current_angle - self.current_angle
            mode = 0
        '''
        if(self.turn_mode == 0):
            self.turned_angle = (current_angle - self.current_angle + 360) %360
        elif(self.turn_mode == 1):
            self.turned_angle = (self.current_angle - current_angle + 360) %360
        self.count += 1
        if(self.count == 20):
            self.get_logger().info("当前转过角度为：%s" % self.turned_angle)
            self.get_logger().info("转弯模式为：%s" % self.turn_mode)
            self.count = 0
        return self.turned_angle
            
            

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