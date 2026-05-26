#!/usr/bin/env python3
'''
机器人运动控制模块
'''
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32MultiArray
import time
from threading import Lock
import os
from robot_control.robot_control_class import Robot_Ctrl
from robot_control.robot_control_cmd_lcmt import robot_control_cmd_lcmt
from robot_control.file_send_lcmt import file_send_lcmt

class MotionController(Node):
    """
    机器人运动控制节点
    """
    def __init__(self):
        super().__init__('motion_controller')
        self.get_logger().info("Initializing MotionController ROS node...")
        
        # 初始化参数
        self.declare_parameter('init_timeout', 5.0)     # 初始化超时
        self.declare_parameter('head_down_angle', 0.5)  # 低头角度(弧度)
        self.declare_parameter('walking_speed', 0.1)    # 行走速度(m/s)

        # 获取参数
        self.head_down_angle = self.get_parameter('head_down_angle').get_parameter_value().double_value
        self.walking_speed = self.get_parameter('walking_speed').get_parameter_value().double_value
        self.init_timeout = self.get_parameter('init_timeout').get_parameter_value().double_value
        
        # 通信设置
        self.ready_publisher = self.create_publisher(Bool, '/robot/ready', 10)
        self.status_publisher = self.create_publisher(Int32MultiArray, '/robot/status', 10)
        self.create_subscription(Int32MultiArray, '/robot/motion_cmd', self.motion_cmd_callback, 10)
        
        # 控制变量
        self.current_mode = 0       # 当前模式 1 = 左圆形走 2 = 右圆形走
        self.progress = 0
        self.is_ready = False       # 就绪状态标志
        self.last_time = time.time()
        self.mode_lock = Lock()
        
        # 初始化LCM控制器
        self.robot_ctrl = Robot_Ctrl()
        self.robot_ctrl.run()
        
        # 控制定时器
        self.motion_timer = self.create_timer(0.05, self.motion_control_loop)
        self.status_timer = self.create_timer(0.2, self.publish_status)
        
        # 执行初始化
        self.init_start_time = time.time()
        self.get_logger().info("开始机器人初始化...")
        self.initialize_robot()


    def initialize_robot(self):
        """
        初始化机器人状态
        """
        self.get_logger().info("发送初始站立命令...")
        self.execute_stand()
        time.sleep(2.0)
        self.is_ready = True
        ready_msg = Bool()
        ready_msg.data = True
        self.ready_publisher.publish(ready_msg)
        self.get_logger().info("机器人初始化完成，已发布就绪状态")

    def motion_cmd_callback(self, msg):
        """
        运动命令回调函数（仅处理站立命令）
        """
        if len(msg.data) < 2:
            self.get_logger().warn("收到的命令格式错误")
            return
        with self.mode_lock:
            new_mode = msg.data[0]
            if new_mode != self.current_mode:  # 仅模式变化时发送
                self.current_mode = new_mode
                self.get_logger().info(f"切换到模式 {new_mode}")
                if new_mode == 0:
                    self.execute_stand()
                elif new_mode == 1:
                    self.execute_left_round_walk()
                elif new_mode == 2:
                    self.execute_right_round_walk()
                elif new_mode == 3:
                    self.execute_left_turn_walk()


    
    def execute_stand(self):
        cmd = robot_control_cmd_lcmt()
        cmd.mode = 12               # 站立模式
        cmd.gait_id = 0             # 标准步态
        cmd.pos_des = [0.0, 0.0, 0.0]
        cmd.rpy_des = [0.0, 0.0, 0.0]
        cmd.acc_des = [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        cmd.ctrl_point = [ 0.0, 0.0, 0.0]
        cmd.step_height = [0.0, 0.0]
        cmd.foot_pose = [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        cmd.duration = 2000 
        self.robot_ctrl.Send_cmd(cmd)
        self.get_logger().info("发送站立指令")

    def execute_left_round_walk(self):
        cmd = robot_control_cmd_lcmt()
        cmd.mode = 11               # LOCOMOTION模式
        cmd.gait_id = 122             # 变频走
        cmd.life_count = 1
        cmd.vel_des = [0.1, 0.0, 0.23]
        cmd.pos_des = [0.0, 0.0, 0.0]
        cmd.rpy_des = [0.0, 0.0, 0.0]
        cmd.acc_des = [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        cmd.ctrl_point = [ 0.0, 0.0, 0.0]
        cmd.step_height = [0.06, 0.06]
        cmd.foot_pose = [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        cmd.duration = 0
        cmd.value = 20
        self.robot_ctrl.Send_cmd(cmd)
        self.get_logger().info("发送左圆形走指令")

    def execute_right_round_walk(self):
        cmd = robot_control_cmd_lcmt()
        cmd.mode = 11               # LOCOMOTION模式
        cmd.gait_id = 122             
        cmd.life_count = 2
        cmd.vel_des = [0.1, 0.0, -0.23]
        cmd.pos_des = [0.0, 0.0, 0.0]
        cmd.rpy_des = [0.0, 0.0, 0.0]
        cmd.acc_des = [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        cmd.ctrl_point = [ 0.0, 0.0, 0.0]
        cmd.step_height = [0.06, 0.06]
        cmd.foot_pose = [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        cmd.duration = 0 
        self.robot_ctrl.Send_cmd(cmd)
        self.get_logger().info("发送右圆形走指令")

    def execute_left_turn_walk(self):
        cmd = robot_control_cmd_lcmt()
        cmd.mode = 11               # LOCOMOTION模式
        cmd.gait_id = 122             
        cmd.life_count = 3
        cmd.vel_des = [0.0, 0.0, 0.2]
        cmd.pos_des = [0.0, 0.0, 0.0]
        cmd.rpy_des = [0.0, 0.0, 0.0]
        cmd.acc_des = [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        cmd.ctrl_point = [ 0.0, 0.0, 0.0]
        cmd.step_height = [0.06, 0.06]
        cmd.foot_pose = [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        cmd.duration = 0
        cmd.value = 20
        self.robot_ctrl.Send_cmd(cmd)
        self.get_logger().info("发送左转指令")

    def publish_status(self):
        """
        发布状态信息（模拟进度）
        """
        status_msg = Int32MultiArray()
        status_msg.data = [self.current_mode, self.progress] 
        self.status_publisher.publish(status_msg)

    def motion_control_loop(self):
        with self.mode_lock:
            now = time.time()
            dt = now - self.last_time
            self.last_time = now
            #self.get_logger().info(f"[调试] 当前模式: {self.current_mode}")
            if not self.is_ready:
                if time.time() - self.init_start_time > self.init_timeout:
                    self.get_logger().warn(f"初始化超时 ({self.init_timeout}秒)，强制设置为就绪状态")
                    self.is_ready = True
                    ready_msg = Bool()
                    ready_msg.data = True
                    self.ready_publisher.publish(ready_msg)
                return

            # 根据当前模式执行相应控制
            if self.current_mode == 0:
                self.progress = 100
                #self.get_logger().info(f"当前模式是站立的进度:{self.progress}")
            elif self.current_mode == 1:
                self.progress = 100
                #self.get_logger().info(f"当前模式是左圆形走的进度:{self.progress}")
            elif self.current_mode == 2:
                self.progress = 100
                #self.get_logger().info(f"当前模式是右圆形走的进度:{self.progress}")
            elif self.current_mode == 3:
                self.progress = 100
            

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MotionController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Ctrl+C received, shutting down...")
    except Exception as e:
        if node:
            node.get_logger().fatal(f"Unhandled exception: {e}", exc_info=True)
        else:
            print(f"FATAL error during MotionController init: {e}")
            import traceback
            traceback.print_exc()
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
            print("ROS 2 shutdown complete.")

if __name__ == '__main__':
    main()