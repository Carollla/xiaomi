#!/usr/bin/env python3
'''
机器人运动控制模块
'''
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32MultiArray
import time
from threading import Lock
from robot_control.robot_control_class import Robot_Ctrl
from robot_control.robot_control_cmd_lcmt import robot_control_cmd_lcmt
from builtin_interfaces.msg import Time
from rosgraph_msgs.msg import Clock
class MotionController(Node):
    """
    机器人运动控制节点
    """
    def __init__(self):
        super().__init__('motion_controller')
        self.get_logger().info("Initializing MotionController ROS node...")
        
        # 初始化参数
        self.declare_parameter('init_timeout', 5.0)     # 初始化超时
        self.declare_parameter('head_down_angle', 0.3)  # 低头角度(弧度)
        self.declare_parameter('walking_speed', 0.2)    # 行走速度(m/s)
        self.declare_parameter('walking_duration', 5.0)  # 目标行走时间
        self.declare_parameter('standing_duration',2.0) # 站立时间
        self.declare_parameter('down_duration',2.0) # 趴下时间
        # 获取参数
        self.head_down_angle = self.get_parameter('head_down_angle').get_parameter_value().double_value
        self.walking_speed = self.get_parameter('walking_speed').get_parameter_value().double_value
        self.init_timeout = self.get_parameter('init_timeout').get_parameter_value().double_value
        self.walking_duration= self.get_parameter('walking_duration').get_parameter_value().double_value
        self.standing_duration= self.get_parameter('standing_duration').get_parameter_value().double_value
        self.down_duration= self.get_parameter('down_duration').get_parameter_value().double_value
        # 通信设置
        self.ready_publisher = self.create_publisher(Bool, '/robot/ready', 10)
        self.status_publisher = self.create_publisher(Int32MultiArray, '/robot/status', 10)
        self.create_subscription(Int32MultiArray, '/robot/motion_cmd', self.motion_cmd_callback, 10)
        #self.clock = Clock(clock_type=ClockType.ROS_TIME)  # 初始化ROS 2时钟
        
        self.create_subscription(Clock, '/clock', self.clock_callback, 10)
        # 控制变量
        self.current_mode = 0       # 当前模式 0=站立 1=低头行走
        self.is_ready = False       # 就绪状态标志
        self.last_time = None
        self.mode_lock = Lock()
        self.sim_time = None  # 初始仿真时间
        self.last_robot_progress=0
        # 初始化LCM控制器
        self.robot_ctrl = Robot_Ctrl()
        self.robot_ctrl.run()
        
        # 控制定时器
        self.motion_timer = self.create_timer(0.05, self.motion_control_loop)
        self.status_timer = self.create_timer(0.2, self.publish_status)
        
        # 执行初始化
        self.init_start_time = None
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
    def clock_callback(self, msg):
        # 更新仿真时间（通过ROS 2时钟接口）
        self.sim_time = msg.clock
        self.get_logger().info(f"收到仿真时间: {self.sim_time.sec}.{self.sim_time.nanosec}")
        if self.init_start_time is None:
            self.init_start_time = self.sim_time
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
                    #time.sleep(2)
                elif new_mode == 1:
                    self.execute_walk()
                    self.walking_duration = msg.data[1]/1000
                elif new_mode == 2:
                    self.target_turning_time=msg.data[1]/1000
                    self.execute_turn_85()
                elif new_mode == 3:
                    self.execute_stand()
                    self.standing_duration = msg.data[1]/1000
                elif new_mode == 4:
                    self.execute_get_down()
                    self.down_duration = msg.data[1]/1000
                elif new_mode == 5:
                    self.execute_walk_back()
                    self.walking_duration =msg.data[1]/1000
                elif new_mode == 6:
                    self.target_turning_time=msg.data[1]/1000
                    self.execute_turn_back()
                self.init_start_time = self.sim_time.sec + self.sim_time.nanosec / 1e9 # 记录动作的开始时间
    
    def execute_stand(self):
        cmd = robot_control_cmd_lcmt()
        cmd.mode = 12               # 站立模式
        cmd.gait_id = 0             # 标准步态
        cmd.pos_des = [0.0, 0.0, 0.2]
        cmd.rpy_des = [0.0, 0.0, 0.0]
        cmd.acc_des = [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        #cmd.vel_des = [0.0, 0.0, 0.0] 
        cmd.ctrl_point = [ 0.0, 0.0, 0.0]
        cmd.step_height = [0.0, 0.0]
        cmd.foot_pose = [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        cmd.duration = 2000 
        self.robot_ctrl.Send_cmd(cmd)
        self.get_logger().info("发送站立指令 (LCM mode=12)")
        

    def execute_head_down_walk(self):
        """执行低头行走控制"""
        cmd = robot_control_cmd_lcmt()
        cmd.mode = 11               # 行走模式
        cmd.gait_id = 27             # 低头步态
        cmd.contact = 0
        cmd.life_count = 1
        cmd.pos_des = [0.0, 0.0, 0.0]
        cmd.rpy_des = [0.0, self.head_down_angle, 0.0]  # 低头角度
        cmd.vel_des = [self.walking_speed, 0.0, 0.0] 
        cmd.acc_des = [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        cmd.foot_pose = [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        cmd.step_height = [ 0.06, 0.06]
        cmd.value = 0
        cmd.duration = 0
        self.robot_ctrl.Send_cmd(cmd)
        self.get_logger().info("执行低头行走命令")
    def execute_walk(self):
        """执行低头行走控制"""
        cmd = robot_control_cmd_lcmt()
        cmd.mode = 11               # 行走模式
        cmd.gait_id = 27             # 低头步态
        cmd.contact = 0
        cmd.life_count = 1
        cmd.pos_des = [0.0, 0.0, 0.0]
        cmd.rpy_des = [0.0, 0.0, 0.0]  # 头角度调平
        cmd.vel_des = [self.walking_speed, 0.0, 0.0] 
        cmd.acc_des = [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        cmd.foot_pose = [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        cmd.step_height = [ 0.06, 0.06]
        cmd.value = 0
        cmd.duration = 0
        self.robot_ctrl.Send_cmd(cmd)
        self.get_logger().info("执行正常行走命令")
    def execute_walk_back(self):
        """执行低头行走控制"""
        cmd = robot_control_cmd_lcmt()
        cmd.mode = 11               # 行走模式
        cmd.gait_id = 27             # 低头步态
        cmd.contact = 0
        cmd.life_count = 1
        cmd.pos_des = [0.0, 0.0, 0.0]
        cmd.rpy_des = [0.0, 0.0, 0.0]  # 头角度调平
        cmd.vel_des = [-self.walking_speed, 0.0, 0.0] 
        cmd.acc_des = [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        cmd.foot_pose = [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        cmd.step_height = [ 0.06, 0.06]
        cmd.value = 0
        cmd.duration = 0
        self.robot_ctrl.Send_cmd(cmd)
        self.get_logger().info("执行反向行走命令")
    def execute_turn_90(self):
        cmd = robot_control_cmd_lcmt()
        cmd.mode = 11               # 行走模式
        cmd.gait_id = 27             # 转90步态
        cmd.contact = 0
        cmd.life_count = 2
        cmd.pos_des = [0.0, 0.0, 0.0]
        cmd.rpy_des = [0.0, 0.0, 0.0] 
        cmd.vel_des = [0.0, 0.0, -0.5] 
        cmd.acc_des = [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        cmd.foot_pose = [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        cmd.step_height = [ 0.06, 0.06]
        cmd.value = 0
        cmd.duration = 4000
        self.robot_ctrl.Send_cmd(cmd)
        self.get_logger().info("执行90度转弯命令")
        
    def execute_turn_85(self):
        cmd = robot_control_cmd_lcmt()
        cmd.mode = 11               # 行走模式
        cmd.gait_id = 27             # 转90步态
        cmd.contact = 0
        cmd.life_count = 2
        cmd.pos_des = [0.0, 0.0, 0.0]
        cmd.rpy_des = [0.0, 0.0, 0.0] 
        cmd.vel_des = [0.0, 0.0, -0.5] 
        cmd.acc_des = [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        cmd.foot_pose = [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        cmd.step_height = [ 0.06, 0.06]
        cmd.value = 0
        cmd.duration = int(self.target_turning_time*1000)
        self.robot_ctrl.Send_cmd(cmd)
        self.get_logger().info("执行85度转弯命令")
    def execute_turn_back(self):
        cmd = robot_control_cmd_lcmt()
        cmd.mode = 11               # 行走模式
        cmd.gait_id = 27             # 转90步态
        cmd.contact = 0
        cmd.life_count = 2
        cmd.pos_des = [0.0, 0.0, 0.0]
        cmd.rpy_des = [0.0, 0.0, 0.0] 
        cmd.vel_des = [0.0, 0.0, 0.5] 
        cmd.acc_des = [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        cmd.foot_pose = [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        cmd.step_height = [ 0.06, 0.06]
        cmd.value = 0
        cmd.duration = int(self.target_turning_time*1000)
        self.robot_ctrl.Send_cmd(cmd)
        self.get_logger().info("执行反向转弯命令")
    def execute_get_down(self):
        cmd = robot_control_cmd_lcmt()
        cmd.mode = 7               
        cmd.gait_id = 0             
        cmd.life_count = 5
        self.robot_ctrl.Send_cmd(cmd)
        self.get_logger().info("执行趴下命令")

    def publish_status(self):
        """
        发布状态信息（模拟进度）
        """
        status_msg = Int32MultiArray()
        status_msg.data = [self.current_mode, self.last_robot_progress] 
        self.status_publisher.publish(status_msg)

    def motion_control_loop(self):
        with self.mode_lock:
            if self.sim_time is None:
                self.get_logger().info("等待仿真时间...")
                return

            now_sec = self.sim_time.sec + self.sim_time.nanosec / 1e9
            if self.init_start_time is None:
                self.init_start_time = now_sec
                return
            now=now_sec
            #self.get_logger().info(f"[调试] 当前模式: {self.current_mode}")

            if not self.is_ready:
                if (now - self.init_start_time)> self.init_timeout:
                    self.get_logger().warn(f"初始化超时 ({self.init_timeout}秒)，强制设置为就绪状态")
                    self.is_ready = True
                    ready_msg = Bool()
                    ready_msg.data = True
                    self.ready_publisher.publish(ready_msg)
                return

            # 根据当前模式执行相应控制并计算进度
            progress = 0
            if self.current_mode == 0:  # 站立模式
                progress=100
            elif self.current_mode == 1:  # 行走模式
                # 计算行走进度
                walking_time = (now- self.init_start_time)
                progress = min(int((walking_time / self.walking_duration) * 100), 100)
            elif self.current_mode == 2:  # 转向模式
                # 计算转向进度
                turning_time = (now- self.init_start_time)
                progress = min(int((turning_time / self.target_turning_time) * 100), 100)
            elif self.current_mode == 3:  # 静止模式
                # 计算转向进度
                standing_time = (now- self.init_start_time)
                progress = min(int((standing_time / self.standing_duration) * 100), 100)
            elif self.current_mode == 4:  # 趴下模式
                down_time = (now- self.init_start_time)
                progress = min(int((down_time / self.down_duration) * 100), 100)   
            elif self.current_mode == 5:  # 反向行走模式
                walk_back_time = (now- self.init_start_time)
                progress = min(int((walk_back_time / self.walking_duration) * 100), 100)   
            elif self.current_mode == 6:  # 反向转弯模式
                turn_back_time = (now- self.init_start_time)
                progress = min(int((turn_back_time / self.target_turning_time) * 100), 100)   
            # 调试日志，输出当前进度
            self.get_logger().info(f"[调试] 模式: {self.current_mode}, 当前时间: {now}, 初始化时间: {self.init_start_time}, 进度: {progress}")
            # 更新并发布状态
            self.last_robot_progress = progress
            status_msg = Int32MultiArray()
            status_msg.data = [self.current_mode, progress]
            self.status_publisher.publish(status_msg)
            

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