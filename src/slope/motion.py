#!/usr/bin/env python3
'''机器人运动控制模块'''
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32MultiArray
import time
import math
from threading import Lock
from robot_control.robot_control_class import Robot_Ctrl
from robot_control.robot_control_cmd_lcmt import robot_control_cmd_lcmt

class MotionController(Node):
    def __init__(self):
        super().__init__('motion_controller')
        # 发布器和订阅
        self.ready_publisher = self.create_publisher(Bool, '/robot/ready', 10)
        self.status_publisher = self.create_publisher(Int32MultiArray, '/robot/status', 10)
        self.create_subscription(Int32MultiArray, '/robot/motion_cmd', self.motion_cmd_callback, 10)
        
        # 状态和参数
        self.mode = 0
        self.target_angle = 0.0
        self.is_ready = False
        self.lock = Lock()  # 简化为单一锁
        
        # 转向控制
        self.turning = {
            'start_time': None,
            'duration': 0.0,
            'completed': False
        }
        
        # 参数初始化
        self.declare_parameter('max_angular_vel', 0.5)
        self.declare_parameter('max_linear_vel', 0.15)
        self.declare_parameter('init_timeout', 5.0)
        self.max_angular_vel = self.get_parameter('max_angular_vel').get_parameter_value().double_value
        self.max_linear_vel = self.get_parameter('max_linear_vel').get_parameter_value().double_value
        self.init_timeout = self.get_parameter('init_timeout').get_parameter_value().double_value
        
        # 机器人控制器
        self.robot_ctrl = Robot_Ctrl()
        self.robot_ctrl.run()
        self.last_progress = 0
        
        # 定时器
        self.create_timer(0.05, self.control_loop)
        self.create_timer(0.2, self.publish_status)
        
        # 初始化
        self.init_time = time.time()
        self.last_time = time.time()
        self.init_robot()

    def init_robot(self):
        """初始化机器人"""
        self.get_logger().info("开始初始化机器人...")
        
        # 检查机器人控制器是否正确初始化
        if not hasattr(self, 'robot_ctrl') or self.robot_ctrl is None:
            self.get_logger().error("机器人控制器未初始化")
            return
        
        # 首先确保机器人是站立状态
        self.get_logger().info("发送站立指令...")
        self.execute_stand()
        
        # 等待站立命令完成，最多等待12秒
        self.get_logger().info("等待站立命令完成...")
        if self.wait_cmd_complete(12, 0):
            self.get_logger().info("站立命令已完成")
        else:
            # 如果等待失败，尝试重新发送命令
            self.get_logger().warn("站立命令等待超时，尝试重新发送...")
            time.sleep(1.0)
            self.execute_stand()
            time.sleep(3.0)  # 强制等待一段时间
        
        # 获取当前机器人状态并记录
        progress = getattr(self.robot_ctrl.rec_msg, 'order_process_bar', 0)
        self.get_logger().info(f"当前机器人进度: {progress}%")
        
        # 等待稳定
        time.sleep(2.0)
        
        # 设置就绪状态并发布
        self.is_ready = True
        ready_msg = Bool()
        ready_msg.data = True
        self.ready_publisher.publish(ready_msg)
        self.get_logger().info("机器人初始化完成，已发布就绪信号")

    def wait_cmd_complete(self, mode, gait_id):
        """等待命令完成"""
        try:
            # 添加重试逻辑
            max_retries = 3
            for attempt in range(max_retries):
                result = self.robot_ctrl.Wait_finish(mode, gait_id)
                if result:
                    self.get_logger().info(f"命令完成: 模式={mode}, 步态={gait_id}")
                    return True
                else:
                    self.get_logger().warn(f"等待命令完成失败(尝试 {attempt + 1}/{max_retries}): 模式={mode}, 步态={gait_id}")
                    if attempt < max_retries - 1:
                        time.sleep(0.5)  # 重试前等待
            
            return False
        except Exception as e:
            self.get_logger().error(f"等待命令完成时发生异常: {str(e)}")
            return False

    def motion_cmd_callback(self, msg):
        """处理运动命令"""
        if len(msg.data) < 2:
            self.get_logger().warn("接收到无效命令格式")
            return
            
        with self.lock:
            mode = msg.data[0]
            value = msg.data[1]
            
            # 相同模式下更新参数
            if mode == self.mode:
                if mode == 1:  # 转向模式
                    old_angle = self.target_angle
                    self.target_angle = value / 100.0
                    self.turning['completed'] = False
                    self.calc_turn_duration()
                    self.get_logger().info(f"更新转向角度: {old_angle}° -> {self.target_angle}°")
                return
            
            # 模式切换
            old_mode = self.mode
            self.mode = mode
            self.get_logger().info(f"模式切换: {old_mode} -> {mode}")
            
            # 执行相应动作
            if mode == 0:      # 站立
                self.get_logger().info("执行站立模式")
                self.execute_stand()
            elif mode == 1:    # 转向
                self.target_angle = value / 100.0
                self.get_logger().info(f"执行转向模式，目标角度: {self.target_angle}°")
                self.turning['completed'] = False
                self.calc_turn_duration()
            elif mode == 2:    # 爬坡
                self.get_logger().info(f"执行爬坡模式，最大速度: {self.max_linear_vel} m/s")
            # 爬坡在control_loop中执行

    def calc_turn_duration(self):
        """计算转向时间"""
        angular_speed = self.max_angular_vel * 180.0 / math.pi
        self.turning['duration'] = abs(self.target_angle) / angular_speed if angular_speed > 0 else 0
        self.turning['start_time'] = None  # 重置开始时间
        self.get_logger().info(f"计算转向时间: {self.turning['duration']}秒 (角速度: {angular_speed}°/s)")

    def execute_stand(self):
        """执行站立命令"""
        try:
            cmd = robot_control_cmd_lcmt()
            cmd.mode = 12
            cmd.gait_id = 0
            cmd.vel_des = [0.0, 0.0, 0.0]
            cmd.pos_des = [0.0, 0.0, 0.32]
            cmd.rpy_des = [0.0, 0.0, 0.0]
            cmd.step_height = [0.0, 0.0]
            cmd.life_count += 1  # 更新life_count使命令生效
            
            # 增加发送重试逻辑
            max_retries = 3
            for attempt in range(max_retries):
                try:
                    self.robot_ctrl.Send_cmd(cmd)
                    self.get_logger().info(f"已发送站立命令(尝试 {attempt + 1}/{max_retries}): 模式={cmd.mode}, 高度={cmd.pos_des[2]}米")
                    # 发送成功后短暂等待
                    time.sleep(0.1)
                    return True
                except Exception as e:
                    self.get_logger().error(f"站立命令发送失败(尝试 {attempt + 1}/{max_retries}): {str(e)}")
                    if attempt < max_retries - 1:
                        time.sleep(0.5)  # 重试前等待
            
            return False
        except Exception as e:
            self.get_logger().error(f"创建站立命令时出错: {str(e)}")
            return False

    def execute_turn(self):
        """执行转向命令"""
        if self.turning['completed']:
            return 100
            
        # 确定转向方向
        angular_z = self.max_angular_vel if self.target_angle < 0 else -self.max_angular_vel
        
        # 初始化转向开始时间
        if self.turning['start_time'] is None:
            self.turning['start_time'] = time.time()
            direction = "顺时针" if angular_z > 0 else "逆时针"
            self.get_logger().info(f"开始转向，目标角度: {self.target_angle}°，预计时间: {self.turning['duration']}秒，方向: {direction}")
            
        # 计算已经过时间
        elapsed = time.time() - self.turning['start_time']
        
        # 完成转向
        if elapsed >= self.turning['duration']:
            # 发送停止转向命令
            cmd = robot_control_cmd_lcmt()
            cmd.mode = 11
            cmd.gait_id = 27
            cmd.vel_des = [0.0, 0.0, 0.0]
            cmd.pos_des = [0.0, 0.0, 0.28]
            cmd.step_height = [0.03, 0.03]
            cmd.rpy_des = [0.0, 0.0, 0.0]
            cmd.life_count += 1
            self.robot_ctrl.Send_cmd(cmd)
            self.get_logger().info("转向完成，发送停止命令")
            
            # 标记转向完成并返回站立姿态
            self.turning['completed'] = True
            self.get_logger().info("转向完成，返回站立状态")
            self.execute_stand()
            return 100
            
        # 继续转向
        cmd = robot_control_cmd_lcmt()
        cmd.mode = 11
        cmd.gait_id = 27
        cmd.vel_des = [0.0, 0.0, angular_z]
        cmd.pos_des = [0.0, 0.0, 0.28]
        cmd.step_height = [0.03, 0.03]
        cmd.rpy_des = [0.0, 0.0, 0.0]
        cmd.life_count += 1
        self.robot_ctrl.Send_cmd(cmd)
        
        # 计算进度
        progress = min(int((elapsed / self.turning['duration']) * 100), 99)
        # 每10%进度记录一次日志
        if progress % 10 == 0 and progress != self.last_progress:
            self.get_logger().debug(f"转向进度: {progress}%")
        
        return progress

    def execute_climb(self):
        """爬坡命令 - 保持原参数不变"""
        cmd = robot_control_cmd_lcmt()
        cmd.mode = 11
        cmd.gait_id = 122
        cmd.vel_des = [self.max_linear_vel, 0.0, 0.0]
        cmd.pos_des = [0.0, 0.0, 0.25]
        cmd.step_height = [0.12, 0.12]
        cmd.rpy_des = [0.0, -0.2, 0.0]
        cmd.value = 20
        cmd.life_count += 1
        self.robot_ctrl.Send_cmd(cmd)

    def control_loop(self):
        """控制主循环"""
        with self.lock:
            now = time.time()
            dt = now - self.last_time
            self.last_time = now
            
            # 检查初始化超时
            if not self.is_ready:
                if time.time() - self.init_time > self.init_timeout:
                    self.get_logger().warn(f"初始化超时 ({self.init_timeout}秒)，强制设置为就绪状态")
                    self.is_ready = True
                    ready_msg = Bool()
                    ready_msg.data = True
                    self.ready_publisher.publish(ready_msg)
                return
            
            # 根据模式执行控制
            progress = 0
            if self.mode == 0:     # 站立模式
                progress = 100
            elif self.mode == 1:   # 转向模式
                progress = self.execute_turn()
            elif self.mode == 2:   # 爬坡模式
                self.execute_climb()
                progress = 100
                
            self.last_progress = progress

    def publish_status(self):
        """发布状态信息"""
        with self.lock:
            # 获取当前进度
            if self.mode == 1:  # 转向模式
                progress = self.last_progress
            else:
                progress = getattr(self.robot_ctrl.rec_msg, 'order_process_bar', 0)
                
            status_msg = Int32MultiArray()
            status_msg.data = [self.mode, progress]
        
        self.status_publisher.publish(status_msg)

    def destroy_node(self):
        """清理节点资源"""
        # 取消定时器
        self.get_logger().info("关闭机器人控制节点...")
        for timer_attr in ['control_timer', 'status_timer']:
            if hasattr(self, timer_attr) and getattr(self, timer_attr):
                getattr(self, timer_attr).cancel()
                
        # 发送站立命令确保安全
        self.get_logger().info("发送最终站立命令，确保机器人安全")
        try:
            cmd = robot_control_cmd_lcmt()
            cmd.mode = 12
            cmd.gait_id = 0
            cmd.life_count += 1
            self.robot_ctrl.Send_cmd(cmd)
            time.sleep(0.1)
        except Exception as e:
            self.get_logger().error(f"发送最终站立命令失败: {str(e)}")
        
        # 关闭机器人控制器
        if hasattr(self, 'robot_ctrl'):
            self.get_logger().info("关闭机器人控制线程")
            try:
                self.robot_ctrl.runing = 0  # 修正为正确的关闭方式
                time.sleep(0.5)  # 等待线程结束
            except Exception as e:
                self.get_logger().error(f"关闭机器人控制线程失败: {str(e)}")
        
        super().destroy_node()
        self.get_logger().info("机器人控制节点已关闭")

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MotionController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n接收到退出信号，正在关闭机器人控制节点...")
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
