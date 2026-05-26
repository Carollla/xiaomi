#!/usr/bin/env python3
'''
机器人状态机管理模块
负责处理状态转换逻辑和协调系统各部分功能
'''
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool, Int32MultiArray
import time
import math

class RobotStateMachine(Node):
    '''
    机器人状态机节点：根据激光检测结果和机器人状态，协调不同操作模式的切换
    '''
    def __init__(self):
        super().__init__('robot_state_machine')
        # 参数设置
        self.declare_parameters(
            '',
            [
                ('turn_precision', 1.0),
                ('final_angle_threshold', 5.0),
                ('robot_init_wait_time', 15.0),
                ('max_angular_vel', 1.0),
            ]
        )
        
        self.angle_threshold = self.get_parameter('final_angle_threshold').get_parameter_value().double_value
        self.init_timeout = self.get_parameter('robot_init_wait_time').get_parameter_value().double_value
        
        # 话题订阅
        self.create_subscription(Bool, '/lidar/initial_data_ready', self.on_lidar_ready, 10)
        self.create_subscription(Float32MultiArray, '/lidar/optimal_angle', self.on_angle_data, 10)
        self.create_subscription(Int32MultiArray, '/robot/status', self.on_robot_status, 10)
        self.create_subscription(Bool, '/robot/ready', self.on_robot_ready, 10)
        
        # 话题发布
        self.cmd_publisher = self.create_publisher(Int32MultiArray, '/robot/motion_cmd', 10)
        self.recollect_publisher = self.create_publisher(Bool, '/lidar/recollect', 10)
        
        # 状态变量
        self.state = "INIT"                    # 初始状态
        self.robot_ready = False               # 机器人是否就绪
        self.target_angle = 0.0                # 目标角度
        self.adjust_count = 0                  # 调整计数
        self.init_time = time.time()           # 初始化时间
        self.robot_state = {
            'mode': 0,                         # 机器人模式
            'progress': 0                      # 机器人进度
        }
        
        # 命令发送标志
        self.cmds_sent = {
            'turn': False,                     # 转向命令
            'stand': False,                    # 站立命令
            'climb': False,                    # 爬坡命令
            'recollect': False                 # 重采集命令
        }
        
        self.waiting_turn_complete = False     # 等待转向完成
        
        # 创建定时器
        self.create_timer(0.05, self.state_machine_loop)
        self.create_timer(1.0, self.check_recollect)


    def on_robot_ready(self, msg):
        """机器人就绪状态回调"""
        if msg.data and not self.robot_ready:
            self.robot_ready = True
            if self.state == "INIT":
                old_state = self.state
                self.state = "WAIT_LIDAR"
                self.init_time = time.time()
                self.get_logger().info(f"机器人就绪，状态切换: {old_state} -> {self.state}")
                # 重置命令标志，确保状态机能够正常发送命令
                self.reset_cmd_flags()
                # 主动触发激光数据采集
                self.trigger_recollect()
        elif not msg.data and self.robot_ready:
            old_state = self.state
            self.robot_ready = False
            self.state = "INIT"
            self.get_logger().warn(f"机器人未就绪，状态切换: {old_state} -> {self.state}")

    def on_robot_status(self, msg):
        """机器人状态回调"""
        if len(msg.data) >= 2:
            old_mode = self.robot_state['mode']
            old_progress = self.robot_state['progress']
            
            self.robot_state['mode'] = msg.data[0]
            self.robot_state['progress'] = msg.data[1]
            
            # 进度变化较大时记录日志
            if abs(self.robot_state['progress'] - old_progress) > 20:
                self.get_logger().debug(f"机器人进度更新: {old_progress}% -> {self.robot_state['progress']}%")
            
            # 模式变化时记录日志
            if self.robot_state['mode'] != old_mode:
                self.get_logger().info(f"机器人模式变化: {old_mode} -> {self.robot_state['mode']}")
            
            # 检查转向完成
            turning_done = (
                (self.state == "TURNING" and self.robot_state['mode'] == 1 and self.robot_state['progress'] >= 99) or
                (self.state == "TURNING" and self.robot_state['mode'] == 0 and old_mode == 1 and old_progress >= 99)
            )
            if turning_done:
                self.waiting_turn_complete = False
                old_state = self.state
                self.state = "STANDING"
                self.get_logger().info(f"转向完成 (进度={self.robot_state['progress']}%)，状态切换: {old_state} -> {self.state}")
                self.reset_cmd_flags()
                self.trigger_recollect()
                
            # 检查站立完成
            if self.state == "STANDING" and self.robot_state['mode'] == 0 and self.robot_state['progress'] >= 95:
                old_state = self.state
                self.state = "RECHECK"
                self.get_logger().info(f"站立完成 (进度={self.robot_state['progress']}%)，状态切换: {old_state} -> {self.state}")
                self.cmds_sent['recollect'] = False

    def on_lidar_ready(self, msg):
        """雷达数据就绪回调"""
        if msg.data and self.robot_ready:
            self.get_logger().info(f"收到激光数据就绪信号，当前状态: {self.state}")
            self.cmds_sent['recollect'] = False

    def on_angle_data(self, msg):
        """角度数据回调"""
        if not self.robot_ready:
            self.get_logger().debug("收到角度数据，但机器人未就绪，忽略")
            return
            
        if len(msg.data) >= 1 and self.state in ["WAIT_LIDAR", "RECHECK"]:
            angle = msg.data[0]
            if math.isnan(angle) or math.isinf(angle):
                self.get_logger().warn("收到无效角度值，请检查激光雷达数据")
                return
                
            self.get_logger().info(f"收到最优角度数据: {angle}°，当前状态: {self.state}")
                
            # 判断角度是否小于阈值
            if abs(angle) <= self.angle_threshold:
                old_state = self.state
                self.state = "CLIMBING"
                self.get_logger().info(f"角度 {angle}° 小于阈值 {self.angle_threshold}°，无需调整，切换到爬坡模式: {old_state} -> {self.state}")
                self.cmds_sent['climb'] = False
                self.adjust_count = 0
                return
                
            # 记录调整次数
            self.adjust_count += 1
            
            # 设置转向参数
            old_state = self.state
            self.target_angle = angle
            self.state = "TURNING"
            self.get_logger().info(f"需要转向调整 {angle}°，状态切换: {old_state} -> {self.state} (第 {self.adjust_count} 次调整)")
            self.cmds_sent['turn'] = False
            self.waiting_turn_complete = True

    def reset_cmd_flags(self):
        """重置命令标志"""
        for key in self.cmds_sent:
            self.cmds_sent[key] = False
        self.get_logger().debug("已重置所有命令发送标志")

    def trigger_recollect(self):
        """触发激光重新采集"""
        recollect_msg = Bool()
        recollect_msg.data = True
        self.recollect_publisher.publish(recollect_msg)
        self.get_logger().info("已发送激光重新采集请求")
        self.cmds_sent['recollect'] = True

    def check_recollect(self):
        """检查是否需要重新采集激光数据"""
        if self.state == "RECHECK" and not self.cmds_sent['recollect']:
            self.get_logger().info(f"触发定时激光数据重新采集，当前状态: {self.state}")
            self.trigger_recollect()

    def check_timeout(self):
        """检查超时"""
        if self.state == "INIT":
            elapsed = time.time() - self.init_time
            if elapsed > self.init_timeout:
                self.get_logger().warn(f"初始化超时 ({self.init_timeout}秒)，强制切换状态: {self.state} -> WAIT_LIDAR")
                self.robot_ready = True
                self.state = "WAIT_LIDAR"
            elif elapsed > self.init_timeout * 0.8 and elapsed % 1 < 0.1:  # 接近超时时每秒输出一次警告
                self.get_logger().warn(f"初始化即将超时，已用时 {elapsed}/{self.init_timeout} 秒")
        
        # 添加对WAIT_LIDAR状态的超时处理
        elif self.state == "WAIT_LIDAR":
            elapsed = time.time() - self.init_time
            if elapsed > self.init_timeout * 2:  # 设置更长的超时时间
                self.get_logger().warn("等待激光数据超时，强制切换到爬坡状态")
                self.state = "CLIMBING"
                self.cmds_sent['climb'] = False

    def state_machine_loop(self):
        """状态机主循环"""
        # 检查机器人初始化超时
        if not self.robot_ready:
            self.check_timeout()
            return
            
        # 发送相应命令
        if self.state == "TURNING" and not self.cmds_sent['turn'] and self.waiting_turn_complete:
            # 发送转向命令
            self.send_motion_cmd(1, int(self.target_angle * 100))
            self.get_logger().info(f"发送转向命令: 角度={self.target_angle}°")
            self.cmds_sent['turn'] = True
            
        elif self.state == "STANDING" and not self.cmds_sent['stand']:
            # 发送站立命令
            self.send_motion_cmd(0, 0)
            self.get_logger().info("发送站立命令")
            self.cmds_sent['stand'] = True
            
        elif self.state == "CLIMBING" and not self.cmds_sent['climb']:
            # 发送爬坡命令
            self.send_motion_cmd(2, 0)
            self.get_logger().info("发送爬坡命令")
            self.cmds_sent['climb'] = True
    
    def send_motion_cmd(self, mode, value):
        """发送运动命令"""
        cmd_msg = Int32MultiArray()
        cmd_msg.data = [mode, value]
        self.cmd_publisher.publish(cmd_msg)
        self.get_logger().debug(f"已发布运动命令: 模式={mode}, 值={value}")

    def destroy_node(self):
        """销毁节点时的清理工作"""
        self.get_logger().info("状态机节点正在关闭...")
        
        # 如果在转向或爬坡模式，发送站立命令
        if self.state in ["TURNING", "CLIMBING"]:
            self.get_logger().info(f"从 {self.state} 状态安全退出，发送站立命令")
            try:
                self.send_motion_cmd(0, 0)
                time.sleep(0.1)  # 确保命令被发送
            except Exception as e:
                self.get_logger().error(f"关闭时发送站立命令失败: {str(e)}")
        
        super().destroy_node()
        self.get_logger().info("状态机节点已关闭")

def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    node = None
    try:
        node = RobotStateMachine()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n接收到退出信号，正在关闭状态机节点...")
    except Exception as e:
        print(f"发生错误: {str(e)}")
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
