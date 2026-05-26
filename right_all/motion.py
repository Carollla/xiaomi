#!/usr/bin/env python3
'''
鏈哄櫒浜鸿繍鍔ㄦ帶鍒舵ā鍧?'''
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
    鏈哄櫒浜鸿繍鍔ㄦ帶鍒惰妭鐐?    """
    def __init__(self):
        super().__init__('motion_controller')
        self.get_logger().info("motion controller init")
        self.ready_publisher = self.create_publisher(Bool, '/robot/ready', 10)
        self.status_publisher = self.create_publisher(Int32MultiArray, '/robot/status', 10)
        self.create_subscription(Int32MultiArray, '/robot/motion_cmd', self.motion_cmd_callback, 10)
        # 鎺у埗鍙橀噺
        self.current_mode = 0       # 褰撳墠妯″紡 0=绔欑珛 1=瓒翠笅 2=浣庡ご琛岃蛋 3=鎶ご鍚庨€€ 4=宸﹁浆寮?5=鍙宠浆寮?6=宸﹀钩绉?7=鍙冲钩绉?8=宸﹀渾璧?9=鍙冲渾璧?10=鐭虫澘璺?11=瓒翠笅璧拌矾
        self.current_command = []
        self.progress = 0
        self.is_ready = False       # 灏辩华鐘舵€佹爣蹇?        self.last_time = time.time()
        self.last_time = time.time()
        self.init_timeout = 10.0
        self.mode_lock = Lock()
        self.active_action = None
        self.active_params = {}
        self.last_lcm_send_at = 0.0
        self.lcm_resend_interval = 1.00
        self.suppress_action_log = False
        self.life_count = 0
        self.action_params = {
            0: {'func': self.execute_stand, 'params': {}},
            1: {'func': self.execute_down, 'params': {}},
            2: {'func': self.execute_head_down_walk, 'params': {'vel_x': 0.2, 'pitch': 0.5}},
            3: {'func': self.execute_down_back_walk, 'params': {'vel_x': -0.15, 'pitch': -0.5}},
            4: {'func': self.execute_left_turn, 'params': {'yaw_rate': 0.15}},
            5: {'func': self.execute_right_turn, 'params': {'yaw_rate': -0.15}},
            6: {'func': self.execute_left_move, 'params': {'vel_y': 0.1}},
            7: {'func': self.execute_right_move, 'params': {'vel_y': -0.1}},
            8: {'func': self.execute_left_round_walk, 'params': {'vel_x': 0.1, 'yaw_rate': 0.23}},
            9: {'func': self.execute_right_round_walk, 'params': {'vel_x': 0.1, 'yaw_rate': -0.23}},
            10: {'func': self.execute_flagstone_walk, 'params': {'vel_x': 0.1, 'height': 0.25, 'pitch': -0.2}},
            11: {'func': self.execute_down_walk, 'params': {}},
            12: {'func': self.execute_velocity_walk, 'params': {'vel_x': 0.12, 'vel_y': 0.0, 'yaw_rate': 0.0, 'pitch': 0.0}},
            13: {'func': self.execute_body_bump, 'params': {'vel_x': 0.45}},
            14: {'func': self.execute_bridge_walk, 'params': {'vel_x': 0.08, 'height': 0.18}},
            15: {'func': self.execute_jump_down, 'params': {'vel_x': 0.35}},
            16: {'func': self.execute_bridge_entry_climb, 'params': {'vel_x': 0.16, 'height': 0.26, 'pitch': -0.16, 'step_height': 0.14}},
            17: {'func': self.execute_jump3d_x30, 'params': {}},
            18: {'func': self.execute_jump3d_x60, 'params': {}},
            19: {'func': self.execute_high_step_yaw, 'params': {'vel_x': 0.10, 'height': 0.24, 'pitch': -0.08, 'yaw_rate': 0.0, 'step_height': 0.14}},
            20: {'func': self.execute_high_step_xy_yaw, 'params': {'vel_x': 0.10, 'vel_y': 0.0, 'height': 0.24, 'pitch': -0.08, 'yaw_rate': 0.0, 'step_height': 0.14}},
        }
        # 鍒濆鍖朙CM鎺у埗鍣?
        self.robot_ctrl = Robot_Ctrl()
        self.robot_ctrl.run()
        # 鎺у埗瀹氭椂鍣?
        self.motion_timer = self.create_timer(0.05, self.motion_control_loop)
        # 鎵ц鍒濆鍖?
        self.init_start_time = time.time()
        self.get_logger().info("motion controller booting")
        self.initialize_robot()
        self.get_logger().info("motion controller ready")
        script_dir = os.path.dirname(os.path.abspath(__file__))
        gait_def_path = os.path.join(script_dir, "Gait_Def_downwalk.toml")
        gait_params_path = os.path.join(script_dir, "Gait_Params_downwalk_full.toml")

        usergait_msg = file_send_lcmt()

        lcm_usergait = self.robot_ctrl.lc_s
        file_obj_gait_def = open(gait_def_path,'r')
        file_obj_gait_params = open(gait_params_path,'r')
        usergait_msg.data = file_obj_gait_def.read()
        lcm_usergait.publish("user_gait_file",usergait_msg.encode())
        time.sleep(0.5)
        usergait_msg.data = file_obj_gait_params.read()
        lcm_usergait.publish("user_gait_file",usergait_msg.encode())
        time.sleep(0.1)
        file_obj_gait_def.close()
        file_obj_gait_params.close()
        self.get_logger().info("motion log")

    def _send_cmd(self, cmd):
        self.life_count = (self.life_count + 1) % 128
        cmd.life_count = self.life_count
        self.robot_ctrl.Send_cmd(cmd)

    def initialize_robot(self):
        """
        鍒濆鍖栨満鍣ㄤ汉鐘舵€?        """
        self.get_logger().info("motion initialize: stand")
        self.execute_stand()
        time.sleep(3.0)
        self.is_ready = True
        ready_msg = Bool()
        ready_msg.data = True
        self.ready_publisher.publish(ready_msg)
        self.get_logger().info("motion initialize: ready published")

    def motion_cmd_callback(self, msg):
        """
        杩愬姩鍛戒护鍥炶皟鍑芥暟
        Args:
            msg.data[0]: 杩愬姩妯″紡
            msg.data[1:]: 鍙€夊弬鏁板垪琛紙鏁存暟锛岄渶瑕侀櫎浠?000杞崲涓烘诞鐐规暟锛?        """
        if len(msg.data) < 1:
            self.get_logger().info("motion log")
            return
            
        with self.mode_lock:
            new_mode = msg.data[0]
            new_command = list(msg.data)
            if new_command != self.current_command:
                self.current_mode = new_mode
                self.current_command = new_command
                self.progress = 0  # 閲嶇疆杩涘害
                self.get_logger().info("motion log")
                if new_mode in self.action_params:
                    action = self.action_params[new_mode]
                    params = action['params'].copy()
                    param_names = list(params.keys())
                    for i, value in enumerate(msg.data[1:], 1):
                        if i <= len(param_names):
                            params[param_names[i-1]] = value / 1000.0
                    self.active_action = action['func']
                    self.active_params = params
                    self.suppress_action_log = False
                    self.active_action(**self.active_params)
                    self.last_lcm_send_at = time.time()
                else:
                    self.active_action = None
                    self.active_params = {}
                    self.get_logger().info("motion log")

    def _send_locomotion(self, vel_x=0.0, vel_y=0.0, yaw_rate=0.0, pitch=0.0, height=0.0, gait_id=27, step_height=0.06, value=0):
        cmd = robot_control_cmd_lcmt()
        cmd.mode = 11
        cmd.gait_id = gait_id
        cmd.contact = 0
        cmd.life_count += 1
        cmd.pos_des = [0.0, 0.0, height]
        cmd.rpy_des = [0.0, pitch, 0.0]
        cmd.vel_des = [vel_x, vel_y, yaw_rate]
        cmd.acc_des = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        cmd.ctrl_point = [0.0, 0.0, 0.0]
        cmd.foot_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        cmd.step_height = [step_height, step_height]
        cmd.value = value
        cmd.duration = 0
        self._send_cmd(cmd)

    def _log_action(self, text):
        if not self.suppress_action_log:
            self.get_logger().info(text)

    def execute_stand(self):
        """鎵ц绔欑珛鍔ㄤ綔 mode = 0"""
        cmd = robot_control_cmd_lcmt()
        cmd.mode = 12               # 绔欑珛妯″紡
        cmd.gait_id = 0             # 鏍囧噯姝ユ€?        cmd.pos_des = [0.0, 0.0, 0.0]
        cmd.rpy_des = [0.0, 0.0, 0.0]
        cmd.acc_des = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        cmd.ctrl_point = [0.0, 0.0, 0.0]
        cmd.step_height = [0.0, 0.0]
        cmd.foot_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        cmd.duration = 0
        self._send_cmd(cmd)
        self._log_action("motion log")

    def execute_down(self):
        """鎵ц瓒翠笅鍔ㄤ綔 mode = 1"""
        cmd = robot_control_cmd_lcmt()
        cmd.mode = 7            # 瓒翠笅鐨勭姸鎬?        cmd.gait_id = 0
        cmd.life_count += 1
        cmd.duration = 0
        self._send_cmd(cmd)
        self._log_action("motion log")

    def execute_head_down_walk(self, vel_x=0.2, pitch=0.5):
        """鎵ц浣庡ご琛岃蛋鎺у埗 mode = 2
        Args:
            vel_x: 鍓嶈繘閫熷害
            pitch: 浣庡ご瑙掑害
        """
        cmd = robot_control_cmd_lcmt()
        cmd.mode = 11               # 鎺у埗妯″紡
        cmd.gait_id = 27             # 鎱㈣蛋姝ユ€?        cmd.contact = 0
        cmd.life_count += 1
        cmd.pos_des = [0.0, 0.0, 0.0]
        cmd.rpy_des = [0.0, pitch, 0.0]
        cmd.vel_des = [vel_x, 0.0, 0.0] 
        cmd.acc_des = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        cmd.foot_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        cmd.step_height = [0.06, 0.06]
        cmd.value = 0
        cmd.duration = 0
        self._send_cmd(cmd)
        self._log_action("motion log")

    def execute_down_back_walk(self, vel_x=-0.15, pitch=-0.5):
        """鎵ц鎶ご鍚庨€€琛岃蛋鎺у埗 mode = 3
        Args:
            vel_x: 鍚庨€€閫熷害
            pitch: 鎶ご瑙掑害
        """
        cmd = robot_control_cmd_lcmt()
        cmd.mode = 11               # 琛岃蛋妯″紡
        cmd.gait_id = 27             # 鎶ご鍚庨€€琛岃蛋姝ユ€?        cmd.contact = 0
        cmd.life_count += 1
        cmd.pos_des = [0.0, 0.0, 0.0]
        cmd.rpy_des = [0.0, pitch, 0.0]  # 鎶ご瑙掑害
        cmd.vel_des = [vel_x, 0.0, 0.0] 
        cmd.acc_des = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        cmd.foot_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        cmd.step_height = [0.06,0.06]
        cmd.value = 0
        cmd.duration = 0
        self._send_cmd(cmd)
        self._log_action("motion log")

    def execute_left_turn(self, yaw_rate=0.15):
        """鎵ц宸﹁浆鍔ㄤ綔 mode = 4
        Args:
            yaw_rate: 宸﹁浆瑙掗€熷害
        """
        cmd = robot_control_cmd_lcmt()
        cmd.mode = 11               # 琛岃蛋妯″紡
        cmd.gait_id = 27             # 鎱㈣蛋姝ユ€?        cmd.contact = 0
        cmd.life_count += 1
        cmd.pos_des = [0.0, 0.0, 0.0]
        cmd.rpy_des = [0.0, 0.0, 0.0] 
        cmd.vel_des = [0.0, 0.0, yaw_rate] 
        cmd.acc_des = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        cmd.foot_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        cmd.step_height = [0.06, 0.06]
        cmd.value = 0
        cmd.duration = 0
        self._send_cmd(cmd)
        self._log_action("motion log")

    def execute_right_turn(self, yaw_rate=-0.15):
        """鎵ц鍙宠浆鍔ㄤ綔 mode = 5
        Args:
            yaw_rate: 鍙宠浆瑙掗€熷害
        """
        cmd = robot_control_cmd_lcmt()
        cmd.mode = 11               # 琛岃蛋妯″紡
        cmd.gait_id = 27            # 鎱㈣蛋姝ユ€?        cmd.contact = 0
        cmd.life_count += 1
        cmd.pos_des = [0.0, 0.0, 0.0]
        cmd.rpy_des = [0.0, 0.0, 0.0]
        cmd.vel_des = [0.0, 0.0, yaw_rate]  # 鍙宠浆锛岃閫熷害涓鸿礋
        cmd.acc_des = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        cmd.foot_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        cmd.step_height = [0.06, 0.06]
        cmd.value = 0
        cmd.duration = 0
        self._send_cmd(cmd)
        self._log_action("motion log")

    def execute_left_move(self, vel_y=0.1):
        """鎵ц鍚戝乏骞崇Щ鍔ㄤ綔 mode = 6
        Args:
            vel_y: 宸︾Щ閫熷害
        """
        cmd = robot_control_cmd_lcmt()
        cmd.mode = 11               # 琛岃蛋妯″紡
        cmd.gait_id = 27            # 姝ユ€両D
        cmd.contact = 0
        cmd.life_count += 1
        cmd.pos_des = [0.0, 0.0, 0.0]
        cmd.rpy_des = [0.0, 0.0, 0.0]
        cmd.vel_des = [0.0, vel_y, 0.0]   # 鍚戝乏骞崇Щ锛寉杞翠负姝?        cmd.acc_des = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        cmd.foot_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        cmd.step_height = [0.06, 0.06]
        cmd.value = 0
        cmd.duration = 0
        self._send_cmd(cmd)
        self._log_action("motion log")

    def execute_right_move(self, vel_y=-0.1):
        """鎵ц鍚戝彸骞崇Щ鍔ㄤ綔 mode = 7
        Args:
            vel_y: 鍙崇Щ閫熷害
        """
        cmd = robot_control_cmd_lcmt()
        cmd.mode = 11               # 琛岃蛋妯″紡
        cmd.gait_id = 27             # 姝ユ€両D
        cmd.contact = 0
        cmd.life_count += 1
        cmd.pos_des = [0.0, 0.0, 0.0]
        cmd.rpy_des = [0.0, 0.0, 0.0] 
        cmd.vel_des = [0.0, vel_y, 0.0]  # 鍚戝彸骞崇Щ锛寉杞翠负璐?        cmd.acc_des = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        cmd.foot_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        cmd.step_height = [0.06, 0.06]
        cmd.value = 0
        cmd.duration = 0
        self._send_cmd(cmd)
        self._log_action("motion log")

    def execute_left_round_walk(self, vel_x=0.1, yaw_rate=0.23):
        """鎵ц宸﹀渾褰㈣璧?mode = 8
        Args:
            vel_x: 鍓嶈繘閫熷害
            yaw_rate: 宸﹁浆瑙掗€熷害
        """
        cmd = robot_control_cmd_lcmt()
        cmd.mode = 11               # LOCOMOTION妯″紡
        cmd.gait_id = 122             # 鍙橀璧?        cmd.life_count = 1
        cmd.vel_des = [vel_x, 0.0, yaw_rate]
        cmd.pos_des = [0.0, 0.0, 0.0]
        cmd.rpy_des = [0.0, 0.0, 0.0]
        cmd.acc_des = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        cmd.ctrl_point = [0.0, 0.0, 0.0]
        cmd.step_height = [0.06, 0.06]
        cmd.foot_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        cmd.duration = 0
        cmd.value = 20
        self._send_cmd(cmd)
        self._log_action("motion log")

    def execute_right_round_walk(self, vel_x=0.1, yaw_rate=-0.23):
        """鎵ц鍙冲渾褰㈣璧?mode = 9
        Args:
            vel_x: 鍓嶈繘閫熷害
            yaw_rate: 鍙宠浆瑙掗€熷害
        """
        cmd = robot_control_cmd_lcmt()
        cmd.mode = 11               # LOCOMOTION妯″紡
        cmd.gait_id = 122             
        cmd.life_count = 2
        cmd.vel_des = [vel_x, 0.0, yaw_rate]
        cmd.pos_des = [0.0, 0.0, 0.0]
        cmd.rpy_des = [0.0, 0.0, 0.0]
        cmd.acc_des = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        cmd.ctrl_point = [0.0, 0.0, 0.0]
        cmd.step_height = [0.06, 0.06]
        cmd.foot_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        cmd.duration = 0 
        self._send_cmd(cmd)
        self._log_action("motion log")

    def execute_flagstone_walk(self, vel_x=0.1, height=0.25, pitch=-0.2):
        """鎵ц鐭虫澘璺鎬?mode = 10
        Args:
            vel_x: 鍓嶈繘閫熷害
            height: 鎶吙楂樺害
            pitch: 淇话瑙掑害
        """
        cmd = robot_control_cmd_lcmt()
        cmd.mode = 11               # 琛岃蛋妯″紡
        cmd.gait_id = 122             # 鐭虫澘璺鎬?        cmd.contact = 0
        cmd.life_count += 1
        cmd.pos_des = [0.0, 0.0, height]
        cmd.rpy_des = [0.0, pitch, 0.0] 
        cmd.vel_des = [vel_x, 0.0, 0.0] 
        cmd.acc_des = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        cmd.foot_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        cmd.step_height = [0.12, 0.12]
        cmd.value = 20
        cmd.duration = 0
        self._send_cmd(cmd)
        self._log_action("motion log")

    def execute_down_walk(self):
        """鎵ц瓒翠笅璧拌矾姝ユ€?mode = 11"""
        cmd = robot_control_cmd_lcmt()
        cmd.mode = 62               # 鑷畾涔夋鎬佹ā寮?        cmd.gait_id = 110             
        cmd.contact = 15
        cmd.life_count += 1
        cmd.pos_des = [0.0, 0.0, 0.0]
        cmd.rpy_des = [0.0, 0.0, 0.0] 
        cmd.vel_des = [0.0, 0.0, 0.0] 
        cmd.acc_des = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        cmd.foot_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        cmd.step_height = [0.0, 0.0]
        cmd.value = 0
        cmd.duration = 0
        self._send_cmd(cmd)
        self._log_action("motion log")

    def execute_velocity_walk(self, vel_x=0.12, vel_y=0.0, yaw_rate=0.0, pitch=0.0):
        """鎵ц閫氱敤閫熷害闂幆琛岃蛋 mode = 12"""
        self._send_locomotion(
            vel_x=vel_x,
            vel_y=vel_y,
            yaw_rate=yaw_rate,
            pitch=pitch,
            gait_id=27,
            step_height=0.06,
        )
        self._log_action("motion log")

    def execute_body_bump(self, vel_x=0.45):
        """鎵ц鐭績鍓嶅啿鎾炲嚮 mode = 13"""
        self._send_locomotion(vel_x=vel_x, gait_id=122, step_height=0.08, value=20)
        self._log_action("motion log")

    def execute_bridge_walk(self, vel_x=0.08, height=0.18):
        """鎵ц鐙湪妗ヤ繚瀹堟參琛?mode = 14"""
        self._send_locomotion(vel_x=vel_x, height=height, gait_id=27, step_height=0.045)
        self._log_action("motion log")

    def execute_jump_down(self, vel_x=0.35):
        """鎵ц鐙湪妗ユ湯绔啿涓?mode = 15"""
        self._send_locomotion(vel_x=vel_x, pitch=-0.12, gait_id=122, step_height=0.10, value=20)
        self._log_action("motion log")

    def execute_bridge_entry_climb(self, vel_x=0.16, height=0.26, pitch=-0.16, step_height=0.14):
        """Bridge-entry lip climb. Short pulses only; sustained use can tip the robot."""
        self._send_locomotion(
            vel_x=vel_x,
            height=height,
            pitch=pitch,
            gait_id=122,
            step_height=step_height,
            value=20,
        )
        self._log_action("motion log")

    def execute_high_step_yaw(self, vel_x=0.10, height=0.24, pitch=-0.08, yaw_rate=0.0, step_height=0.14):
        """High step forward with a small yaw correction for tunnel/bridge recovery."""
        self._send_locomotion(
            vel_x=vel_x,
            yaw_rate=yaw_rate,
            height=height,
            pitch=pitch,
            gait_id=122,
            step_height=step_height,
            value=20,
        )
        self._log_action("motion log")

    def execute_high_step_xy_yaw(self, vel_x=0.10, vel_y=0.0, height=0.24, pitch=-0.08, yaw_rate=0.0, step_height=0.14):
        """High step with lateral and yaw correction for narrow bridge recovery."""
        self._send_locomotion(
            vel_x=vel_x,
            vel_y=vel_y,
            yaw_rate=yaw_rate,
            height=height,
            pitch=pitch,
            gait_id=122,
            step_height=step_height,
            value=20,
        )
        self._log_action("motion log")

    def _send_jump3d(self, jump_id):
        cmd = robot_control_cmd_lcmt()
        cmd.mode = 16
        cmd.gait_id = int(jump_id)
        cmd.contact = 0
        cmd.pos_des = [0.0, 0.0, 0.0]
        cmd.rpy_des = [0.0, 0.0, 0.0]
        cmd.vel_des = [0.0, 0.0, 0.0]
        cmd.acc_des = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        cmd.ctrl_point = [0.0, 0.0, 0.0]
        cmd.foot_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        cmd.step_height = [0.0, 0.0]
        cmd.value = 0
        cmd.duration = 0
        self._send_cmd(cmd)

    def execute_jump3d_x30(self):
        """Official jump3D forward 30cm trajectory, jump_id=4."""
        self._send_jump3d(4)
        self._log_action("motion log")

    def execute_jump3d_x60(self):
        """Official jump3D forward 60cm trajectory, jump_id=1."""
        self._send_jump3d(1)
        self._log_action("motion log")

    def motion_control_loop(self):
        with self.mode_lock:
            now = time.time()
            dt = now - self.last_time
            self.last_time = now
            if not self.is_ready:
                if time.time() - self.init_start_time > self.init_timeout:
                    self._log_action("motion log")
                    self.is_ready = True
                    ready_msg = Bool()
                    ready_msg.data = True
                    self.ready_publisher.publish(ready_msg)
                return
            if (
                self.active_action is not None
                and self.current_mode != 1
                and now - self.last_lcm_send_at >= self.lcm_resend_interval
            ):
                self.suppress_action_log = True
                self.active_action(**self.active_params)
                self.suppress_action_log = False
                self.last_lcm_send_at = now

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
