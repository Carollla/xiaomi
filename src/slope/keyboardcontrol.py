#!/usr/bin/env python3
import lcm
import sys
import tty
import termios
import select
from time import sleep
from robot_control.robot_control_cmd_lcmt import robot_control_cmd_lcmt

# LCM配置
LCM_CMD_CHANNEL = "robot_control_cmd"
LCM_GROUP = "239.255.76.67"
LCM_PORT = 7671

class LCMKeyboardControl:
    def __init__(self):
        # LCM初始化
        self.lc = lcm.LCM(f"udpm://{LCM_GROUP}:{LCM_PORT}?ttl=255")
        self.msg = robot_control_cmd_lcmt()
        
        # 控制参数
        self.running = True
        self.speed = 0.5
        self.max_speed = 1.5
        self.angular_speed = 0.8
        self.current_mode = 0  # 0:初始状态 11:运动模式 12:站立模式
        self.old_settings = termios.tcgetattr(sys.stdin)
        
        # 初始化默认消息
        self._reset_msg()

    def _reset_msg(self):
        """重置消息到运动模式默认值"""
        self.msg.mode = 11
        self.msg.gait_id = 26
        self.msg.pos_des = [0.0, 0.0, 0.25]
        self.msg.step_height = [0.15, 0.15]
        self.msg.rpy_des = [0.0, 0.0, 0.0]
        self.msg.duration = 0
        self.msg.life_count = 0
        
    def _update_life_count(self):
        """安全更新life_count（保持在int8范围）"""
        new_count = self.msg.life_count + 1
        self.msg.life_count = new_count if new_count <= 127 else -128

    def _send_cmd(self, vx=0.0, vy=0.0, wz=0.0):
        """发送运动指令"""
        if self.current_mode == 12:
            stand_msg = robot_control_cmd_lcmt()
            stand_msg.mode = 12
            stand_msg.gait_id = 0
            self._update_life_count()  # 使用安全更新方法
            stand_msg.life_count = self.msg.life_count
            self.lc.publish(LCM_CMD_CHANNEL, stand_msg.encode())
        else:
            self.msg.vel_des = [vx, vy, wz]
            self._update_life_count()  # 使用安全更新方法
            self.lc.publish(LCM_CMD_CHANNEL, self.msg.encode())

    def _stand_up(self):
        """进入持续站立模式"""
        self.current_mode = 12
        # 发送初始站立指令
        stand_msg = robot_control_cmd_lcmt()
        stand_msg.mode = 12
        stand_msg.gait_id = 0
        stand_msg.life_count = 1
        self.lc.publish(LCM_CMD_CHANNEL, stand_msg.encode())
        self.msg.life_count = 1  # 同步计数器
        print("\n进入站立模式，持续保持站立状态!")

    def _exit_stand(self):
        """退出站立模式"""
        self.current_mode = 11
        self._reset_msg()
        print("\n退出站立模式，返回运动控制")

    def _get_key(self):
        """非阻塞获取键盘输入"""
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        return key

    def _print_help(self):
        print("\nLCM直连控制模式")
        print("---------------------------")
        print("运动控制：")
        print("  w   前进 | s 后退")
        print("  a 左转 | d 右转")
        print("  z 左移 | c 右移")
        print("  x 站立 | v 急停")
        print("  q 加速 | e 减速")
        print("  CTRL-C 退出")

    def run(self):
        self._print_help()
        try:
            while self.running:
                key = self._get_key()
                
                if key == '\x03':  # CTRL-C
                    break
                    
                # 站立控制
                if key == 'x':
                    self._stand_up()
                    continue
                    
                # 如果当前是站立模式，任何运动按键退出站立
                if self.current_mode == 12 and key in ['w','s','a','d','z','c']:
                    self._exit_stand()

                # 速度调节
                if key == 'q':
                    self.speed = min(self.speed*1.1, self.max_speed)
                    print(f"\n当前速度: {self.speed:.2f} m/s")
                elif key == 'e':
                    self.speed = max(self.speed*0.9, 0.1)
                    print(f"\n当前速度: {self.speed:.2f} m/s")

                # 运动控制
                vx, vy, wz = 0.0, 0.0, 0.0
                if self.current_mode == 11:  # 只在运动模式下响应
                    if key == 'w':     vx = self.speed
                    elif key == 's':  vx = -self.speed
                    elif key == 'a':  wz = self.angular_speed
                    elif key == 'd':  wz = -self.angular_speed
                    elif key == 'z':   vy = self.speed
                    elif key == 'c':   vy = -self.speed
                    elif key == 'v':  vx = vy = wz = 0.0
                    
                # 发送指令（站立模式会自动发送保持指令）
                self._send_cmd(vx, vy, wz)
                sleep(0.05)

        finally:
            self._send_cmd(0, 0, 0)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
            print("\n控制已断开!")

if __name__ == '__main__':
    try:
        controller = LCMKeyboardControl()
        controller.run()
    except Exception as e:
        print(f"错误发生: {str(e)}")