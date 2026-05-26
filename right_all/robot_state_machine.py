#!/usr/bin/env python3
'''
鏈哄櫒浜虹姸鎬佹満绠＄悊妯″潡
璐熻矗澶勭悊鏈哄櫒浜虹姸鎬佽浆鎹㈠拰瑙嗚妫€娴嬫暣鍚?'''
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32MultiArray
import time
import cv2
from collections import Counter, deque
from detect.RGB_detect import detect_yellow, slope_move_right_end, detect_arrow
from detect.D435_detect import detect_top_bar
from detect.IMU_detect import is_facing_cardinal_direction, get_current_yaw, get_adjustment_direction
from detect.track_detector import TrackDetector

class RobotStateMachine(Node):
    '''
    鐘舵€佹満娴佺▼锛?    1. 浠庡垵濮嬩綅缃埌A搴擄細
    INIT_GO_FORWARD -> INIT_RIGHT_TURN_AND_SCAN_QR_CODE -> INIT_SCAN_QR_CODE -> WAREHOUSE_A_2_TURN_RIGHT_TO_SOUTH -> WAREHOUSE_A_2_TURN_RIGHT_TO_WEST -> WAREHOUSE_A_2_IN -> WAREHOUSE_A_2_DOWN
    
    2. 浠嶢2搴撳埌S寮細
    WAREHOUSE_A_2_DOWN -> WAREHOUSE_A_2_OUT -> WAREHOUSE_A_2_BACK_TURN_RIGHT -> WAREHOUSE_A_2_BACK_LEFT_TURN -> TO_S_RIGHT_TURN -> S_CURVE_GO
    
    3. 浠嶴寮埌B2搴擄細
    S_CURVE_GO -> WAREHOUSE_B_GO_PREPARE -> WAREHOUSE_B_2_GO_RIGHT_MOVE -> WAREHOUSE_B_2_GO_SLOPE_CLIMB -> WAREHOUSE_B_2_GO_YELLOW_LIGHT -> WAREHOUSE_B_2_GO_SCAN_B_QR -> WAREHOUSE_B_2_IN -> WAREHOUSE_B_2_GO_DOWN
    
    4. B2鍒癇1锛?    WAREHOUSE_B_2_GO_DOWN -> WAREHOUSE_B_2_OUT -> WAREHOUSE_B_2_TO_1_TURN_LEFT -> WAREHOUSE_B_2_TO_1_DOWN_WALK -> WAREHOUSE_B_2_TO_1_TURN_RIGHT -> WAREHOUSE_B_1_IN -> WAREHOUSE_B_1_DOWN
    
    5. B1杩斿洖S寮細
    WAREHOUSE_B_1_DOWN -> WAREHOUSE_B_1_BACK_TURN_REVERSE -> WAREHOUSE_B_1_BACK_FORWARD -> WAREHOUSE_B_1_BACK_LIMIT_BAR -> WAREHOUSE_B_1_BACK_STONE_ROAD -> WAREHOUSE_B_1_BACK_TURN_LEFT -> WAREHOUSE_B_1_BACK_TO_S -> S_CURVE_BACK
    
    6. 浠嶴寮繑鍥濧1搴擄細
    S_CURVE_BACK -> OUT_S_LEFT_TURN -> WAREHOUSE_A_1_TURN_LEFT_TO_NORTH -> WAREHOUSE_A_1_TURN_LEFT_TO_WEST -> WAREHOUSE_A_1_IN -> WAREHOUSE_A_1_DOWN
    
    7. 浠嶢1搴撹繑鍥炶捣鐐癸細
    WAREHOUSE_A_1_DOWN -> WAREHOUSE_A_1_OUT -> WAREHOUSE_A_1_BACK_TURN_LEFT -> WAREHOUSE_A_1_BACK_RIGHT_TURN -> TO_HOME_LEFT_TURN -> TO_HOME_IN -> TO_HOME_DOWN -> FINISH
    
    姣忎釜鐘舵€佸垏鎹㈠墠閮戒細鍏堝彂閫佸仠姝㈠懡浠わ紝纭繚鏈哄櫒浜鸿繍鍔ㄧǔ瀹氥€?    '''
    def __init__(self, imu_provider, d435_provider, rgb_provider):
        super().__init__('robot_state_machine')
        self.create_subscription(Bool, '/robot/ready', self.robot_ready_callback, 10)
        self.motion_cmd_publisher = self.create_publisher(Int32MultiArray, '/robot/motion_cmd', 10)
        self.completion_publisher = self.create_publisher(Bool, '/robot/completion', 10)  # 娣诲姞瀹屾垚淇″彿鍙戝竷鑰?        
        self.state = "INIT_GO_FORWARD"  # 鍒濆鐘舵€佽涓篒NIT_GO_FORWARD
        self.robot_initialized = False
        self.qr_data = None  # 娣诲姞浜岀淮鐮佹暟鎹瓨鍌ㄥ彉閲?        
        self.publish_timer = self.create_timer(0.1, self.publish_control_command)
        self.track_detector = TrackDetector()
        self._qr_history = deque(maxlen=7)
        
        # ---- 1. 鍒濆浣嶇疆鍒癆搴撶浉鍏冲彉閲?----
        self._init_forward_cmd_sent = False
        self._init_right_turn_cmd_sent = False
        self._init_right_turn_started_at = None
        self._init_scan_qr_code_cmd_sent = False
        self._a2_turn_right_to_south_cmd_sent = False
        self._a2_turn_right_to_west_cmd_sent = False
        self._a2_in_cmd_sent = False
        self._a2_down_cmd_sent = False
        # ---- 2. A2搴撳埌S寮浉鍏冲彉閲?----
        self._up_back_walk_cmd_sent = False
        self._turn_reverse_cmd_sent = False
        self._turn_right_cmd_sent = False
        self._forward_cmd_sent = False
        # ---- 3. S寮埌B2搴撶浉鍏冲彉閲?----
        self._prepare_cmd_sent = False
        self._right_move_cmd_sent = False
        self._slope_climb_cmd_sent = False
        self._scan_b_qr_cmd_sent = False
        self._warehouse_cmd_sent = False
        self._down_cmd_sent = False
        # ---- 4. B2鍒癇1鐩稿叧鍙橀噺 ----
        self._turn_left_cmd_sent = False
        self._down_walk_cmd_sent = False
        self._turn_right_cmd_sent = False
        # ---- 5. B1杩斿洖S寮浉鍏冲彉閲?----
        self._turn_left_back_cmd_sent = False
        self._back_to_s_cmd_sent = False
        self._back_to_s_stage = 0
        self._limit_bar_detected_once = False
        self._limit_bar_start_time = None
        self._limit_bar_cmd_sent = False
        # ---- 6. S寮繑鍥濧1搴撶浉鍏冲彉閲?----
        self._s_curve_back_stage = 0
        self._s_curve_back_cmd_sent = False
        self._out_s_left_turn_cmd_sent = False
        self._out_s_left_turn_stage = 0
        self._a1_turn_left_to_north_cmd_sent = False
        self._a1_turn_left_to_west_cmd_sent = False
        self._a1_turn_left_to_west_stage = 0
        self._a1_in_cmd_sent = False
        self._a1_down_cmd_sent = False
        # ---- 7. A1搴撹繑鍥炶捣鐐圭浉鍏冲彉閲?----
        self._a1_back_turn_left_cmd_sent = False
        self._a1_back_turn_left_stage = 0
        self._a1_back_right_turn_cmd_sent = False
        self._a1_back_right_turn_stage = 0
        self._to_home_left_turn_cmd_sent = False
        self._to_home_left_turn_stage = 0
        self._to_home_in_cmd_sent = False
        self._to_home_in_stage = 0
        self._to_home_down_cmd_sent = False
        # ---- 8. 閫氱敤鍙橀噺 ----
        self.current_angle = 0 # 褰撳墠瑙掑害
        self.turned_angle = 0 # 宸茶浆瑙掑害
        self.turn_mode = 0 # 杞悜妯″紡
        
        # 浼犳劅鍣ㄦ彁渚涜€?
        self.imu_provider = imu_provider
        self.d435_provider = d435_provider
        self.rgb_provider = rgb_provider
        self.debug_mode = True
        
        self.get_logger().info("[鍒濆鍖朷 鏈哄櫒浜虹姸鎬佹満鑺傜偣鍚姩")
        self.get_logger().info(f"[鍒濆鐘舵€乚 {self.state}")
    
    def __del__(self):
        self.get_logger().info("[鍏抽棴] 鏈哄櫒浜虹姸鎬佹満鑺傜偣鍏抽棴")
        cmd_msg = Int32MultiArray()
        cmd_msg.data = [0, 0]
        self.motion_cmd_publisher.publish(cmd_msg)
        self.destroy_node()
        rclpy.shutdown()
    
    def robot_ready_callback(self, msg):
        if msg.data and not self.robot_initialized:
            self.robot_initialized = True
            self.get_logger().info("[鍒濆鍖朷 鏀跺埌鏈哄櫒浜哄氨缁俊鍙凤紝绯荤粺鍑嗗灏辩华")
        elif not msg.data:
            self.robot_initialized = False
            self.get_logger().warn("[璀﹀憡] 鏈哄櫒浜哄彉涓烘湭灏辩华鐘舵€侊紝閲嶇疆鐘舵€佹満")
            self.reset_state_machine()
    
    def reset_state_machine(self):
        self.state = "INIT_GO_FORWARD"
        self._forward_cmd_sent = False
        self._right_move_cmd_sent = False
        self._left_turn_cmd_sent = False
        self._slope_climb_cmd_sent = False
        self._warehouse_cmd_sent = False
        self._scan_b_qr_cmd_sent = False
        
        # A鍖轰粨搴撶浉鍏虫爣蹇楅噸缃?
        self._init_forward_cmd_sent = False
        self._init_right_turn_cmd_sent = False
        self._init_right_turn_started_at = None
        self._init_scan_qr_code_cmd_sent = False
        self._a2_turn_right_to_south_cmd_sent = False
        self._a2_turn_right_to_west_cmd_sent = False
        self._a2_in_cmd_sent = False
        self._a2_down_cmd_sent = False
        
        # Reverse_Park鐩稿叧鏍囧織閲嶇疆
        self._up_back_walk_cmd_sent = False
        self._turn_left_cmd_sent = False
        self._turn_right_cmd_sent = False
        self._down_walk_cmd_sent = False
        self._down_cmd_sent = False
        self._turn_reverse_cmd_sent = False

        # B1杩斿洖鐩稿叧鏍囧織閲嶇疆
        self._turn_left_back_cmd_sent = False
        self._back_to_s_cmd_sent = False
        self._back_to_s_stage = 0

        # A1搴撶浉鍏虫爣蹇楅噸缃?
        self._a1_turn_left_to_north_cmd_sent = False
        self._a1_turn_left_to_west_cmd_sent = False
        self._a1_turn_left_to_west_stage = 0
        self._a1_in_cmd_sent = False
        self._a1_down_cmd_sent = False
        self._a1_back_turn_left_cmd_sent = False
        self._a1_back_turn_left_stage = 0
        self._a1_back_right_turn_cmd_sent = False
        self._a1_back_right_turn_stage = 0

        # 杩斿洖璺緞鐩稿叧鏍囧織閲嶇疆
        self._out_s_left_turn_cmd_sent = False
        self._out_s_left_turn_stage = 0
        self._to_home_left_turn_cmd_sent = False
        self._to_home_left_turn_stage = 0
        self._to_home_in_cmd_sent = False
        self._to_home_in_stage = 0
        self._to_home_down_cmd_sent = False

        # S寮繑鍥炵浉鍏虫爣蹇楅噸缃?
        self._s_curve_back_stage = 0
        self._s_curve_back_cmd_sent = False
        self.current_angle = 0
        self.turned_angle = 0
        self.turn_mode = 0

        # 闄愰珮鏉跨浉鍏虫爣蹇楅噸缃?
        self._limit_bar_detected_once = False
        self._limit_bar_start_time = None
        self._limit_bar_cmd_sent = False
        self._qr_history.clear()

    def _append_qr_candidate(self, qr_data):
        if not qr_data:
            return
        candidate = qr_data[0] if isinstance(qr_data, (list, tuple)) else qr_data
        if isinstance(candidate, bytes):
            candidate = candidate.decode(errors='ignore')
        if isinstance(candidate, str):
            candidate = candidate.strip()
        if candidate:
            self._qr_history.append(candidate)

    def _get_confirmed_qr(self, valid_values=None, min_hits=3):
        if not self._qr_history:
            return None
        counter = Counter(self._qr_history)
        value, hits = counter.most_common(1)[0]
        if hits < min_hits:
            return None
        if valid_values is not None and value not in valid_values:
            return None
        return value

    def _publish_track_follow_command(self, cmd_msg, track_result, cruise_speed=120):
        if not track_result.found or track_result.confidence < 0.45:
            cmd_msg.data = [2, max(80, cruise_speed - 30), 500]
            self.motion_cmd_publisher.publish(cmd_msg)
            return

        error = track_result.error
        if error < -0.16:
            cmd_msg.data = [8, 60, 220]
        elif error < -0.06:
            cmd_msg.data = [8, 80, 140]
        elif error > 0.16:
            cmd_msg.data = [9, 60, -220]
        elif error > 0.06:
            cmd_msg.data = [9, 80, -140]
        else:
            cmd_msg.data = [2, cruise_speed, 500]
        self.motion_cmd_publisher.publish(cmd_msg)

    def _track_lost_long_enough(self, key, track_result, timeout_sec=0.8, confidence_thresh=0.42):
        attr_name = f"_{key}_track_lost_since"
        if track_result.found and track_result.confidence >= confidence_thresh:
            setattr(self, attr_name, None)
            return False
        lost_since = getattr(self, attr_name, None)
        if lost_since is None:
            setattr(self, attr_name, time.time())
            return False
        return time.time() - lost_since >= timeout_sec

    def _confirm_arrow_direction(self, img, key="return_arrow", min_hits=2):
        if img is None:
            return None
        history_attr = f"_{key}_history"
        if not hasattr(self, history_attr):
            setattr(self, history_attr, deque(maxlen=5))
        history = getattr(self, history_attr)
        detected, arrow_direction, _, _ = detect_arrow(img.copy())
        if detected and arrow_direction in ("Left", "Right"):
            history.append(arrow_direction)
        if not history:
            return None
        value, hits = Counter(history).most_common(1)[0]
        return value if hits >= min_hits else None

    def publish_control_command(self):
        if not self.robot_initialized:
            return
            
        img = self.rgb_provider.get_latest_image()
        depth_img = self.d435_provider.get_latest_depth()
        if img is None:
            return
        cmd_msg = Int32MultiArray()

        # 初始化状态处理
        if self.state == "INIT_GO_FORWARD":
            if not self._init_forward_cmd_sent:
                cmd_msg.data = [2, 200, 500]
                self.motion_cmd_publisher.publish(cmd_msg)
                self._init_forward_cmd_sent = True

            time.sleep(2.0)
            cmd_msg.data = [0]
            self.motion_cmd_publisher.publish(cmd_msg)
            self.get_logger().info("[INIT_GO_FORWARD] 前进2秒完成，准备切换到右转扫描状态")
            time.sleep(0.5)
            self.state = "INIT_RIGHT_TURN_AND_SCAN_QR_CODE"
            self._init_right_turn_cmd_sent = False
            self._init_right_turn_started_at = None
            return
        elif self.state == "INIT_RIGHT_TURN_AND_SCAN_QR_CODE":
            if getattr(self, '_init_right_turn_started_at', None) is None:
                self._init_right_turn_started_at = time.time()

            if not self._init_right_turn_cmd_sent:
                need_adjust, adjust_direction, angle_error = get_adjustment_direction(
                    self.imu_provider, 270.0, tolerance_deg=4.0
                )
                current_angle = get_current_yaw(self.imu_provider)
                if not need_adjust:
                    cmd_msg.data = [0]
                    self.motion_cmd_publisher.publish(cmd_msg)
                    time.sleep(0.5)
                    self.get_logger().info(
                        f"[INIT_RIGHT_TURN_AND_SCAN_QR_CODE] 已接近目标角度，当前角度: {current_angle:.2f}°"
                    )
                    self.state = "INIT_SCAN_QR_CODE"
                    self._init_scan_qr_code_cmd_sent = False
                    return

                cmd_msg.data = [4, 350] if adjust_direction == 'left' else [5, -350]
                self.motion_cmd_publisher.publish(cmd_msg)
                self._init_right_turn_cmd_sent = True
                self.get_logger().info(
                    f"[INIT_RIGHT_TURN_AND_SCAN_QR_CODE] 当前角度: {current_angle:.2f}°，目标270°，发送{adjust_direction}转命令"
                )
                return
            
            if self._init_right_turn_started_at is not None and time.time() - self._init_right_turn_started_at >= 6.0:
                cmd_msg.data = [0]
                self.motion_cmd_publisher.publish(cmd_msg)
                time.sleep(0.5)
                self.get_logger().warn("[INIT_RIGHT_TURN_AND_SCAN_QR_CODE] 转向超时，直接进入扫码阶段")
                self.state = "INIT_SCAN_QR_CODE"
                self._init_scan_qr_code_cmd_sent = False
                return

            is_west, current_angle, angle_diff = is_facing_cardinal_direction(self.imu_provider, 270.0, tolerance_deg=4.0)
            if is_west:
                cmd_msg.data = [0]  # 绔欑珛
                self.motion_cmd_publisher.publish(cmd_msg)
                time.sleep(1)
                self.get_logger().info("[INIT_RIGHT_TURN_AND_SCAN_QR_CODE] 转向完成，进入扫描状态")
                self.state = "INIT_SCAN_QR_CODE"
                self._init_scan_qr_code_cmd_sent = False
                return
            else:
                self._init_right_turn_cmd_sent = False  # 閲嶆柊鍙戦€佽浆鍚戝懡浠?
        elif self.state == "INIT_SCAN_QR_CODE":
            qr_detected, qr_data = self.rgb_provider.detect_qrcode(img)
            if not hasattr(self, '_init_scan_started_at'):
                self._init_scan_started_at = time.time()
                self._init_scan_yellow_gone_at = None

            # 边扫边跟线，避免单纯定时前进导致偏出识别区。
            track_result = self.track_detector.detect(img)
            self._publish_track_follow_command(cmd_msg, track_result, cruise_speed=110)
            self._init_scan_qr_code_cmd_sent = True

            if qr_detected:
                self.get_logger().info(f"[INIT_SCAN_QR_CODE] 妫€娴嬪埌浜岀淮鐮? {qr_data}")
                self._append_qr_candidate(qr_data)
                confirmed = self._get_confirmed_qr(valid_values={"A-1", "A-2"})
                if confirmed is not None:
                    self.qr_data = [confirmed]
            # 只关注底部区域的黄线，避免整张图误触发
            bottom_yellow_detected = detect_yellow(
                img,
                x_start=0.2,
                x_end=0.8,
                y_start=0.85,
                y_end=1.0,
                yellow_thresh=0.03
            )

            if not bottom_yellow_detected:
                if self._init_scan_yellow_gone_at is None:
                    self._init_scan_yellow_gone_at = time.time()
                elif time.time() - self._init_scan_yellow_gone_at >= 0.5:
                    time.sleep(0.5)
                    cmd_msg.data = [0]  # 绔欑珛
                    self.motion_cmd_publisher.publish(cmd_msg)
                    time.sleep(0.5)
                    confirmed = self._get_confirmed_qr(valid_values={"A-1", "A-2"})
                    self.get_logger().info(f"[INIT_SCAN_QR_CODE] 妫€娴嬪埌鐨勪簩缁寸爜鏁版嵁: {confirmed}")
                    if confirmed == "A-1":
                        self.get_logger().info("[INIT_SCAN_QR_CODE] 妫€娴嬪埌A-1锛屾殏鏃朵笉鐢ㄧ")
                        self.state = "WAREHOUSE_A_1"
                    elif confirmed == "A-2":
                        self.get_logger().info("[LOG_REPAIRED]")
                        self.state = "WAREHOUSE_A_2_TURN_RIGHT_TO_SOUTH"
                    else:
                        self.get_logger().warn("[INIT_SCAN_QR_CODE] 鏈瘑鍒埌鏈夋晥浜岀淮鐮侊紝榛樿鎸堿-2澶勭悊")
                        self.state = "WAREHOUSE_A_2_TURN_RIGHT_TO_SOUTH"
                    self._a2_turn_right_to_south_cmd_sent = False
                    return
            else:
                self._init_scan_yellow_gone_at = None

        elif self.state == "WAREHOUSE_A_2_TURN_RIGHT_TO_SOUTH":
            if not hasattr(self, '_a2_turn_right_to_south_stage'):
                self._a2_turn_right_to_south_stage = 0
            if getattr(self, '_a2_turn_right_to_south_started_at', None) is None:
                self._a2_turn_right_to_south_started_at = None

            if self._a2_turn_right_to_south_stage == 0:
                if self._a2_turn_right_to_south_started_at is None:
                    self._a2_turn_right_to_south_started_at = time.time()
                if not self._a2_turn_right_to_south_cmd_sent:
                    need_adjust, adjust_direction, angle_error = get_adjustment_direction(
                        self.imu_provider, 180.0, tolerance_deg=4.0
                    )
                    if not need_adjust:
                        cmd_msg.data = [0]
                        self.motion_cmd_publisher.publish(cmd_msg)
                        self.get_logger().info(
                            f"[WAREHOUSE_A_2_TURN_RIGHT_TO_SOUTH] 宸叉帴杩戝崡鍚戯紝褰撳墠瑙掑害: {current_angle:.2f}掳"
                        )
                        time.sleep(1.0)
                        self._a2_turn_right_to_south_stage = 1
                        return

                    cmd_msg.data = [4, 250] if adjust_direction == 'left' else [5, -250]
                    self.motion_cmd_publisher.publish(cmd_msg)
                    self._a2_turn_right_to_south_cmd_sent = True
                    self.get_logger().info(
                        f"[LOG_REPAIRED]"
                    )
                    return
                if self._a2_turn_right_to_south_started_at is not None and time.time() - self._a2_turn_right_to_south_started_at >= 6.0:
                    cmd_msg.data = [0]
                    self.motion_cmd_publisher.publish(cmd_msg)
                    self.get_logger().warn("[LOG_REPAIRED]")
                    time.sleep(1.0)
                    self._a2_turn_right_to_south_stage = 1
                    return
                is_south, current_angle, angle_diff = is_facing_cardinal_direction(self.imu_provider, 180.0, tolerance_deg=4.0)
                if is_south:
                    cmd_msg.data = [0]  # 绔欑珛
                    self.motion_cmd_publisher.publish(cmd_msg)
                    self.get_logger().info("[LOG_REPAIRED]")
                    time.sleep(2)
                    self._a2_turn_right_to_south_stage = 1
                    return
                else:
                    self._a2_turn_right_to_south_cmd_sent = False
                    return

            elif self._a2_turn_right_to_south_stage == 1: 
                if getattr(self, '_a2_turn_right_to_south_forward_started_at', None) is None:
                    self._a2_turn_right_to_south_forward_started_at = time.time()
                cmd_msg.data = [2, 200, 500]  # 浣庡ご琛岃蛋锛岄€熷害0.2锛屼刊浠拌0.5
                self.motion_cmd_publisher.publish(cmd_msg)
                self.get_logger().info("[LOG_REPAIRED]")
                yellow_gone = not detect_yellow(
                    img,
                    x_start=0.0,
                    x_end=1.0,
                    y_start=0.9,
                    y_end=1.0,
                    yellow_thresh=0.03
                )
                if yellow_gone or time.time() - self._a2_turn_right_to_south_forward_started_at >= 6.0:
                    time.sleep(2)
                    cmd_msg.data = [0]  # 绔欑珛
                    self.motion_cmd_publisher.publish(cmd_msg)
                    time.sleep(1)
                    self.state = "WAREHOUSE_A_2_TURN_RIGHT_TO_WEST"
                    self._a2_turn_right_to_west_cmd_sent = False
                    self._a2_turn_right_to_south_forward_started_at = None
                    return

        elif self.state == "WAREHOUSE_A_2_TURN_RIGHT_TO_WEST":
            if getattr(self, '_a2_turn_right_to_west_started_at', None) is None:
                self._a2_turn_right_to_west_started_at = None
            if self._a2_turn_right_to_west_started_at is None:
                self._a2_turn_right_to_west_started_at = time.time()
            if not self._a2_turn_right_to_west_cmd_sent:
                need_adjust, adjust_direction, angle_error = get_adjustment_direction(
                    self.imu_provider, 90.0, tolerance_deg=4.0
                )
                current_angle = get_current_yaw(self.imu_provider)
                if not need_adjust:
                    cmd_msg.data = [0]
                    self.motion_cmd_publisher.publish(cmd_msg)
                    time.sleep(0.5)
                    self.state = "WAREHOUSE_A_2_IN"
                    self._a2_in_cmd_sent = False
                    return

                cmd_msg.data = [4, 250] if adjust_direction == 'left' else [5, -250]
                self.motion_cmd_publisher.publish(cmd_msg)
                self._a2_turn_right_to_west_cmd_sent = True
                self.get_logger().info(
                    f"[LOG_REPAIRED]"
                )
                return
            if self._a2_turn_right_to_west_started_at is not None and time.time() - self._a2_turn_right_to_west_started_at >= 6.0:
                cmd_msg.data = [0]
                self.motion_cmd_publisher.publish(cmd_msg)
                self.get_logger().warn("[WAREHOUSE_A_2_TURN_RIGHT_TO_WEST] 杞悜瓒呮椂锛岀洿鎺ヨ繘鍏2浠撳簱")
                time.sleep(0.5)
                self.state = "WAREHOUSE_A_2_IN"
                self._a2_in_cmd_sent = False
                return
            
            # 妫€鏌ユ槸鍚︽瀵硅タ鏂?
            is_west, current_angle, angle_diff = is_facing_cardinal_direction(self.imu_provider, 90.0, tolerance_deg=4.0)
            if is_west:
                cmd_msg.data = [0]  # 绔欑珛
                self.motion_cmd_publisher.publish(cmd_msg)
                time.sleep(0.5)
                self.state = "WAREHOUSE_A_2_IN"
                self._a2_in_cmd_sent = False
                return
            else:
                self._a2_turn_right_to_west_cmd_sent = False  # 閲嶆柊鍙戦€佽浆鍚戝懡浠?
        elif self.state == "WAREHOUSE_A_2_IN":
            if not self._a2_in_cmd_sent:
                cmd_msg.data = [2, 200, 500]  # 浣庡ご琛岃蛋锛岄€熷害0.2锛屼刊浠拌0.5
                self.motion_cmd_publisher.publish(cmd_msg)
                self._a2_in_cmd_sent = True
                time.sleep(6.0)
                cmd_msg.data = [0]  # 绔欑珛
                self.motion_cmd_publisher.publish(cmd_msg)
                time.sleep(2.0)
                self.state = "WAREHOUSE_A_2_DOWN"
                self._a2_down_cmd_sent = False
                return

        elif self.state == "WAREHOUSE_A_2_DOWN":
            if not self._a2_down_cmd_sent:
                cmd_msg.data = [1]  # 瓒翠笅
                self.motion_cmd_publisher.publish(cmd_msg)
                self._a2_down_cmd_sent = True
                self.get_logger().info("[LOG_REPAIRED]")
                time.sleep(5.0)  # 绛夊緟5绉?                cmd_msg.data = [0]  # 绔欑珛
                self.motion_cmd_publisher.publish(cmd_msg)
                time.sleep(3.0)
                self.get_logger().info("[WAREHOUSE_A_2_DOWN] 瓒翠笅瀹屾垚")
                self.state = "WAREHOUSE_A_2_OUT"  # 鍒囨崲鍒板嚭搴撶姸鎬?                return
            
        elif self.state == "WAREHOUSE_A_2_OUT":
            if not self._up_back_walk_cmd_sent:
                cmd_msg.data = [3, -200, -500]  # 鎶ご鍚庨€€琛岃蛋锛岄€熷害-0.15锛屼刊浠拌-0.5
                self.motion_cmd_publisher.publish(cmd_msg)
                self._up_back_walk_cmd_sent = True
                self.get_logger().info("[WAREHOUSE_A_2_OUT] 鍙戦€佹姮澶村悗閫€鍛戒护 [9,0]")
                time.sleep(4.5)  # 鍚庨€€5绉?                cmd_msg.data = [0]  # 绔欑珛
                self.motion_cmd_publisher.publish(cmd_msg)
                self.get_logger().info("[LOG_REPAIRED]")
                time.sleep(1.0)
                self.state = "WAREHOUSE_A_2_BACK_TURN_RIGHT"
                self._turn_reverse_cmd_sent = False
                self._up_back_walk_cmd_sent = False
                return
            
        elif self.state == "WAREHOUSE_A_2_BACK_TURN_RIGHT":
            if getattr(self, '_a2_back_turn_right_started_at', None) is None:
                self._a2_back_turn_right_started_at = time.time()
            if not self._turn_reverse_cmd_sent:
                need_adjust, adjust_direction, angle_error = get_adjustment_direction(
                    self.imu_provider, 0.0, tolerance_deg=3.0
                )
                current_angle = get_current_yaw(self.imu_provider)
                if not need_adjust:
                    cmd_msg.data = [0]
                    self.motion_cmd_publisher.publish(cmd_msg)
                    self.get_logger().info("[WAREHOUSE_A_2_BACK_TURN_RIGHT] 宸叉帴杩戝寳鍚戯紝鐩存帴杩涘叆鍓嶈繘")
                    time.sleep(1.0)
                    cmd_msg.data = [2, 200, 500]
                    self.motion_cmd_publisher.publish(cmd_msg)
                    time.sleep(6.0)
                    cmd_msg.data = [0]
                    self.motion_cmd_publisher.publish(cmd_msg)
                    time.sleep(1.0)
                    self.state = "WAREHOUSE_A_2_BACK_LEFT_TURN"
                    self._forward_cmd_sent = False
                    return

                cmd_msg.data = [4, 250] if adjust_direction == 'left' else [5, -250]
                self.motion_cmd_publisher.publish(cmd_msg)
                self._turn_reverse_cmd_sent = True
                self.get_logger().info(
                    f"[LOG_REPAIRED]"
                )
                return
            if self._a2_back_turn_right_started_at is not None and time.time() - self._a2_back_turn_right_started_at >= 3.0:
                cmd_msg.data = [0]
                self.motion_cmd_publisher.publish(cmd_msg)
                self.get_logger().warn("[LOG_REPAIRED]")
                time.sleep(1.0)
                cmd_msg.data = [2, 200, 500]
                self.motion_cmd_publisher.publish(cmd_msg)
                time.sleep(6.0)
                cmd_msg.data = [0]
                self.motion_cmd_publisher.publish(cmd_msg)
                time.sleep(1.0)
                self.state = "WAREHOUSE_A_2_BACK_LEFT_TURN"
                self._forward_cmd_sent = False
                self._turn_reverse_cmd_sent = False
                return

            is_north, current_angle, angle_diff = is_facing_cardinal_direction(self.imu_provider, 0.0, tolerance_deg=2.0)
            self.get_logger().info(f"[WAREHOUSE_A_2_BACK_TURN_RIGHT] 鏄惁姝ｅ鍖? {is_north}锛岃搴﹀樊{angle_diff:.2f}掳")
            if is_north:
                cmd_msg.data = [0]  # 绔欑珛
                self.motion_cmd_publisher.publish(cmd_msg)
                self.get_logger().info("[LOG_REPAIRED]")
                time.sleep(1.0)
                cmd_msg.data = [2, 200, 500]
                self.motion_cmd_publisher.publish(cmd_msg)
                time.sleep(6.0)
                cmd_msg.data = [0]  # 绔欑珛
                self.motion_cmd_publisher.publish(cmd_msg)
                time.sleep(1.0)
                self.state = "WAREHOUSE_A_2_BACK_LEFT_TURN"
                self._forward_cmd_sent = False
                self._a2_back_turn_right_started_at = None
                return
            else:
                self._turn_reverse_cmd_sent = False  # 閲嶆柊鍙戦€佽浆鍚戝懡浠?            
        elif self.state == "WAREHOUSE_A_2_BACK_LEFT_TURN":
            if not hasattr(self, '_a2_back_left_turn_stage'):
                self._a2_back_left_turn_stage = 0
            if getattr(self, '_a2_back_left_turn_started_at', None) is None:
                self._a2_back_left_turn_started_at = None

            if self._a2_back_left_turn_stage == 0:  # 杞集闃舵
                if self._a2_back_left_turn_started_at is None:
                    self._a2_back_left_turn_started_at = time.time()
                if not self._turn_right_cmd_sent:
                    need_adjust, adjust_direction, angle_error = get_adjustment_direction(
                        self.imu_provider, 90.0, tolerance_deg=3.0
                    )
                    current_angle = get_current_yaw(self.imu_provider)
                    if not need_adjust:
                        cmd_msg.data = [0]
                        self.motion_cmd_publisher.publish(cmd_msg)
                        self.get_logger().info("[WAREHOUSE_A_2_BACK_LEFT_TURN] 宸叉帴杩戣タ鍚戯紝杩涘叆鍓嶈繘闃舵")
                        time.sleep(1.0)
                        self._a2_back_left_turn_stage = 1
                        return

                    cmd_msg.data = [4, 300] if adjust_direction == 'left' else [5, -300]
                    self.motion_cmd_publisher.publish(cmd_msg)
                    self._turn_right_cmd_sent = True
                    self.get_logger().info(
                        f"[WAREHOUSE_A_2_BACK_LEFT_TURN] 当前角度: {current_angle:.2f}°，目标90°，发送{adjust_direction}转命令"
                    )
                    return
                if self._a2_back_left_turn_started_at is not None and time.time() - self._a2_back_left_turn_started_at >= 8.0:
                    cmd_msg.data = [0]
                    self.motion_cmd_publisher.publish(cmd_msg)
                    self.get_logger().warn("[WAREHOUSE_A_2_BACK_LEFT_TURN] 转向超时，进入前进阶段")
                    time.sleep(1.0)
                    self._a2_back_left_turn_stage = 1
                    return
                is_west, current_angle, angle_diff = is_facing_cardinal_direction(self.imu_provider, 90.0, tolerance_deg=2.0)
                self.get_logger().info(f"[WAREHOUSE_A_2_BACK_LEFT_TURN] 是否正对西方: {is_west}，角度差{angle_diff:.2f}°")
                if is_west:
                    cmd_msg.data = [0]  # 绔欑珛
                    self.motion_cmd_publisher.publish(cmd_msg)
                    self.get_logger().info("[LOG_REPAIRED]")
                    time.sleep(1.0)
                    self._a2_back_left_turn_stage = 1
                    return
                else:
                    self._turn_right_cmd_sent = False
                    return
            elif self._a2_back_left_turn_stage == 1:  # 鍓嶈繘闃舵
                if getattr(self, '_a2_back_left_forward_started_at', None) is None:
                    self._a2_back_left_forward_started_at = time.time()
                cmd_msg.data = [2, 200, 500]  # 浣庡ご琛岃蛋锛岄€熷害0.2锛屼刊浠拌0.5
                self.motion_cmd_publisher.publish(cmd_msg)
                self.get_logger().info("[LOG_REPAIRED]")
                if detect_yellow(img, x_start=0, x_end=1.0, y_start=0.9, y_end=1.0, yellow_thresh=0.05) or \
                   time.time() - self._a2_back_left_forward_started_at >= 6.0:
                    time.sleep(4)
                    cmd_msg.data = [0]  # 绔欑珛
                    self.motion_cmd_publisher.publish(cmd_msg)
                    self.get_logger().info("[WAREHOUSE_A_2_BACK_LEFT_TURN] 鍓嶈繘瀹屾垚")
                    time.sleep(1.0)
                    self.state = "TO_S_RIGHT_TURN"
                    self._a2_back_left_forward_started_at = None
                else:
                    self._forward_cmd_sent = False
                    return

        elif self.state == "TO_S_RIGHT_TURN":
            if not hasattr(self, '_to_s_right_turn_stage'):
                self._to_s_right_turn_stage = 0
            if getattr(self, '_to_s_right_turn_started_at', None) is None:
                self._to_s_right_turn_started_at = None
            if getattr(self, '_to_s_right_turn_micro_adjust_started_at', None) is None:
                self._to_s_right_turn_micro_adjust_started_at = None

            if self._to_s_right_turn_stage == 0:  # 鍙宠浆闃舵
                if self._to_s_right_turn_started_at is None:
                    self._to_s_right_turn_started_at = time.time()
                if not self._forward_cmd_sent:
                    need_adjust, adjust_direction, angle_error = get_adjustment_direction(
                        self.imu_provider, 0.0, tolerance_deg=2.0
                    )
                    current_angle = get_current_yaw(self.imu_provider)
                    if not need_adjust:
                        cmd_msg.data = [0]
                        self.motion_cmd_publisher.publish(cmd_msg)
                        self.get_logger().info("[TO_S_RIGHT_TURN] 宸叉帴杩戠洰鏍囨湞鍚戯紝鍑嗗鍓嶈繘")
                        time.sleep(1.0)
                        self._to_s_right_turn_stage = 1
                        self._forward_cmd_sent = False
                        return

                    cmd_msg.data = [4, 150] if adjust_direction == 'left' else [5, -150]
                    self.motion_cmd_publisher.publish(cmd_msg)
                    self._forward_cmd_sent = True
                    self.get_logger().info(
                        f"[LOG_REPAIRED]"
                    )
                    return
                if self._to_s_right_turn_started_at is not None and time.time() - self._to_s_right_turn_started_at >= 8.0:
                    cmd_msg.data = [0]
                    self.motion_cmd_publisher.publish(cmd_msg)
                    self.get_logger().warn("[LOG_REPAIRED]")
                    time.sleep(1.0)
                    self._to_s_right_turn_stage = 1
                    self._forward_cmd_sent = False
                    return
                
                is_target, current_angle, angle_diff = is_facing_cardinal_direction(self.imu_provider, 0, tolerance_deg=1)
                self.get_logger().info(f"[TO_S_RIGHT_TURN] 褰撳墠瑙掑害: {current_angle:.2f}掳锛岀洰鏍囪搴? 0掳锛岃搴﹀樊: {angle_diff:.2f}掳")
                if is_target:
                    cmd_msg.data = [0]  # 绔欑珛
                    self.motion_cmd_publisher.publish(cmd_msg)
                    self.get_logger().info("[LOG_REPAIRED]")
                    time.sleep(1.0)
                    self._to_s_right_turn_stage = 1
                    self._forward_cmd_sent = False
                    return
                else:
                    self._forward_cmd_sent = False  # 閲嶆柊鍙戦€佽浆鍚戝懡浠?                    return

            elif self._to_s_right_turn_stage == 1:  # 鍓嶈繘闃舵
                if not self._forward_cmd_sent:
                    cmd_msg.data = [2, 200, 500]  # 浣庡ご琛岃蛋锛岄€熷害0.2锛屼刊浠拌0.5
                    self.motion_cmd_publisher.publish(cmd_msg)
                    self._forward_cmd_sent = True
                    self._forward_start_time = time.time()
                    self.get_logger().info("[LOG_REPAIRED]")
                    return
                
                # 鍓嶈繘鍒癝寮紝妫€娴嬪埌鐗瑰畾鏍囪鎴栧墠杩涜冻澶熺殑鏃堕棿
                if (not detect_yellow(img, y_start=0.0, y_end=1.0)) or (time.time() - self._forward_start_time >= 4.5):
                    cmd_msg.data = [0]  # 绔欑珛
                    self.motion_cmd_publisher.publish(cmd_msg)
                    self.get_logger().info("[LOG_REPAIRED]")
                    time.sleep(1.0)
                    self._to_s_right_turn_stage = 2
                    self._forward_cmd_sent = False
                    self._to_s_right_turn_micro_adjust_started_at = None
                    return

            elif self._to_s_right_turn_stage == 2:  # 寰皟闃舵
                if self._to_s_right_turn_micro_adjust_started_at is None:
                    self._to_s_right_turn_micro_adjust_started_at = time.time()
                if not self._forward_cmd_sent:
                    need_adjust, adjust_direction, angle_error = get_adjustment_direction(
                        self.imu_provider, 2.7, tolerance_deg=0.8
                    )
                    cmd_msg.data = [4, 10] if adjust_direction == 'left' else [5, -10]
                    self.motion_cmd_publisher.publish(cmd_msg)
                    self._forward_cmd_sent = True
                    self.get_logger().info("[LOG_REPAIRED]")
                    return
                
                is_target, current_angle, angle_diff = is_facing_cardinal_direction(self.imu_provider, 2.7, tolerance_deg=0.8)
                self.get_logger().info(f"[TO_S_RIGHT_TURN] 褰撳墠瑙掑害: {current_angle:.2f}掳锛岀洰鏍囪搴? 2.7掳锛岃搴﹀樊: {angle_diff:.2f}掳")
                if is_target or (time.time() - self._to_s_right_turn_micro_adjust_started_at >= 3.0):
                    cmd_msg.data = [0]  # 绔欑珛
                    self.motion_cmd_publisher.publish(cmd_msg)
                    if is_target:
                        self.get_logger().info("[LOG_REPAIRED]")
                    else:
                        self.get_logger().warn("[LOG_REPAIRED]")
                    time.sleep(1.0)
                    self.state = "S_CURVE_GO"
                    self._forward_cmd_sent = False
                    self._to_s_right_turn_micro_adjust_started_at = None
                    return
                else:
                    self._forward_cmd_sent = False  # 閲嶆柊鍙戦€佽浆鍚戝懡浠?                    return

        elif self.state == "S_CURVE_GO":
            if not hasattr(self, '_s_curve_stage'):
                self._s_curve_stage = 0
                self._s_curve_entered_at = time.time()
                self._s_curve_track_lost_since = None
                self.get_logger().info("[S_CURVE_GO] 切换为视觉闭环跟线")

            track_result = self.track_detector.detect(img)
            self._publish_track_follow_command(cmd_msg, track_result, cruise_speed=90)

            elapsed = time.time() - self._s_curve_entered_at
            exit_by_track_loss = elapsed >= 4.5 and self._track_lost_long_enough(
                "s_curve", track_result, timeout_sec=0.7, confidence_thresh=0.40
            )
            exit_by_timeout = elapsed >= 12.0
            exit_by_bottom_clear = (
                elapsed >= 4.5 and
                not detect_yellow(img, x_start=0.15, x_end=0.85, y_start=0.88, y_end=1.0, yellow_thresh=0.025)
            )

            if exit_by_track_loss or exit_by_timeout or exit_by_bottom_clear:
                cmd_msg.data = [0]
                self.motion_cmd_publisher.publish(cmd_msg)
                time.sleep(0.6)
                self.state = "WAREHOUSE_B_GO_PREPARE"
                self._prepare_stage = 0
                self._prepare_cmd_sent = False
                self._prepare_stage_started_at = None
                self.get_logger().info("[S_CURVE_GO] 完成，切换到 B 区准备段")
                return

        elif self.state == "WAREHOUSE_B_GO_PREPARE":
            if not hasattr(self, '_prepare_stage'):
                self._prepare_stage = 0
                self._prepare_cmd_sent = False
                self._prepare_stage_started_at = None

            if self._prepare_stage == 0:  # 宸﹁浆闃舵
                if not self._prepare_cmd_sent:
                    cmd_msg.data = [4, 200]  # 宸﹁浆锛岃閫熷害0.15
                    self.motion_cmd_publisher.publish(cmd_msg)
                    self._prepare_cmd_sent = True
                    if self._prepare_stage_started_at is None:
                        self._prepare_stage_started_at = time.time()
                    self.get_logger().info("[LOG_REPAIRED]")
                    return
                is_forward, current_angle, angle_diff = is_facing_cardinal_direction(self.imu_provider, 92, tolerance_deg=1)
                self.get_logger().info(f"[准备] 是否正对西: {is_forward}，角度差{angle_diff:.2f}°，当前角度{current_angle:.2f}°")
                if is_forward or (
                    self._prepare_stage_started_at is not None and time.time() - self._prepare_stage_started_at >= 6.0
                ):
                    # 鍏堝彂閫佸仠姝㈠懡浠?                    cmd_msg.data = [0]  # 绔欑珛
                    self.motion_cmd_publisher.publish(cmd_msg)
                    self.get_logger().info("[LOG_REPAIRED]")
                    time.sleep(0.5)  # 绛夊緟鍋滄鍛戒护鐢熸晥
                    
                    self.get_logger().info("[妫€娴媇 姝ｅ瑗匡紝鍑嗗鍓嶈繘")
                    self._prepare_stage = 1  # 杩涘叆鍓嶈繘闃舵
                    self._prepare_cmd_sent = False
                    self._prepare_stage_started_at = None
                    self._forward_start_time = time.time()  # 璁板綍寮€濮嬪墠杩涚殑鏃堕棿
                    return
                else:
                    self._prepare_cmd_sent = False
                    self.get_logger().info("[LOG_REPAIRED]")
                    return

            elif self._prepare_stage == 1:  # 鍓嶈繘闃舵
                track_result = self.track_detector.detect(img)
                self._publish_track_follow_command(cmd_msg, track_result, cruise_speed=100)
                self._prepare_cmd_sent = True

                if (
                    time.time() - self._forward_start_time >= 3.0 and
                    not detect_yellow(img, x_start=0.55, x_end=1.0, y_start=0.0, y_end=1.0, yellow_thresh=0.015)
                ) or (time.time() - self._forward_start_time >= 6.5):
                    cmd_msg.data = [0]
                    self.motion_cmd_publisher.publish(cmd_msg)
                    time.sleep(0.5)
                    self.state = "WAREHOUSE_B_2_GO_RIGHT_MOVE"
                    self._right_move_cmd_sent = False
                    return
        # ---- 浠嶴寮嚭鏉ュ埌鍏2搴?----
        elif self.state == "WAREHOUSE_B_2_GO_RIGHT_MOVE":
            if not hasattr(self, '_right_move_stage'):
                self._right_move_stage = 0
                self._right_move_cmd_sent = False
                self._right_move_started_at = None

            if self._right_move_stage == 0:  # 绗竴闃舵锛氬彸骞崇Щ
                if not self._right_move_cmd_sent:
                    cmd_msg.data = [7, -150]  # 鍙冲钩绉伙紝閫熷害-0.1
                    self.motion_cmd_publisher.publish(cmd_msg)
                    self._right_move_cmd_sent = True
                    if self._right_move_started_at is None:
                        self._right_move_started_at = time.time()
                    self.get_logger().info("[鍔ㄤ綔] 鍙戦€佸彸骞崇Щ鍛戒护")
                    return
                
                if (not detect_yellow(img, x_start=0.2, x_end=1.0, y_start=0, y_end=1.0)) or (
                    self._right_move_started_at is not None and time.time() - self._right_move_started_at >= 6.0
                ):
                    cmd_msg.data = [0]
                    self.motion_cmd_publisher.publish(cmd_msg)
                    self.get_logger().info("[LOG_REPAIRED]")
                    time.sleep(0.5)  # 绛夊緟鍋滄鍛戒护鐢熸晥
                    
                    self.get_logger().info("[妫€娴媇 鍙充晶鏃犻粍鑹诧紝鍑嗗寰皟")
                    self._right_move_stage = 1
                    self._right_move_cmd_sent = False
                    return
                else:
                    self._right_move_cmd_sent = False
                    return

            elif self._right_move_stage == 1:
                if getattr(self, '_right_move_adjust_started_at', None) is None:
                    self._right_move_adjust_started_at = time.time()
                need_adjust, adjust_direction, angle_error = get_adjustment_direction(
                    self.imu_provider, 89.8, tolerance_deg=2.0
                )
                current_angle = get_current_yaw(self.imu_provider)
                angle_diff = abs(angle_error)
                self.get_logger().info(
                    f"[妫€娴媇 褰撳墠瑙掑害: {current_angle:.2f}掳锛岀洰鏍囪搴? 90掳锛岃搴﹀樊: {angle_diff:.2f}掳锛屽井璋冩柟鍚? {adjust_direction}"
                )
                if not need_adjust:
                    is_target = True
                elif not self._right_move_cmd_sent:
                    cmd_msg.data = [4, 50] if adjust_direction == 'left' else [5, -50]
                    self.motion_cmd_publisher.publish(cmd_msg)
                    self._right_move_cmd_sent = True
                    self.get_logger().info("[LOG_REPAIRED]")
                    return
                else:
                    is_target = False

                if is_target or (time.time() - self._right_move_adjust_started_at >= 3.0):
                    cmd_msg.data = [0]  # 绔欑珛
                    self.motion_cmd_publisher.publish(cmd_msg)
                    self.get_logger().info("[鍔ㄤ綔] 寰皟瀹屾垚锛屽噯澶囧垏鎹㈠埌鐖潯")
                    time.sleep(0.5)  # 绛夊緟鍋滄鍛戒护鐢熸晥
                    self.state = "WAREHOUSE_B_2_GO_SLOPE_CLIMB"
                    self._slope_climb_cmd_sent = False
                    self._right_move_adjust_started_at = None
                    return
                else:
                    self._right_move_cmd_sent = False
                    return

        elif self.state == "WAREHOUSE_B_2_GO_SLOPE_CLIMB":
            if not hasattr(self, '_slope_climb_stage'):
                self._slope_climb_stage = 0  # 0: 姝ｅ父琛岃蛋, 1: 寰皟
                self._slope_climb_cmd_sent = False
                self._last_adjust_time = time.time()
                self._slope_climb_stage = 0
                self._slope_climb_target_yaw = get_current_yaw(self.imu_provider)
                self.get_logger().info(
                    f"[WAREHOUSE_B_2_GO_SLOPE_CLIMB] 锁定石板路目标朝向: {self._slope_climb_target_yaw:.2f}°"
                )

            current_time = time.time()
            # 每隔一段时间做一次短暂保向，避免石板路步态长时间漂移。
            if self._slope_climb_stage == 0 and current_time - self._last_adjust_time >= 6.0:
                self._slope_climb_stage = 1
                self.get_logger().info("[WAREHOUSE_B_2_GO_SLOPE_CLIMB] 进入保向微调阶段")
                self._last_adjust_time = current_time

            if self._slope_climb_stage == 1:
                need_adjust, adjust_direction, angle_error = get_adjustment_direction(
                    self.imu_provider, self._slope_climb_target_yaw, tolerance_deg=3.0
                )
                current_angle = get_current_yaw(self.imu_provider)
                angle_diff = abs(angle_error)
                if need_adjust:
                    cmd_msg.data = [4, 20] if adjust_direction == 'left' else [5, -20]
                    self.motion_cmd_publisher.publish(cmd_msg)
                    self.get_logger().info(
                        f"[WAREHOUSE_B_2_GO_SLOPE_CLIMB] 保向微调，当前: {current_angle:.2f}°，目标: {self._slope_climb_target_yaw:.2f}°，偏差: {angle_diff:.2f}°，方向: {adjust_direction}"
                    )
                    return
                else:
                    cmd_msg.data = [0]  # 绔欑珛
                    self.motion_cmd_publisher.publish(cmd_msg)
                    self.get_logger().info("[WAREHOUSE_B_2_GO_SLOPE_CLIMB] 保向完成，恢复石板路步态")
                    self._slope_climb_stage = 0
                    self._slope_climb_cmd_sent = False
                    self._last_adjust_time = time.time()
                    return

            # 姝ｅ父琛岃蛋闃舵
            if self._slope_climb_stage == 0 and not self._slope_climb_cmd_sent:
                cmd_msg.data = [10, 120, 250, -200]
                self.motion_cmd_publisher.publish(cmd_msg)
                self._slope_climb_cmd_sent = True
                self.get_logger().info("[WAREHOUSE_B_2_GO_SLOPE_CLIMB] 开始石板路步态前进")
                return
            if depth_img is not None and detect_top_bar(depth_img):
                self.state = "WAREHOUSE_B_2_GO_YELLOW_LIGHT"
                self._bar_detected_once = True
                self._last_adjust_time = time.time()
                return
        
        elif self.state == "WAREHOUSE_B_2_GO_YELLOW_LIGHT":
            if not hasattr(self, '_yellow_light_initialized'):
                self._yellow_light_initialized = True
                self._yellow_light_cmd_sent = False
                self.get_logger().info("[LOG_REPAIRED]")

            if not self._yellow_light_cmd_sent:
                cmd_msg.data = [2, 200, 500]  # 浣庡ご琛岃蛋锛岄€熷害0.2锛屼刊浠拌0.5
                self.motion_cmd_publisher.publish(cmd_msg)
                self._yellow_light_cmd_sent = True
                self.get_logger().info("[LOG_REPAIRED]")
                return

            # 鎵ц鍘熸湁閫昏緫
            if depth_img is not None and detect_top_bar(depth_img):
                self.get_logger().info("[LOG_REPAIRED]")
                self._bar_detected_once = True
            elif depth_img is not None and not detect_top_bar(depth_img):
                if self._bar_detected_once:  # 鍙湁涔嬪墠妫€娴嬪埌杩嘼ar鎵嶅鐞嗘秷澶辩殑鎯呭喌
                    cmd_msg.data = [0]
                    self.motion_cmd_publisher.publish(cmd_msg)
                    self.get_logger().info("[LOG_REPAIRED]")
                    time.sleep(5.0)
                    self.state = "WAREHOUSE_B_2_GO_SCAN_B_QR"
                    self._scan_b_qr_start_time = None
                    return
        
        elif self.state == "WAREHOUSE_B_2_GO_SCAN_B_QR":
            if not hasattr(self, '_scan_b_qr_initialized'):
                self._scan_b_qr_initialized = True
                self._scan_b_qr_cmd_sent = False
                self.get_logger().info("[LOG_REPAIRED]")

            track_result = self.track_detector.detect(img)
            self._publish_track_follow_command(cmd_msg, track_result, cruise_speed=110)
            self._scan_b_qr_cmd_sent = True

            # 妫€娴嬩簩缁寸爜
            if img is not None:
                qr_code_detected, qr_code_data = self.rgb_provider.detect_qrcode(img)
                if qr_code_detected:
                    self.get_logger().info(f"[WAREHOUSE_B_2_GO_SCAN_B_QR] 妫€娴嬪埌浜岀淮鐮侊紝鏁版嵁: {qr_code_data}")
                    self._append_qr_candidate(qr_code_data)
                    confirmed = self._get_confirmed_qr(valid_values={"B-1", "B-2"})
                    if confirmed in ["B-1", "B-2"]:
                        cmd_msg.data = [0]
                        self.motion_cmd_publisher.publish(cmd_msg)
                        self.get_logger().info("[LOG_REPAIRED]")
                        time.sleep(2.0)
                        if confirmed == "B-2":
                            self.state = "WAREHOUSE_B_2_IN"
                        elif confirmed == "B-1":
                            self.state = "WAREHOUSE_B_1_IN"
                        self._warehouse_cmd_sent = False
                        self._warehouse_start_time = None
                        return
        elif self.state == "WAREHOUSE_B_2_IN":
            if not self._warehouse_cmd_sent:
                cmd_msg.data = [2, 200, 500]  # 浣庡ご琛岃蛋锛岄€熷害0.2锛屼刊浠拌0.5
                self.motion_cmd_publisher.publish(cmd_msg)
                self._warehouse_cmd_sent = True
                self._warehouse_stage = 0
                self.get_logger().info("[LOG_REPAIRED]")
                return
            # 妫€娴嬩笅鏂归粍鑹叉秷澶?
            if not hasattr(self, '_warehouse_stage') or self._warehouse_stage == 0:
                if img is not None:
                    yellow_gone = not detect_yellow(img, y_start=0.0)
                    self.get_logger().info(f"[WAREHOUSE_B_2_IN] 涓嬫柟榛勮壊娑堝け: {yellow_gone}")
                    if yellow_gone:
                        self._warehouse_stage = 1
                        self.get_logger().info("[LOG_REPAIRED]")
                        time.sleep(2.0)  # 绛夊緟2绉?
                        cmd_msg.data = [0]  # 绔欑珛
                        self.motion_cmd_publisher.publish(cmd_msg)
                        self.get_logger().info("[LOG_REPAIRED]")
                        self._warehouse_stage = 2
                        self.get_logger().info("[WAREHOUSE_B_2_IN] 瀹屾垚浠诲姟锛屽噯澶囪繘鍏AREHOUSE_B_2_GO_DOWN")
                        self.state = "WAREHOUSE_B_2_GO_DOWN"
                        self._warehouse_cmd_sent = False
                        self._down_cmd_sent = False
                        return

        elif self.state == "WAREHOUSE_B_2_GO_DOWN":
            if not self._down_cmd_sent:
                cmd_msg.data = [1]  # 瓒翠笅
                self.motion_cmd_publisher.publish(cmd_msg)
                self.get_logger().info("[LOG_REPAIRED]")
                self._down_cmd_sent = True
                time.sleep(5.0)
                cmd_msg.data = [0]  # 绔欑珛
                self.motion_cmd_publisher.publish(cmd_msg)
                self.get_logger().info("[LOG_REPAIRED]")
                time.sleep(2.0)
                self.get_logger().info("[DOWN] 绔欑珛瀹屾垚锛屽噯澶囪繘鍏AREHOUSE_B_2_OUT")
                self.state = "WAREHOUSE_B_2_OUT"
                self._turn_reverse_cmd_sent = False
                return
        
        # ---- 浠嶣2搴撳嚭鏉ュ埌B1搴?----
        elif self.state == "WAREHOUSE_B_2_OUT":
            if not self._up_back_walk_cmd_sent:
                cmd_msg.data = [3, -150, -500]  # 鎶ご鍚庨€€琛岃蛋锛岄€熷害-0.15锛屼刊浠拌-0.5
                self.motion_cmd_publisher.publish(cmd_msg)
                self._up_back_walk_cmd_sent = True
                self.get_logger().info("[WAREHOUSE_B_2_OUT] 鍙戦€佹姮澶村悗閫€鍛戒护")
                return
                
            # 妫€娴嬮粍鑹诧紝鍐冲畾鏄惁杞悜
            if img is not None:
                yellow_detected = detect_yellow(img)
                if yellow_detected:
                    self.get_logger().info("[LOG_REPAIRED]")
                    cmd_msg.data = [0]  # 绔欑珛
                    self.motion_cmd_publisher.publish(cmd_msg)
                    time.sleep(1.0)  # 绛夊緟鍋滄绋冲畾
                    self.state = "WAREHOUSE_B_2_TO_1_TURN_LEFT"
                    self._turn_left_cmd_sent = False   
                    self._up_back_walk_cmd_sent = False
        
        elif self.state == "WAREHOUSE_B_2_TO_1_TURN_LEFT":
            if not self._turn_left_cmd_sent:
                cmd_msg.data = [4, 150]  # 宸﹁浆锛岃閫熷害0.15
                self.motion_cmd_publisher.publish(cmd_msg)
                self._turn_left_cmd_sent = True
                self.get_logger().info("[LOG_REPAIRED]")
                return
                
            is_north, current_angle, angle_diff = is_facing_cardinal_direction(self.imu_provider, 180.0, tolerance_deg=1)
            self.get_logger().info(f"[TURN_LEFT] 鏄惁姝ｅ鍗? {is_north}锛岃搴﹀樊{angle_diff:.2f}掳")
            if is_north:
                cmd_msg.data = [0]  # 绔欑珛
                self.motion_cmd_publisher.publish(cmd_msg)
                self.get_logger().info("[LOG_REPAIRED]")
                time.sleep(1.0)  # 绛夊緟绋冲畾
                self.state = "WAREHOUSE_B_2_TO_1_DOWN_WALK"
                self._down_walk_cmd_sent = False
            else:
                self._turn_left_cmd_sent = False  # 閲嶆柊鍙戦€佽浆鍚戝懡浠?        
        elif self.state == "WAREHOUSE_B_2_TO_1_DOWN_WALK":
            if not self._down_walk_cmd_sent:
                cmd_msg.data = [2, 200, 500]  # 浣庡ご琛岃蛋锛岄€熷害0.2锛屼刊浠拌0.5
                self.motion_cmd_publisher.publish(cmd_msg)
                self._down_walk_cmd_sent = True
                self.get_logger().info("[LOG_REPAIRED]")
                return
            # 妫€娴嬩笉鍒伴粍鑹叉椂锛屽噯澶囧彸杞?
            if img is not None:
                yellow_gone = not detect_yellow(img, y_start=0.0)
                if yellow_gone:
                    self.get_logger().info("[LOG_REPAIRED]")
                    time.sleep(1.5)
                    cmd_msg.data = [0]  # 绔欑珛
                    self.motion_cmd_publisher.publish(cmd_msg)
                    time.sleep(1.5)  # 绛夊緟鍋滄绋冲畾
                    self.state = "WAREHOUSE_B_2_TO_1_TURN_RIGHT"
                    self._turn_right_cmd_sent = False
        
        elif self.state == "WAREHOUSE_B_2_TO_1_TURN_RIGHT":
            if not self._turn_right_cmd_sent:
                cmd_msg.data = [5, -150]  # 鍙宠浆锛岃閫熷害-0.15
                self.motion_cmd_publisher.publish(cmd_msg)
                self._turn_right_cmd_sent = True
                self.get_logger().info("[LOG_REPAIRED]")
                return
                
            is_east, current_angle, angle_diff = is_facing_cardinal_direction(self.imu_provider, 90.0, tolerance_deg=1)
            self.get_logger().info(f"[WAREHOUSE_B_2_TO_1_TURN_RIGHT] 鏄惁姝ｅ涓? {is_east}锛岃搴﹀樊{angle_diff:.2f}掳")
            if is_east:
                cmd_msg.data = [0]  # 绔欑珛
                self.motion_cmd_publisher.publish(cmd_msg)
                self.get_logger().info("[WAREHOUSE_B_2_TO_1_TURN_RIGHT] 鍙宠浆瀹屾垚锛岃繘鍏ヤ粨搴揃1")
                time.sleep(2.0)  # 绛夊緟绋冲畾
                self.state = "WAREHOUSE_B_1_IN"
                self._warehouse_cmd_sent = False
                self._warehouse_start_time = None
            else:
                self._turn_right_cmd_sent = False  # 閲嶆柊鍙戦€佽浆鍚戝懡浠?        
        elif self.state == "WAREHOUSE_B_1_IN":
            if not self._warehouse_cmd_sent:
                cmd_msg.data = [2, 200, 500]  # 浣庡ご琛岃蛋锛岄€熷害0.2锛屼刊浠拌0.5
                self.motion_cmd_publisher.publish(cmd_msg)
                self._warehouse_cmd_sent = True
                self._warehouse_stage = 0
                self.get_logger().info("[LOG_REPAIRED]")
                return
            # 妫€娴嬩笅鏂归粍鑹叉秷澶?
            if not hasattr(self, '_warehouse_stage') or self._warehouse_stage == 0:
                if img is not None:
                    yellow_gone = not detect_yellow(img, y_start=0.0)
                    self.get_logger().info(f"[WAREHOUSE_B_1_IN] 涓嬫柟榛勮壊娑堝け: {yellow_gone}")
                    if yellow_gone:
                        self._warehouse_stage = 1
                        self.get_logger().info("[LOG_REPAIRED]")
                        time.sleep(2.0)  # 绛夊緟2绉?
                        cmd_msg.data = [0]  # 绔欑珛
                        self.motion_cmd_publisher.publish(cmd_msg)
                        self.get_logger().info("[LOG_REPAIRED]")
                        self._warehouse_stage = 2
                        self.get_logger().info("[WAREHOUSE_B_1_IN] 瀹屾垚浠诲姟锛屽噯澶囪繘鍏AREHOUSE_B_1_DOWN")
                        self.state = "WAREHOUSE_B_1_DOWN"
                        self._warehouse_cmd_sent = False
                        self._down_cmd_sent = False
                        return
        
        elif self.state == "WAREHOUSE_B_1_DOWN":
            if not self._down_cmd_sent:
                cmd_msg.data = [1]  # 瓒翠笅
                self.motion_cmd_publisher.publish(cmd_msg)
                self.get_logger().info("[LOG_REPAIRED]")
                self._down_cmd_sent = True
                time.sleep(5.0)
                cmd_msg.data = [0]  # 绔欑珛
                self.motion_cmd_publisher.publish(cmd_msg)
                self.get_logger().info("[LOG_REPAIRED]")
                time.sleep(2.0)
                self.get_logger().info("[WAREHOUSE_B_1_DOWN] 绔欑珛瀹屾垚锛屽噯澶囪繘鍏AREHOUSE_B_1_BACK_TURN_REVERSE")
                self.state = "WAREHOUSE_B_1_BACK_TURN_REVERSE"
                self._turn_reverse_cmd_sent = False
                return
        
        # ---- B-1搴撹繑鍥?----
        elif self.state == "WAREHOUSE_B_1_BACK_TURN_REVERSE":
            if not hasattr(self, '_turn_reverse_cmd_sent') or not self._turn_reverse_cmd_sent:
                cmd_msg.data = [4, 150]  # 宸﹁浆锛岃閫熷害0.15
                self.motion_cmd_publisher.publish(cmd_msg)
                self._turn_reverse_cmd_sent = True
                self.get_logger().info("[LOG_REPAIRED]")
                return
            is_east, current_angle, angle_diff = is_facing_cardinal_direction(self.imu_provider, 270.0, tolerance_deg=1)
            self.get_logger().info(f"[WAREHOUSE_B_1_BACK_TURN_REVERSE] 鏄惁姝ｅ涓? {is_east}锛岃搴﹀樊{angle_diff:.2f}掳")
            if is_east:
                cmd_msg.data = [0]  # 绔欑珛
                self.motion_cmd_publisher.publish(cmd_msg)
                self.get_logger().info("[LOG_REPAIRED]")
                self.state = "WAREHOUSE_B_1_BACK_FORWARD"
                self._limit_bar_detected_once = False
                self._limit_bar_start_time = None
                self._limit_bar_cmd_sent = False
                return
            else:
                self._turn_reverse_cmd_sent = False

        elif self.state == "WAREHOUSE_B_1_BACK_FORWARD":
            if not self._warehouse_cmd_sent:
                cmd_msg.data = [2, 200, 500]  # 浣庡ご琛岃蛋锛岄€熷害0.2锛屼刊浠拌0.5
                self.motion_cmd_publisher.publish(cmd_msg)
                self._warehouse_cmd_sent = True
                self.get_logger().info("[LOG_REPAIRED]")
                
            if depth_img is not None and detect_top_bar(depth_img, distance_threshold=0.8, top_height_ratio=0.2, max_valid_depth=4.0):
                self.get_logger().info("[LOG_REPAIRED]")
                time.sleep(1)
                cmd_msg.data = [0]  # 绔欑珛
                self.motion_cmd_publisher.publish(cmd_msg)
                time.sleep(1)
                self.state = "WAREHOUSE_B_1_BACK_LIMIT_BAR"
                self._limit_bar_cmd_sent = False
                self._limit_bar_start_time = None
                return

        elif self.state == "WAREHOUSE_B_1_BACK_LIMIT_BAR":
            if not self._limit_bar_cmd_sent:
                time.sleep(1.0)
                cmd_msg.data = [11]  # 瓒翠笅璧拌矾
                self.motion_cmd_publisher.publish(cmd_msg)
                self.get_logger().info("[LOG_REPAIRED]")
                self._limit_bar_cmd_sent = True
                time.sleep(5.0)
                cmd_msg.data = [0]  # 绔欑珛
                self.motion_cmd_publisher.publish(cmd_msg)
                self.get_logger().info("[WAREHOUSE_B_1_BACK_LIMIT_BAR] 鍓嶈繘10绉掑悗锛屽彂閫佸仠姝㈠懡浠わ紝鍒囨崲鍒癝TONE_ROAD")
                time.sleep(0.5)
                self.state = "WAREHOUSE_B_1_BACK_STONE_ROAD"
                self._stone_road_cmd_sent = False
                return

        elif self.state == "WAREHOUSE_B_1_BACK_STONE_ROAD":
            if not hasattr(self, '_stone_road_init'):
                self._stone_road_init = True
                self._stone_road_cmd_sent = False
                self._yellow_gone_start_time = None
                self._stone_road_started_at = time.time()
                # 鍙湪杩涘叆鏃跺井璋冧竴娆?
                is_target, current_angle, angle_diff = is_facing_cardinal_direction(self.imu_provider, 0.0, tolerance_deg=0.5)
                if not is_target:
                    if angle_diff > 0:
                        cmd_msg.data = [4, 20]  # 宸﹁浆
                    else:
                        cmd_msg.data = [5, -20]  # 鍙宠浆
                    self.motion_cmd_publisher.publish(cmd_msg)
                    self.get_logger().info(f"[WAREHOUSE_B_1_BACK_STONE_ROAD] 杩涘叆鐘舵€侊紝寰皟瀵瑰噯0搴︼紝褰撳墠瑙掑害宸? {angle_diff:.2f}掳")
                    time.sleep(1.0)
                    cmd_msg.data = [0]
                    self.motion_cmd_publisher.publish(cmd_msg)
                    time.sleep(0.5)
                else:
                    self.get_logger().info("[WAREHOUSE_B_1_BACK_STONE_ROAD] 宸叉瀵?搴︼紝鏃犻渶寰皟")
            # 姝ｅ父琛岃蛋闃舵
            if not self._stone_road_cmd_sent:
                cmd_msg.data = [10, 150, 300, -200]
                self.motion_cmd_publisher.publish(cmd_msg)
                self._stone_road_cmd_sent = True
                self.get_logger().info("[LOG_REPAIRED]")
                return
            if time.time() - self._stone_road_started_at >= 2.0 and int((time.time() - self._stone_road_started_at) * 10) % 12 == 0:
                need_adjust, adjust_direction, angle_error = get_adjustment_direction(
                    self.imu_provider, 0.0, tolerance_deg=2.0
                )
                if need_adjust:
                    cmd_msg.data = [4, 35] if adjust_direction == 'left' else [5, -35]
                    self.motion_cmd_publisher.publish(cmd_msg)
                    return
                cmd_msg.data = [10, 150, 300, -200]
                self.motion_cmd_publisher.publish(cmd_msg)
            # 妫€娴嬪彸涓嬭榛勮壊娑堝け
            if img is not None:
                yellow_detected = detect_yellow(img, x_start=0.5, x_end=1.0, y_start=0.7, y_end=1.0)
                if not yellow_detected:
                    if self._yellow_gone_start_time is None:
                        self._yellow_gone_start_time = time.time()
                        self.get_logger().info("[LOG_REPAIRED]")
                    elif time.time() - self._yellow_gone_start_time >= 3.5:
                        self.get_logger().info("[LOG_REPAIRED]")
                        cmd_msg.data = [0]
                        self.motion_cmd_publisher.publish(cmd_msg)
                        time.sleep(0.5)
                        self.state = "WAREHOUSE_B_1_BACK_TURN_LEFT"
                        self._turn_left_back_cmd_sent = False
                        return
                else:
                    self._yellow_gone_start_time = None

        elif self.state == "WAREHOUSE_B_1_BACK_TURN_LEFT":
            if not self._turn_left_back_cmd_sent:
                cmd_msg.data = [4, 150]  # 宸﹁浆锛岃閫熷害0.15
                self.motion_cmd_publisher.publish(cmd_msg)
                self._turn_left_back_cmd_sent = True
                self.get_logger().info("[LOG_REPAIRED]")
                return
            is_north, current_angle, angle_diff = is_facing_cardinal_direction(self.imu_provider, 0.0)
            self.get_logger().info(f"[WAREHOUSE_B_1_BACK_TURN_LEFT] 鏄惁姝ｅ鍖? {is_north}锛岃搴﹀樊{angle_diff:.2f}掳")
            if is_north:
                cmd_msg.data = [0]  # 绔欑珛
                self.motion_cmd_publisher.publish(cmd_msg)
                time.sleep(1.0)  # 绛夊緟绋冲畾
                self.state = "WAREHOUSE_B_1_BACK_TO_S"
                self._back_to_s_cmd_sent = False
                self._back_to_s_stage = 0
                return

        elif self.state == "WAREHOUSE_B_1_BACK_TO_S":
            if not hasattr(self, '_back_to_s_stage'):
                self._back_to_s_stage = 0
                self._back_to_s_cmd_sent = False

            if self._back_to_s_stage == 0:  # 鍓嶈繘闃舵
                track_result = self.track_detector.detect(img)
                self._publish_track_follow_command(cmd_msg, track_result, cruise_speed=95)
                self._back_to_s_cmd_sent = True

                # 妫€娴嬪乏涓嬭榛勮壊
                if img is not None:
                    yellow_detected = detect_yellow(img, x_start=0, x_end=0.5, y_start=0.9, y_end=1.0)
                    if not yellow_detected:
                        time.sleep(1.5)
                        cmd_msg.data = [0]  # 绔欑珛
                        self.motion_cmd_publisher.publish(cmd_msg)
                        time.sleep(1.0)
                        self._back_to_s_stage = 1
                        self._back_to_s_cmd_sent = False
                        self.get_logger().info("[WAREHOUSE_B_1_BACK_TO_S] 宸︿笅瑙掗粍鑹叉秷澶憋紝鍑嗗鍙宠浆")
                        return

            elif self._back_to_s_stage == 1:  # 鍙宠浆闃舵
                if not self._back_to_s_cmd_sent:
                    cmd_msg.data = [5, -150]  # 鍙宠浆锛岃閫熷害-0.15
                    self.motion_cmd_publisher.publish(cmd_msg)
                    self._back_to_s_cmd_sent = True
                    self.get_logger().info("[LOG_REPAIRED]")
                    return

                # 妫€鏌ユ槸鍚﹁浆鍒?61搴?
                is_target, current_angle, angle_diff = is_facing_cardinal_direction(self.imu_provider, 261.0, tolerance_deg=0.5)
                self.get_logger().info(f"[WAREHOUSE_B_1_BACK_TO_S] 鏄惁姝ｅ261搴? {is_target}锛岃搴﹀樊{angle_diff:.2f}掳")
                if is_target:
                    cmd_msg.data = [0]  # 绔欑珛
                    self.motion_cmd_publisher.publish(cmd_msg)
                    time.sleep(1.0)
                    self._back_to_s_stage = 2
                    self._back_to_s_cmd_sent = False
                    self.get_logger().info("[WAREHOUSE_B_1_BACK_TO_S] 鍙宠浆鍒?61搴︼紝鍑嗗鍓嶈繘")
                    return

            elif self._back_to_s_stage == 2:  # 鍓嶈繘闃舵
                track_result = self.track_detector.detect(img)
                self._publish_track_follow_command(cmd_msg, track_result, cruise_speed=90)
                self._back_to_s_cmd_sent = True

                # 妫€娴嬩笅鏂归粍鑹?
                if img is not None:
                    yellow_detected = detect_yellow(img, y_start=0.8, y_end=1.0)
                    if not yellow_detected:
                        cmd_msg.data = [0]  # 绔欑珛
                        self.motion_cmd_publisher.publish(cmd_msg)
                        time.sleep(1.0)
                        self.return_arrow = self._confirm_arrow_direction(img) or "Left"
                        self.state = "S_CURVE_BACK"
                        self.get_logger().info(f"[WAREHOUSE_B_1_BACK_TO_S] 返回箭头方向: {self.return_arrow}")
                        return

        elif self.state == "S_CURVE_BACK":
            if not hasattr(self, '_s_curve_back_stage'):
                self._s_curve_back_stage = 0
                self._s_curve_back_entered_at = time.time()
                self._s_curve_back_track_lost_since = None
                self.get_logger().info("[S_CURVE_BACK] 切换为视觉闭环返程")

            track_result = self.track_detector.detect(img)
            self._publish_track_follow_command(cmd_msg, track_result, cruise_speed=88)

            elapsed = time.time() - self._s_curve_back_entered_at
            exit_by_track_loss = elapsed >= 4.0 and self._track_lost_long_enough(
                "s_curve_back", track_result, timeout_sec=0.7, confidence_thresh=0.40
            )
            exit_by_timeout = elapsed >= 12.0
            exit_by_bottom_clear = (
                elapsed >= 4.0 and
                not detect_yellow(img, x_start=0.2, x_end=0.8, y_start=0.88, y_end=1.0, yellow_thresh=0.02)
            )

            if exit_by_track_loss or exit_by_timeout or exit_by_bottom_clear:
                cmd_msg.data = [0]
                self.motion_cmd_publisher.publish(cmd_msg)
                time.sleep(0.6)
                self.state = "OUT_S_LEFT_TURN"
                self._out_s_left_turn_cmd_sent = False
                self._out_s_left_turn_stage = 0
                self.get_logger().info("[S_CURVE_BACK] 完成，切换到出 S 弯")

        # ---- 浠嶴寮繑鍥炲悗宸﹁浆鍑哄簱 ----
        elif self.state == "OUT_S_LEFT_TURN":
            if not hasattr(self, '_out_s_left_turn_stage'):
                self._out_s_left_turn_stage = 0
                self._out_s_left_turn_cmd_sent = False

            if self._out_s_left_turn_stage == 0:
                if not self._out_s_left_turn_cmd_sent:
                    self.motion_cmd_publisher.publish(cmd_msg)
                    self._out_s_left_turn_cmd_sent = True
                    self._out_s_left_turn_start_time = time.time()
                    self.get_logger().info("[LOG_REPAIRED]")
                    return

                if time.time() - self._out_s_left_turn_start_time >= 2.0:
                    cmd_msg.data = [0]  # 绔欑珛
                    self.motion_cmd_publisher.publish(cmd_msg)
                    time.sleep(1.0)
                    self._out_s_left_turn_stage = 1
                    self._out_s_left_turn_cmd_sent = False
                    self.get_logger().info("[OUT_S_LEFT_TURN] 鍓嶈繘2绉掑畬鎴愶紝鍑嗗宸﹁浆")
                    return

            elif self._out_s_left_turn_stage == 1:
                if not self._out_s_left_turn_cmd_sent:
                    self.motion_cmd_publisher.publish(cmd_msg)
                    self._out_s_left_turn_cmd_sent = True
                    self.get_logger().info("[LOG_REPAIRED]")
                    return

                is_west, current_angle, angle_diff = is_facing_cardinal_direction(self.imu_provider, 270.0, tolerance_deg=1.0)
                if is_west:
                    cmd_msg.data = [0]  # 绔欑珛
                    self.motion_cmd_publisher.publish(cmd_msg)
                    time.sleep(1.0)
                    self._out_s_left_turn_stage = 2
                    self._out_s_left_turn_cmd_sent = False
                    self.get_logger().info("[OUT_S_LEFT_TURN] 宸﹁浆鍒?70搴﹀畬鎴愶紝鍑嗗鍓嶈繘")
                    return
                else:
                    self._out_s_left_turn_cmd_sent = False
                    return

            elif self._out_s_left_turn_stage == 2:
                if not self._out_s_left_turn_cmd_sent:
                    self.motion_cmd_publisher.publish(cmd_msg)
                    self._out_s_left_turn_cmd_sent = True
                    self.get_logger().info("[OUT_S_LEFT_TURN] 寮€濮嬪墠杩涳紝绛夊緟榛勮壊娑堝け")
                    return

                if not detect_yellow(img, y_start=0):
                    time.sleep(1.0)
                    cmd_msg.data = [0]  # 绔欑珛
                    self.motion_cmd_publisher.publish(cmd_msg)
                    time.sleep(1.0)
                    self.get_logger().info("[LOG_REPAIRED]")
                    self.state = "WAREHOUSE_A_1_TURN_LEFT_TO_NORTH"
                    self._a1_turn_left_to_north_cmd_sent = False
                    return

        elif self.state == "WAREHOUSE_A_1_TURN_LEFT_TO_NORTH":
            if not self._a1_turn_left_to_north_cmd_sent:
                cmd_msg.data = [4, 250]  # 宸﹁浆锛岃閫熷害0.25
                self.motion_cmd_publisher.publish(cmd_msg)
                self._a1_turn_left_to_north_cmd_sent = True
                self.get_logger().info("[LOG_REPAIRED]")
                return

            is_north, current_angle, angle_diff = is_facing_cardinal_direction(self.imu_provider, 0.0, tolerance_deg=1.0)
            if is_north:
                cmd_msg.data = [0]  # 绔欑珛
                self.motion_cmd_publisher.publish(cmd_msg)
                time.sleep(1.0)
                self.state = "WAREHOUSE_A_1_TURN_LEFT_TO_WEST"
                self._a1_turn_left_to_west_cmd_sent = False
                return
            else:
                self._a1_turn_left_to_north_cmd_sent = False

        elif self.state == "WAREHOUSE_A_1_TURN_LEFT_TO_WEST":
            if not hasattr(self, '_a1_turn_left_to_west_stage'):
                self._a1_turn_left_to_west_stage = 0

            if self._a1_turn_left_to_west_stage == 0:  # 鍓嶈繘闃舵
                if not self._a1_turn_left_to_west_cmd_sent:
                    cmd_msg.data = [2, 200, 500]  # 浣庡ご琛岃蛋锛岄€熷害0.2锛屼刊浠拌0.5
                    self.motion_cmd_publisher.publish(cmd_msg)
                    self._a1_turn_left_to_west_cmd_sent = True
                    self.get_logger().info("[LOG_REPAIRED]")
                    return

                if not detect_yellow(img, y_start=0):
                    time.sleep(2.0)
                    cmd_msg.data = [0]  # 绔欑珛
                    self.motion_cmd_publisher.publish(cmd_msg)
                    time.sleep(1.0)
                    self._a1_turn_left_to_west_stage = 1
                    self._a1_turn_left_to_west_cmd_sent = False
                    return

            elif self._a1_turn_left_to_west_stage == 1:  # 宸﹁浆闃舵
                if not self._a1_turn_left_to_west_cmd_sent:
                    cmd_msg.data = [4, 150]  # 宸﹁浆锛岃閫熷害0.15
                    self.motion_cmd_publisher.publish(cmd_msg)
                    self._a1_turn_left_to_west_cmd_sent = True
                    self.get_logger().info("[LOG_REPAIRED]")
                    return

                is_west, current_angle, angle_diff = is_facing_cardinal_direction(self.imu_provider, 90.0, tolerance_deg=1.0)
                if is_west:
                    cmd_msg.data = [0]  # 绔欑珛
                    self.motion_cmd_publisher.publish(cmd_msg)
                    time.sleep(1.0)
                    self.state = "WAREHOUSE_A_1_IN"
                    self._a1_in_cmd_sent = False
                    return
                else:
                    self._a1_turn_left_to_west_cmd_sent = False

        elif self.state == "WAREHOUSE_A_1_IN":
            if not self._a1_in_cmd_sent:
                cmd_msg.data = [2, 200, 500]  # 浣庡ご琛岃蛋锛岄€熷害0.2锛屼刊浠拌0.5
                self.motion_cmd_publisher.publish(cmd_msg)
                self._a1_in_cmd_sent = True
                time.sleep(6.0)
                cmd_msg.data = [0]  # 绔欑珛
                self.motion_cmd_publisher.publish(cmd_msg)
                time.sleep(2.0)
                self.state = "WAREHOUSE_A_1_DOWN"
                self._a1_down_cmd_sent = False
                return

        elif self.state == "WAREHOUSE_A_1_DOWN":
            if not self._a1_down_cmd_sent:
                cmd_msg.data = [1]  # 瓒翠笅
                self.motion_cmd_publisher.publish(cmd_msg)
                self._a1_down_cmd_sent = True
                self.get_logger().info("[LOG_REPAIRED]")
                time.sleep(5.0)  # 绛夊緟5绉?                cmd_msg.data = [0]  # 绔欑珛
                self.motion_cmd_publisher.publish(cmd_msg)
                time.sleep(3.0)
                self.get_logger().info("[WAREHOUSE_A_1_DOWN] 瓒翠笅瀹屾垚")
                self.state = "WAREHOUSE_A_1_OUT"  # 鍒囨崲鍒板嚭搴撶姸鎬?                return

        elif self.state == "WAREHOUSE_A_1_OUT":
            if not self._up_back_walk_cmd_sent:
                cmd_msg.data = [3, -200, -500]  # 鎶ご鍚庨€€琛岃蛋锛岄€熷害-0.15锛屼刊浠拌-0.5
                self.motion_cmd_publisher.publish(cmd_msg)
                self._up_back_walk_cmd_sent = True
                self.get_logger().info("[WAREHOUSE_A_1_OUT] 鍙戦€佹姮澶村悗閫€鍛戒护")
                time.sleep(4.5)  # 鍚庨€€4.5绉?                cmd_msg.data = [0]  # 绔欑珛
                self.motion_cmd_publisher.publish(cmd_msg)
                time.sleep(1.0)
                self.state = "WAREHOUSE_A_1_BACK_TURN_LEFT"
                self._a1_back_turn_left_cmd_sent = False
                return

        elif self.state == "WAREHOUSE_A_1_BACK_TURN_LEFT":
            if not hasattr(self, '_a1_back_turn_left_stage'):
                self._a1_back_turn_left_stage = 0

            if self._a1_back_turn_left_stage == 0:  # 宸﹁浆闃舵
                if not self._a1_back_turn_left_cmd_sent:
                    cmd_msg.data = [4, 250]  # 宸﹁浆锛岃閫熷害0.25
                    self.motion_cmd_publisher.publish(cmd_msg)
                    self._a1_back_turn_left_cmd_sent = True
                    self.get_logger().info("[LOG_REPAIRED]")
                    return

                is_south, current_angle, angle_diff = is_facing_cardinal_direction(self.imu_provider, 180.0, tolerance_deg=2.0)
                if is_south:
                    cmd_msg.data = [0]  # 绔欑珛
                    self.motion_cmd_publisher.publish(cmd_msg)
                    time.sleep(1.0)
                    self._a1_back_turn_left_stage = 1
                    self._a1_back_turn_left_cmd_sent = False
                    self.get_logger().info("[LOG_REPAIRED]")
                    return
                else:
                    self._a1_back_turn_left_cmd_sent = False
                    return

            elif self._a1_back_turn_left_stage == 1:  # 鍓嶈繘闃舵
                if not self._a1_back_turn_left_cmd_sent:
                    cmd_msg.data = [2, 200, 500]  # 浣庡ご琛岃蛋锛岄€熷害0.2锛屼刊浠拌0.5
                    self.motion_cmd_publisher.publish(cmd_msg)
                    self._a1_back_turn_left_cmd_sent = True
                    self.get_logger().info("[LOG_REPAIRED]")
                    time.sleep(6.0)  # 鐩存帴绛夊緟5绉?                    cmd_msg.data = [0]  # 绔欑珛
                    self.motion_cmd_publisher.publish(cmd_msg)
                    time.sleep(1.0)
                    self.state = "WAREHOUSE_A_1_BACK_RIGHT_TURN"
                    self._a1_back_right_turn_cmd_sent = False
                    return

        elif self.state == "WAREHOUSE_A_1_BACK_RIGHT_TURN":
            if not hasattr(self, '_a1_back_right_turn_stage'):
                self._a1_back_right_turn_stage = 0

            if self._a1_back_right_turn_stage == 0:  # 鍙宠浆闃舵
                if not self._a1_back_right_turn_cmd_sent:
                    cmd_msg.data = [5, -250]  # 鍙宠浆锛岃閫熷害-0.25
                    self.motion_cmd_publisher.publish(cmd_msg)
                    self._a1_back_right_turn_cmd_sent = True
                    self.get_logger().info("[LOG_REPAIRED]")
                    return

                is_east, current_angle, angle_diff = is_facing_cardinal_direction(self.imu_provider, 90.0, tolerance_deg=2.0)
                if is_east:
                    cmd_msg.data = [0]  # 绔欑珛
                    self.motion_cmd_publisher.publish(cmd_msg)
                    time.sleep(1.0)
                    self._a1_back_right_turn_stage = 1
                    self._a1_back_right_turn_cmd_sent = False
                    self.get_logger().info("[LOG_REPAIRED]")
                    return
                else:
                    self._a1_back_right_turn_cmd_sent = False
                    return

            elif self._a1_back_right_turn_stage == 1:  # 鍓嶈繘闃舵
                if not self._a1_back_right_turn_cmd_sent:
                    cmd_msg.data = [2, 200, 500]  # 浣庡ご琛岃蛋锛岄€熷害0.2锛屼刊浠拌0.2
                    self.motion_cmd_publisher.publish(cmd_msg)
                    self._a1_back_right_turn_cmd_sent = True
                    self.get_logger().info("[LOG_REPAIRED]")
                    return

                if detect_yellow(img, x_start=0, x_end=1.0, y_start=0.9, y_end=1.0, yellow_thresh=0.9):
                    time.sleep(3.5)
                    cmd_msg.data = [0]  # 绔欑珛
                    self.motion_cmd_publisher.publish(cmd_msg)
                    time.sleep(1.0)
                    self.state = "TO_HOME_LEFT_TURN"
                    self._to_home_left_turn_cmd_sent = False
                    return

        elif self.state == "TO_HOME_LEFT_TURN":
            if not hasattr(self, '_to_home_left_turn_stage'):
                self._to_home_left_turn_stage = 0

            if self._to_home_left_turn_stage == 0:  # 宸﹁浆闃舵
                if not self._to_home_left_turn_cmd_sent:
                    cmd_msg.data = [4, 250]  # 宸﹁浆锛岃閫熷害0.25
                    self.motion_cmd_publisher.publish(cmd_msg)
                    self._to_home_left_turn_cmd_sent = True
                    self.get_logger().info("[LOG_REPAIRED]")
                    return

                is_south, current_angle, angle_diff = is_facing_cardinal_direction(self.imu_provider, 180.0, tolerance_deg=2.0)
                if is_south:
                    cmd_msg.data = [0]  # 绔欑珛
                    self.motion_cmd_publisher.publish(cmd_msg)
                    time.sleep(1.0)
                    self._to_home_left_turn_stage = 1
                    self._to_home_left_turn_cmd_sent = False
                    self.get_logger().info("[LOG_REPAIRED]")
                    return
                else:
                    self._to_home_left_turn_cmd_sent = False
                    return

            elif self._to_home_left_turn_stage == 1:  # 鍓嶈繘闃舵
                if not self._to_home_left_turn_cmd_sent:
                    cmd_msg.data = [2, 200, 500]  # 浣庡ご琛岃蛋锛岄€熷害0.2锛屼刊浠拌0.5
                    self.motion_cmd_publisher.publish(cmd_msg)
                    self._to_home_left_turn_cmd_sent = True
                    self.get_logger().info("[LOG_REPAIRED]")
                    return

                if not detect_yellow(img, y_start=0):
                    time.sleep(1.5)
                    cmd_msg.data = [0]  # 绔欑珛
                    self.motion_cmd_publisher.publish(cmd_msg)
                    time.sleep(1.0)
                    self.state = "TO_HOME_IN"
                    self._to_home_in_cmd_sent = False
                    return

        elif self.state == "TO_HOME_IN":
            if not self._to_home_in_cmd_sent:
                cmd_msg.data = [5, -250]  # 鍙宠浆锛岃閫熷害-0.25
                self.motion_cmd_publisher.publish(cmd_msg)
                self._to_home_in_cmd_sent = True
                self.get_logger().info("[LOG_REPAIRED]")
                return

            is_north, current_angle, angle_diff = is_facing_cardinal_direction(self.imu_provider, 0.0, tolerance_deg=2.0)
            if is_north:
                cmd_msg.data = [0]  # 绔欑珛
                self.motion_cmd_publisher.publish(cmd_msg)
                time.sleep(1.0)
                self.state = "TO_HOME_DOWN"
                self._to_home_down_cmd_sent = False
                return
            else:
                self._to_home_in_cmd_sent = False
                return

        elif self.state == "TO_HOME_DOWN":
            if not self._to_home_down_cmd_sent:
                cmd_msg.data = [1]  # 瓒翠笅
                self.motion_cmd_publisher.publish(cmd_msg)
                self._to_home_down_cmd_sent = True
                self.get_logger().info("[TO_HOME_DOWN] 瓒翠笅瀹屾垚")
                self.state = "FINISH"  # 鍒囨崲鍒板畬鎴愮姸鎬?                return
        
        elif self.state == "FINISH":
            self.get_logger().info("[FINISH] 瀹屾垚")
            # 鍙戝竷瀹屾垚淇″彿
            completion_msg = Bool()
            completion_msg.data = True
            self.completion_publisher.publish(completion_msg)
            self.get_logger().info("[LOG_REPAIRED]")
            return

def main(args=None):
    rclpy.init(args=args)
    node = RobotStateMachine()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.provider.shutdown()
        node.d435_provider.shutdown()
        node.imu_provider.shutdown() 
        cv2.destroyAllWindows()
        node.get_logger().info("閿€姣佹満鍣ㄤ汉鐘舵€佹満鑺傜偣...")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
